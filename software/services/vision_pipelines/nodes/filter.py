"""
filter nodes — narrow a pose_list down to a subset of its entries.

Two simple filters that compose well in series:

    filter.id_range
        Keep poses whose `tag_id` falls within [id_min, id_max].
        Useful for "anchors only" (20..23) or "robots only" (1..10).

    filter.classification
        Keep poses whose `classification` field matches a selected set
        (own / opponent / anchor / other). The localization node already
        labels poses based on the current team color, so the same
        `keep_own` checkbox produces the right output for blue and yellow
        — that's the team-conditional behavior the user asked about.

Both nodes pass the rest of each pose dict through untouched.
"""

from __future__ import annotations

from ..pipeline import NodeIO, Port, PortKind
from .base import Node, register_node


@register_node('filter.id_range')
class FilterIdRangeNode(Node):
    IO = NodeIO(
        inputs=[Port('pose_list', PortKind.POSE_LIST)],
        outputs=[Port('pose_list', PortKind.POSE_LIST,
                      'pose_list filtered to [id_min, id_max]')],
    )
    params_schema = {
        'id_min':  {'type': 'int', 'default': 1,  'label': 'min tag id',
                    'min': 0, 'max': 99,
                    'description': 'lowest tag id kept (inclusive)'},
        'id_max':  {'type': 'int', 'default': 10, 'label': 'max tag id',
                    'min': 0, 'max': 99,
                    'description': 'highest tag id kept (inclusive)'},
        'invert':  {'type': 'bool', 'default': False, 'label': 'invert',
                    'description': 'when on, DROP poses in range and keep the rest'},
    }

    def process(self, inputs):
        poses = inputs.get('pose_list')
        if poses is None:
            return {}
        lo = int(self._params.get('id_min', 1))
        hi = int(self._params.get('id_max', 10))
        invert = bool(self._params.get('invert', False))
        out = []
        for q in poses:
            if not isinstance(q, dict):
                continue
            tid = q.get('tag_id')
            if tid is None:
                continue
            in_range = (lo <= int(tid) <= hi)
            if in_range != invert:
                out.append(q)
        return {'pose_list': out}

    def get_state(self):
        return {
            'id_min': int(self._params.get('id_min', 1)),
            'id_max': int(self._params.get('id_max', 10)),
            'invert': bool(self._params.get('invert', False)),
            'last_error': self._last_error,
        }


@register_node('filter.classification')
class FilterClassificationNode(Node):
    """Keep poses whose classification matches one of the enabled classes.

    The classification labels come from the upstream localization node and
    are TEAM-AWARE — `own` means "own team's robots" regardless of whether
    the team is blue or yellow. So a single 'keep_own' checkbox does the
    right thing for both teams (this is the team-condition the user asked
    about, encoded directly in the data flow).
    """
    IO = NodeIO(
        inputs=[Port('pose_list', PortKind.POSE_LIST)],
        outputs=[Port('pose_list', PortKind.POSE_LIST,
                      'pose_list filtered by classification')],
    )
    params_schema = {
        'keep_own':      {'type': 'bool', 'default': True,  'label': 'keep own team'},
        'keep_opponent': {'type': 'bool', 'default': False, 'label': 'keep opponent'},
        'keep_anchor':   {'type': 'bool', 'default': False, 'label': 'keep anchors'},
        'keep_other':    {'type': 'bool', 'default': False, 'label': 'keep other'},
    }

    def process(self, inputs):
        poses = inputs.get('pose_list')
        if poses is None:
            return {}
        keep = {
            'own':      bool(self._params.get('keep_own',      True)),
            'opponent': bool(self._params.get('keep_opponent', False)),
            'anchor':   bool(self._params.get('keep_anchor',   False)),
            'other':    bool(self._params.get('keep_other',    False)),
            # Synonyms in case upstream uses different labels
            'own robot': bool(self._params.get('keep_own',     True)),
            'object':   bool(self._params.get('keep_other',    False)),
        }
        out = [q for q in poses
               if isinstance(q, dict)
               and keep.get(str(q.get('classification', 'other')), False)]
        return {'pose_list': out}

    def get_state(self):
        return {
            'keeping': [k for k in ('own', 'opponent', 'anchor', 'other')
                        if self._params.get(f'keep_{k}', False)],
            'last_error': self._last_error,
        }
