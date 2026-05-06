"""
services.vision — multi-source vision pipeline framework.

The match-time `VisionBackend` (in services/vision_backend.py) sits on top
of this. New pipelines / nodes live here so the design can grow without
bloating the backend module.

Public surface:
    Pipeline      — DAG of Nodes; runs in its own worker thread
    Node, NodeIO  — base classes for nodes + their typed I/O
    NODE_KINDS    — registry { kind_str: NodeClass }
    PortKind      — frame | detection | pose_list | mask | json
"""
from .pipeline import Pipeline, PipelineRegistry, NodeIO, Port, PortKind
from .nodes import Node, NODE_KINDS, register_node

__all__ = [
    'Pipeline', 'PipelineRegistry',
    'Node', 'NodeIO', 'Port', 'PortKind',
    'NODE_KINDS', 'register_node',
]
