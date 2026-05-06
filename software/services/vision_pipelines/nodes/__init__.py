"""
Node implementations for services.vision.

Each node is a subclass of `Node` registered via `register_node('kind', cls)`.
The Pipeline scheduler instantiates nodes from the saved JSON graph using
NODE_KINDS.
"""
from .base import Node, register_node, NODE_KINDS

# Importing the node modules registers them as side effect.
from . import source        # noqa: F401
from . import undistort     # noqa: F401
from . import rectify       # noqa: F401
from . import aruco         # noqa: F401
from . import preprocess    # noqa: F401
from . import localization  # noqa: F401
from . import parallax      # noqa: F401
from . import camera        # noqa: F401  (camera.manual + camera.auto)
from . import overlay       # noqa: F401
from . import filter as _filter  # noqa: F401  (avoids shadowing builtin)
from . import output        # noqa: F401
from . import output_data   # noqa: F401  (output.pose_list + output.json)

__all__ = ['Node', 'register_node', 'NODE_KINDS']
