"""
DEPRECATED — coord-frame handling moved INTO `rectify`.

The world frame is now defined directly on the rectify node (params:
`world_reference_tag_id`, `world_tag_corner`, `world_table_w_mm`,
`world_table_h_mm`, `world_lock_when_seen`). Rectify emits a `coord_state`
JSON output that downstream nodes (`localization`, `parallax`, etc.)
read via an optional input port to flip their world coords.

This module is kept as a stub so importing it doesn't fail on graphs
saved before the migration. Nothing is registered.
"""
