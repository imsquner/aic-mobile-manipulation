"""Competition task package.

Read ``orchestration`` for task order and state changes, ``planning`` for cube
selection and routes, ``navigation`` and ``grasping`` for execution, and
``placement`` for release. World2 slot handling lives in
``placement.zone_slots``. Root-level modules with the old names are import
compatibility shims; ``task_orchestrator_node`` remains the ROS node entry.
"""
