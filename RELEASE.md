# Release Notes

## v0.2.12

### Bug Fixes

#### 1. Unicode Encoding Errors (Windows GBK codec)
Replaced all emoji characters with ASCII text for Windows compatibility:
- `ik_op.py`: Replaced checkmark/cross emojis with `SUCCESS`/`FAILED`
- `planner_ompl_with_collision_op.py`: Replaced checkmark/cross emojis with `SUCCESS`/`FAILED`
- `collision_check_op.py`: Replaced status emojis with `COLLISION`/`CLEAR`
- `demo_node.py`: Replaced emojis with `[OK]`/`[FAIL]`/`[COLLISION]`/`[CLEAR]`
- `motion_commander.py`: Removed all emoji characters
- `collision_lib.py`: Replaced checkmark emoji with `[OK]`

#### 2. Scene Update Parsing Error ('object' key missing)
Fixed scene update handling in `collision_check_op.py` and `planner_ompl_with_collision_op.py`:
- Now supports both full scene broadcasts (`{"world_objects": [...]}`) and single object commands (`{"action": "add", "object": {...}}`)
- Full scene sync clears and rebuilds all obstacles from `planning_scene_op.py` broadcasts
- Single object commands continue to work for incremental updates
