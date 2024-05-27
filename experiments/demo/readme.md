## Common commands

`change_tool(tool_name1:str, tool_name2:str)`
- ask the robot to change the gripper in its hand from tool_name1 to tool_name2

`joint_move(object_name: str)`
- ask the robot to move a pre-defined joint position

`cartesian_move(object_name: str)`
- ask the robot to move a pre-defined cartesian position

`teach_object_TCP(object_name: str)`
- store the current robot TCP as the TCP pose of the object
- the data is stored in mongoDB with `object_name` as the key
- joint pose is stored as well

`backup_mios_environment(backup_name: str)`
- backup the current environment to a file with the name `backup_name`

`restore_to_mios_environment(backup_name: str)`
- restore the environment from a file with the name `backup_name`

`set_tool(tool_name: str)`
- set the gripper in the robot hand.
- the default tool is `default_gripper`
- after setting the gripper, the TCP of the robot is changed accordingly

`grasp()`
- ask the robot to grasp the object
- based on the api in libfranka

`move_gripper(width: float)`
- ask the robot to move the gripper to a certain width
- range: [0.01, 0.08]
- with a gripper in the hand, the maximum width is 0.04

## Grippers
- `default_gripper`
- `clampgripper`
- `parallelgripper`
- `outwardgripper`
- `inwardgripper`

## Assembly relationships
1. `shaft2` is inserted into `gearbase_hole2`
2. `shaft1` is inserted into `gearbase_hole1`
3. `shaft3` is inserted into `gearbase_hole3`
4. `gear2` is inserted into `gearbase_shaft2`
5. `gear1` is inserted into `gearbase_shaft1`
6. `gear3` is inserted into `gearbase_shaft3`
