README!

# Sample Commands

## 0 initial start orientations

**plan_to_orientation:**

execute:

`ros2 service call /go_here plan_execute_interface/srv/GoHere '{start_pose: [], goal_pose: {position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: .707, y: 0.707, z: 0.0, w: 1}}, execute: 'true'}'`

plan:

`ros2 service call /go_here plan_execute_interface/srv/GoHere '{start_pose: [], goal_pose: {position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: 0.707, y: 0.707, z: 0.0, w: 1}}, execute: 'false'}'`

**plan_to_pos:**

execute:

`ros2 service call /go_here plan_execute_interface/srv/GoHere '{start_pose: [], goal_pose: {position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: .707, y: 0.707, z: 0.0, w: 1}}, execute: 'true'}'`

plan:

`ros2 service call /go_here plan_execute_interface/srv/GoHere '{start_pose: [], goal_pose: {position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: .707, y: 0.707, z: 0.0, w: 1}}, execute: 'false'}'`

**plan_to_position:**

execute:

`ros2 service call /go_here plan_execute_interface/srv/GoHere '{start_pose: [], goal_pose: {position: {x: 0.3, y: 0.3, z: 0.3}, orientation: {x: .707, y: 0.707, z: 0.0, w: 1}}, execute: 'true'}'`

plan:

`ros2 service call /go_here plan_execute_interface/srv/GoHere '{start_pose: [], goal_pose: {position: {x: 0.3, y: 0.3, z: 0.3}, orientation: {x: .707, y: 0.707, z: 0.0, w: 1}}, execute: 'false'}'`

## 1 initial start orientations

Will only ever plan even if the execute value is true since the robot can't teleport. 
plan_to_orientation:

plan:

`ros2 service call /go_here plan_execute_interface/srv/GoHere '{start_pose: [{position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}], goal_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.707, y: 0.707, z: 0, w: 1}},execute: 'true'}'`

**plan_to_pos:**

plan:

`ros2 service call /go_here plan_execute_interface/srv/GoHere '{start_pose: [{position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}], goal_pose: {position: {x: 0.2, y: 0.2, z: 0.2}, orientation: {x: 0.707, y: 0.707, z: 0, w: 1}},execute: 'true'}'`

**plan_to_position:**

plan:

`ros2 service call /go_here plan_execute_interface/srv/GoHere '{start_pose: [{position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}], goal_pose: {position: {x: 0.2, y: 0.2, z: 0.2}, orientation: {x: 0.707, y: 0.707, z: 0, w: 1}},execute: 'true'}'`

## Place the block

`ros2 service call /place plan_execute_interface/srv/Place '{place: {position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: 0.5, y: 0.5, z: 0.5, w: 1}}}'`