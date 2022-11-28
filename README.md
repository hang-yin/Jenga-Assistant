**Group Members:**
Katie Hughes, Alyssa Chen, Hang Yin, Liz Metzger

**Instructions:**

To plan or execute you need to call our service `/go_here` (there are sample commands below), which
takes in start pose, goal pose, and a boolean to indicate whether to plan or execute. If there is no 
start position indicated then the plan/execute happens with the robot's current position. If there 
is one given then the robot will plan from that postion. If there is more than one start position 
given then the program throws you an error.

To place a obstacle call the `/place` service (sample command below) and give it a pose. A block
will spawn at the indicated location realtive to the end effector. If the service is called again 
then a new block is not made, the current block just moves. 

Our node defaults to using our `plan_to_pose` function which takes the user input and moves the 
robot to the indicated pose. To see the robot move and execute you can run one of the service calls 
with no start position indicated and execute set to true. To test the other functions (`plan_to_orientation` or `plan_to_position`) you can
comment out the current call in the node and uncomment one of the other ones.

Sometime our IK service doesn't load up and the node crashes, to fix it relaunch the program.

# Sample Commands

## Run the Launch File
`ros2 launch plan_execute simple_move.launch.py`

## 0 initial start positions

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

## 1 initial start positions

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


####################################################################

Install April Tag packages: 
`vcs import --recursive --input src/jengavision/camera/config/jenga_vision.repo src/`

How to run:

1. Plug into the Franka and the realsense camera.
2. `ssh student@station`, then `ros2 launch franka_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot`
3. `ros2 launch franka_moveit_config rviz.launch.py robot_ip:=panda0.robot`
4. From the workspace containing our packages, run `ros2 run plan_execute cv_test`
5. Run `ros2 service call /calibrate std_srvs/srv/Empty` to move the robot into the calibration position. Insert the april tag into the grippers.
6. Run `ros2 launch camera jenga_vision.launch.py calibrate:=true`. After you see a message that indicates that `tf.yaml` has been written to, you can CTRL-C. The file `tf.yaml` will be saved in `${root_workspace}/install/camera/share/camera` and will be loaded automatically in future runs. 
7. From terminal 5 run `ros2 service call /ready std_srvs/srv/Empty` to return the robot to the ready position. Remove the april tag from the grippers.
8. Run `ros2 launch camera jenga_vision.launch.py`
9.  In the pop up window, ensure that the tower is visible in the color frame and make sure that it is inside the bounding square (if not, adjust the size with the trackbars)
10. Run `ros2 service call /findtower std_srvs/srv/Empty`. This will find the top of the jenga tower and the table
11. Remove a piece about halfway from the tower (and ensure you can see it from the camera frame). Run `ros2 service call /scan std_srvs/srv/Empty`. This will look for the piece in between the top of the tower and the table.
12. Theoretically this should be it!