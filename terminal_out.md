# TEST
```zsh
➜  plan_ws git:(main) ✗ ros2 launch plan_manage test_node.launch.py
[INFO] [launch]: All log files can be found below /home/jack/.ros/log/2025-08-06-22-29-54-800371-jack-120497
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rviz2-1]: process started with pid [120498]
[INFO] [test_node-2]: process started with pid [120500]
[INFO] [map_server-3]: process started with pid [120502]
[INFO] [lifecycle_manager-4]: process started with pid [120504]
[rviz2-1] Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
[test_node-2] [INFO] [1754490594.856289313] [test.test_node]: test_node started.
[test_node-2] [INFO] [1754490594.859376635] [test.test_node]: 固定占用栅格地图初始化完成，边长: 4.00 m, 分辨率: 0.05 m/像素, 像素: 80
[test_node-2] [INFO] [1754490594.859410285] [test.test_node]: 地图布局：下1/3障碍物，中间1/3地面，上1/3未知区域
[test_node-2] [INFO] [1754490594.859418080] [test.test_node]: Subscribed to /clicked_point and /plan/cmd_vel, will publish to /task_manager/PoseArray
[lifecycle_manager-4] [INFO] [1754490594.861587040] [map.lifecycle_manager]: Creating
[lifecycle_manager-4] [INFO] [1754490594.863334886] [map.lifecycle_manager]: Creating and initializing lifecycle service clients
[lifecycle_manager-4] [INFO] [1754490594.870310675] [map.lifecycle_manager]: Starting managed nodes bringup...
[lifecycle_manager-4] [INFO] [1754490594.870352092] [map.lifecycle_manager]: Configuring map_server
[map_server-3] [INFO] [1754490594.870864046] [map.map_server]: 
[map_server-3] 	map_server lifecycle node launched. 
[map_server-3] 	Waiting on external lifecycle transitions to activate
[map_server-3] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[map_server-3] [INFO] [1754490594.870945545] [map.map_server]: Creating
[map_server-3] [INFO] [1754490594.871161248] [map.map_server]: Configuring
[map_server-3] [INFO] [1754490594.871189722] [map_io]: Loading yaml file: /home/jack/plan_ws/install/nav2_map_server/share/nav2_map_server/map/map.yaml
[map_server-3] [INFO] [1754490594.871382305] [map_io]: resolution: 0.2
[map_server-3] [INFO] [1754490594.871390233] [map_io]: origin[0]: -50.6863
[map_server-3] [INFO] [1754490594.871393877] [map_io]: origin[1]: -16.8485
[map_server-3] [INFO] [1754490594.871397044] [map_io]: origin[2]: 0
[map_server-3] [INFO] [1754490594.871400186] [map_io]: free_thresh: 0.33
[map_server-3] [INFO] [1754490594.871403360] [map_io]: occupied_thresh: 0.65
[map_server-3] [INFO] [1754490594.871406573] [map_io]: mode: trinary
[map_server-3] [INFO] [1754490594.871409858] [map_io]: negate: 1
[map_server-3] [INFO] [1754490594.871522051] [map_io]: Loading image_file: /home/jack/plan_ws/install/nav2_map_server/share/nav2_map_server/map/map.pgm
[map_server-3] [INFO] [1754490594.896456805] [map_io]: Read map /home/jack/plan_ws/install/nav2_map_server/share/nav2_map_server/map/map.pgm: 550 X 401 map @ 0.2 m/cell
[lifecycle_manager-4] [INFO] [1754490594.899128079] [map.lifecycle_manager]: Activating map_server
[map_server-3] [INFO] [1754490594.899202985] [map.map_server]: Activating
[map_server-3] [INFO] [1754490594.899447375] [map.map_server]: Creating bond (map_server) to lifecycle manager.
[lifecycle_manager-4] [INFO] [1754490595.001344152] [map.lifecycle_manager]: Server map_server connected with bond.
[lifecycle_manager-4] [INFO] [1754490595.001406537] [map.lifecycle_manager]: Managed nodes are active
[lifecycle_manager-4] [INFO] [1754490595.001418490] [map.lifecycle_manager]: Creating bond timer...
[rviz2-1] [INFO] [1754490595.076663291] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-1] [INFO] [1754490595.076730036] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-1] [INFO] [1754490595.089532556] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-1] [INFO] [1754490595.436118217] [rviz2]: Trying to create a map of size 550 x 401 using 1 swatches
[rviz2-1] [ERROR] [1754490595.468502626] [rviz2]: Vertex Program:rviz/glsl120/indexed_8bit_image.vert Fragment Program:rviz/glsl120/indexed_8bit_image.frag GLSL link result : 
[rviz2-1] active samplers with a different type refer to the same texture image unit
[test_node-2] [INFO] [1754490924.679625787] [test.test_node]: Received clicked point: x=-2.07, y=6.58, z=0.00
[test_node-2] [INFO] [1754490924.679716368] [test.test_node]: 发布PoseArray，包含 2 个目标点
[test_node-2] [INFO] [1754490924.679732571] [test.test_node]: 目标点1: (4.8, 8.6), 目标点2: (-2.5, 11.0)
```

# PLAN
```zsh
➜  plan_ws git:(main) ✗ ros2 launch plan_manage plan.launch.py     
[INFO] [launch]: All log files can be found below /home/jack/.ros/log/2025-08-06-22-30-07-982065-jack-120664
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [plan_manage_node-1]: process started with pid [120665]
[INFO] [planner_server-2]: process started with pid [120667]
[INFO] [global_path_node-3]: process started with pid [120669]
[INFO] [controller_server-4]: process started with pid [120671]
[INFO] [lifecycle_manager-5]: process started with pid [120673]
[plan_manage_node-1] [INFO] [1754490608.045633942] [plan.plan_manage_node]: plan manage node started
[global_path_node-3] [INFO] [1754490608.051755138] [plan.global_path_node]: global path node started
[plan_manage_node-1] [INFO] [1754490608.053664608] [plan.plan_manage_node]: Created FollowPath action client
[global_path_node-3] [INFO] [1754490608.059146375] [plan.global_path_node]: Subscribed to /task_manager/PoseArray, will call compute_path_through_poses action and publish to /global_path
[lifecycle_manager-5] [INFO] [1754490608.060920544] [plan.lifecycle_manager]: Creating
[controller_server-4] [INFO] [1754490608.062886857] [plan.local_plan_server]: 
[controller_server-4] 	local_plan_server lifecycle node launched. 
[controller_server-4] 	Waiting on external lifecycle transitions to activate
[controller_server-4] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[lifecycle_manager-5] [INFO] [1754490608.063392600] [plan.lifecycle_manager]: Creating and initializing lifecycle service clients
[planner_server-2] [INFO] [1754490608.064213383] [plan.global_plan_server]: 
[planner_server-2] 	global_plan_server lifecycle node launched. 
[planner_server-2] 	Waiting on external lifecycle transitions to activate
[planner_server-2] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[controller_server-4] [INFO] [1754490608.064816522] [plan.local_plan_server]: Creating controller server
[planner_server-2] [INFO] [1754490608.064934460] [plan.global_plan_server]: Creating
[lifecycle_manager-5] [INFO] [1754490608.065350550] [plan.lifecycle_manager]: Starting managed nodes bringup...
[lifecycle_manager-5] [INFO] [1754490608.065616633] [plan.lifecycle_manager]: Configuring global_plan_server
[controller_server-4] [INFO] [1754490608.691144854] [plan.local_costmap.local_costmap]: 
[controller_server-4] 	local_costmap lifecycle node launched. 
[controller_server-4] 	Waiting on external lifecycle transitions to activate
[controller_server-4] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[controller_server-4] [INFO] [1754490608.691961729] [plan.local_costmap.local_costmap]: Creating Costmap
[planner_server-2] [INFO] [1754490608.733713776] [plan.global_costmap.global_costmap]: 
[planner_server-2] 	global_costmap lifecycle node launched. 
[planner_server-2] 	Waiting on external lifecycle transitions to activate
[planner_server-2] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[planner_server-2] [INFO] [1754490608.734222529] [plan.global_costmap.global_costmap]: Creating Costmap
[planner_server-2] [INFO] [1754490612.513542276] [plan.global_plan_server]: Configuring
[planner_server-2] [INFO] [1754490612.513580817] [plan.global_costmap.global_costmap]: Configuring
[planner_server-2] [INFO] [1754490612.513614661] [plan.global_costmap.global_costmap]: Default plugins: [static_layer, obstacle_layer, inflation_layer]
[planner_server-2] [INFO] [1754490612.513621785] [plan.global_costmap.global_costmap]: Loaded plugin_names_ size: 2
[planner_server-2] [INFO] [1754490612.513627133] [plan.global_costmap.global_costmap]:   plugin_names_[0]: static_layer
[planner_server-2] [INFO] [1754490612.513632117] [plan.global_costmap.global_costmap]:   plugin_names_[1]: inflation_layer
[planner_server-2] [INFO] [1754490612.657844099] [plan.global_costmap.global_costmap]: Using plugin "static_layer"
[planner_server-2] [INFO] [1754490612.683178993] [plan.global_costmap.global_costmap]: Subscribing to the map topic (/map/global_map) with transient local durability
[planner_server-2] [INFO] [1754490612.691860464] [plan.global_costmap.global_costmap]: Initialized plugin "static_layer"
[planner_server-2] [INFO] [1754490612.691908464] [plan.global_costmap.global_costmap]: Using plugin "inflation_layer"
[planner_server-2] [INFO] [1754490612.698778529] [plan.global_costmap.global_costmap]: Initialized plugin "inflation_layer"
[planner_server-2] [INFO] [1754490613.154391674] [plan.global_costmap.global_costmap]: StaticLayer: Resizing costmap to 550 X 401 at 0.200000 m/pix
[planner_server-2] [INFO] [1754490613.154604983] [plan.global_plan_server]: Created global planner plugin GridBased of type nav2_theta_star_planner/ThetaStarPlanner
[planner_server-2] [INFO] [1754490613.159671469] [plan.global_plan_server]: Planner Server has GridBased  planners available.
[lifecycle_manager-5] [INFO] [1754490613.988625421] [plan.lifecycle_manager]: Configuring local_plan_server
[controller_server-4] [INFO] [1754490613.988853492] [plan.local_plan_server]: Configuring controller interface
[controller_server-4] [INFO] [1754490613.989070845] [plan.local_plan_server]: getting goal checker plugins..
[controller_server-4] [INFO] [1754490613.989382827] [plan.local_plan_server]: Controller frequency set to 20.0000Hz
[controller_server-4] [INFO] [1754490613.989423377] [plan.local_costmap.local_costmap]: Configuring
[controller_server-4] [INFO] [1754490613.989482893] [plan.local_costmap.local_costmap]: Default plugins: [static_layer, obstacle_layer, inflation_layer]
[controller_server-4] [INFO] [1754490613.989498265] [plan.local_costmap.local_costmap]: Loaded plugin_names_ size: 3
[controller_server-4] [INFO] [1754490613.989505986] [plan.local_costmap.local_costmap]:   plugin_names_[0]: static_layer
[controller_server-4] [INFO] [1754490613.989511177] [plan.local_costmap.local_costmap]:   plugin_names_[1]: local_layer
[controller_server-4] [INFO] [1754490613.989516190] [plan.local_costmap.local_costmap]:   plugin_names_[2]: inflation_layer
[controller_server-4] [INFO] [1754490614.055995257] [plan.local_costmap.local_costmap]: Using plugin "static_layer"
[controller_server-4] [INFO] [1754490614.060844273] [plan.local_costmap.local_costmap]: Subscribing to the map topic (/map/global_map) with transient local durability
[controller_server-4] [INFO] [1754490614.088297386] [plan.local_costmap.local_costmap]: Initialized plugin "static_layer"
[controller_server-4] [INFO] [1754490614.088336803] [plan.local_costmap.local_costmap]: Using plugin "local_layer"
[controller_server-4] [INFO] [1754490614.127687107] [plan.local_costmap.local_costmap]: Initialized plugin "local_layer"
[controller_server-4] [INFO] [1754490614.127733912] [plan.local_costmap.local_costmap]: Using plugin "inflation_layer"
[controller_server-4] [INFO] [1754490614.152276591] [plan.local_costmap.local_costmap]: Initialized plugin "inflation_layer"
[controller_server-4] [INFO] [1754490614.380224688] [plan.local_costmap.local_costmap]: StaticLayer: Resizing static layer to 550 X 401 at 0.200000 m/pix
[controller_server-4] [INFO] [1754490614.380450880] [plan.local_plan_server]: Created progress_checker : progress_checker of type nav2_controller::SimpleProgressChecker
[controller_server-4] [INFO] [1754490614.391149465] [plan.local_plan_server]: Created goal checker : goal_checker of type nav2_controller::SimpleGoalChecker
[controller_server-4] [INFO] [1754490614.398483877] [plan.local_plan_server]: Controller Server has goal_checker  goal checkers available.
[controller_server-4] [INFO] [1754490614.399529876] [plan.local_plan_server]: Created controller : FollowPath of type nav2_mppi_controller::MPPIController
[controller_server-4] [INFO] [1754490614.429588798] [plan.local_plan_server]: Controller period is equal to model dt. Control sequence shifting is ON
[controller_server-4] [INFO] [1754490614.449545080] [plan.local_plan_server]: ConstraintCritic instantiated with 1 power and 4.000000 weight.
[controller_server-4] [INFO] [1754490614.449603034] [plan.local_plan_server]: Critic loaded : mppi::critics::ConstraintCritic
[controller_server-4] [INFO] [1754490614.470074073] [plan.local_plan_server]: InflationCostCritic instantiated with 1 power and 300.000000 / 0.015000 weights. Critic will collision check based on circular cost.
[controller_server-4] [INFO] [1754490614.470148802] [plan.local_plan_server]: Critic loaded : mppi::critics::CostCritic
[controller_server-4] [INFO] [1754490614.483633934] [plan.local_plan_server]: GoalCritic instantiated with 1 power and 5.000000 weight.
[controller_server-4] [INFO] [1754490614.483679021] [plan.local_plan_server]: Critic loaded : mppi::critics::GoalCritic
[controller_server-4] [INFO] [1754490614.503238105] [plan.local_plan_server]: GoalAngleCritic instantiated with 1 power, 3.000000 weight, and 0.500000 angular threshold.
[controller_server-4] [INFO] [1754490614.503287665] [plan.local_plan_server]: Critic loaded : mppi::critics::GoalAngleCritic
[controller_server-4] [INFO] [1754490614.518448444] [plan.local_plan_server]: ReferenceTrajectoryCritic instantiated with 1 power and 14.000000 weight
[controller_server-4] [INFO] [1754490614.518520781] [plan.local_plan_server]: Critic loaded : mppi::critics::PathAlignCritic
[controller_server-4] [INFO] [1754490614.537124455] [plan.local_plan_server]: Critic loaded : mppi::critics::PathFollowCritic
[controller_server-4] [INFO] [1754490614.557663179] [plan.local_plan_server]: PathAngleCritic instantiated with 1 power and 2.000000 weight. Reversing allowed.
[controller_server-4] [INFO] [1754490614.557715509] [plan.local_plan_server]: Critic loaded : mppi::critics::PathAngleCritic
[controller_server-4] [INFO] [1754490614.570661428] [plan.local_plan_server]: PreferForwardCritic instantiated with 1 power and 5.000000 weight.
[controller_server-4] [INFO] [1754490614.570703393] [plan.local_plan_server]: Critic loaded : mppi::critics::PreferForwardCritic
[controller_server-4] [INFO] [1754490614.574248999] [plan.local_plan_server]: Optimizer reset
[controller_server-4] [INFO] [1754490614.703959625] [MPPIController]: Configured MPPI Controller: FollowPath
[controller_server-4] [INFO] [1754490614.703999872] [plan.local_plan_server]: Controller Server has FollowPath  controllers available.
[lifecycle_manager-5] [INFO] [1754490615.158261374] [plan.lifecycle_manager]: Activating global_plan_server
[planner_server-2] [INFO] [1754490615.158794084] [plan.global_plan_server]: Activating
[planner_server-2] [INFO] [1754490615.158851701] [plan.global_costmap.global_costmap]: Activating
[planner_server-2] [INFO] [1754490615.158866849] [plan.global_costmap.global_costmap]: Checking transform
[planner_server-2] [INFO] [1754490615.158984927] [plan.global_costmap.global_costmap]: start
[planner_server-2] [INFO] [1754490615.209093321] [plan.global_plan_server]: Activating plugin GridBased of type nav2_theta_star_planner
[planner_server-2] [INFO] [1754490615.328475847] [plan.global_plan_server]: Creating bond (global_plan_server) to lifecycle manager.
[lifecycle_manager-5] [INFO] [1754490616.480808315] [plan.lifecycle_manager]: Server global_plan_server connected with bond.
[lifecycle_manager-5] [INFO] [1754490616.480850737] [plan.lifecycle_manager]: Activating local_plan_server
[controller_server-4] [INFO] [1754490616.481077122] [plan.local_plan_server]: Activating
[controller_server-4] [INFO] [1754490616.481108447] [plan.local_costmap.local_costmap]: Activating
[controller_server-4] [INFO] [1754490616.481119636] [plan.local_costmap.local_costmap]: Checking transform
[controller_server-4] [INFO] [1754490616.481197927] [plan.local_costmap.local_costmap]: start
[controller_server-4] [WARN] [1754490616.531386861] [plan.local_plan_server]: Parameter local_plan_server.verbose not found
[controller_server-4] [INFO] [1754490616.532698723] [plan.local_plan_server]: Optimizer reset
[controller_server-4] [INFO] [1754490616.532970911] [MPPIController]: Activated MPPI Controller: FollowPath
[controller_server-4] [INFO] [1754490616.533021045] [plan.local_plan_server]: Creating bond (local_plan_server) to lifecycle manager.
[lifecycle_manager-5] [INFO] [1754490616.657692365] [plan.lifecycle_manager]: Server local_plan_server connected with bond.
[lifecycle_manager-5] [INFO] [1754490616.657786656] [plan.lifecycle_manager]: Managed nodes are active
[lifecycle_manager-5] [INFO] [1754490616.657798236] [plan.lifecycle_manager]: Creating bond timer...
[global_path_node-3] [INFO] [1754490924.679870587] [plan.global_path_node]: Received PoseArray with 2 poses
[global_path_node-3] [INFO] [1754490924.679936018] [plan.global_path_node]: Sending compute_path_through_poses action goal
[global_path_node-3] [INFO] [1754490924.680484672] [plan.global_path_node]: ComputePathThroughPoses goal accepted by server
[plan_manage_node-1] [INFO] [1754490924.682951889] [plan.plan_manage_node]: Received global_path
[global_path_node-3] [INFO] [1754490924.683069272] [plan.global_path_node]: Action succeeded! Path computed successfully
[global_path_node-3] [INFO] [1754490924.683143441] [plan.global_path_node]: Published path with 74 poses to /global_path
[plan_manage_node-1] [INFO] [1754490924.683187203] [plan.plan_manage_node]: Sending FollowPath action goal
[plan_manage_node-1] [INFO] [1754490924.683371979] [plan.plan_manage_node]: Received global_path
[plan_manage_node-1] [INFO] [1754490924.683453854] [plan.plan_manage_node]: Sending FollowPath action goal
[plan_manage_node-1] [INFO] [1754490924.683581487] [plan.plan_manage_node]: FollowPath goal accepted by server
[plan_manage_node-1] [INFO] [1754490924.683756380] [plan.plan_manage_node]: FollowPath goal accepted by server
[controller_server-4] [INFO] [1754490924.683700835] [plan.local_plan_server]: Received a goal, begin computing control effort.
[controller_server-4] [WARN] [1754490924.683753793] [plan.local_plan_server]: No goal checker was specified in parameter 'current_goal_checker'. Server will use only plugin loaded goal_checker . This warning will appear once.
[controller_server-4] [INFO] [1754490924.683808795] [plan.local_plan_server]: Passing new path to controller.
[plan_manage_node-1] [ERROR] [1754490924.683940046] [plan.plan_manage_node]: FollowPath action was aborted
[controller_server-4] [INFO] [1754490924.685503431] [plan.local_plan_server]: Optimizer reset
[controller_server-4] [INFO] [1754490984.288403225] [plan.local_plan_server]: Reached the goal!
[plan_manage_node-1] [INFO] [1754490984.288599553] [plan.plan_manage_node]: FollowPath action succeeded
```