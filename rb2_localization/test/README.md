Simple AMCL + bag configuration to test localization parameters.

Plays a bag, publishes a map and runs amcl.

How to use:
roslaunch rb2_localization test_amcl_with_bag.launch

Parameters:
- bag_file: bag to be played (must be set!)
- map_file: map to be loaded (must be set!)
- play_rate: at which rate will be the bag played (default 1)
- play_delay: delay to begin playing after loading. gives nodes time to subscribe to topics (default 5s)
- play_start: how much time to advance in the bag, maybe movement starts late (default 0)
-
- x_init_pose: initial pose (default 0). must be correct for amcl to work
- y_init_pose: iniital pose (default 0). must be correct for amcl to work
- a_init_pose: initial rotation (default 0). must be correct for amcl to work

