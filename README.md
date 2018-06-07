rb2_common
===============
Common packages of the RB2: URDF description of the RB2, platform messages and other files for simulation.

<h1>Packages</h1>

<h2>rb2_control</h2>
This package contains the launch and configuration files to spawn the joint controllers with the ROS controller_manager. It allows to launch the joint controllers for the RB2.

<h2>rb2_description</h2>

The urdf, meshes, and other elements needed in the description are contained here. This package includes the description of the RB2 robot.
The package includes also some launch files to publish the robot state and to test the urdf files in rviz.

<h2>rb2_localization</h2>

Contains the configuration and launch files to use the robot localiztion packages along the real or simulated robot.

<h2>rb2_navigation</h2>

Contains the configuration and launch files to work with the ROS navigation stack along the real or simulated robot.

<h2>rb2_pad</h2>

This package contains the node that subscribes to /joy messages and publishes command messages for the robot platform including speed level control. The joystick output is feed to a mux (http://wiki.ros.org/twist_mux) so that the final command to the robot can be set by different components (move_base, etc.)

The node allows to load different types of joysticks (PS4, PS3, Logitech, Thrustmaster). New models can be easily added by creating new .yaml files.

