/*
 * rb2_pad
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation, SLL
 * \brief Allows to use a pad with the roboy controller, sending the messages
 * received from the joystick device
 */

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
// Not yet catkinized 9/2013
// #include <sound_play/sound_play.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <rb2_pad/enable_disable_pad.h>
#include <robotnik_msgs/ElevatorAction.h>
#include <robotnik_msgs/SetElevator.h>
#include <robotnik_msgs/home.h>
#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/set_mode.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <robotnik_msgs/SetLaserMode.h>
#include <unistd.h>

#define DEFAULT_NUM_OF_BUTTONS 16
#define DEFAULT_AXIS_LINEAR_X 1
#define DEFAULT_AXIS_LINEAR_Y 0
#define DEFAULT_AXIS_ANGULAR 2
#define DEFAULT_AXIS_LINEAR_Z 3
#define DEFAULT_SCALE_LINEAR 1.0
#define DEFAULT_SCALE_ANGULAR 2.0
#define DEFAULT_SCALE_LINEAR_Z 1.0

#define ITERATIONS_CALL_SAFETY_MODULE 2
#define ITERATIONS_AFTER_DEADMAN 3.0

#define JOY_ERROR_TIME 1.0

//!//////////////////////////////////////////////////////////////////////
//!                               NOTE:                                //
//! This configuration is made for a PS4 Dualshock                     //
//!   please feel free to modify to adapt for your own joystick.       //
//! 								       //
//!//////////////////////////////////////////////////////////////////////

class RB2Pad {
   public:
    RB2Pad();
    void Update();

   private:
    bool checkButtonPressed(const std::vector<int> &buttons, const int number);
    void padCallback(const sensor_msgs::Joy::ConstPtr &joy);
    bool EnableDisablePad(rb2_pad::enable_disable_pad::Request &req,
                          rb2_pad::enable_disable_pad::Response &res);
    int setSafetyOverride(bool value);
    int setManualRelease(bool value);
    int loopBetweenLaserModes();

    ros::NodeHandle nh_;

    int manual_release_true_number_, manual_release_false_number_,
        safety_override_false_number_, safety_override_true_number_;
    int linear_x_, linear_y_, linear_z_, angular_;
    double l_scale_, a_scale_, l_scale_z_;
    //! It will publish into command velocity (for the robot)
    ros::Publisher vel_pub_;
    //! It will publish into command velocity (for the robot)
    ros::Publisher unsafe_vel_pub_;
    bool has_unsafe_vel_;
    //! It will be suscribed to the joystick
    ros::Subscriber pad_sub_;
    //! Name of the topic where it will be publishing the velocity
    std::string cmd_topic_vel_;
    //! Name of the topic where it will be publishing the velocity for unsafe
    std::string cmd_topic_unsafe_vel_;
    //! Name of the service where it will be modifying the digital outputs
    std::string cmd_service_io_;
    //! Name of the service where to actuate the elevator
    std::string elevator_service_name_;
    //! If it is True, it will check the timeout message
    bool check_message_timeout_;
    double current_vel;
  //! Name of service where to initialize pose
  std::string initialize_pose_service_name_;
    //! Number of the DEADMAN button
    int dead_man_button_, dead_man_unsafe_button_, safety_override_button_, laser_mode_button_;
    //! Number of the button for increase or decrease the speed max of the
    //! joystick
    int speed_up_button_, speed_down_button_;
    int button_raise_elevator_, button_lower_elevator_, button_stop_elevator_,
        axis_elevator_;
    bool bOutput1, bOutput2;
    //! button to change kinematic mode
    int button_kinematic_mode_;
    //! kinematic mode
  //! button to initialize pose
  int button_initialize_pose_;
    int kinematic_mode_;
    //! Service to modify the kinematic mode
    ros::ServiceClient setKinematicMode;
    //! Name of the service to change the mode
    std::string cmd_set_mode_;
    //! button to start the homing service
    int button_home_;
    //! Service to start homing
    ros::ServiceClient doHome;
    //! Name of the service to do the homing
    std::string cmd_home_;
    //! Enables/disables the pad
    // ros::ServiceServer enable_disable_srv_;

    //! Service to modify the digital outputs
    ros::ServiceClient set_digital_outputs_client_;
    //! Service to activate the elevator
    ros::ServiceClient set_elevator_client_;
    //! Service to safety module
    ros::ServiceClient set_manual_release_client_, set_safety_override_client_, set_laser_mode_client_;
    //! Service to call pose initialization
    ros::ServiceClient initialize_pose_client_;

    //! Number of buttons of the joystick
    int num_of_buttons_;
    //! Pointer to a vector for controlling the event when pushing the buttons
    bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
    // DIAGNOSTICS
    //! Diagnostic to control the frequency of the published command velocity
    //! topic
    diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq;
    //! Diagnostic to control the reception frequency of the subscribed joy
    //! topic
    diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq;
    //! General status diagnostic updater
    diagnostic_updater::Updater updater_pad;
    //! Diagnostics min freq
    double min_freq_command, min_freq_joy;  //
    //! Diagnostics max freq
    double max_freq_command, max_freq_joy;  //
    //! Flag to enable/disable the communication with the publishers topics
    // bool bEnable;
    //! Client of the sound play service
    //  sound_play::SoundClient sc
    //! sets the elevator by reading from an axis, otherwise reading from buttons
    bool use_axis_for_elevator;
    //! Laser modes, to loop between them;
    std::vector<std::string> laser_modes_;
    int current_laser_mode_;
};

RB2Pad::RB2Pad() : linear_x_(1), linear_y_(0), angular_(2), linear_z_(3) {
    current_vel = 0.1;
    //
    nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);

    // MOTION CONF
    nh_.param("axis_linear_x", linear_x_, DEFAULT_AXIS_LINEAR_X);
    nh_.param("axis_linear_y", linear_y_, DEFAULT_AXIS_LINEAR_Y);
    nh_.param("axis_linear_z", linear_z_, DEFAULT_AXIS_LINEAR_Z);
    nh_.param("axis_angular", angular_, DEFAULT_AXIS_ANGULAR);
    nh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
    nh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);
    nh_.param("scale_linear_z", l_scale_z_, DEFAULT_SCALE_LINEAR_Z);
    nh_.param("cmd_topic_vel", cmd_topic_vel_, cmd_topic_vel_);
    nh_.param("cmd_topic_unsafe_vel", cmd_topic_unsafe_vel_, cmd_topic_vel_);
    nh_.param("button_dead_man", dead_man_button_, dead_man_button_);
    nh_.param("button_dead_man_unsafe", dead_man_unsafe_button_,
              -1);  // NOT SET BY DEFAULT
    nh_.param("button_safety_override", safety_override_button_,
              safety_override_button_);
    nh_.param("button_laser_mode", laser_mode_button_,
              laser_mode_button_);
    nh_.param("button_speed_up", speed_up_button_,
              speed_up_button_);  // 4 Thrustmaster
    nh_.param("button_speed_down", speed_down_button_,
              speed_down_button_);  // 5 Thrustmaster

    // DIGITAL OUTPUTS CONF
    nh_.param("cmd_service_io", cmd_service_io_, cmd_service_io_);
   // PANTILT CONF
    nh_.param("button_home", button_home_, button_home_);
    nh_.param("button_lower_elevator", button_lower_elevator_, 6);
    nh_.param("button_raise_elevator", button_raise_elevator_, 4);
    nh_.param("button_stop_elevator", button_stop_elevator_, 16);

    nh_.param("axis_elevator", axis_elevator_, 1);
    if(axis_elevator_ < 0)
        use_axis_for_elevator = false;
    else
        use_axis_for_elevator = true;

   nh_.param("initialize_pose", button_initialize_pose_, 4);

    nh_.param("cmd_service_home", cmd_home_, cmd_home_);

    nh_.param("check_message_timeout", check_message_timeout_,
              check_message_timeout_);

    nh_.param<std::string>("elevator_service_name", elevator_service_name_,
                           "set_elevator");

    ROS_INFO("RB2Pad num_of_buttons_ = %d", num_of_buttons_);
    for (int i = 0; i < num_of_buttons_; i++) {
        bRegisteredButtonEvent[i] = false;
        ROS_INFO("bREG %d", i);
    }

  nh_.param<std::string>("initialize_pose_service_name", initialize_pose_service_name_, "initialize_pose");

    // Publish through the node handle Twist type messages to the
    // guardian_controller/command topicç
    if (cmd_topic_unsafe_vel_ == "") {
        ROS_ERROR(
            "RB2Pad: cmd_topic_vel is empty, so things are going to be crazy");
    }
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
    if (cmd_topic_unsafe_vel_ != "" and dead_man_unsafe_button_ != -1) {
        ROS_WARN(
            "RB2Pad: We have an unsafe cmd vel in topic \"%s\". Be aware of it "
            "(press button %d for it)",
            cmd_topic_unsafe_vel_.c_str(), dead_man_unsafe_button_);
        unsafe_vel_pub_ =
            nh_.advertise<geometry_msgs::Twist>(cmd_topic_unsafe_vel_, 1);
        has_unsafe_vel_ = true;
    } else {
        ROS_WARN(
            "RB2Pad: We do not have an unsafe cmd_vel (button %d, topic %s)",
            dead_man_unsafe_button_, cmd_topic_unsafe_vel_.c_str());
        has_unsafe_vel_ = false;
    }

    // Listen through the node handle sensor_msgs::Joy messages from joystick
    // (these are the references that we will sent to cmd_vel)
    pad_sub_ =
        nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RB2Pad::padCallback, this);

    // Request service to activate / deactivate digital I/O
    set_digital_outputs_client_ =
        nh_.serviceClient<robotnik_msgs::set_digital_output>(cmd_service_io_);
    set_elevator_client_ =
        nh_.serviceClient<robotnik_msgs::SetElevator>(elevator_service_name_);
    set_manual_release_client_ = nh_.serviceClient<std_srvs::SetBool>(
        "safety_module/set_manual_release");
    set_safety_override_client_ = nh_.serviceClient<std_srvs::SetBool>(
        "safety_module/set_safety_override");
    set_laser_mode_client_ = nh_.serviceClient<robotnik_msgs::SetLaserMode>(
        "safety_module/set_laser_mode");

    laser_modes_.clear();
    nh_.param<std::vector<std::string> >("laser_modes", laser_modes_, laser_modes_);
    current_laser_mode_ = 0;

 initialize_pose_client_ =
      nh_.serviceClient<std_srvs::Trigger>(initialize_pose_service_name_);

    bOutput1 = bOutput2 = false;

    manual_release_false_number_ = 0;
    manual_release_true_number_ = 0;
    safety_override_false_number_ = 0;
    safety_override_true_number_ = 0;

    // Request service to start homing
    doHome = nh_.serviceClient<robotnik_msgs::home>(cmd_home_);

    // Diagnostics
    updater_pad.setHardwareID("None");
    // Topics freq control
    min_freq_command = min_freq_joy = 5.0;
    max_freq_command = max_freq_joy = 50.0;
    sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(
        "/joy", updater_pad, diagnostic_updater::FrequencyStatusParam(
                                 &min_freq_joy, &max_freq_joy, 0.1, 10));

    pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(
        cmd_topic_vel_.c_str(), updater_pad,
        diagnostic_updater::FrequencyStatusParam(&min_freq_command,
                                                 &max_freq_command, 0.1, 10));

    // Advertises new service to enable/disable the pad
    // enable_disable_srv_ = nh_.advertiseService("/rb2_pad/enable_disable_pad",
    // &RB2Pad::EnableDisablePad, this);
    // bEnable = true;	// Communication flag enabled by default
}

/*
 *	\brief Updates the diagnostic component. Diagnostics
 *
 */
void RB2Pad::Update() { updater_pad.update(); }

/*
 *	\brief Enables/Disables the pad
 *
 */
/*
bool RB2Pad::EnableDisablePad(rb2_pad::enable_disable_pad::Request &req,
rb2_pad::enable_disable_pad::Response &res )
{
        bEnable = req.value;

        ROS_INFO("RB2Pad::EnablaDisablePad: Setting to %d", req.value);
        res.ret = true;
        return true;
}
*/
bool RB2Pad::checkButtonPressed(const std::vector<int> &buttons,
                                const int number) {
    if (number < 0 or number > buttons.size()) {
        ROS_WARN_THROTTLE(
            2,
            "RB2Pad::checkButtonPressed: You are pressing a disabled button");
        return false;
    }
    return buttons[number] == true;
}

void RB2Pad::padCallback(const sensor_msgs::Joy::ConstPtr &joy) {
    geometry_msgs::Twist vel;
    static int send_iterations_after_dead_man = 0;

    // Checks the ROS time to avoid noise in the pad
    if (check_message_timeout_ &&
        ((ros::Time::now() - joy->header.stamp).toSec() > JOY_ERROR_TIME))
        return;

    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

    // Actions dependant on dead-man button
    if (checkButtonPressed(joy->buttons, dead_man_button_) == true) {
        // ROS_ERROR("RB2Pad::padCallback: DEADMAN button %d",
        // dead_man_button_);
        // Set the current velocity level

        manual_release_false_number_ = 0;
        // MANUAL RELEASE -> 1
        // write the signal X number of times
        // if (manual_release_true_number_ < ITERATIONS_CALL_SAFETY_MODULE) {
        //     setManualRelease(true);
        //     manual_release_true_number_++;
        // }

        // L1 pressed -> Safety override 1
        if (checkButtonPressed(joy->buttons, safety_override_button_) == true) {
            if (safety_override_true_number_ < ITERATIONS_CALL_SAFETY_MODULE) {
                setSafetyOverride(true);
                safety_override_true_number_++;
            }
            safety_override_false_number_ = 0;
        } else {
            //	if(safety_override_false_number_ < ITERATIONS_CALL_SAFETY_MODULE){
            //		setSafetyOverride(false);
            //		safety_override_false_number_++;
            //	}
            //	safety_override_true_number_ = 0;
        }
        
        if (checkButtonPressed(joy->buttons, laser_mode_button_) == true) {
            if (!bRegisteredButtonEvent[laser_mode_button_]) 
            {
                bRegisteredButtonEvent[laser_mode_button_] = true;
                loopBetweenLaserModes();
            }
        } else {
            bRegisteredButtonEvent[laser_mode_button_] = false;
        }

        double speed_step = 0.1;
        double minimum_speed = 0.1;
        double maximum_speed = 1.0;
        if (checkButtonPressed(joy->buttons, speed_down_button_) == true) {
            if (!bRegisteredButtonEvent[speed_down_button_]) 
            {
                current_vel = current_vel - speed_step;
                if (current_vel < minimum_speed)
                   current_vel = minimum_speed; 
                bRegisteredButtonEvent[speed_down_button_] = true;
                ROS_INFO("Velocity: %f%%", current_vel * 100.0);
                char buf[50] = "\0";
                int percent = (int)(current_vel * 100.0);
                sprintf(buf, " %d percent", percent);
                // sc.say(buf);
            }
        } else {
            bRegisteredButtonEvent[speed_down_button_] = false;
        }

        if (checkButtonPressed(joy->buttons, speed_up_button_) == true) {
            if (!bRegisteredButtonEvent[speed_up_button_])
            {
                current_vel = current_vel + speed_step;
                if (current_vel > maximum_speed)
                    current_vel = maximum_speed;
                bRegisteredButtonEvent[speed_up_button_] = true;
                ROS_INFO("Velocity: %f%%", current_vel * 100.0);
                char buf[50] = "\0";
                int percent = (int)(current_vel * 100.0);
                sprintf(buf, " %d percent", percent);
                // sc.say(buf);
            }

        } else {
            bRegisteredButtonEvent[speed_up_button_] = false;
        }

        vel.angular.x = current_vel * (a_scale_ * joy->axes[angular_]);
        vel.angular.y = current_vel * (a_scale_ * joy->axes[angular_]);
        vel.angular.z = current_vel * (a_scale_ * joy->axes[angular_]);
        vel.linear.x = current_vel * l_scale_ * joy->axes[linear_x_];
        vel.linear.y = current_vel * l_scale_ * joy->axes[linear_y_];
        vel.linear.z = current_vel * l_scale_z_ * joy->axes[linear_z_];
		
		
        // ELEVATOR
        robotnik_msgs::SetElevator elevator_msg_srv;
		if(use_axis_for_elevator){
			if (joy->axes[axis_elevator_] > 0.99) {
				 ROS_INFO_THROTTLE(10, "RB2Pad::padCallback: button %d calling service:%sRAISE", button_stop_elevator_,elevator_service_name_.c_str());
				

				elevator_msg_srv.request.action.action =
					robotnik_msgs::ElevatorAction::RAISE;
				set_elevator_client_.call(elevator_msg_srv);
			}

			if (joy->axes[axis_elevator_] < -0.99) {
				 ROS_INFO_THROTTLE(10, "RB2Pad::padCallback: button %d calling service:%s LOWER", button_stop_elevator_,elevator_service_name_.c_str());

				elevator_msg_srv.request.action.action =
					robotnik_msgs::ElevatorAction::LOWER;
				set_elevator_client_.call(elevator_msg_srv);
			}
		}else{
			 if (checkButtonPressed(joy->buttons, button_lower_elevator_) == true) {
				if (!bRegisteredButtonEvent[button_lower_elevator_]){
					bRegisteredButtonEvent[button_lower_elevator_] = true;
					elevator_msg_srv.request.action.action =
					robotnik_msgs::ElevatorAction::LOWER;
					set_elevator_client_.call(elevator_msg_srv);
				}

			} else {
				bRegisteredButtonEvent[button_lower_elevator_] = false;
			}
			 if (checkButtonPressed(joy->buttons, button_raise_elevator_) == true) {
				if (!bRegisteredButtonEvent[button_raise_elevator_]){
					bRegisteredButtonEvent[button_raise_elevator_] = true;
					elevator_msg_srv.request.action.action =
					robotnik_msgs::ElevatorAction::RAISE;
					set_elevator_client_.call(elevator_msg_srv);
				}

			} else {
				bRegisteredButtonEvent[button_raise_elevator_] = false;
			}
		}
    } else {
        // MANUAL RELEASE -> 0
        // if (manual_release_false_number_ < ITERATIONS_CALL_SAFETY_MODULE) {
        //     setManualRelease(false);
        //     // setSafetyOverride(false);
        //     manual_release_false_number_++;
        // }

        manual_release_true_number_ = 0;
        manual_release_false_number_ = 0;
        safety_override_false_number_ = 0;
        safety_override_true_number_ = 0;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = 0.0;
        vel.linear.x = 0.0;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;
    }

  static bool initialized_pose = false;

  if (joy->buttons[dead_man_button_] == 1)
  {
    if (initialized_pose == false and joy->buttons[button_initialize_pose_] == 1)
    {
      initialized_pose = true;
      bool success = false;
      std_srvs::Trigger init_pose;
      if (initialize_pose_client_.exists() == true)
      {
        success = initialize_pose_client_.call(init_pose);
      }
      else
      {
        success = false;
      }
      if (success == false) 
      {
        ROS_ERROR_STREAM("Pad: Cannot call to initialize pose. Service name: " << initialize_pose_client_.getService());
      }
      else if (init_pose.response.success == false)
      {
        ROS_ERROR_STREAM("Pad: Call resulted in error: " << init_pose.response.message << ". Service name: " << initialize_pose_client_.getService());
      }
      else
      {
        ROS_INFO_STREAM("Pad: initialized pose"); 
      }
    }
  }
  else
  {
    initialized_pose = false;
  }




    sus_joy_freq->tick();  // Ticks the reception of joy events

    // Publish only with deadman button pushed for twist use
    if (checkButtonPressed(joy->buttons, dead_man_button_) == true) {
        send_iterations_after_dead_man = ITERATIONS_AFTER_DEADMAN;
        if (checkButtonPressed(joy->buttons, dead_man_unsafe_button_) ==
                true and
            has_unsafe_vel_) {
            unsafe_vel_pub_.publish(vel);
        } else {
            vel_pub_.publish(vel);
        }
        pub_command_freq->tick();
    } else {  // send some 0 if deadman is released
        if (send_iterations_after_dead_man > 0) {
            send_iterations_after_dead_man--;
            if (checkButtonPressed(joy->buttons, dead_man_unsafe_button_) ==
                    true and
                has_unsafe_vel_) {
                unsafe_vel_pub_.publish(vel);
            } else {
                vel_pub_.publish(vel);
            }
            pub_command_freq->tick();
        }
    }
}

int RB2Pad::setManualRelease(bool value) {
    std_srvs::SetBool set_bool_msg;

    set_bool_msg.request.data = value;
    set_manual_release_client_.call(set_bool_msg);

    return 0;
}

int RB2Pad::setSafetyOverride(bool value) {
    std_srvs::SetBool set_bool_msg;

    set_bool_msg.request.data = value;
    set_safety_override_client_.call(set_bool_msg);

    return 0;
}

int RB2Pad::loopBetweenLaserModes() {

    int next_mode = (current_laser_mode_ + 1) % laser_modes_.size();
    
    robotnik_msgs::SetLaserMode set_laser_msg;
    set_laser_msg.request.mode = laser_modes_[next_mode];
    
    if (set_laser_mode_client_.call(set_laser_msg) == false or set_laser_msg.response.ret == false) {
        ROS_INFO("RB2Pad: error setting laser mode = %s", set_laser_msg.request.mode.c_str());
        return -1;
    }
    current_laser_mode_ = next_mode;
    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rb2_pad");
    RB2Pad rb2_pad;

    ros::Rate r(50.0);

    while (ros::ok()) {
        // UPDATING DIAGNOSTICS
        rb2_pad.Update();
        ros::spinOnce();
        r.sleep();
    }
}
