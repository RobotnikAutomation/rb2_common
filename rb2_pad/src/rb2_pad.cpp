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
 * \brief Allows to use a pad with the roboy controller, sending the messages received from the joystick device
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
// Not yet catkinized 9/2013
// #include <sound_play/sound_play.h>
#include <unistd.h>
#include <robotnik_msgs/set_mode.h>
#include <rb2_pad/enable_disable_pad.h>
#include <robotnik_msgs/set_digital_output.h>
#include <std_srvs/SetBool.h>
#include <robotnik_msgs/home.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <robotnik_msgs/SetElevator.h>
#include <robotnik_msgs/ElevatorAction.h>

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_LINEAR_Y       0
#define DEFAULT_AXIS_ANGULAR		2
#define DEFAULT_AXIS_LINEAR_Z       3	
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		2.0
#define DEFAULT_SCALE_LINEAR_Z      1.0 

#define ITERATIONS_WRITE_MODBUS		2
#define ITERATIONS_AFTER_DEADMAN    3.0

#define JOY_ERROR_TIME					1.0

//!//////////////////////////////////////////////////////////////////////
//!                               NOTE:                                //
//! This configuration is made for a PS4 Dualshock                     //
//!   please feel free to modify to adapt for your own joystick.       //
//! 								       //
//!//////////////////////////////////////////////////////////////////////


class RB2Pad
{
	public:
	RB2Pad();
	void Update();

	private:
        bool checkButtonPressed(const std::vector<int> &buttons, const int number);
	void padCallback(const sensor_msgs::Joy::ConstPtr& joy);
	bool EnableDisablePad(rb2_pad::enable_disable_pad::Request &req, rb2_pad::enable_disable_pad::Response &res );
	int setBumperOverride(bool value);
	int setManualRelease(bool value);

	ros::NodeHandle nh_;

	int manual_release_true_number_, manual_release_false_number_, bumper_override_false_number_, bumper_override_true_number_;
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
	//! Number of the DEADMAN button
	int dead_man_button_, dead_man_unsafe_button_, bumper_override_button_;
	//! Number of the button for increase or decrease the speed max of the joystick	
	int speed_up_button_, speed_down_button_;
	int button_output_1_, button_output_2_;
        int button_raise_elevator_, button_lower_elevator_, button_stop_elevator_,axis_elevator_;
	int output_1_, output_2_;
	bool bOutput1, bOutput2;
	//! button to change kinematic mode
	int button_kinematic_mode_;
	//! kinematic mode
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
	ros::ServiceClient  set_manual_release_client_, set_bumper_override_client_;
	//! Number of buttons of the joystick
	int num_of_buttons_;
	//! Pointer to a vector for controlling the event when pushing the buttons
	bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
	// DIAGNOSTICS
	//! Diagnostic to control the frequency of the published command velocity topic
	diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq; 
	//! Diagnostic to control the reception frequency of the subscribed joy topic 
	diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq; 
	//! General status diagnostic updater
	diagnostic_updater::Updater updater_pad;	
	//! Diagnostics min freq
	double min_freq_command, min_freq_joy; // 
	//! Diagnostics max freq
	double max_freq_command, max_freq_joy; // 	
	//! Flag to enable/disable the communication with the publishers topics
	// bool bEnable;
    //! Client of the sound play service
    //  sound_play::SoundClient sc;
};


RB2Pad::RB2Pad():
  linear_x_(1),
  linear_y_(0),
  angular_(2),
  linear_z_(3)
{
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
	nh_.param("button_dead_man_unsafe", dead_man_unsafe_button_, -1);  // NOT SET BY DEFAULT
	nh_.param("button_bumber_override", bumper_override_button_, bumper_override_button_);
	nh_.param("button_speed_up", speed_up_button_, speed_up_button_);  //4 Thrustmaster
	nh_.param("button_speed_down", speed_down_button_, speed_down_button_); //5 Thrustmaster
	
	// DIGITAL OUTPUTS CONF
	nh_.param("cmd_service_io", cmd_service_io_, cmd_service_io_);
	nh_.param("button_output_1", button_output_1_, button_output_1_);
	nh_.param("button_output_2", button_output_2_, button_output_2_);
	nh_.param("output_1", output_1_, output_1_);
	nh_.param("output_2", output_2_, output_2_);
    // PANTILT CONF
    nh_.param("button_home", button_home_, button_home_);
	nh_.param("button_lower_elevator",button_lower_elevator_, 6);
	nh_.param("button_raise_elevator",button_raise_elevator_, 4);
	nh_.param("button_stop_elevator",button_stop_elevator_, 16);

    nh_.param("axis_elevator",axis_elevator_, 1);

    nh_.param("cmd_service_home", cmd_home_, cmd_home_);

	nh_.param("check_message_timeout", check_message_timeout_, check_message_timeout_);
    
    nh_.param<std::string>("elevator_service_name", elevator_service_name_, "set_elevator");
	
	ROS_INFO("RB2Pad num_of_buttons_ = %d", num_of_buttons_);	
	for(int i = 0; i < num_of_buttons_; i++){
		bRegisteredButtonEvent[i] = false;
		ROS_INFO("bREG %d", i);
		}

  	// Publish through the node handle Twist type messages to the guardian_controller/command topicç
        if (cmd_topic_unsafe_vel_ == "")
	{
		ROS_ERROR("RB2Pad: cmd_topic_vel is empty, so things are going to be crazy");
	}
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
        if (cmd_topic_unsafe_vel_ != "" and dead_man_unsafe_button_ != -1) {
            ROS_WARN("RB2Pad: We have an unsafe cmd vel in topic \"%s\". Be aware of it (press button %d for it)", cmd_topic_unsafe_vel_.c_str(), dead_man_unsafe_button_);
	    unsafe_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_unsafe_vel_, 1);
            has_unsafe_vel_ = true;
        }
	else {
            ROS_WARN("RB2Pad: We do not have an unsafe cmd_vel (button %d, topic %s)", dead_man_unsafe_button_, cmd_topic_unsafe_vel_.c_str());
            has_unsafe_vel_ = false;
	}

 	// Listen through the node handle sensor_msgs::Joy messages from joystick 
    // (these are the references that we will sent to cmd_vel)
	pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RB2Pad::padCallback, this);
	
 	// Request service to activate / deactivate digital I/O
	set_digital_outputs_client_ = nh_.serviceClient<robotnik_msgs::set_digital_output>(cmd_service_io_);
	set_elevator_client_ = nh_.serviceClient<robotnik_msgs::SetElevator>(elevator_service_name_);
	set_manual_release_client_ = nh_.serviceClient<std_srvs::SetBool>("safety_module/set_manual_release");
	set_bumper_override_client_ = nh_.serviceClient<std_srvs::SetBool>("safety_module/set_bumper_override");

	bOutput1 = bOutput2 = false;

	manual_release_false_number_  = 0;
	manual_release_true_number_   = 0;
	bumper_override_false_number_ = 0;
	bumper_override_true_number_  = 0;

    // Request service to start homing
	doHome = nh_.serviceClient<robotnik_msgs::home>(cmd_home_);

	// Diagnostics
	updater_pad.setHardwareID("None");
	// Topics freq control 
	min_freq_command = min_freq_joy = 5.0;
	max_freq_command = max_freq_joy = 50.0;
	sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

	pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(cmd_topic_vel_.c_str(), updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));

	// Advertises new service to enable/disable the pad
	// enable_disable_srv_ = nh_.advertiseService("/rb2_pad/enable_disable_pad",  &RB2Pad::EnableDisablePad, this);
	// bEnable = true;	// Communication flag enabled by default

}


/*
 *	\brief Updates the diagnostic component. Diagnostics
 *
 */
void RB2Pad::Update(){
	updater_pad.update();
}

/*
 *	\brief Enables/Disables the pad
 *
 */
/*
bool RB2Pad::EnableDisablePad(rb2_pad::enable_disable_pad::Request &req, rb2_pad::enable_disable_pad::Response &res )
{
	bEnable = req.value;

	ROS_INFO("RB2Pad::EnablaDisablePad: Setting to %d", req.value);
	res.ret = true;
	return true;
}
*/
bool RB2Pad::checkButtonPressed(const std::vector<int> &buttons, const int number)
{
    if (number < 0 or number > buttons.size())
    {
        ROS_WARN_THROTTLE(2, "RB2Pad::checkButtonPressed: You are pressing a disabled button");
        return false;
    }
    return buttons[number] == true;
}

void RB2Pad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist vel;
	static int send_iterations_after_dead_man = 0;
	
	// Checks the ROS time to avoid noise in the pad
	if(check_message_timeout_ && ((ros::Time::now() - joy->header.stamp).toSec() > JOY_ERROR_TIME))
		return;

	vel.angular.x = 0.0;  vel.angular.y = 0.0; vel.angular.z = 0.0;
	vel.linear.x = 0.0;   vel.linear.y = 0.0; vel.linear.z = 0.0;

  	// Actions dependant on dead-man button
 	if (checkButtonPressed(joy->buttons,dead_man_button_) == true) {
		//ROS_ERROR("RB2Pad::padCallback: DEADMAN button %d", dead_man_button_);
		// Set the current velocity level

		manual_release_false_number_  = 0;
		// MANUAL RELEASE -> 1
		// write the signal X number of times
		if(manual_release_true_number_ < ITERATIONS_WRITE_MODBUS){
			setManualRelease(true);
			manual_release_true_number_++;
		}

		// L1 pressed -> Bumper override 1
		if(checkButtonPressed(joy->buttons,bumper_override_button_) == true){
			if(bumper_override_true_number_ < ITERATIONS_WRITE_MODBUS)
                        {
				setBumperOverride(true);
				bumper_override_true_number_++;
			}
			bumper_override_false_number_ = 0;
		}
                else{
		//	if(bumper_override_false_number_ < ITERATIONS_WRITE_MODBUS){
		//		setBumperOverride(false);
		//		bumper_override_false_number_++;
		//	}
		//	bumper_override_true_number_ = 0;
		}

		if ( checkButtonPressed(joy->buttons,speed_down_button_) == true ){

			if(!bRegisteredButtonEvent[speed_down_button_]) 
				if(current_vel > 0.1){
		  			current_vel = current_vel - 0.1;
					bRegisteredButtonEvent[speed_down_button_] = true;
					ROS_INFO("Velocity: %f%%", current_vel*100.0);	
					char buf[50]="\0";
 					int percent = (int) (current_vel*100.0);
					sprintf(buf," %d percent", percent);
                    // sc.say(buf);
				}	 	
		}else{
			bRegisteredButtonEvent[speed_down_button_] = false;
		 }
		 
		if (checkButtonPressed(joy->buttons,speed_up_button_) == true){
			if(!bRegisteredButtonEvent[speed_up_button_])
				if(current_vel < 0.9){
					current_vel = current_vel + 0.1;
					bRegisteredButtonEvent[speed_up_button_] = true;
			 	 	ROS_INFO("Velocity: %f%%", current_vel*100.0);
  					char buf[50]="\0";
					int percent = (int) (current_vel*100.0);
					sprintf(buf," %d percent", percent);
                    // sc.say(buf);
				}
		  
		}else{
			bRegisteredButtonEvent[speed_up_button_] = false;
		}
		 
		vel.angular.x = current_vel*(a_scale_*joy->axes[angular_]);
		vel.angular.y = current_vel*(a_scale_*joy->axes[angular_]);
		vel.angular.z = current_vel*(a_scale_*joy->axes[angular_]);
		vel.linear.x = current_vel*l_scale_*joy->axes[linear_x_];
		vel.linear.y = current_vel*l_scale_*joy->axes[linear_y_];
		vel.linear.z = current_vel*l_scale_z_*joy->axes[linear_z_];

		// ELEVATOR

        if (joy->axes[axis_elevator_]>0.99){
            //ROS_INFO("RB2Pad::padCallback: button %d calling service:%s RAISE", button_stop_elevator_,elevator_service_name_.c_str());
            robotnik_msgs::SetElevator elevator_msg_srv;

            elevator_msg_srv.request.action.action = robotnik_msgs::ElevatorAction::RAISE;
            set_elevator_client_.call( elevator_msg_srv );

        }

        if (joy->axes[axis_elevator_]<-0.99){
            //ROS_INFO("RB2Pad::padCallback: button %d calling service:%s LOWER", button_stop_elevator_,elevator_service_name_.c_str());
            robotnik_msgs::SetElevator elevator_msg_srv;

            elevator_msg_srv.request.action.action = robotnik_msgs::ElevatorAction::LOWER;
            set_elevator_client_.call( elevator_msg_srv );
        }

	}
   	else {

		// MANUAL RELEASE -> 0
		if(manual_release_false_number_ < ITERATIONS_WRITE_MODBUS){
			setManualRelease(false);
			//setBumperOverride(false);
			manual_release_false_number_++;
		}

		manual_release_true_number_ = 0;
		bumper_override_false_number_ = 0;
		bumper_override_true_number_ = 0;
		vel.angular.x = 0.0;	vel.angular.y = 0.0; vel.angular.z = 0.0;
		vel.linear.x = 0.0; vel.linear.y = 0.0; vel.linear.z = 0.0;
	}

	sus_joy_freq->tick();	// Ticks the reception of joy events

        // Publish only with deadman button pushed for twist use
        if (checkButtonPressed(joy->buttons,dead_man_button_) == true) {
                send_iterations_after_dead_man = ITERATIONS_AFTER_DEADMAN;
                if (checkButtonPressed(joy->buttons, dead_man_unsafe_button_) == true and has_unsafe_vel_) {
		    unsafe_vel_pub_.publish(vel);
                }
else
{
		    vel_pub_.publish(vel);
}
		    pub_command_freq->tick();
		}else { // send some 0 if deadman is released
          if (send_iterations_after_dead_man >0) {
		    send_iterations_after_dead_man--;
                if (checkButtonPressed(joy->buttons, dead_man_unsafe_button_) == true and has_unsafe_vel_) {
		    unsafe_vel_pub_.publish(vel);
                }
else
{
		    vel_pub_.publish(vel);
}
	            pub_command_freq->tick(); 
	        }
        }
}

int RB2Pad::setManualRelease(bool value){
	std_srvs::SetBool set_bool_msg;

	set_bool_msg.request.data = value;
	set_manual_release_client_.call(set_bool_msg);

	return 0;
} 

int RB2Pad::setBumperOverride(bool value){
	std_srvs::SetBool set_bool_msg;

	set_bool_msg.request.data = value;
	set_bumper_override_client_.call(set_bool_msg);

	return 0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rb2_pad");
	RB2Pad rb2_pad;

	ros::Rate r(50.0);

	while( ros::ok() ){
		// UPDATING DIAGNOSTICS
		rb2_pad.Update();
		ros::spinOnce();
		r.sleep();
		}
}

