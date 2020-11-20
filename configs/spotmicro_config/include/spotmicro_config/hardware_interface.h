#ifndef ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

#include <boost/scoped_ptr.hpp>
#include <chrono>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tr1cpp/tr1.h>
#include <spotmicro_config/hardware.h>

#include <i2cpwm_board/i2cpwm_lib.h>


using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;


namespace ROBOT_hardware_interface
{
    class ROBOTHardwareInterface: public ROBOT_hardware_interface::ROBOTHardware
    {
        public:
            ROBOTHardwareInterface(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle);
            ~ROBOTHardwareInterface();
            void read(const ros::Time& time, const ros::Duration& elapsed_time);
            void write(const ros::Time& time, const ros::Duration& elapsed_time);

        protected:
            tr1cpp::TR1 TR1;
            std::unique_ptr<I2cPWMLib> pwm_controller;
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Duration control_period_;
            PositionJointInterface positionJointInterface;
            PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
    };

}

#endif
