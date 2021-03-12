/*******************************************************************************
** MIT License
**
** Copyright(c) 2021 QuartzYan https://github.com/QuartzYan
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files(the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions :
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*******************************************************************************/

#ifndef AUBO_BASE_AUBO_HARDWARE_H
#define AUBO_BASE_AUBO_HARDWARE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <boost/assign.hpp>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <map>

#include "CJsonObject.hpp"

using namespace neb;
using boost::asio::ip::tcp;

namespace aubo_base
{
  class AuboHardware : public hardware_interface::RobotHW
  {
  public:
    AuboHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, boost::asio::io_service& io_service);
    ~AuboHardware();

    void updateJointsFromHardware();
    void writeCommandsToHardware();

  private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher aubo_status_pub_;

    // ROS param
    std::string ip_;
    int32_t port_;
    double max_joint_vel_;
    double max_joint_acc_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    // socket
    boost::asio::io_service& io_service_;
    boost::asio::ip::tcp::socket socket_;

    //inint aubo
    bool initAubo();
    bool setJointMaxVel(const double max_vel);
    bool setJointMaxAcc(const double max_acc);
    bool setTCP2CANBUS();
    bool cancelTCP2CANBUS();

    bool requestFromAubo(const std::string name, const std::string comm);
    
    // Joint structure that is hooked to ros_control's InterfaceManager, 
    // to allow control via diff_drive_controller
    struct Joint
    {
      double effort;
      double position;
      double position_offset;
      double position_command;
      double velocity;
      double velocity_command;

      Joint() :
        effort(0), position(0), position_offset(0), position_command(0), velocity(0), velocity_command(0)
      { }
    } joints_[6];
  };
}  // namespace aubo_base
#endif  // AUBO_BASE_AUBO_HARDWARE_H