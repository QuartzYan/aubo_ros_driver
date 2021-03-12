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

#include "aubo_base/aubo_hardware.h"

namespace aubo_base
{
  AuboHardware::AuboHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, boost::asio::io_service &io_service)
      : nh_(nh), private_nh_(private_nh), io_service_(io_service), socket_(io_service_)
  {
    ros::V_string joint_names = boost::assign::list_of("shoulder_joint")("upperArm_joint")("foreArm_joint")("wrist1_joint")("wrist2_joint")("wrist3_joint");

    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position,
                                                              &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle1(joint_state_handle,
                                                    &joints_[i].velocity_command);
      hardware_interface::JointHandle joint_handle2(joint_state_handle,
                                                    &joints_[i].position_command);
      velocity_joint_interface_.registerHandle(joint_handle1);
      position_joint_interface_.registerHandle(joint_handle2);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&position_joint_interface_);

    private_nh_.param<std::string>("robot_ip", ip_, "192.168.3.10");
    private_nh_.param<int32_t>("robot_port", port_, 9090);
    private_nh_.param<double>("max_joint_vel", max_joint_vel_, 1.0);
    private_nh_.param<double>("max_joint_acc", max_joint_acc_, 1.0);

    ROS_INFO("waiting connect to robot!");

    tcp::resolver resolver_(io_service_);
    auto endpoint_iterator_ = resolver_.resolve(tcp::endpoint(boost::asio::ip::address::from_string(ip_), port_));
    socket_.open(boost::asio::ip::tcp::v4());
    //socket_.io_control(boost::asio::ip::tcp::socket::non_blocking_io(true));
    //boost::asio::socket_base::bytes_readable command(true); //non block
    //socket_.io_control(command);
    socket_.non_blocking(true);
    boost::system::error_code error;
    socket_.connect(tcp::endpoint(boost::asio::ip::address::from_string(ip_), port_), error);
    if (error)
    {
      ROS_ERROR_STREAM("conect error:" << error.message());
      exit(-1);
    }

    ROS_INFO("connected to robot!!");

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    initAubo();
    setJointMaxVel(max_joint_vel_);
    setJointMaxAcc(max_joint_acc_);
    // setTCP2CANBUS();
    // //cancelTCP2CANBUS();
  }

  AuboHardware::~AuboHardware()
  {
    socket_.close();
  }

  bool AuboHardware::initAubo()
  {
    CJsonObject command;
    command.Add("command", "robot_startup");
    return requestFromAubo("robot_startup", command.ToString());
  }

  bool AuboHardware::setJointMaxVel(const double max_vel)
  {
    CJsonObject command;
    command.Add("command", "set_joint_maxvelc");
    command.AddEmptySubArray("joint_maxvelc");
    command["joint_maxvelc"].Add(max_vel);
    command["joint_maxvelc"].Add(max_vel);
    command["joint_maxvelc"].Add(max_vel);
    command["joint_maxvelc"].Add(max_vel);
    command["joint_maxvelc"].Add(max_vel);
    command["joint_maxvelc"].Add(max_vel);

    return requestFromAubo("set_joint_maxvelc", command.ToString());
  }

  bool AuboHardware::setJointMaxAcc(const double max_acc)
  {
    CJsonObject command;
    command.Add("command", "set_joint_maxacc");
    command.AddEmptySubArray("joint_maxacc");
    command["joint_maxacc"].Add(max_acc);
    command["joint_maxacc"].Add(max_acc);
    command["joint_maxacc"].Add(max_acc);
    command["joint_maxacc"].Add(max_acc);
    command["joint_maxacc"].Add(max_acc);
    command["joint_maxacc"].Add(max_acc);

    return requestFromAubo("set_joint_maxacc", command.ToString());
  }

  bool AuboHardware::setTCP2CANBUS()
  {
    CJsonObject command;
    command.Add("command", "enter_tcp2canbus_mode");
    return requestFromAubo("enter_tcp2canbus_mode", command.ToString());
  }

  bool AuboHardware::cancelTCP2CANBUS()
  {
    CJsonObject command;
    command.Add("command", "leave_tcp2canbus_mode");
    return requestFromAubo("leave_tcp2canbus_mode", command.ToString());
  }

  // Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  void AuboHardware::updateJointsFromHardware()
  {
    //get joint rad
    CJsonObject get_joint_comm;
    get_joint_comm.Add("command", "get_current_waypoint");
    //std::cout << get_joint_comm.ToString() << std::endl;

    //request/message from client
    boost::system::error_code error;
    socket_.write_some(boost::asio::buffer(get_joint_comm.ToString()), error);
    if (error)
    {
      ROS_ERROR("send faild!");
      return;
    }

    ros::Time send_time = ros::Time::now();

    //getting response from arm
    boost::array<char, 1024> buf;
    while (true)
    {
      size_t len = socket_.read_some(boost::asio::buffer(buf), error);
      if (error && error != boost::asio::error::eof && error.value() != 11)
      {
        ROS_ERROR_STREAM("receive faild: " << error.message());
        return;
      }
      if (len > 0)
      {
        std::string str(buf.data(), len);
        //std::cout << str << std::endl;
        CJsonObject json(str);
        if (!json.GetErrMsg().empty())
        {
          ROS_ERROR_STREAM("json recode error: " << json.GetErrMsg());
          return;
        }

        std::string jointKey("joint");
        while (json["ret"].GetKey(jointKey))
        {
          CJsonObject joint;
          if (json["ret"].Get("joint", joint))
          {
            if (joint.IsArray() && joint.GetArraySize() == 6)
            {
              for (size_t i = 0; i < 6; i++)
              {
                joint.Get(i, joints_[i].position);
              }
              //std::cout << str << std::endl;
              return;
            }
          }
        }
      }

      //time out return
      if (ros::Time::now().toSec() > (send_time.toSec() + 0.5))
      {
        ROS_WARN_STREAM("request data time out!!");
        return;
      }

      //sleep
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // for (size_t i = 0; i < 6; i++)
    // {
    //  joints_[i].position = joints_[i].position_command;
    // }
  }

  // Get latest velocity commands from ros_control via joint structure, and send to arm
  void AuboHardware::writeCommandsToHardware()
  {
    static bool first_flage = true;
    static double last_comm[6] = {0,};
    static double comm[6] = {0,};

    // static int cc = 0;

    // if(cc < 2)
    // {
    //   cc++;
    //   return;
    // }

    if (first_flage)
    {
      first_flage = false;
      for (size_t i = 0; i < 6; i++)
      {
        last_comm[i] = joints_[i].position_command;
      }
      return;
    }

    bool check_falage = true;

    for (size_t i = 0; i < 6; i++)
    {
      if(abs(last_comm[i] - joints_[i].position_command) > 0.001)
      {
        check_falage = false;
        break;
      }
      else
      {
        //joints_[i].position_command = last_comm[i];
      }
    }

    if(check_falage)
    {
      return;
    }

    for (size_t i = 0; i < 6; i++)
    {
      last_comm[i] = joints_[i].position_command;
    }
    
    //get joint rad
    CJsonObject set_joint_comm;
    set_joint_comm.Add("command", "move_joint");
    set_joint_comm.AddEmptySubArray("joint_radian");
    set_joint_comm["joint_radian"].Add(joints_[0].position_command);
    set_joint_comm["joint_radian"].Add(joints_[1].position_command);
    set_joint_comm["joint_radian"].Add(joints_[2].position_command);
    set_joint_comm["joint_radian"].Add(joints_[3].position_command);
    set_joint_comm["joint_radian"].Add(joints_[4].position_command);
    set_joint_comm["joint_radian"].Add(joints_[5].position_command);

    std::cout << set_joint_comm.ToString() << std::endl;

    boost::system::error_code error;
    //send msg to arm
    socket_.write_some(boost::asio::buffer(set_joint_comm.ToString()), error);
    if (error)
    {
      ROS_ERROR_STREAM("send faild: " << error.message());
      return;
    }

    ros::Time send_time = ros::Time::now();

    //getting response from arm
    boost::array<char, 1024> buf;
    while (true)
    {
      size_t len = socket_.read_some(boost::asio::buffer(buf), error);
      if (error && error != boost::asio::error::eof && error.value() != 11)
      {
        ROS_ERROR_STREAM("receive faild: " << error.message());
        return;
      }
      if (len > 0)
      {
        std::string rte(buf.data(), len);

        CJsonObject json(rte);
        if (!json.GetErrMsg().empty())
        {
          ROS_ERROR_STREAM("json recode error: " << json.GetErrMsg());
          return;
        }

        std::string msgKey("msg");
        while (json.GetKey(msgKey))
        {
          std::string msgType;
          if (json.Get(msgKey, msgType))
          {
            if (msgType == "succ")
            {
              return;
            }
            else if (msgType == "faild")
            {
              ROS_INFO_STREAM(json.ToString());
              return;
            }
            else if (msgType == "Move Error!")
            {
              ROS_INFO_STREAM(json.ToString());
              return;
            }
            else
            {
              ROS_INFO_STREAM(json.ToString());
            }
          }
        }
      }

      if (ros::Time::now().toSec() > (send_time.toSec() + 1.0))
      {
        ROS_WARN_STREAM("request data time out!!");
        return;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  bool AuboHardware::requestFromAubo(const std::string name, const std::string comm)
  {
    ROS_INFO_STREAM(name << "......");

    boost::system::error_code error;
    //send msg to arm
    socket_.write_some(boost::asio::buffer(comm), error);
    if (error)
    {
      ROS_ERROR_STREAM("send faild: " << error.message());
      ROS_ERROR_STREAM(name << ": faild!");
      return false;
    }

    ros::Time send_time = ros::Time::now();

    //getting response from arm
    boost::array<char, 1024> buf;
    while (true)
    {
      size_t len = socket_.read_some(boost::asio::buffer(buf), error);
      if (error && error != boost::asio::error::eof && error.value() != 11)
      {
        ROS_ERROR_STREAM("receive faild: " << error.message());
        ROS_ERROR_STREAM(name << ": faild!");
        return false;
      }
      if (len > 0)
      {
        std::string rte(buf.data(), len);

        CJsonObject json(rte);
        if (!json.GetErrMsg().empty())
        {
          ROS_ERROR_STREAM("json recode error: " << json.GetErrMsg());
          return false;
        }

        std::string msgKey("msg");
        while (json.GetKey(msgKey))
        {
          std::string msgType;
          if (json.Get(msgKey, msgType))
          {
            if (msgType == "succ")
            {
              ROS_INFO_STREAM(name << ": successful!");
              return true;
            }
            else if (msgType == "faild")
            {
              ROS_ERROR_STREAM(name << ": faild!");
              return false;
            }
            else if (msgType == "Move Error!")
            {
              ROS_ERROR_STREAM(name << ": faild!");
              return false;
            }
            else
            {
              //std::cout << json.ToString() << std::endl;
              ROS_INFO_STREAM(json.ToString());
            }
          }
        }
      }

      if (ros::Time::now().toSec() > (send_time.toSec() + 5.0))
      {
        ROS_ERROR_STREAM("request data time out!!");
        ROS_ERROR_STREAM(name << ": faild!");
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

} // namespace aubo_base
