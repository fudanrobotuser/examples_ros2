// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include "ecat_data_buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>



// std::vector<std::string> joint_names = {"M1L", "M2L", "M3L", "M4L", "M5L", "M6L", "M1R", "M2R", "M3R", "M4R", "M5R", "M6R"};

std::vector<std::string> joint_names = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7","joint_8","joint_9","joint_10","joint_11","joint_12","joint_13","joint_14"};



using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {

    subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/trajectory_controller/joint_trajectory", 1,
        std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

    subscription2_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/trajectory_controller/joint_trajectory2", 1,
        std::bind(&MinimalSubscriber::topic_callback2, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    publisher_2 = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_target", 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2), std::bind(&MinimalSubscriber::timer_callback, this));
  }

private:
  void topic_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    size_t num_points = msg->points.size();
    size_t num_joints = msg->joint_names.size();

    // std::vector<std::vector<double>> angles(num_points, std::vector<double>(num_joints, 0.0));

    for (size_t i = 0; i < num_points; i++)
    {
      memset(&new_ref, 0, sizeof(new_ref));
      for (size_t j = 0; j < num_joints; j++)
      {
        for (int i2 = 0; i2 < num_joints; i2++)
        {
          std::string name = joint_names[i2];
          if (name.compare(msg->joint_names[j].c_str()) == 0)
          {
            // angles[i][i2] = msg->points[i].positions[i2];
            new_ref.motor_ref[i2 + P_START].target_postion = static_cast<int>(std::floor(msg->points[i].positions[i2]));
            // fprintf(stderr, "a %d : %d , ", j, new_ref.motor_ref[i2].target_postion);
            break;
          }
        }
      }
      bool dataOk = edb_push_ref(&new_ref);
      if (!dataOk)
      {
        fprintf(stderr, "dataOk %d , line: %ld \n ", dataOk, i);
      }
    }
  }

  void topic_callback2(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    size_t num_points = msg->points.size();
    size_t num_joints = msg->joint_names.size();

    // std::vector<std::vector<double>> angles(num_points, std::vector<double>(num_joints, 0.0));

    for (size_t i = 0; i < num_points; i++)
    {
      memset(&new_ref, 0, sizeof(new_ref));
      for (size_t j = 0; j < num_joints; j++)
      {
        for (int i2 = 0; i2 < num_joints; i2++)
        {
          std::string name = joint_names[i2];
          if (name.compare(msg->joint_names[j].c_str()) == 0)
          {
            new_ref.motor_ref[i2 + P_START].default_position = static_cast<int>(std::floor(msg->points[i].positions[i2]));
            break;
          }
        }
      }
      bool dataOk = edb_push_ref(&new_ref);
      if (!dataOk)
      {
        fprintf(stderr, "dataOk %d , line: %ld \n ", dataOk, i);
      }
    }
  }

  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.name = joint_names;
    message.position.resize(message.name.size());
    message.effort.resize(message.name.size());
    message.velocity.resize(message.name.size());

    auto message2 = sensor_msgs::msg::JointState();
    message2.header.stamp = this->now();
    message2.name = joint_names;
    message2.position.resize(message.name.size());
    message2.effort.resize(message.name.size());

    memset(&feedback, 0, sizeof(feedback));
    bool dataOk = edb_pull_fdbk(&feedback);

    if (dataOk)
    {
      for (size_t i = 0; i < message.name.size(); ++i)
      {
        message.position[i] = feedback.motor_fdbk[i + P_START].feedbk_postion;
        message.effort[i] = feedback.motor_fdbk[i + P_START].feedbk_torque;
        message.velocity[i] = feedback.motor_fdbk[i + P_START].feedbk_speed;

        message2.position[i] = feedback.motor_fdbk[i + P_START].target_position;
        message2.effort[i] = feedback.motor_fdbk[i + P_START].target_torque_offset;
      }
    }
    publisher_->publish(message);
    publisher_2->publish(message2);
  }

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription2_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_2;
  rclcpp::TimerBase::SharedPtr timer_;
  GROUP_REFERENCE new_ref;
  GROUP_FEEDBACK feedback;
};

int main(int argc, char *argv[])
{

  // Get the shared memory segment
  int shmid = shmget(SHM_KEY, SHM_SIZE, 0666);
  if (shmid == -1)
  {
    perror("shmget");
    return EXIT_FAILURE;
  }

  void *appPtr;

  appPtr = shmat(shmid, 0, 0);
  if (appPtr == (void *)-1)
  {
    return -1;
  }

  edb_init(appPtr, SHM_SIZE, false);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}