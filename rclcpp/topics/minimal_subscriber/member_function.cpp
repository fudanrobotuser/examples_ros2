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

#define SHM_KEY 12345
#define SHM_SIZE (1024 * 1024 * 2)

std::vector<std::string> joint_names = {"W1L", "W2R", "W3B", "M1L", "M2L", "M3L", "M4L", "M5L", "M6L", "M1R", "M2R", "M3R", "M4R", "M5R", "M6R"};

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

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    publisher_2 = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_target", 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(8), std::bind(&MinimalSubscriber::timer_callback, this));
  }

private:
  void topic_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    size_t num_points = msg->points.size();
    size_t num_joints = msg->joint_names.size();

    std::vector<std::vector<double>> angles(num_points, std::vector<double>(num_joints, 0.0));

    for (size_t i = 0; i < num_points; ++i)
    {
      memset(&new_ref, 0, sizeof(new_ref));
      for (size_t j = 0; j < num_joints; j++)
      {
        for (int i2 = 0; i2 < num_joints; i2++)
        {
          std::string name = joint_names[i2];
          if (name.compare(msg->joint_names[j].c_str()) == 0)
          {
            angles[i][i2] = msg->points[i].positions[i2];
            new_ref.motor_ref[i2].target_postion = angles[i][i2];
            fprintf(stderr, "a %d : %d , ", j, angles[i][i2]);
            break;
          }
        }
      }
      bool dataOk = edb_push_ref(&new_ref);
      fprintf(stderr, "dataOk %d , line: %ld \n ", dataOk, i);
    }
  }

  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.name = joint_names;
    message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9", "joint10", "joint11", "joint12", "joint13", "joint14", "joint15"};
    message.position.resize(message.name.size());
    message.effort.resize(message.name.size());
    message.velocity.resize(message.name.size());

    auto message2 = sensor_msgs::msg::JointState();
    message2.header.stamp = this->now();
    message2.name = joint_names;
    message2.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9", "joint10", "joint11", "joint12", "joint13", "joint14", "joint15"};
    message2.position.resize(message.name.size());

    memset(&feedback, 0, sizeof(feedback));
    bool dataOk = edb_pull_fdbk(&feedback);

    if (dataOk)
    {
      for (size_t i = 0; i < message.name.size(); ++i)
      {
        message.position[i] = feedback.motor_fdbk[i].feedbk_postion;
        message.effort[i] = feedback.motor_fdbk[i].feedbk_torque;
        message.velocity[i] = feedback.motor_fdbk[i].feedbk_speed;

        message2.position[i] = feedback.motor_fdbk[i].target_position;
      }
    }
    publisher_->publish(message);
    publisher_2->publish(message2);
  }

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_2;
  rclcpp::TimerBase::SharedPtr timer_;
  GROUP_REFERENCE new_ref;
  GROUP_FEEDBACK feedback;
};

int main(int argc, char *argv[])
{

  // Get the shared memory segment
  int shmid = shmget(SHM_KEY, SHM_SIZE, IPC_CREAT | 0666);

  if (shmid == -1)
  {
    return EXIT_FAILURE;
  }
  void *appPtr;

  appPtr = shmat(shmid, 0, 0);
  if (appPtr == (void *)-1)
  {
    return -1;
  }

  edb_init(appPtr, SHM_SIZE, true);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}