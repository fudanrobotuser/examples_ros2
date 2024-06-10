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



using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {

    subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/trajectory_controller/joint_trajectory", 10,
        std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&MinimalSubscriber::timer_callback, this));
  }

  

private:
  void topic_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    size_t num_points = msg->points.size();
    size_t num_joints = msg->joint_names.size();

    std::vector<std::vector<double>> angles(num_points, std::vector<double>(num_joints, 0.0));

    for (size_t i = 0; i < num_points; ++i)
    {
      for (size_t j = 0; j < num_joints; ++j)
      {
        angles[i][j] = msg->points[i].positions[j];
        
        memset(&new_ref, 0, sizeof(new_ref));
        new_ref.motor_ref[j].target_postion = angles[i][j];
        bool dataOk = edb_push_ref(&new_ref);
        fprintf(stderr, "dataOk %d \n ", dataOk);
      }
    }
  }

  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9", "joint10", "joint11", "joint12", "joint13", "joint14", "joint15", "joint16", "joint17", "joint18"};
    message.position.resize(message.name.size());
    message.effort.resize(message.name.size());
    message.velocity.resize(message.name.size());

    memset(&feedback, 0, sizeof(feedback));
    bool dataOk = edb_pull_fdbk(&feedback);

    if(dataOk){
      for (size_t i = 0; i < message.name.size(); ++i) {
        message.position[i] = feedback.motor_fdbk[i].feedbk_postion;
        message.effort[i] = feedback.motor_fdbk[i].feedbk_torque;
        message.velocity[i] = feedback.motor_fdbk[i].feedbk_speed;
        
      }

      
    }
    publisher_->publish(message);
  }

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_ ;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
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