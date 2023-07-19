/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Pro Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MAVLINK_INTERFACE_HH_
#define MAVLINK_INTERFACE_HH_

#include <vector>
#include <regex>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>

#include <iostream>
#include <random>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <gz/common5/gz/common.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include "gz/sim/components/Actuators.hh"
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>
#include <Pressure.pb.h>
#include <MagneticField.pb.h>
#include <SITLGps.pb.h>

#include <common.h>

#include <development/mavlink.h>
#include "msgbuffer.h"
#include "mavlink_interface.h"


using lock_guard = std::lock_guard<std::recursive_mutex>;

//! Default distance sensor model joint naming
static const std::regex kDefaultLidarModelLinkNaming("(lidar|sf10a)(.*::link)");
static const std::regex kDefaultSonarModelLinkNaming("(sonar|mb1240-xl-ez4)(.*::link)");

// Default values
static const std::string kDefaultNamespace = "";

// This just proxies the motor commands from command/motor_speed to the single motors via internal
// ConsPtr passing, such that the original commands don't have to go n_motors-times over the wire.
static const std::string kDefaultMotorVelocityReferencePubTopic = "/gazebo/command/motor_speed";

static const std::string kDefaultImuTopic = "/imu";
static const std::string kDefaultOpticalFlowTopic = "/px4flow/link/opticalFlow";
static const std::string kDefaultIRLockTopic = "/camera/link/irlock";
static const std::string kDefaultGPSTopic = "/gps";
static const std::string kDefaultVisionTopic = "/vision_odom";
static const std::string kDefaultMagTopic = "/mag";
static const std::string kDefaultBarometerTopic = "/baro";

namespace mavlink_interface
{
  class GZ_SIM_VISIBLE GazeboMavlinkInterface:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    public: GazeboMavlinkInterface();

    public: ~GazeboMavlinkInterface() override;
    public: void Configure(const gz::sim::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &/*_eventMgr*/);
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm);
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
    private:
      gz::common::ConnectionPtr sigIntConnection_;
      std::shared_ptr<MavlinkInterface> mavlink_interface_;
      bool received_first_actuator_{false};
      Eigen::VectorXd input_reference_;

      gz::sim::Entity entity_{gz::sim::kNullEntity};
      gz::sim::Model model_{gz::sim::kNullEntity};
      gz::sim::Entity modelLink_{gz::sim::kNullEntity};
      std::string model_name_;

      float protocol_version_{2.0};

      std::string namespace_{kDefaultNamespace};
      std::string motor_velocity_reference_pub_topic_{kDefaultMotorVelocityReferencePubTopic};
      std::string mavlink_control_sub_topic_;
      std::string link_name_;

      bool use_propeller_pid_{false};
      bool use_elevator_pid_{false};
      bool use_left_elevon_pid_{false};
      bool use_right_elevon_pid_{false};

      void ImuCallback(const gz::msgs::IMU &_msg);
      void BarometerCallback(const sensor_msgs::msgs::Pressure &_msg);
      void MagnetometerCallback(const sensor_msgs::msgs::MagneticField &_msg);
      void GpsCallback(const sensor_msgs::msgs::SITLGps &_msg);
      void SendSensorMessages(const gz::sim::UpdateInfo &_info);
      void SendGroundTruth();
      void PublishRotorVelocities(gz::sim::EntityComponentManager &_ecm,
          const Eigen::VectorXd &_vels);
      void handle_actuator_controls(const gz::sim::UpdateInfo &_info);
      void handle_control(double _dt);
      void onSigInt();
      bool IsRunning();

      static const unsigned n_out_max = 16;

      double input_offset_[n_out_max];
      Eigen::VectorXd input_scaling_;
      std::string joint_control_type_[n_out_max];
      std::string gztopic_[n_out_max];
      double zero_position_disarmed_[n_out_max];
      double zero_position_armed_[n_out_max];
      int input_index_[n_out_max];

      /// \brief gz communication node.
      gz::transport::Node node;

      std::string opticalFlow_sub_topic_{kDefaultOpticalFlowTopic};
      std::string irlock_sub_topic_{kDefaultIRLockTopic};
      std::string gps_sub_topic_{kDefaultGPSTopic};
      std::string groundtruth_sub_topic_;
      std::string vision_sub_topic_{kDefaultVisionTopic};
      std::string mag_sub_topic_{kDefaultMagTopic};
      std::string baro_sub_topic_{kDefaultBarometerTopic};

      std::mutex last_imu_message_mutex_ {};

      gz::msgs::IMU last_imu_message_;
      gz::msgs::Actuators rotor_velocity_message_;

      std::chrono::steady_clock::duration last_imu_time_{0};
      std::chrono::steady_clock::duration lastControllerUpdateTime{0};
      std::chrono::steady_clock::duration last_actuator_time_{0};
      std::chrono::steady_clock::duration last_heartbeat_sent_time_{0};

      bool mag_updated_{false};
      bool baro_updated_;
      bool diff_press_updated_;

      double groundtruth_lat_rad{0.0};
      double groundtruth_lon_rad{0.0};
      double groundtruth_altitude{0.0};

      double imu_update_interval_ = 0.004; ///< Used for non-lockstep

      gz::math::Vector3d gravity_W_{gz::math::Vector3d(0.0, 0.0, -9.8)};
      gz::math::Vector3d velocity_prev_W_;
      gz::math::Vector3d mag_n_;

      double temperature_;
      double pressure_alt_;
      double abs_pressure_;

      bool use_tcp_ = false;
      bool close_conn_ = false;

      double optflow_distance;
      double sonar_distance;

      bool enable_lockstep_ = false;
      bool serial_enabled_;
      double speed_factor_ = 1.0;
      int64_t previous_imu_seq_ = 0;
      unsigned update_skip_factor_ = 1;

      bool hil_mode_{false};
      bool hil_state_level_{false};

      std::atomic<bool> gotSigInt_ {false};
  };
}

#endif
