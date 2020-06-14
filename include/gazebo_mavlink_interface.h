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

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/imu.pb.h>

#include <common.h>

#include <mavlink/v2.0/common/mavlink.h>
#include "msgbuffer.h"

static const uint32_t kDefaultMavlinkUdpPort = 14560;
static const uint32_t kDefaultMavlinkTcpPort = 4560;
static const uint32_t kDefaultQGCUdpPort = 14550;
static const uint32_t kDefaultSDKUdpPort = 14540;

using lock_guard = std::lock_guard<std::recursive_mutex>;
static constexpr auto kDefaultDevice = "/dev/ttyACM0";
static constexpr auto kDefaultBaudRate = 921600;


//! Maximum buffer size with padding for CRC bytes (280 + padding)
static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
static constexpr size_t MAX_TXQ_SIZE = 1000;

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

//! Rx packer framing status. (same as @p mavlink::mavlink_framing_t)
enum class Framing : uint8_t {
	incomplete = MAVLINK_FRAMING_INCOMPLETE,
	ok = MAVLINK_FRAMING_OK,
	bad_crc = MAVLINK_FRAMING_BAD_CRC,
	bad_signature = MAVLINK_FRAMING_BAD_SIGNATURE,
};


//! Enumeration to use on the bitmask in HIL_SENSOR
enum class SensorSource {
  ACCEL		= 0b111,
  GYRO		= 0b111000,
  MAG		= 0b111000000,
  BARO		= 0b1101000000000,
  DIFF_PRESS	= 0b10000000000,
};

namespace mavlink_interface
{
  class GazeboMavlinkInterface:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate
  {
    public: GazeboMavlinkInterface();
 
    public: ~GazeboMavlinkInterface() override;
    public: void Configure(const ignition::gazebo::Entity &_entity, 
                            const std::shared_ptr<const sdf::Element> &_sdf, 
                            ignition::gazebo::EntityComponentManager &_ecm, 
                            ignition::gazebo::EventManager &/*_eventMgr*/);
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm);
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;
    private:
      bool received_first_actuator_;
      Eigen::VectorXd input_reference_;

      float protocol_version_;

      std::string namespace_;
      std::string motor_velocity_reference_pub_topic_;
      std::string mavlink_control_sub_topic_;
      std::string link_name_;

      bool use_propeller_pid_;
      bool use_elevator_pid_;
      bool use_left_elevon_pid_;
      bool use_right_elevon_pid_;
      
      bool vehicle_is_tailsitter_;

      bool send_vision_estimation_;
      bool send_odometry_;

      void ImuCallback(const ignition::msgs::IMU &_msg);
      void send_mavlink_message(const mavlink_message_t *message);
      void forward_mavlink_message(const mavlink_message_t *message);
      void handle_message(mavlink_message_t *msg, bool &received_actuator);
      void acceptConnections();
      void pollForMAVLinkMessages();
      void pollFromQgcAndSdk();
      void SendSensorMessages();
      void SendGroundTruth();
      void handle_control(double _dt);
      bool IsRunning();


      // Serial interface
      void open();
      void close();
      void do_read();
      void parse_buffer(const boost::system::error_code& err, std::size_t bytes_t);
      void do_write(bool check_tx_state);
      inline bool is_open(){
        return serial_dev.is_open();
      }

      static const unsigned n_out_max = 16;

      double input_offset_[n_out_max];
      double input_scaling_[n_out_max];
      std::string joint_control_type_[n_out_max];
      std::string gztopic_[n_out_max];
      double zero_position_disarmed_[n_out_max];
      double zero_position_armed_[n_out_max];
      int input_index_[n_out_max];
      
      /// \brief Ignition communication node.
      ignition::transport::Node node;

      std::string imu_sub_topic_;
      std::string opticalFlow_sub_topic_;
      std::string irlock_sub_topic_;
      std::string gps_sub_topic_;
      std::string groundtruth_sub_topic_;
      std::string vision_sub_topic_;
      std::string mag_sub_topic_;
      std::string baro_sub_topic_;

      std::mutex last_imu_message_mutex_ {};
      std::condition_variable last_imu_message_cond_ {};
      ignition::msgs::IMU last_imu_message_;
      ignition::common::Time last_time_;
      ignition::common::Time last_imu_time_;
      ignition::common::Time last_actuator_time_;

      bool mag_updated_;
      bool baro_updated_;
      bool diff_press_updated_;

      double groundtruth_lat_rad;
      double groundtruth_lon_rad;
      double groundtruth_altitude;

      double imu_update_interval_ = 0.004; ///< Used for non-lockstep

      ignition::math::Vector3d gravity_W_;
      ignition::math::Vector3d velocity_prev_W_;
      ignition::math::Vector3d mag_n_;

      double temperature_;
      double pressure_alt_;
      double abs_pressure_;

      std::default_random_engine random_generator_;
      std::normal_distribution<float> standard_normal_distribution_;

      struct sockaddr_in local_simulator_addr_;
      socklen_t local_simulator_addr_len_;
      struct sockaddr_in remote_simulator_addr_;
      socklen_t remote_simulator_addr_len_;

      int qgc_udp_port_;
      struct sockaddr_in remote_qgc_addr_;
      socklen_t remote_qgc_addr_len_;
      struct sockaddr_in local_qgc_addr_;
      socklen_t local_qgc_addr_len_;

      int sdk_udp_port_;
      struct sockaddr_in remote_sdk_addr_;
      socklen_t remote_sdk_addr_len_;
      struct sockaddr_in local_sdk_addr_;
      socklen_t local_sdk_addr_len_;

      unsigned char _buf[65535];
      enum FD_TYPES {
        LISTEN_FD,
        CONNECTION_FD,
        N_FDS
      };
      struct pollfd fds_[N_FDS];
      bool use_tcp_ = false;
      bool close_conn_ = false;

      double optflow_distance;
      double sonar_distance;

      in_addr_t mavlink_addr_;
      int mavlink_udp_port_; // MAVLink refers to the PX4 simulator interface here
      int mavlink_tcp_port_; // MAVLink refers to the PX4 simulator interface here

      int simulator_socket_fd_;
      int simulator_tcp_client_fd_;

      int qgc_socket_fd_ {-1};
      int sdk_socket_fd_ {-1};

      bool enable_lockstep_ = false;
      double speed_factor_ = 1.0;
      int64_t previous_imu_seq_ = 0;
      unsigned update_skip_factor_ = 1;

      // Serial interface
      mavlink_status_t m_status;
      mavlink_message_t m_buffer;
      bool serial_enabled_;
      std::thread io_thread;
      std::string device_;
      std::array<uint8_t, MAX_SIZE> rx_buf;
      std::recursive_mutex mutex;
      unsigned int baudrate_;
      std::atomic<bool> tx_in_progress;
      std::deque<gazebo::MsgBuffer> tx_q;
      boost::asio::io_service io_service;
      boost::asio::serial_port serial_dev;

      bool hil_mode_;
      bool hil_state_level_;

      std::atomic<bool> gotSigInt_ {false};
  };
}

#endif