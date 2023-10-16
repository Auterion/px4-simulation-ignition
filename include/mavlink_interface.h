/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2020 PX4 Pro Development Team
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

#pragma once

#include <vector>
#include <queue>
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
#include <cstdint>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <Eigen/Eigen>
#include<Eigen/StdVector>

#include <development/mavlink.h>
#include "msgbuffer.h"

static const uint32_t kDefaultMavlinkUdpRemotePort = 14560;
static const uint32_t kDefaultMavlinkUdpLocalPort = 0;
static const uint32_t kDefaultMavlinkTcpPort = 4560;

static const size_t kMaxRecvBufferSize = 20;
static const size_t kMaxSendBufferSize = 30;

using lock_guard = std::lock_guard<std::recursive_mutex>;
static constexpr auto kDefaultDevice = "/dev/ttyACM0";
static constexpr auto kDefaultBaudRate = 921600;

//! Maximum buffer size with padding for CRC bytes (280 + padding)
static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
static constexpr size_t MAX_TXQ_SIZE = 1000;

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

namespace SensorData {
    struct Imu {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d accel_b;
        Eigen::Vector3d gyro_b;
    };

    struct Barometer {
        double temperature;
        double abs_pressure;
        double pressure_alt;
    };

    struct Magnetometer {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d mag_b;
    };

    struct Airspeed {
        double diff_pressure;
    };
}

/*
struct HILData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id=-1;
    bool baro_updated{false};
    bool diff_press_updated{false};
    bool mag_updated{false};
    bool imu_updated{false};
    double temperature;
    double pressure_alt;
    double abs_pressure;
    double diff_pressure;
    Eigen::Vector3d mag_b;
    Eigen::Vector3d accel_b;
    Eigen::Vector3d gyro_b;
};
*/

class MavlinkInterface {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MavlinkInterface();
    ~MavlinkInterface();
    void ReadMAVLinkMessages();
    std::shared_ptr<mavlink_message_t> PopRecvMessage();
    void PushSendMessage(std::shared_ptr<mavlink_message_t> msg);
    void PushSendMessage(mavlink_message_t* msg);
    void send_mavlink_message(const mavlink_message_t *message);
    void forward_mavlink_message(const mavlink_message_t *message);
    void open();
    void close();
    void Load();
    void SendSensorMessages(const uint64_t time_usec);
    void UpdateBarometer(const SensorData::Barometer &data);
    void UpdateAirspeed(const SensorData::Airspeed &data);
    void UpdateIMU(const SensorData::Imu &data);
    void UpdateMag(const SensorData::Magnetometer &data);
    Eigen::VectorXd GetActuatorControls();
    bool GetArmedState();
    void onSigInt();
    uint16_t FinalizeOutgoingMessage(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
        uint8_t min_length, uint8_t length, uint8_t crc_extra);
    bool GetReceivedFirstActuator() {return received_first_actuator_;}
    void SetBaudrate(int baudrate) {baudrate_ = baudrate;}
    void SetUseTcp(bool use_tcp) {use_tcp_ = use_tcp;}
    void SetUseTcpClientMode(bool tcp_client_mode) {tcp_client_mode_ = tcp_client_mode;}
    void SetDevice(std::string device) {device_ = device;}
    void SetEnableLockstep(bool enable_lockstep) {enable_lockstep_ = enable_lockstep;}
    void SetMavlinkAddr(std::string mavlink_addr) {mavlink_addr_str_ = mavlink_addr;}
    void SetMavlinkTcpPort(int mavlink_tcp_port) {mavlink_tcp_port_ = mavlink_tcp_port;}
    void SetMavlinkUdpRemotePort(int mavlink_udp_port) {mavlink_udp_remote_port_ = mavlink_udp_port;}
    void SetMavlinkUdpLocalPort(int mavlink_udp_port) {mavlink_udp_local_port_ = mavlink_udp_port;}
    bool IsRecvBuffEmpty() {return receiver_buffer_.empty();}

    bool ReceivedHeartbeats() const { return received_heartbeats_; }

private:
    bool received_actuator_{false};
    bool received_first_actuator_{false};
    bool armed_;
    bool messages_handled_{false};
    Eigen::VectorXd input_reference_;

    void handle_message(mavlink_message_t *msg);
    void handle_heartbeat(mavlink_message_t *msg);
    void handle_actuator_controls(mavlink_message_t *msg);
    void acceptConnections();
    void RegisterNewHILSensorInstance(int id);
    bool tryConnect();

    // UDP/TCP send/receive thread workers
    void ReceiveWorker();
    void SendWorker();

    static const unsigned n_out_max = 16;

    int input_index_[n_out_max];

    struct sockaddr_in local_simulator_addr_;
    socklen_t local_simulator_addr_len_;
    struct sockaddr_in remote_simulator_addr_;
    socklen_t remote_simulator_addr_len_;

    unsigned char buf_[65535];
    enum FD_TYPES {
        LISTEN_FD,
        CONNECTION_FD,
        N_FDS
    };
    struct pollfd fds_[N_FDS];
    bool use_tcp_{false};
    bool tcp_client_mode_{false};
    bool close_conn_{false};

    in_addr_t mavlink_addr_;
    std::string mavlink_addr_str_{"INADDR_ANY"};
    int mavlink_udp_remote_port_{kDefaultMavlinkUdpRemotePort}; // MAVLink refers to the PX4 simulator interface here
    int mavlink_udp_local_port_{kDefaultMavlinkUdpLocalPort}; // MAVLink refers to the PX4 simulator interface here
    int mavlink_tcp_port_{kDefaultMavlinkTcpPort}; // MAVLink refers to the PX4 simulator interface here


    int simulator_socket_fd_{0};
    int simulator_tcp_client_fd_{0};

    bool enable_lockstep_{false};

    mavlink_status_t m_status_{};
    mavlink_message_t m_buffer_{};
    std::thread io_thread_;
    std::string device_{kDefaultDevice};

    std::recursive_mutex mutex_;
    std::mutex actuator_mutex_;
    std::mutex sensor_msg_mutex_;

    std::array<uint8_t, MAX_SIZE> rx_buf_{};
    unsigned int baudrate_{kDefaultBaudRate};
    std::atomic<bool> tx_in_progress_;
    std::deque<MsgBuffer> tx_q_{};

    bool baro_updated_;
    bool diff_press_updated_;
    bool mag_updated_;
    bool imu_updated_;

    double temperature_;
    double pressure_alt_;
    double abs_pressure_;
    double diff_pressure_;
    Eigen::Vector3d mag_b_;
    Eigen::Vector3d accel_b_;
    Eigen::Vector3d gyro_b_;

    //std::vector<HILData, Eigen::aligned_allocator<HILData>> hil_data_;
    std::atomic<bool> gotSigInt_ {false};

    bool received_heartbeats_ {false};

    std::mutex receiver_buff_mtx_;
    std::queue<std::shared_ptr<mavlink_message_t>> receiver_buffer_;
    std::thread receiver_thread_;

    std::mutex sender_buff_mtx_;
    std::queue<std::shared_ptr<mavlink_message_t>> sender_buffer_;
    std::thread sender_thread_;
    std::condition_variable sender_cv_;
    std::mutex mav_status_mutex_;
    mavlink_status_t sender_m_status_{};

};
