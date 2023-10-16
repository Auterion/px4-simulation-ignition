#include "mavlink_interface.h"

MavlinkInterface::~MavlinkInterface() {
  close();
}

void MavlinkInterface::Load()
{
  mavlink_addr_ = htonl(INADDR_ANY);
  if (mavlink_addr_str_ != "INADDR_ANY") {
    mavlink_addr_ = inet_addr(mavlink_addr_str_.c_str());
    if (mavlink_addr_ == INADDR_NONE) {
      std::cerr << "Invalid mavlink_addr: " << mavlink_addr_str_ << ", aborting" << std::endl;
      abort();
    }
  }

  // initialize sender status to zero
  memset((char *)&sender_m_status_, 0, sizeof(sender_m_status_));

  memset((char *)&remote_simulator_addr_, 0, sizeof(remote_simulator_addr_));
  remote_simulator_addr_.sin_family = AF_INET;
  remote_simulator_addr_len_ = sizeof(remote_simulator_addr_);

  memset((char *)&local_simulator_addr_, 0, sizeof(local_simulator_addr_));
  local_simulator_addr_.sin_family = AF_INET;
  local_simulator_addr_len_ = sizeof(local_simulator_addr_);

  if (use_tcp_) {

      if ((simulator_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Creating TCP socket failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      int yes = 1;
      int result = setsockopt(simulator_socket_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      struct linger nolinger {};
      nolinger.l_onoff = 1;
      nolinger.l_linger = 0;

      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      // The socket reuse is necessary for reconnecting to the same address
      // if the socket does not close but gets stuck in TIME_WAIT. This can happen
      // if the server is suddenly closed, for example, if the robot is deleted in gazebo.
      int socket_reuse = 1;
      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      // Same as above but for a given port
      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

    if (tcp_client_mode_) {
      // TCP client mode
      local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
      local_simulator_addr_.sin_port = htons(0);
      remote_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
      remote_simulator_addr_.sin_port = htons(mavlink_tcp_port_);
      memset(fds_, 0, sizeof(fds_));
    } else {
      // TCP server mode
      local_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
      local_simulator_addr_.sin_port = htons(mavlink_tcp_port_);

      if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
        std::cerr << "bind failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      errno = 0;
      if (listen(simulator_socket_fd_, 0) < 0) {
        std::cerr << "listen failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      memset(fds_, 0, sizeof(fds_));
      fds_[LISTEN_FD].fd = simulator_socket_fd_;
      fds_[LISTEN_FD].events = POLLIN; // only listens for new connections on tcp
    }
  } else {
    // When connecting to SITL, we specify the port where the mavlink traffic originates from.
    remote_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
    remote_simulator_addr_.sin_port = htons(mavlink_udp_remote_port_);
    local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    local_simulator_addr_.sin_port = htons(mavlink_udp_local_port_);

    std::cout << "Creating UDP socket for SITL input on local port : " << mavlink_udp_local_port_ << " and remote port " << mavlink_udp_remote_port_ << std::endl;

    if ((simulator_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      std::cerr << "Creating UDP socket failed: " << strerror(errno) << ", aborting" << std::endl;
      abort();
    }

    if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
      std::cerr << "bind failed: " << strerror(errno) << ", aborting" << std::endl;
      abort();
    }

    memset(fds_, 0, sizeof(fds_));
    fds_[CONNECTION_FD].fd = simulator_socket_fd_;
    fds_[CONNECTION_FD].events = POLLIN | POLLOUT; // read/write

  }

  // Start mavlink message receiver thread
  receiver_thread_ = std::thread([this] () {
    ReceiveWorker();
  });
  // Start mavlink message sender thread
  sender_thread_ = std::thread([this] () {
    SendWorker();
  });
}


/*******************************************************
 * Receive buffer handling
 */

std::shared_ptr<mavlink_message_t> MavlinkInterface::PopRecvMessage() {
  std::shared_ptr<mavlink_message_t> msg(nullptr);
  const std::lock_guard<std::mutex> guard(receiver_buff_mtx_);
  if (!receiver_buffer_.empty()) {
    msg = receiver_buffer_.front();
    receiver_buffer_.pop();
  }
  return msg;
}

void MavlinkInterface::ReceiveWorker() {
  char thrd_name[64] = {0};
  sprintf(thrd_name, "MAV_Recver_%d", gettid());
  pthread_setname_np(pthread_self(), thrd_name);

  std::cout << "[" << thrd_name << "] starts" << std::endl;

  // Wait for connection:
  if ((fds_[CONNECTION_FD].fd <= 0) && use_tcp_) {
    // Client mode
    std::cout << "[" << thrd_name << "] Wait for TCP connection.." << std::endl;
    while (fds_[CONNECTION_FD].fd <= 0) {
        if (!tcp_client_mode_) acceptConnections();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::cout << "[" << thrd_name << "] TCP connection detected" << std::endl;
  }

  std::cout << "[" << thrd_name << "] Start receiving..." << std::endl;

  while(!close_conn_ && !gotSigInt_) {
    int ret = recvfrom(fds_[CONNECTION_FD].fd, buf_, sizeof(buf_), 0, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);
    if (ret < 0) {
      std::cerr << "[" << thrd_name << "] recvfrom error: " << strerror(errno) << std::endl;
      if (errno == ECONNRESET) {
        close_conn_ = true;
      }
      continue;
    }

    // client closed the connection orderly, only makes sense on tcp
    if (use_tcp_ && ret == 0) {
      std::cerr << "[" << thrd_name << "] Connection closed by client." << std::endl;
      close_conn_ = true;
      continue;
    }

    // data received
    int len = ret;
    mavlink_status_t status;
    mavlink_message_t message;

    for(int i = 0; i < len; i++)
    {
      auto msg_received = static_cast<Framing>(mavlink_frame_char_buffer(&m_buffer_, &m_status_, buf_[i], &message, &status));
      if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
        _mav_parse_error(&m_status_);
        m_status_.msg_received = MAVLINK_FRAMING_INCOMPLETE;
        m_status_.parse_state = MAVLINK_PARSE_STATE_IDLE;
        if (buf_[i] == MAVLINK_STX) {
          m_status_.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
          m_buffer_.len = 0;
          mavlink_start_checksum(&m_buffer_);
        }
      }

      if (msg_received != Framing::incomplete) {
        auto msg = std::make_shared<mavlink_message_t>(message);
        const std::lock_guard<std::mutex> guard(receiver_buff_mtx_);
        if (receiver_buffer_.size() > kMaxRecvBufferSize) {
          std::cerr << "[" << thrd_name << "] Messages buffer overflow!" << std::endl;
          // clear the buffer
          while (!receiver_buffer_.empty()) {
            receiver_buffer_.pop();
          }
        }
        receiver_buffer_.push(msg);
      }
    }
  }
  std::cout << "[" << thrd_name << "] shutdown" << std::endl;

}

/*******************************************************
 * Send buffer handling
 */

void MavlinkInterface::PushSendMessage(std::shared_ptr<mavlink_message_t> msg) {
  const std::lock_guard<std::mutex> guard(sender_buff_mtx_);
  sender_buffer_.push(msg);
  sender_cv_.notify_one();

  if (sender_buffer_.size() > kMaxSendBufferSize) {
    sender_buffer_.pop();
    // Starts reporting buffer overflows only after the connection is established to FC
    if (received_first_actuator_) {
      std::cerr << "PushSendMessage - Messages buffer overflow!" << std::endl;
    }
  }
}

void MavlinkInterface::PushSendMessage(mavlink_message_t *msg) {
    auto msg_shared = std::make_shared<mavlink_message_t>(*msg);
    PushSendMessage(msg_shared);
}

void MavlinkInterface::SendWorker() {
  char thrd_name[64] = {0};
  sprintf(thrd_name, "MAV_Sender_%d", gettid());
  pthread_setname_np(pthread_self(), thrd_name);

  if ((fds_[CONNECTION_FD].fd <= 0) && tcp_client_mode_) {
    std::cout << "[" << thrd_name << "] Try to connect to PX4 TCP server.. " << std::endl;
    while (!tryConnect()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "[" << thrd_name << "] Client connected to PX4 TCP server" << std::endl;

  }

  while(!close_conn_ && !gotSigInt_) {
    std::unique_lock<std::mutex> lock{sender_buff_mtx_};
    sender_cv_.wait(lock, [&]()
    {
      return close_conn_ || gotSigInt_ || !sender_buffer_.empty();
    });

    std::shared_ptr<mavlink_message_t> msg;
    msg = sender_buffer_.front();
    if (msg) {
      sender_buffer_.pop();
      lock.unlock();
      send_mavlink_message(msg.get());
    } else {
      lock.unlock();
    }
  }

  std::cout << "[" << thrd_name << "] Shutdown.." << std::endl;
}

void MavlinkInterface::SendSensorMessages(uint64_t time_usec) {
  mavlink_hil_sensor_t sensor_msg;
  sensor_msg.fields_updated = 0;
  /* Workaround for mavlinkv2 zero-suppression bug
     Set last byte of message to non-zero to avoid zero-suppression. PX4
     assumes that sensor_id of hil_sensors message is always 0, so there
     is similar workaround in receiver end to reset the field back to zero.
  */
  sensor_msg.id = 1;
  sensor_msg.time_usec = time_usec;
  if (imu_updated_) {
    sensor_msg.xacc = accel_b_[0];
    sensor_msg.yacc = accel_b_[1];
    sensor_msg.zacc = accel_b_[2];
    sensor_msg.xgyro = gyro_b_[0];
    sensor_msg.ygyro = gyro_b_[1];
    sensor_msg.zgyro = gyro_b_[2];
    // std::cout <<gyro_b[2] << std::endl;

    sensor_msg.fields_updated = (uint16_t)SensorSource::ACCEL | (uint16_t)SensorSource::GYRO;

    imu_updated_ = false;
  }

  // Sensor mutes not used for imu
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);

  // send only mag data
  if (mag_updated_) {
    sensor_msg.xmag = mag_b_[0];
    sensor_msg.ymag = mag_b_[1];
    sensor_msg.zmag = mag_b_[2];
    sensor_msg.fields_updated = sensor_msg.fields_updated | (uint16_t)SensorSource::MAG;

    mag_updated_ = false;
  }

  // send only baro data
  if (baro_updated_) {
    sensor_msg.temperature = temperature_;
    sensor_msg.abs_pressure = abs_pressure_;
    sensor_msg.pressure_alt = pressure_alt_;
    sensor_msg.fields_updated = sensor_msg.fields_updated | (uint16_t)SensorSource::BARO;

    baro_updated_ = false;
  }

  // send only diff pressure data
  if (diff_press_updated_) {
    sensor_msg.diff_pressure = diff_pressure_;
    sensor_msg.fields_updated = sensor_msg.fields_updated | (uint16_t)SensorSource::DIFF_PRESS;

    diff_press_updated_ = false;
  }
  sensor_msg_mutex_.unlock();

  mavlink_message_t msg;
  mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  // Override default global mavlink channel status with instance specific status
  FinalizeOutgoingMessage(&msg, 1, 200,
    MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN,
    MAVLINK_MSG_ID_HIL_SENSOR_LEN,
    MAVLINK_MSG_ID_HIL_SENSOR_CRC);
  auto msg_shared = std::make_shared<mavlink_message_t>(msg);
  PushSendMessage(msg_shared);
}

void MavlinkInterface::UpdateBarometer(const SensorData::Barometer &data) {
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);
  temperature_ = data.temperature;
  abs_pressure_ = data.abs_pressure;
  pressure_alt_ = data.pressure_alt;

  baro_updated_ = true;
}

void MavlinkInterface::UpdateAirspeed(const SensorData::Airspeed &data) {
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);
  diff_pressure_ = data.diff_pressure;

  diff_press_updated_ = true;
}

void MavlinkInterface::UpdateIMU(const SensorData::Imu &data) {
  // Imu is updated only before sending, so locking handled there
  //   by last_imu_message_mutex_
  accel_b_ = data.accel_b;
  gyro_b_ = data.gyro_b;

  imu_updated_ = true;
}

void MavlinkInterface::UpdateMag(const SensorData::Magnetometer &data) {
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);
  mag_b_ = data.mag_b;

  mag_updated_ = true;
}

void MavlinkInterface::ReadMAVLinkMessages()
{
  if (gotSigInt_) {
    return;
  }

  received_actuator_ = false;

  if ((fds_[CONNECTION_FD].fd <= 0) && tcp_client_mode_) {
    return;
  }

  if (!enable_lockstep_ && IsRecvBuffEmpty()) {
    // Receive buffer is empty, exit
    return;
  }

  do {
    //std::cerr << "[MavlinkInterface] check message from msg buffer.. " << std::endl;
    auto msg = PopRecvMessage();
    if (msg) {
      //std::cerr << "[MavlinkInterface] ReadMAVLinkMessages -> handle_message " << std::endl;
      handle_message(msg.get());
    }
  } while( (!enable_lockstep_ && !IsRecvBuffEmpty()) ||
           ( enable_lockstep_ && !received_actuator_ && received_first_actuator_) );

}

void MavlinkInterface::acceptConnections()
{
  if (fds_[CONNECTION_FD].fd > 0) {
    return;
  }

  // accepting incoming connections on listen fd
  int ret =
    accept(fds_[LISTEN_FD].fd, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);

  if (ret < 0) {
    if (errno != EWOULDBLOCK) {
      std::cerr << "accept error: " << strerror(errno) << std::endl;
    }
    return;
  }

  // assign socket to connection descriptor on success
  fds_[CONNECTION_FD].fd = ret; // socket is replaced with latest connection
  fds_[CONNECTION_FD].events = POLLIN;
}

bool MavlinkInterface::tryConnect()
{
  if (fds_[CONNECTION_FD].fd > 0) {
    return true;
  }

  // Try connecting to px4 simulator TCP server
  int ret = connect(simulator_socket_fd_, (struct sockaddr *)&remote_simulator_addr_, remote_simulator_addr_len_);

  if (ret < 0) {
    return false;
  }

  // assign socket to connection descriptor on success
  fds_[CONNECTION_FD].events = POLLIN | POLLOUT; // read/write
  fds_[CONNECTION_FD].fd = simulator_socket_fd_;

  return true;
}

void MavlinkInterface::handle_message(mavlink_message_t *msg)
{
  switch (msg->msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT:
    handle_heartbeat(msg);
    break;
  case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
    handle_actuator_controls(msg);
    break;
  }
}

void MavlinkInterface::handle_heartbeat(mavlink_message_t *)
{
  received_heartbeats_ = true;
}

void MavlinkInterface::handle_actuator_controls(mavlink_message_t *msg)
{
  const std::lock_guard<std::mutex> lock(actuator_mutex_);

  mavlink_hil_actuator_controls_t controls;
  mavlink_msg_hil_actuator_controls_decode(msg, &controls);

  armed_ = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);

  for (unsigned i = 0; i < n_out_max; i++) {
    input_index_[i] = i;
  }

  // set rotor speeds, controller targets
  input_reference_.resize(n_out_max);
  for (int i = 0; i < input_reference_.size(); i++) {
    input_reference_[i] = controls.controls[i];
  }

  received_actuator_ = true;
  received_first_actuator_ = true;
}

void MavlinkInterface::send_mavlink_message(const mavlink_message_t *message)
{
  assert(message != nullptr);

  if (gotSigInt_ || close_conn_) {
    return;
  }

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int packetlen = mavlink_msg_to_send_buffer(buffer, message);

  if (fds_[CONNECTION_FD].fd > 0) {
    ssize_t len;
    if (use_tcp_) {
      len = send(fds_[CONNECTION_FD].fd, buffer, packetlen, 0);
    } else {
      len = sendto(fds_[CONNECTION_FD].fd, buffer, packetlen, 0, (struct sockaddr *)&remote_simulator_addr_, remote_simulator_addr_len_);
    }
    if (len < 0) {
      if (received_first_actuator_) {
        std::cerr << "Failed sending mavlink message: " << strerror(errno) << std::endl;
        if (errno == ECONNRESET || errno == EPIPE) {
          if (use_tcp_) { // udp socket remains alive
            std::cerr << "Closing connection." << std::endl;
            close_conn_ = true;
          }
        }
      }
    }
  }
}

void MavlinkInterface::close()
{
  // Shutdown receiver side
  shutdown(fds_[CONNECTION_FD].fd, SHUT_RD);

  if (receiver_thread_.joinable())
    receiver_thread_.join();

  if (sender_thread_.joinable()) {
    sender_cv_.notify_one();
    sender_thread_.join();
  }

  ::close(fds_[CONNECTION_FD].fd);
  fds_[CONNECTION_FD] = { 0, 0, 0 };
  fds_[CONNECTION_FD].fd = -1;

  received_first_actuator_ = false;
}



void MavlinkInterface::onSigInt() {
  gotSigInt_ = true;
  close();
}

Eigen::VectorXd MavlinkInterface::GetActuatorControls() {
  const std::lock_guard<std::mutex> lock(actuator_mutex_);
  return input_reference_;
}

bool MavlinkInterface::GetArmedState() {
  const std::lock_guard<std::mutex> lock(actuator_mutex_);
  return armed_;
}

// Mavlink helper function to finalize message without global channel status
uint16_t MavlinkInterface::FinalizeOutgoingMessage(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
    const std::lock_guard<std::mutex> guard(mav_status_mutex_);
    return mavlink_finalize_message_buffer(msg, system_id, component_id, &sender_m_status_,
        min_length, length, crc_extra);
}
