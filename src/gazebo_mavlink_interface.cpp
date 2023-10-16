/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2021 PX4 Development Team
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

#include <gazebo_mavlink_interface.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <random>

#include <gz/plugin/Register.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sim/components/AirPressureSensor.hh>
#include <gz/sim/components/Magnetometer.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Pose.hh>

GZ_ADD_PLUGIN(
    mavlink_interface::GazeboMavlinkInterface,
    gz::sim::System,
    mavlink_interface::GazeboMavlinkInterface::ISystemConfigure,
    mavlink_interface::GazeboMavlinkInterface::ISystemPreUpdate,
    mavlink_interface::GazeboMavlinkInterface::ISystemPostUpdate)
using namespace mavlink_interface;

GazeboMavlinkInterface::GazeboMavlinkInterface() :
    input_scaling_ {},
    input_index_ {}
    {
      mavlink_interface_ = std::make_shared<MavlinkInterface>();
    }

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  mavlink_interface_->close();
}

void GazeboMavlinkInterface::Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_em) {

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->Get<std::string>("robotNamespace");
  } else {
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace." << std::endl;
  }

  entity_ = _entity;
  model_ = gz::sim::Model(_entity);
  model_name_ = model_.Name(_ecm);

  if (_sdf->HasElement("protocol_version")) {
    protocol_version_ = _sdf->Get<float>("protocol_version");
  }


  gazebo::getSdfParam<std::string>(_sdf, "poseSubTopic", pose_sub_topic_, pose_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "gpsSubTopic", gps_sub_topic_, gps_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "visionSubTopic", vision_sub_topic_, vision_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "opticalFlowSubTopic", opticalFlow_sub_topic_, opticalFlow_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "irlockSubTopic", irlock_sub_topic_, irlock_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "magSubTopic", mag_sub_topic_, mag_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "baroSubTopic", baro_sub_topic_, baro_sub_topic_);

  // set input_reference_ from inputs.control
  input_reference_.resize(n_out_max);

  ///TODO: Parse input reference
  input_scaling_.resize(n_out_max);
  input_scaling_(0) = 1000;
  input_scaling_(1) = 1000;
  input_scaling_(2) = 1000;
  input_scaling_(3) = 1000;

  bool use_tcp = false;
  if (_sdf->HasElement("use_tcp"))
  {
    use_tcp = _sdf->Get<bool>("use_tcp");
    mavlink_interface_->SetUseTcp(use_tcp);
  }

  bool tcp_client_mode = false;
  if (_sdf->HasElement("tcp_client_mode"))
  {
    tcp_client_mode = _sdf->Get<bool>("tcp_client_mode");
    mavlink_interface_->SetUseTcpClientMode(tcp_client_mode);
  }
  gzmsg << "Connecting to PX4 SITL using " << (use_tcp ? (tcp_client_mode ? "TCP (client mode)" : "TCP (server mode)") : "UDP") << std::endl;

  if (_sdf->HasElement("enable_lockstep"))
  {
    enable_lockstep_ = _sdf->Get<bool>("enable_lockstep");
    mavlink_interface_->SetEnableLockstep(enable_lockstep_);
  }
  gzmsg << "Lockstep is " << (enable_lockstep_ ? "enabled" : "disabled") << std::endl;

  // When running in lockstep, we can run the simulation slower or faster than
  // realtime. The speed can be set using the env variable PX4_SIM_SPEED_FACTOR.
  if (enable_lockstep_)
  {
    const char *speed_factor_str = std::getenv("PX4_SIM_SPEED_FACTOR");
    if (speed_factor_str)
    {
      speed_factor_ = std::atof(speed_factor_str);
      if (!std::isfinite(speed_factor_) || speed_factor_ <= 0.0)
      {
        gzerr << "Invalid speed factor '" << speed_factor_str << "', aborting" << std::endl;
        abort();
      }
    }
    gzmsg << "Speed factor set to: " << speed_factor_ << std::endl;
  }

  // // Listen to Ctrl+C / SIGINT.
  sigIntConnection_ = _em.Connect<gz::sim::events::Stop>(std::bind(&GazeboMavlinkInterface::onSigInt, this));

  auto world_name = "/" + gz::sim::scopedName(gz::sim::worldEntity(_ecm), _ecm);

  auto vehicle_scope_prefix = world_name + gz::sim::topicFromScopedName(
    _ecm.EntityByComponents(gz::sim::components::Name(model_name_)), _ecm, false);

  // Subscribe to messages of sensors.
  auto imu_topic = vehicle_scope_prefix + imu_sub_topic_;
  node.Subscribe(imu_topic, &GazeboMavlinkInterface::ImuCallback, this);

  auto baro_topic = vehicle_scope_prefix + baro_sub_topic_;
  node.Subscribe(baro_topic, &GazeboMavlinkInterface::BarometerCallback, this);

  auto mag_topic = vehicle_scope_prefix + mag_sub_topic_;
  node.Subscribe(mag_topic, &GazeboMavlinkInterface::MagnetometerCallback, this);

  auto gps_topic = vehicle_scope_prefix + gps_sub_topic_;
  node.Subscribe(gps_topic, &GazeboMavlinkInterface::GpsCallback, this);

  // Subscribe to entity pose info message
  auto pose_topic = world_name + pose_sub_topic_;
  node.Subscribe(pose_topic, &GazeboMavlinkInterface::PoseCallback, this);

  // This doesn't seem to be used anywhere but we leave it here
  // for potential compatibility
  if (_sdf->HasElement("imu_rate")) {
    imu_update_interval_ = 1 / _sdf->Get<int>("imu_rate");
  }

  if (_sdf->HasElement("mavlink_hostname")) {
    mavlink_hostname_str_ = _sdf->Get<std::string>("mavlink_hostname");
    if (! mavlink_hostname_str_.empty()) {
      // Start hostname resolver thread
      hostname_resolver_thread_ = std::thread([this] () {
        ResolveWorker();
      });
    }
  }

  if (_sdf->HasElement("mavlink_addr")) {
    std::string mavlink_addr_str = _sdf->Get<std::string>("mavlink_addr");
    if (mavlink_addr_str != "INADDR_ANY") {
      mavlink_interface_->SetMavlinkAddr(mavlink_addr_str);
    }
  }

  if (_sdf->HasElement("mavlink_udp_remote_port")) {
    int mavlink_udp_remote_port = _sdf->Get<int>("mavlink_udp_remote_port");
    mavlink_interface_->SetMavlinkUdpRemotePort(mavlink_udp_remote_port);
  }

  if (_sdf->HasElement("mavlink_udp_local_port")) {
    int mavlink_udp_local_port = _sdf->Get<int>("mavlink_udp_local_port");
    mavlink_interface_->SetMavlinkUdpLocalPort(mavlink_udp_local_port);
  }

  if (_sdf->HasElement("mavlink_tcp_port")) {
    int mavlink_tcp_port = _sdf->Get<int>("mavlink_tcp_port");
    mavlink_interface_->SetMavlinkTcpPort(mavlink_tcp_port);
  }

  mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

  // set the Mavlink protocol version to use on the link
  if (protocol_version_ == 2.0) {
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    gzmsg << "Using MAVLink protocol v2.0" << std::endl;
  }
  else if (protocol_version_ == 1.0) {
    chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    gzmsg << "Using MAVLink protocol v1.0" << std::endl;
  }
  else {
    gzerr << "Unkown protocol version! Using v" << protocol_version_ << "by default " << std::endl;
  }

  std::default_random_engine rnd_gen_;

  if (hostptr_ || mavlink_hostname_str_.empty()) {
    gzmsg << "--> load mavlink_interface_" << std::endl;
    mavlink_interface_->Load();
    mavlink_loaded_ = true;
  }
}

void GazeboMavlinkInterface::PreUpdate(const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm) {

  // Always run at 250 Hz. At 500 Hz, the skip factor should be 2, at 1000 Hz 4.
  if (!(previous_imu_seq_++ % update_skip_factor_ == 0)) {
    return;
  }

  if (!mavlink_loaded_) {
    // mavlink not loaded, exit
    return;
  }

  double dt;

  mavlink_interface_->ReadMAVLinkMessages();

  // Always send Gyro and Accel data at full rate (= sim update rate)
  SendSensorMessages(_info);

  handle_actuator_controls(_info);

  handle_control(dt);

  if (received_first_actuator_) {
    PublishRotorVelocities(_ecm, input_reference_);
  }
}

void GazeboMavlinkInterface::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) {
}

void GazeboMavlinkInterface::PoseCallback(const gz::msgs::Pose_V &_msg){
  for (int p = 0; p < _msg.pose_size(); p++) {
    if (_msg.pose(p).name() == model_name_) {
      gz::msgs::Vector3d pose_position = _msg.pose(p).position();
      gz::msgs::Quaternion pose_orientation = _msg.pose(p).orientation();

      // orientation transform
      gz::math::Quaterniond q_gr = gz::math::Quaterniond(
                    pose_orientation.w(),
                    pose_orientation.x(),
                    pose_orientation.y(),
                    pose_orientation.z());

      gz::math::Quaterniond q_nb;
      RotateQuaternion(q_nb, q_gr);

      // send pose info
      mavlink_hil_state_quaternion_t hil_state_quat;

      hil_state_quat.attitude_quaternion[0] = q_nb.W();
      hil_state_quat.attitude_quaternion[1] = q_nb.X();
      hil_state_quat.attitude_quaternion[2] = q_nb.Y();
      hil_state_quat.attitude_quaternion[3] = q_nb.Z();

      hil_state_quat.lat = pose_position.x() * 1e3;
      hil_state_quat.lon = pose_position.y() * 1e3;
      hil_state_quat.alt = pose_position.z() * 1e3;

      mavlink_message_t msg;
      mavlink_msg_hil_state_quaternion_encode_chan(254, 25, MAVLINK_COMM_0, &msg, &hil_state_quat);
      // Override default global mavlink channel status with instance specific status
      mavlink_interface_->FinalizeOutgoingMessage(&msg, 254, 25,
        MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN,
        MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN,
        MAVLINK_MSG_ID_HIL_STATE_QUATERNION_CRC);
      mavlink_interface_->PushSendMessage(&msg);
    }
  }



}

void GazeboMavlinkInterface::ImuCallback(const gz::msgs::IMU &_msg) {
  const std::lock_guard<std::mutex> lock(last_imu_message_mutex_);
  last_imu_message_ = _msg;
}

void GazeboMavlinkInterface::BarometerCallback(const gz::msgs::FluidPressure &_msg) {
  SensorData::Barometer baro_data;

  const float absolute_pressure = AddSimpleNoise((float) _msg.pressure(), 0, 1.5);
  const float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
  const float pressure_msl = 101325.0f; // pressure at MSL
  const float temperature_msl = 288.0f; // temperature at MSL (Kelvin)

  // Calculate local temperature:
  // absolute_pressure = pressure_msl / pressure_ratio
  // =>
  const float pressure_ratio = pressure_msl / absolute_pressure;
  // pressure_ratio = powf(temperature_msl / temperature_local, 5.256f)
  // =>
  // temperature_local = temperature_msl / powf(pressure_ratio, 1/5.256f)
  const float temperature_local = temperature_msl / powf(pressure_ratio, 0.19025875);

  // Calculate altitude from pressure:
  // temperature_local = temperature_msl - lapse_rate * alt_msl;
  // =>
  const float alt_msl = (temperature_msl - temperature_local) / lapse_rate;

  //gzmsg << "[BarometerCallback] temperature_local: " << temperature_local << " abs_press: " << absolute_pressure << std::endl;

  baro_data.temperature = temperature_local - 273.15f;
  baro_data.abs_pressure = absolute_pressure / 100.0f;
  baro_data.pressure_alt = alt_msl;
  mavlink_interface_->UpdateBarometer(baro_data);
}

void GazeboMavlinkInterface::MagnetometerCallback(const gz::msgs::Magnetometer &_msg) {
  SensorData::Magnetometer mag_data;
  mag_data.mag_b = Eigen::Vector3d(
    AddSimpleNoise(_msg.field_tesla().x(), 0, 0.0001),
    AddSimpleNoise(_msg.field_tesla().y(), 0, 0.0001),
    AddSimpleNoise(_msg.field_tesla().z(), 0, 0.0001)
  );
  mavlink_interface_->UpdateMag(mag_data);
}

//void GazeboMavlinkInterface::GpsCallback(const sensor_msgs::msgs::SITLGps &_msg) {
void GazeboMavlinkInterface::GpsCallback(const gz::msgs::NavSat &_msg) {
    // fill HIL GPS Mavlink msg
  //std::cerr << "GpsCallback" << std::endl;
  mavlink_hil_gps_t hil_gps_msg;
  const auto header = _msg.header();
  hil_gps_msg.time_usec = static_cast<uint64_t>((header.stamp().sec() * 1000000) + (header.stamp().nsec() / 1000));
  hil_gps_msg.fix_type = 3;
  hil_gps_msg.lat = static_cast<int32_t>(_msg.latitude_deg() * 1e7);
  hil_gps_msg.lon = static_cast<int32_t>(_msg.longitude_deg() * 1e7);
  hil_gps_msg.alt = static_cast<int32_t>(_msg.altitude() * 1000.0);
  hil_gps_msg.eph = 100;
  hil_gps_msg.epv = 100;
  Eigen::Vector3d v(_msg.velocity_north(), _msg.velocity_east(), -_msg.velocity_up());
  hil_gps_msg.vel = static_cast<uint16_t>(v.norm() * 100.0);
  hil_gps_msg.vn = static_cast<int16_t>(_msg.velocity_north() * 100.0);
  hil_gps_msg.ve = static_cast<int16_t>(_msg.velocity_east() * 100.0);
  hil_gps_msg.vd = static_cast<int16_t>(-_msg.velocity_up() * 100.0);
  // MAVLINK_HIL_GPS_T CoG is [0, 360]. math::Angle::Normalize() is [-pi, pi].
  gz::math::Angle cog(atan2(_msg.velocity_east(), _msg.velocity_north()));
  cog.Normalize();
  hil_gps_msg.cog = static_cast<uint16_t>(gazebo::GetDegrees360(cog) * 100.0);
  hil_gps_msg.satellites_visible = 10;
  hil_gps_msg.id = 0; // Workaround for mavlink zero trimming feature

  //gzmsg << "[GpsCallback] alt: " << _msg.altitude() << std::endl;

  // send HIL_GPS Mavlink msg
  mavlink_message_t msg;
  mavlink_msg_hil_gps_encode_chan(254, 25, MAVLINK_COMM_0, &msg, &hil_gps_msg);
  // Override default global mavlink channel status with instance specific status
  mavlink_interface_->FinalizeOutgoingMessage(&msg, 254, 25,
    MAVLINK_MSG_ID_HIL_GPS_MIN_LEN,
    MAVLINK_MSG_ID_HIL_GPS_LEN,
    MAVLINK_MSG_ID_HIL_GPS_CRC);
  mavlink_interface_->PushSendMessage(&msg);
}

void GazeboMavlinkInterface::SendSensorMessages(const gz::sim::UpdateInfo &_info) {
  const std::lock_guard<std::mutex> lock(last_imu_message_mutex_);
  const gz::msgs::IMU last_imu_message = last_imu_message_;
  last_imu_message_mutex_.unlock();


  // send always accel and gyro data (not dependent of the bitmask)
  // required so to keep the timestamps on sync and the lockstep can
  // work properly
  // gz::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
  //   AddSimpleNoise(last_imu_message.linear_acceleration().x(), 0, 0.006),
  //   AddSimpleNoise(last_imu_message.linear_acceleration().y(), 0, 0.006),
  //   AddSimpleNoise(last_imu_message.linear_acceleration().z(), 0, 0.030)));

  // gz::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
  //   AddSimpleNoise(last_imu_message.angular_velocity().x(), 0, 0.001),
  //   AddSimpleNoise(last_imu_message.angular_velocity().y(), 0, 0.001),
  //   AddSimpleNoise(last_imu_message.angular_velocity().z(), 0, 0.001)));

  gz::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
    last_imu_message.linear_acceleration().x(),
    last_imu_message.linear_acceleration().y(),
    last_imu_message.linear_acceleration().z()));

  gz::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
    last_imu_message.angular_velocity().x(),
    last_imu_message.angular_velocity().y(),
    last_imu_message.angular_velocity().z()));

  uint64_t time_usec = std::chrono::duration_cast<std::chrono::duration<uint64_t>>(_info.simTime * 1e6).count();
  SensorData::Imu imu_data;
  imu_data.accel_b = Eigen::Vector3d(accel_b.X(), accel_b.Y(), accel_b.Z());
  imu_data.gyro_b = Eigen::Vector3d(gyro_b.X(), gyro_b.Y(), gyro_b.Z());
  mavlink_interface_->UpdateIMU(imu_data);
  mavlink_interface_->SendSensorMessages(time_usec);
}

void GazeboMavlinkInterface::handle_actuator_controls(const gz::sim::UpdateInfo &_info) {
  bool armed = mavlink_interface_->GetArmedState();

  last_actuator_time_ = _info.simTime;

  for (unsigned i = 0; i < n_out_max; i++) {
    input_index_[i] = i;
  }
  // Read Input References
  input_reference_.resize(n_out_max);

  Eigen::VectorXd actuator_controls = mavlink_interface_->GetActuatorControls();
  if (actuator_controls.size() < n_out_max) return; //TODO: Handle this properly

  for (int i = 0; i < input_reference_.size(); i++) {
    if (armed) {
      input_reference_[i] = actuator_controls[input_index_[i]] * input_scaling_(i);
    } else {
      input_reference_[i] = 0;
      // std::cout << input_reference_ << ", ";
    }
  }
  received_first_actuator_ = mavlink_interface_->GetReceivedFirstActuator();
}

void GazeboMavlinkInterface::handle_control(double _dt)
{
  // set joint positions
  // for (int i = 0; i < input_reference_.size(); i++) {
  // }
}

bool GazeboMavlinkInterface::IsRunning()
{
  return true; //TODO;
}

void GazeboMavlinkInterface::onSigInt() {
  mavlink_interface_->onSigInt();
}

// The following snippet was copied from https://github.com/gzrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/multicopter_control/MulticopterVelocityControl.cc
void GazeboMavlinkInterface::PublishRotorVelocities(
    gz::sim::EntityComponentManager &_ecm,
    const Eigen::VectorXd &_vels)
{
  if (_vels.size() != rotor_velocity_message_.velocity_size())
  {
    rotor_velocity_message_.mutable_velocity()->Resize(_vels.size(), 0);
  }
  for (int i = 0; i < _vels.size(); ++i)
  {
    rotor_velocity_message_.set_velocity(i, _vels(i));
  }
  // Publish the message by setting the Actuators component on the model entity.
  // This assumes that the MulticopterMotorModel system is attached to this
  // model
  auto actuatorMsgComp =
      _ecm.Component<gz::sim::components::Actuators>(model_.Entity());

  if (actuatorMsgComp)
  {
    auto compFunc = [](const gz::msgs::Actuators &_a, const gz::msgs::Actuators &_b)
    {
      return std::equal(_a.velocity().begin(), _a.velocity().end(),
                        _b.velocity().begin());
    };
    auto state = actuatorMsgComp->SetData(this->rotor_velocity_message_, compFunc)
                     ? gz::sim::ComponentState::PeriodicChange
                     : gz::sim::ComponentState::NoChange;
    _ecm.SetChanged(model_.Entity(), gz::sim::components::Actuators::typeId, state);
  }
  else
  {
    _ecm.CreateComponent(model_.Entity(),
                         gz::sim::components::Actuators(this->rotor_velocity_message_));
  }
}

bool GazeboMavlinkInterface::resolveHostName()
{
  if (!mavlink_hostname_str_.empty()) {
    gzmsg << "Try to resolve hostname: '"  << mavlink_hostname_str_ << "'" << std::endl;
    hostptr_ = gethostbyname(mavlink_hostname_str_.c_str());
    if (hostptr_ && hostptr_->h_length && hostptr_->h_addrtype == AF_INET) {
      struct in_addr **addr_l = (struct in_addr **)hostptr_->h_addr_list;
      char *addr_str = inet_ntoa(*addr_l[0]);
      std::string ip_addr = std::string(addr_str);
      mavlink_interface_->SetMavlinkAddr(ip_addr);
      gzmsg << "Host name '" << mavlink_hostname_str_ << "' resolved to IP: " << ip_addr << std::endl;
      return true;
    }
    return false;
  } else {
    // Assume resolved in case hostname is not given at all
    return true;
  }

}

void GazeboMavlinkInterface::ResolveWorker()
{
  gzmsg << "[ResolveWorker] Start Resolving hostname" << std::endl;
  while (!resolveHostName()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  gzmsg << "[ResolveWorker] --> load mavlink_interface_" << std::endl;
  mavlink_interface_->Load();
  mavlink_loaded_ = true;
}

float GazeboMavlinkInterface::AddSimpleNoise(float value, float mean, float stddev) {
  std::normal_distribution<float> dist(mean, stddev);
  return value + dist(rnd_gen_);
}

void GazeboMavlinkInterface::RotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED,
    const gz::math::Quaterniond q_FLU_to_ENU)
{
	// FLU (ROS) to FRD (PX4) static rotation
	static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

	/**
	 * @brief Quaternion for rotation between ENU and NED frames
	 *
	 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
	 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
	 * This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
	 */
	static const auto q_ENU_to_NED = gz::math::Quaterniond(0, 0.70711, 0.70711, 0);

	// final rotation composition
	q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.Inverse();
}