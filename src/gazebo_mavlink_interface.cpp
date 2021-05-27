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

#include "gazebo_mavlink_interface.h"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    mavlink_interface::GazeboMavlinkInterface,
    ignition::gazebo::System,
    mavlink_interface::GazeboMavlinkInterface::ISystemConfigure,
    mavlink_interface::GazeboMavlinkInterface::ISystemPreUpdate,
    mavlink_interface::GazeboMavlinkInterface::ISystemPostUpdate)
using namespace mavlink_interface;

GazeboMavlinkInterface::GazeboMavlinkInterface() : 
    input_offset_ {},
    input_scaling_ {},
    zero_position_disarmed_ {},
    zero_position_armed_ {},
    input_index_ {}
    {
      mavlink_interface_ = std::make_shared<MavlinkInterface>();
    }

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  mavlink_interface_->close();
}

void GazeboMavlinkInterface::Configure(const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &_em) {

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->Get<std::string>("robotNamespace");
  } else {
    ignerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  }

  if (_sdf->HasElement("protocol_version")) {
    protocol_version_ = _sdf->Get<float>("protocol_version");
  }

  gazebo::getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_,
      motor_velocity_reference_pub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "gpsSubTopic", gps_sub_topic_, gps_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "visionSubTopic", vision_sub_topic_, vision_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "opticalFlowSubTopic",
      opticalFlow_sub_topic_, opticalFlow_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "irlockSubTopic", irlock_sub_topic_, irlock_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "magSubTopic", mag_sub_topic_, mag_sub_topic_);
  gazebo::getSdfParam<std::string>(_sdf, "baroSubTopic", baro_sub_topic_, baro_sub_topic_);
  groundtruth_sub_topic_ = "/groundtruth";

  // set input_reference_ from inputs.control
  input_reference_.resize(n_out_max);

  ///TODO: Parse input reference
  input_scaling_.resize(n_out_max);
  input_scaling_(0) = 1000;
  input_scaling_(1) = 1000;
  input_scaling_(2) = 1000;
  input_scaling_(3) = 1000;

  if(_sdf->HasElement("hil_mode"))
  {
    hil_mode_ = _sdf->Get<bool>("hil_mode");
    mavlink_interface_->SetHILMode(hil_mode_);
  }

  if(_sdf->HasElement("hil_state_level"))
  {
    hil_state_level_ = _sdf->Get<bool>("hil_state_level");
    mavlink_interface_->SetHILStateLevel(hil_state_level_);
  }

  bool serial_enabled=false;
  if(_sdf->HasElement("serialEnabled"))
  {
    serial_enabled = _sdf->Get<bool>("serialEnabled");
    mavlink_interface_->SetSerialEnabled(serial_enabled);
  }

  bool use_tcp = false;
  if (!serial_enabled && _sdf->HasElement("use_tcp"))
  {
    use_tcp = _sdf->Get<bool>("use_tcp");
    mavlink_interface_->SetUseTcp(use_tcp);
  }
  ignmsg << "Connecting to PX4 SITL using " << (serial_enabled ? "serial" : (use_tcp ? "TCP" : "UDP")) << "\n";

  if (!hil_mode_ && _sdf->HasElement("enable_lockstep"))
  {
    enable_lockstep_ = _sdf->Get<bool>("enable_lockstep");
    mavlink_interface_->SetEnableLockstep(enable_lockstep_);
  }
  ignmsg << "Lockstep is " << (enable_lockstep_ ? "enabled" : "disabled") << "\n";


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
        ignerr << "Invalid speed factor '" << speed_factor_str << "', aborting\n";
        abort();
      }
    }
    ignmsg << "Speed factor set to: " << speed_factor_ << "\n";

  }

  // // Listen to Ctrl+C / SIGINT.
  sigIntConnection_ = _em.Connect<ignition::gazebo::events::Stop>(std::bind(&GazeboMavlinkInterface::onSigInt, this));

  // Subscribe to messages of other plugins.
  node.Subscribe("/imu", &GazeboMavlinkInterface::ImuCallback, this);
  node.Subscribe("/world/quadcopter/model/X3/link/base_link/sensor/barometer", &GazeboMavlinkInterface::BarometerCallback, this);
  node.Subscribe("/world/quadcopter/model/X3/link/base_link/sensor/magnetometer", &GazeboMavlinkInterface::MagnetometerCallback, this);
  node.Subscribe("/world/quadcopter/model/X3/link/base_link/sensor/gps", &GazeboMavlinkInterface::GpsCallback, this);

  // This doesn't seem to be used anywhere but we leave it here
  // for potential compatibility
  if (_sdf->HasElement("imu_rate")) {
    imu_update_interval_ = 1 / _sdf->Get<int>("imu_rate");
  }

  if (_sdf->HasElement("mavlink_addr")) {
    std::string mavlink_addr_str = _sdf->Get<std::string>("mavlink_addr");
    if (mavlink_addr_str != "INADDR_ANY") {
      mavlink_interface_->SetMavlinkAddr(mavlink_addr_str);
    }
  }

  int mavlink_udp_port;
  if (_sdf->HasElement("mavlink_udp_port")) {
    mavlink_udp_port = _sdf->Get<int>("mavlink_udp_port");
    //TODO: Set Malvink UDP port
  }
  // model_param(worldName, model_->GetName(), "mavlink_udp_port", mavlink_udp_port);
  mavlink_interface_->SetMavlinkUdpPort(mavlink_udp_port);

  int mavlink_tcp_port;
  if (_sdf->HasElement("mavlink_tcp_port")) {
    mavlink_tcp_port = _sdf->Get<int>("mavlink_tcp_port");
  }
  // model_param(worldName, model_->GetName(), "mavlink_tcp_port", mavlink_tcp_port);
  mavlink_interface_->SetMavlinkTcpPort(mavlink_tcp_port);

  if (_sdf->HasElement("qgc_addr")) {
    std::string qgc_addr = _sdf->Get<std::string>("qgc_addr");
    if (qgc_addr != "INADDR_ANY") {
      mavlink_interface_->SetGcsAddr(qgc_addr);
    }
  }
  if (_sdf->HasElement("qgc_udp_port")) {
    int qgc_udp_port = _sdf->Get<int>("qgc_udp_port");
    mavlink_interface_->SetGcsUdpPort(qgc_udp_port);
  }

  if (_sdf->HasElement("sdk_addr")) {
    std::string sdk_addr = _sdf->Get<std::string>("sdk_addr");
    if (sdk_addr != "INADDR_ANY") {
      mavlink_interface_->SetSdkAddr(sdk_addr);
    }
  }
  if (_sdf->HasElement("sdk_udp_port")) {
    int sdk_udp_port = _sdf->Get<int>("sdk_udp_port");
    mavlink_interface_->SetSdkUdpPort(sdk_udp_port);
  }

  if (serial_enabled_) {
    // Set up serial interface
    if(_sdf->HasElement("serialDevice"))
    {
      std::string device = _sdf->Get<std::string>("serialDevice");
    }

    if (_sdf->HasElement("baudRate")) {
      int baudrate = _sdf->Get<int>("baudRate");
    }
  }

  mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

  // set the Mavlink protocol version to use on the link
  if (protocol_version_ == 2.0) {
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    ignmsg << "Using MAVLink protocol v2.0\n";
  }
  else if (protocol_version_ == 1.0) {
    chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    ignmsg << "Using MAVLink protocol v1.0\n";
  }
  else {
    ignerr << "Unkown protocol version! Using v" << protocol_version_ << "by default \n";
  }
  mavlink_interface_->Load();

  entity_ = _entity;
  model_ = ignition::gazebo::Model(_entity);
  model_name_ = model_.Name(_ecm);
}

void GazeboMavlinkInterface::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm) {
  const std::lock_guard<std::mutex> lock(last_imu_message_mutex_);

  // Always run at 250 Hz. At 500 Hz, the skip factor should be 2, at 1000 Hz 4.
  if (!(previous_imu_seq_ % update_skip_factor_ == 0)) {
    return;
  }
  
  double dt;

  bool close_conn_ = false;

  if (hil_mode_) {
    mavlink_interface_->pollFromGcsAndSdk();
  } else {
    mavlink_interface_->pollForMAVLinkMessages();
  }

  // Always send Gyro and Accel data at full rate (= sim update rate)
  SendSensorMessages(_info);

  // Send groudntruth at full rate
  SendGroundTruth();

  if (close_conn_) { // close connection if required
    mavlink_interface_->close();
  }

  handle_actuator_controls(_info);

  handle_control(dt);

  if (received_first_actuator_) {
    PublishRotorVelocities(_ecm, input_reference_);
  }
}

void GazeboMavlinkInterface::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm) {
}

void GazeboMavlinkInterface::ImuCallback(const ignition::msgs::IMU &_msg) {
  const std::lock_guard<std::mutex> lock(last_imu_message_mutex_);
  last_imu_message_ = _msg;

}

void GazeboMavlinkInterface::BarometerCallback(const sensor_msgs::msgs::Pressure &_msg) {
  const std::lock_guard<std::mutex> lock(last_imu_message_mutex_);
  SensorData::Barometer baro_data;
  baro_data.temperature = _msg.temperature();
  baro_data.abs_pressure = _msg.absolute_pressure();
  baro_data.pressure_alt = _msg.pressure_altitude();
  mavlink_interface_->UpdateBarometer(baro_data);
}

void GazeboMavlinkInterface::MagnetometerCallback(const sensor_msgs::msgs::MagneticField &_msg) {
  const std::lock_guard<std::mutex> lock(last_imu_message_mutex_);
  SensorData::Magnetometer mag_data;
  mag_data.mag_b = Eigen::Vector3d(_msg.magnetic_field().x(),
    _msg.magnetic_field().y(), _msg.magnetic_field().z());
  mavlink_interface_->UpdateMag(mag_data);
}

void GazeboMavlinkInterface::GpsCallback(const sensor_msgs::msgs::SITLGps &_msg) {
  const std::lock_guard<std::mutex> lock(last_imu_message_mutex_);
  SensorData::Gps gps_data;
  gps_data.time_utc_usec = _msg.time_utc_usec();
  gps_data.fix_type = 3;
  gps_data.latitude_deg = _msg.latitude_deg() * 1e7;
  gps_data.longitude_deg = _msg.longitude_deg() * 1e7;
  gps_data.altitude = _msg.altitude() * 1000.0;
  gps_data.eph = _msg.eph() * 100.0;
  gps_data.epv = _msg.epv() * 100.0;
  gps_data.velocity = _msg.velocity() * 100.0;
  gps_data.velocity_north = _msg.velocity_north() * 100.0;
  gps_data.velocity_east = _msg.velocity_east() * 100.0;
  gps_data.velocity_down = -_msg.velocity_up() * 100.0;
  // MAVLINK_HIL_GPS_T CoG is [0, 360]. math::Angle::Normalize() is [-pi, pi].
  ignition::math::Angle cog(atan2(_msg.velocity_east(), _msg.velocity_north()));
  cog.Normalize();
  gps_data.cog = static_cast<uint16_t>(gazebo::GetDegrees360(cog) * 100.0);
  gps_data.satellites_visible = 10;
  gps_data.id = 0;

  mavlink_interface_->SendGpsMessages(gps_data);

}

void GazeboMavlinkInterface::SendSensorMessages(const ignition::gazebo::UpdateInfo &_info) {
  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
    last_imu_message_.orientation().w(),
    last_imu_message_.orientation().x(),
    last_imu_message_.orientation().y(),
    last_imu_message_.orientation().z());

  bool should_send_imu = false;
  if (!enable_lockstep_) {
    std::chrono::steady_clock::duration current_time = _info.simTime;

    double dt = (current_time - last_imu_time_).count();

    if (imu_update_interval_!=0 && dt >= imu_update_interval_) {
      should_send_imu = true;
      last_imu_time_ = current_time;
    }
  }

  int time_usec = std::chrono::duration_cast<std::chrono::duration<int>>(_info.simTime * 1e6).count();

  // send always accel and gyro data (not dependent of the bitmask)
  // required so to keep the timestamps on sync and the lockstep can
  // work properly
  ignition::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(ignition::math::Vector3d(
    last_imu_message_.linear_acceleration().x(),
    last_imu_message_.linear_acceleration().y(),
    last_imu_message_.linear_acceleration().z()));

  ignition::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(ignition::math::Vector3d(
    last_imu_message_.angular_velocity().x(),
    last_imu_message_.angular_velocity().y(),
    last_imu_message_.angular_velocity().z()));
  
  SensorData::Imu imu_data;
  imu_data.accel_b = Eigen::Vector3d(accel_b.X(), accel_b.Y(), accel_b.Z());
  imu_data.gyro_b = Eigen::Vector3d(gyro_b.X(), gyro_b.Y(), gyro_b.Z());
  mavlink_interface_->UpdateIMU(imu_data);

  mavlink_interface_->SendSensorMessages(time_usec);
}

void GazeboMavlinkInterface::SendGroundTruth()
{
  // ground truth
  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
    last_imu_message_.orientation().w(),
    last_imu_message_.orientation().x(),
    last_imu_message_.orientation().y(),
    last_imu_message_.orientation().z());

  ignition::math::Quaterniond q_gb = q_gr*q_FLU_to_FRD.Inverse();
  ignition::math::Quaterniond q_nb = q_ENU_to_NED*q_gb;

  // ignition::math::Vector3d vel_b = q_FLU_to_FRD.RotateVector(model_->RelativeLinearVel());
  // ignition::math::Vector3d vel_n = q_ENU_to_NED.RotateVector(model_->WorldLinearVel());
  // ignition::math::Vector3d omega_nb_b = q_br.RotateVector(model_->RelativeAngularVel());
  ignition::math::Vector3d vel_b;
  ignition::math::Vector3d vel_n;
  ignition::math::Vector3d omega_nb_b;

  // ignition::math::Vector3d accel_true_b = q_FLU_to_FRD.RotateVector(model_->RelativeLinearAccel());
  ignition::math::Vector3d accel_true_b; //TODO: Get model pointer

  // send ground truth
  mavlink_hil_state_quaternion_t hil_state_quat;
// #if GAZEBO_MAJOR_VERSION >= 9
//   hil_state_quat.time_usec = world_->SimTime().Double() * 1e6;
// #else
//   hil_state_quat.time_usec = world_->GetSimTime().Double() * 1e6;
// #endif
  hil_state_quat.attitude_quaternion[0] = q_nb.W();
  hil_state_quat.attitude_quaternion[1] = q_nb.X();
  hil_state_quat.attitude_quaternion[2] = q_nb.Y();
  hil_state_quat.attitude_quaternion[3] = q_nb.Z();

  hil_state_quat.rollspeed = omega_nb_b.X();
  hil_state_quat.pitchspeed = omega_nb_b.Y();
  hil_state_quat.yawspeed = omega_nb_b.Z();

  hil_state_quat.lat = groundtruth_lat_rad * 180 / M_PI * 1e7;
  hil_state_quat.lon = groundtruth_lon_rad * 180 / M_PI * 1e7;
  hil_state_quat.alt = groundtruth_altitude * 1000;

  hil_state_quat.vx = vel_n.X() * 100;
  hil_state_quat.vy = vel_n.Y() * 100;
  hil_state_quat.vz = vel_n.Z() * 100;

  // assumed indicated airspeed due to flow aligned with pitot (body x)
  hil_state_quat.ind_airspeed = vel_b.X();

// #if GAZEBO_MAJOR_VERSION >= 9
//   hil_state_quat.true_airspeed = model_->WorldLinearVel().Length() * 100;  //no wind simulated
// #else
//   hil_state_quat.true_airspeed = model_->GetWorldLinearVel().GetLength() * 100;  //no wind simulated
// #endif

  hil_state_quat.xacc = accel_true_b.X() * 1000;
  hil_state_quat.yacc = accel_true_b.Y() * 1000;
  hil_state_quat.zacc = accel_true_b.Z() * 1000;

  if (!hil_mode_ || (hil_mode_ && hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_state_quat);
    mavlink_interface_->send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::handle_actuator_controls(const ignition::gazebo::UpdateInfo &_info) {
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
      input_reference_[i] = (actuator_controls[input_index_[i]] + input_offset_[i])
          * input_scaling_(i) + zero_position_armed_[i];
    } else {
      input_reference_[i] = zero_position_disarmed_[i];
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

// The following snippet was copied from https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/multicopter_control/MulticopterVelocityControl.cc
void GazeboMavlinkInterface::PublishRotorVelocities(
    ignition::gazebo::EntityComponentManager &_ecm,
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
      _ecm.Component<ignition::gazebo::components::Actuators>(model_.Entity());

  if (actuatorMsgComp)
  {
    auto compFunc = [](const ignition::msgs::Actuators &_a, const ignition::msgs::Actuators &_b)
    {
      return std::equal(_a.velocity().begin(), _a.velocity().end(),
                        _b.velocity().begin());
    };
    auto state = actuatorMsgComp->SetData(this->rotor_velocity_message_, compFunc)
                     ? ignition::gazebo::ComponentState::PeriodicChange
                     : ignition::gazebo::ComponentState::NoChange;
    _ecm.SetChanged(model_.Entity(), ignition::gazebo::components::Actuators::typeId, state);
  }
  else
  {
    _ecm.CreateComponent(model_.Entity(),
                         ignition::gazebo::components::Actuators(this->rotor_velocity_message_));
  }
}
