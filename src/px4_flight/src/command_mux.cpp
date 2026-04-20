#include "px4_flight/command_mux.hpp"

namespace px4_flight
{

CommandMux::CommandMux(const rclcpp::NodeOptions & options)
: Node("command_mux", options)
{
  this->declare_parameter("arbitration_rate", 50.0);
  this->declare_parameter("command_timeout", 0.5);
  this->declare_parameter("transition_duration", 0.5);
  this->declare_parameter("enable_smoothing", true);

  arbitration_rate_ = this->get_parameter("arbitration_rate").as_double();
  command_timeout_ = this->get_parameter("command_timeout").as_double();
  transition_duration_ = this->get_parameter("transition_duration").as_double();
  enable_smoothing_ = this->get_parameter("enable_smoothing").as_bool();

  source_priorities_[CommandSource::EMERGENCY] = 100;
  source_priorities_[CommandSource::RC] = 80;
  source_priorities_[CommandSource::GROUND_STATION] = 60;
  source_priorities_[CommandSource::MISSION_MANAGER] = 40;
  source_priorities_[CommandSource::UNKNOWN] = 0;

  rc_cmd_sub_ = this->create_subscription<px4_flight::msg::ExternalCommand>(
    "/rc/cmd", 10,
    std::bind(&CommandMux::rc_command_callback, this, std::placeholders::_1));

  gs_cmd_sub_ = this->create_subscription<px4_flight::msg::ExternalCommand>(
    "/ground_station/cmd", 10,
    std::bind(&CommandMux::gs_command_callback, this, std::placeholders::_1));

  mission_cmd_sub_ = this->create_subscription<px4_flight::msg::ExternalCommand>(
    "/mission_manager/cmd", 10,
    std::bind(&CommandMux::mission_command_callback, this, std::placeholders::_1));

  emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/emergency/stop", 10,
    std::bind(&CommandMux::emergency_callback, this, std::placeholders::_1));

  active_source_pub_ = this->create_publisher<std_msgs::msg::String>("active_command_source", 10);
  output_cmd_pub_ = this->create_publisher<px4_flight::msg::ExternalCommand>("mux_output_cmd", 10);

  auto arbitration_period = std::chrono::milliseconds(static_cast<int>(1000.0 / arbitration_rate_));
  arbitration_timer_ = this->create_wall_timer(
    arbitration_period, std::bind(&CommandMux::arbitration_timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Command Mux initialized");
}

bool CommandMux::get_current_command(px4_flight::msg::ExternalCommand & cmd)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  
  auto it = command_buffers_.find(active_source_);
  if (it == command_buffers_.end()) {
    return false;
  }

  cmd = it->second;
  return true;
}

void CommandMux::rc_command_callback(const px4_flight::msg::ExternalCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  command_buffers_[CommandSource::RC] = *msg;
  last_update_times_[CommandSource::RC] = this->get_clock()->now();
}

void CommandMux::gs_command_callback(const px4_flight::msg::ExternalCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  command_buffers_[CommandSource::GROUND_STATION] = *msg;
  last_update_times_[CommandSource::GROUND_STATION] = this->get_clock()->now();
}

void CommandMux::mission_command_callback(const px4_flight::msg::ExternalCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  command_buffers_[CommandSource::MISSION_MANAGER] = *msg;
  last_update_times_[CommandSource::MISSION_MANAGER] = this->get_clock()->now();
}

void CommandMux::emergency_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    active_source_ = CommandSource::EMERGENCY;
    
    px4_flight::msg::ExternalCommand emergency_cmd;
    emergency_cmd.control_type = px4_flight::msg::ExternalCommand::EMERGENCY_LAND;
    command_buffers_[CommandSource::EMERGENCY] = emergency_cmd;
    last_update_times_[CommandSource::EMERGENCY] = this->get_clock()->now();
  }
}

void CommandMux::arbitration_timer_callback()
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  
  CommandSource new_source = select_priority_source();
  
  if (new_source != active_source_) {
    RCLCPP_INFO(this->get_logger(), "Command source switched from %d to %d", 
      static_cast<int>(active_source_), static_cast<int>(new_source));
    active_source_ = new_source;
  }

  std_msgs::msg::String source_msg;
  switch(active_source_) {
    case CommandSource::RC: source_msg.data = "RC"; break;
    case CommandSource::GROUND_STATION: source_msg.data = "GROUND_STATION"; break;
    case CommandSource::MISSION_MANAGER: source_msg.data = "MISSION_MANAGER"; break;
    case CommandSource::EMERGENCY: source_msg.data = "EMERGENCY"; break;
    default: source_msg.data = "UNKNOWN";
  }
  active_source_pub_->publish(source_msg);

  auto it = command_buffers_.find(active_source_);
  if (it != command_buffers_.end()) {
    output_cmd_pub_->publish(it->second);
  }
}

CommandSource CommandMux::select_priority_source()
{
  auto now = this->get_clock()->now();
  CommandSource selected = CommandSource::UNKNOWN;
  int highest_priority = -1;

  for (const auto & [source, time] : last_update_times_) {
    if ((now - time).seconds() > command_timeout_) {
      continue;
    }

    int priority = source_priorities_[source];
    if (priority > highest_priority) {
      highest_priority = priority;
      selected = source;
    }
  }

  return selected;
}

bool CommandMux::validate_command(const px4_flight::msg::ExternalCommand::SharedPtr & cmd)
{
  if (std::isnan(cmd->pose.position.x) || std::isnan(cmd->pose.position.y) || 
      std::isnan(cmd->pose.position.z)) {
    return false;
  }
  
  if (std::abs(cmd->pose.position.x) > 1000 || std::abs(cmd->pose.position.y) > 1000) {
    return false;
  }

  return true;
}

px4_flight::msg::ExternalCommand CommandMux::smooth_transition(
  const px4_flight::msg::ExternalCommand & from,
  const px4_flight::msg::ExternalCommand & to)
{
  (void)from;
  return to;
}

}  // namespace px4_flight
