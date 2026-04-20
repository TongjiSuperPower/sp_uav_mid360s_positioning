#ifndef PX4_FLIGHT__COMMAND_MUX_HPP_
#define PX4_FLIGHT__COMMAND_MUX_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include "px4_flight/msg/external_command.hpp"

#include <queue>
#include <mutex>
#include <map>

namespace px4_flight
{

enum class CommandSource {
  RC,                 // 遥控器
  GROUND_STATION,     // 地面站
  MISSION_MANAGER,    // 任务管理器
  EMERGENCY,          // 紧急指令
  UNKNOWN
};

/**
 * @brief 指令多路复用器
 * 仲裁多源控制指令，确保单一控制源
 */
class CommandMux : public rclcpp::Node
{
public:
  explicit CommandMux(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CommandMux() = default;

  // 获取输出指令（供OffboardController查询）
  bool get_current_command(px4_flight::msg::ExternalCommand & cmd);
  CommandSource get_active_source() const { return active_source_; }

private:
  // 输入回调
  void rc_command_callback(const px4_flight::msg::ExternalCommand::SharedPtr msg);
  void gs_command_callback(const px4_flight::msg::ExternalCommand::SharedPtr msg);
  void mission_command_callback(const px4_flight::msg::ExternalCommand::SharedPtr msg);
  void emergency_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // 仲裁逻辑
  void arbitration_timer_callback();
  CommandSource select_priority_source();
  bool validate_command(const px4_flight::msg::ExternalCommand::SharedPtr & cmd);

  // 指令融合/过渡
  px4_flight::msg::ExternalCommand smooth_transition(
    const px4_flight::msg::ExternalCommand & from,
    const px4_flight::msg::ExternalCommand & to);

  // 参数
  std::map<CommandSource, int> source_priorities_;
  double arbitration_rate_;
  double command_timeout_;
  double transition_duration_;
  bool enable_smoothing_;

  // 订阅者
  rclcpp::Subscription<px4_flight::msg::ExternalCommand>::SharedPtr rc_cmd_sub_;
  rclcpp::Subscription<px4_flight::msg::ExternalCommand>::SharedPtr gs_cmd_sub_;
  rclcpp::Subscription<px4_flight::msg::ExternalCommand>::SharedPtr mission_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;

  // 发布者（调试）
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_source_pub_;
  rclcpp::Publisher<px4_flight::msg::ExternalCommand>::SharedPtr output_cmd_pub_;

  // 状态
  std::map<CommandSource, px4_flight::msg::ExternalCommand> command_buffers_;
  std::map<CommandSource, rclcpp::Time> last_update_times_;
  CommandSource active_source_{CommandSource::UNKNOWN};
  std::mutex buffer_mutex_;

  // 定时器
  rclcpp::TimerBase::SharedPtr arbitration_timer_;
};

}  // namespace px4_flight

#endif  // PX4_FLIGHT__COMMAND_MUX_HPP_