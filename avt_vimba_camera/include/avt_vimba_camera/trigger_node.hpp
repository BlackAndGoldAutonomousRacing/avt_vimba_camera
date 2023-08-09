#ifndef AVT_VIMBA_CAMERA_TRIGGER_H
#define AVT_VIMBA_CAMERA_TRIGGER_H

#include <VmbCPP/VmbCPP.h>

#include <arpa/inet.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

using VimbaSystem = VmbCPP::VmbSystem;
using VmbCPP::InterfacePtr;
using VmbCPP::FeaturePtr;

namespace trigger
{
class TriggerNode : public rclcpp::Node
{
public:
  TriggerNode();
  ~TriggerNode();

  void Init();

private:
  void LoadParams();
  void InitializeAddress();
  bool PrepareActionCommand();
  bool SetIntFeatureValue(const std::string& name, int64_t value);

  void TimerCb();
  void TriggerCb(const std_msgs::msg::Bool::SharedPtr msg);
  void SendActionCommand();

  VimbaSystem& vimba_system_;
  InterfacePtr interface_ptr_;
  rclcpp::Clock clock_;
  rclcpp::TimerBase::SharedPtr trigger_timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;

  // Params
  struct in_addr destination_ip_;
  std::string trigger_src_;
  double timer_period_;
  int64_t action_device_key_;
  int64_t action_group_key_;
  int64_t action_group_mask_;
};

}  // namespace trigger

#endif  // AVT_VIMBA_CAMERA_TRIGGER_H
