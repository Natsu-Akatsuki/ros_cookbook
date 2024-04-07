
#ifndef O3D_DASHBOARD_O3D_DASHBOARD_H
#define O3D_DASHBOARD_O3D_DASHBOARD_H

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include <rviz_common/panel.hpp>

namespace rviz_plugins {
  class LogPanels : public rviz_common::Panel {

    // Q_OBJECT
  public:
    // todo：放在声明还是定义？
    // 等于 0 的意思？
    explicit LogPanels(QWidget *parent = nullptr);

    void load(const rviz_common::Config &config) override;

    void save(rviz_common::Config config) const override;
    ~LogPanels() override = default;

  private:
    std::shared_ptr<rclcpp::Node> velocity_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  };
} // namespace rviz_plugins

#endif //O3D_DASHBOARD_O3D_DASHBOARD_H
