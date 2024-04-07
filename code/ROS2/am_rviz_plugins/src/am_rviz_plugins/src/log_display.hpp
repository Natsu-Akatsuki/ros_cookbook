#ifndef SAMPLE_RVIZ_PLUGINS_LOG_DISPLAY_HPP
#define SAMPLE_RVIZ_PLUGINS_LOG_DISPLAY_HPP


#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <Overlay/OgreBorderPanelOverlayElement.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayContainer.h>
#include <Overlay/OgreOverlayElement.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>
#include <Overlay/OgreTextAreaOverlayElement.h>
#include <am_rviz_plugins/msg/bounding_box_array.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_rendering/render_system.hpp>

namespace rviz_plugins {
  using MSG_T = am_rviz_plugins::msg::BoundingBoxArray;
  using BoundingBox = am_rviz_plugins::msg::BoundingBox;

  class LogDisplay : public rviz_common::RosTopicDisplay<MSG_T> {
    Q_OBJECT
  public:
    LogDisplay();
    ~LogDisplay() override;

    void onInitialize() override;
    void reset() override;
    void onDisable() override;
    void onEnable() override;
    void initTextElement(const std::string &caption, int row_id = 0,
                         int col_offset = 0, int row_offset = 0, int text_size = 16);

    // (optional) 用基类的即可，无需重写
    // void load(const rviz_common::Config &config) override;
    // void save(rviz_common::Config config) const override;

  private Q_SLOTS:
    void updateOverlay();

  private:
    // 属性字段成员
    rviz_common::properties::IntProperty *height_property_;
    rviz_common::properties::IntProperty *width_property_;
    rviz_common::properties::IntProperty *size_property_;
    rviz_common::properties::IntProperty *x_coordinate_property_;
    rviz_common::properties::IntProperty *y_coordinate_property_;

    void processMessage(MSG_T::ConstSharedPtr msg) override;
    void createMaterial(const std::string &mat_name);
    void destroyMaterial(const std::string &mat_name);
    std::deque<rcl_interfaces::msg::Log> log_msgs_;

    Ogre::OverlayManager *overlay_manager_;
    Ogre::Overlay *overlay_;
    Ogre::PanelOverlayElement *panel_;
    std::vector<Ogre::TextAreaOverlayElement *> text_elements_;
  };

} // namespace rviz_plugins

#endif // SAMPLE_RVIZ_PLUGINS_LOG_DISPLAY_HPP