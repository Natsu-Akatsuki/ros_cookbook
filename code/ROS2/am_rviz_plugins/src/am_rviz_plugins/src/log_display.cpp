#include "log_display.hpp"

#include <OgreMovableObject.h>
#include <QColor>
#include <QImage>
#include <fmt/core.h>

namespace rviz_plugins {

  /**
   * 构造函数
   * */
  LogDisplay::LogDisplay() {

    // 添加属性
    height_property_ = new rviz_common::properties::IntProperty("panel_height", 180, "panel height", this, SLOT(updateOverlay()));
    width_property_ = new rviz_common::properties::IntProperty("panel_width", 280, "panel width", this, SLOT(updateOverlay()));
    size_property_ = new rviz_common::properties::IntProperty("font_size", 16, "font size", this, SLOT(updateOverlay()));
    x_coordinate_property_ = new rviz_common::properties::IntProperty("x_coordinate", 20, "x coordinate", this, SLOT(updateOverlay()));
    y_coordinate_property_ = new rviz_common::properties::IntProperty("y_coordinate", 20, "y coordinate", this, SLOT(updateOverlay()));

    // 创建 overlay
    overlay_manager_ = Ogre::OverlayManager::getSingletonPtr();
    overlay_ = overlay_manager_->create("LogOverlay");

    // 创建 overlay 元素即 panel
    panel_ = dynamic_cast<Ogre::PanelOverlayElement *>(overlay_manager_->createOverlayElement("Panel", "Panel"));
    panel_->setMetricsMode(Ogre::GuiMetricsMode::GMM_PIXELS);

    // 给 overlay 添加元素
    overlay_->add2D(panel_);

    // 创建材质
    createMaterial("BackGround");
    panel_->setMaterialName("BackGround");

    // 给 overlay 添加材质
    overlay_->add2D((Ogre::OverlayContainer *) panel_);

    // 创建 TextOverlay 元素
    std::string caption = ">>> Statistical Information >>>";
    initTextElement(caption, 0, 10, 10);
    caption = "True Positives:";
    initTextElement(caption, 1, 5, 20);
    caption = "False Positives:";
    initTextElement(caption, 2, 5, 20);
    caption = "False Negatives:";
    initTextElement(caption, 3, 5, 20);
    caption = "Prediction:";
    initTextElement(caption, 4, 5, 20);
    caption = "Groundtruth:";
    initTextElement(caption, 5, 5, 20);
    caption = "Current frame:";
    initTextElement(caption, 6, 40, 40);
    caption = "TF(Red) FP(Blue) FN(Green) TP(Yellow)";
    initTextElement(caption, 7, 5, 50);

    for (auto text_element: text_elements_) {
      panel_->addChild(text_element);
    }

    // 显示 overlay
    overlay_->show();
    updateOverlay();
  }

  void LogDisplay::initTextElement(const std::string &caption, int row_id,
                                   int col_offset, int row_offset, int text_size) {
    std::string text_name = std::string("Text") + std::to_string(row_id);
    auto text_element = (Ogre::TextAreaOverlayElement *) overlay_manager_->createOverlayElement("TextArea", text_name);
    text_element->setMetricsMode(Ogre::GuiMetricsMode::GMM_PIXELS);
    text_element->setColour(Ogre::ColourValue::White);
    text_element->setFontName("Liberation Sans");
    text_element->setCaption(caption);
    text_element->setCharHeight(float(text_size));
    text_element->setPosition(float(col_offset), float(row_offset + text_size * row_id));
    text_elements_.push_back(text_element);
  }

  /**
   * 析构函数
   */
  LogDisplay::~LogDisplay() {

    // 移除 overlay
    overlay_manager_->destroy("LogOverlay");

    // 移除 overlay 的元素，即 panel
    overlay_manager_->destroyOverlayElement(panel_);

    for (auto element: text_elements_) {
      overlay_manager_->destroyOverlayElement(element);
    }

    // 移除材质元素（也包含移除容器中的）
    destroyMaterial("BackGround");
  }

  /**
   * （mandatory）重写函数
   */
  void LogDisplay::onInitialize() {
    RTDClass::onInitialize();
    // Prepare a scene_manager to render overlays. Needed for Ogre >= 1.9 to use fonts;
    rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  }

  /**
   * (mandatory) 重写函数，使能插件后触发
   */
  void LogDisplay::reset() {
    RTDClass::reset();
  }

  /**
   * (optional) 重写函数，实测用基类的实现即可
   */
  // void LogDisplay::save(rviz_common::Config config) const {
  //   rviz_common::Display::save(config);
  //   config.mapSetValue("Height", height_property_->getInt());
  //   config.mapSetValue("Width", width_property_->getInt());
  //   config.mapSetValue("Size", size_property_->getInt());
  // }

  /**
   * (optional) 重写函数，使能插件后触发
   */
  // void LogDisplay::load(const rviz_common::Config &config) {
  //   rviz_common::Display::load(config);
  //   int temp_int;
  //   if (config.mapGetInt("Height", &temp_int))
  //     height_property_->setInt(temp_int);
  //   if (config.mapGetInt("Width", &temp_int))
  //     width_property_->setInt(temp_int);
  //   if (config.mapGetInt("Size", &temp_int))
  //     size_property_->setInt(temp_int);
  // }

  // Display 的回调函数
  void LogDisplay::processMessage(const MSG_T::ConstSharedPtr bounding_box_3d_array) {
    int static total_tp_count = 0, total_fp_count = 0, total_fn_count = 0, total_pred_count = 0, total_gt_count = 0;
    int tp_count = 0, fp_count = 0, fn_count = 0, pred_count = 0, gt_count = 0;

    if (!isEnabled()) {
      overlay_->hide();
      return;
    }
    overlay_->show();

    // 统计框的属性
    for (const auto& bounding_box_3d: bounding_box_3d_array->boxes) {
      if (bounding_box_3d.attr == BoundingBox::TP) {
        total_tp_count += 1;
        tp_count += 1;
        pred_count += 1;
        total_pred_count += 1;
      }
      if (bounding_box_3d.attr == BoundingBox::FP) {
        total_fp_count += 1;
        fp_count += 1;

        pred_count += 1;
        total_pred_count += 1;
      }
      if (bounding_box_3d.attr == BoundingBox::GT) {
        total_gt_count += 1;
        gt_count += 1;
      }
      if (bounding_box_3d.attr == BoundingBox::FN) {
        total_fn_count += 1;
        fn_count += 1;

        total_gt_count += 1;
        gt_count += 1;
      }
    }

    std::string caption = fmt::format("True Positives:{}    {:04}/{:04}", tp_count, total_tp_count, total_gt_count);
    text_elements_[1]->setCaption(caption);
    caption = fmt::format("False Positives: {}    {:04}/{:04}", fp_count, total_fp_count, total_gt_count);
    text_elements_[2]->setCaption(caption);
    caption = fmt::format("False Negatives: {}    {:04}/{:04}", fn_count, total_fn_count, total_gt_count);
    text_elements_[3]->setCaption(caption);
    caption = fmt::format("Predictions: {}    {:04}", pred_count, total_pred_count);
    text_elements_[4]->setCaption(caption);
    caption = fmt::format("Groundtruth: {}    {:04}", gt_count, total_gt_count);
    text_elements_[5]->setCaption(caption);
    caption = "Current frame: 0000 / 0000";
    text_elements_[6]->setCaption(caption);

    updateOverlay();
  }

  void LogDisplay::updateOverlay() {
    // 调整 overlay 的位置
    int panel_width = width_property_->getInt();
    int panel_height = height_property_->getInt();
    int x_coordinate = x_coordinate_property_->getInt();
    int y_coordinate = y_coordinate_property_->getInt();

    panel_->setPosition(float(x_coordinate), float(y_coordinate));
    panel_->setDimensions(float(panel_width), float(panel_height));
  }

  void LogDisplay::onDisable() {
    RTDClass::onDisable();
    overlay_->hide();
  }

  /**
   * (optional) 重写函数，使能插件后触发
   */
  void LogDisplay::onEnable() {
    RTDClass::onEnable();
    overlay_->show();
  }

  void LogDisplay::createMaterial(const std::string &mat_name) {
    int width = 180;
    int height = 280;
    std::string texture_name = mat_name + "Texture";
    QColor bg_color(0, 0, 0, 70.0);

    Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().createManual(
      texture_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, width, height, 0,
      Ogre::PF_A8R8G8B8, Ogre::TU_DEFAULT);

    Ogre::MaterialPtr mat = (Ogre::MaterialPtr) Ogre::MaterialManager::getSingleton().create(
      mat_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

    mat->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name);
    mat->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture->getBuffer();
    pixel_buffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
    const Ogre::PixelBox &pixelBox = pixel_buffer->getCurrentLock();
    auto *pDest = static_cast<Ogre::uint8 *>(pixelBox.data);
    memset(pDest, 0, width * height);
    QImage Hud = QImage(pDest, width, height, QImage::Format_ARGB32);
    for (int i = 0; i < width; i++) {
      for (int j = 0; j < height; j++) {
        Hud.setPixel(i, j, bg_color.rgba());
      }
    }
    pixel_buffer->unlock();
  }

  void LogDisplay::destroyMaterial(const std::string &mat_name) {
    std::string texture_name = mat_name + "Texture";
    Ogre::TextureManager::getSingleton().destroyResourcePool(texture_name);
    Ogre::MaterialManager::getSingleton().destroyResourcePool(mat_name);
  }

} // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::LogDisplay, rviz_common::Display)