
#include "log_panel.hpp"

#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/panel.hpp>

namespace rviz_plugins {
  LogPanels::LogPanels(QWidget *parent) {

    // connect(line_edit_, SIGNAL(returnPressed()), this, SLOT(updateLabel()));

    // auto log_output_ = new QTextEdit;


    QHBoxLayout *hlayout = new QHBoxLayout;

    QLabel *label_tp = new QLabel;
    label_tp->setText("True Positives: ");
    label_tp->adjustSize();

    QTextEdit *edit_tp = new QTextEdit;
    edit_tp->setReadOnly(true);
    edit_tp->setPlainText("0");
    // 通过设置子部件的高度，来间接调整 layout 的高度
    edit_tp->setFixedHeight(30);
    edit_tp->setFixedWidth(100);

    QLabel *label_fp = new QLabel;
    label_fp->setText("False Positives: ");
    label_fp->adjustSize();

    QTextEdit *edit_fp = new QTextEdit;
    edit_fp->setReadOnly(true);
    edit_fp->setPlainText("0");
    edit_fp->setFixedHeight(30);
    edit_fp->setFixedWidth(100);

    QLabel *label_fn = new QLabel;
    label_fn->setText("False Negatives: ");
    label_fn->adjustSize();

    QTextEdit *edit_fn = new QTextEdit;
    edit_fn->setReadOnly(true);
    edit_fn->setPlainText("0");
    edit_fn->setFixedHeight(30);
    edit_fn->setFixedWidth(100);


    QLabel *label_frame_id = new QLabel;
    label_frame_id->setText("Current Frame ID: ");
    label_fn->adjustSize();
    QTextEdit *edit_frame_id = new QTextEdit;
    edit_frame_id->setReadOnly(true);
    edit_frame_id->setPlainText("0 / 3250");
    edit_frame_id->setAlignment(Qt::AlignCenter);
    edit_frame_id->setFixedHeight(30);
    edit_frame_id->setFixedWidth(100);

    hlayout->addWidget(label_tp);
    hlayout->addWidget(edit_tp);
    hlayout->addWidget(label_fp);
    hlayout->addWidget(edit_fp);
    hlayout->addWidget(label_fn);
    hlayout->addWidget(edit_fn);
    hlayout->addStretch();
    hlayout->addWidget(label_frame_id);
    hlayout->addWidget(edit_frame_id);

    setLayout(hlayout);
  }

  void LogPanels::save(rviz_common::Config config) const {
    rviz_common::Panel::save(config);
    // config.mapSetValue("Topic", output_topic_);
  }

  // Load all configuration data for this panel from the given Config object.
  void LogPanels::load(const rviz_common::Config &config) {
    rviz_common::Panel::load(config);
    // QString topic;
    // if (config.mapGetString("Topic", &topic)) {
    //   output_topic_editor_->setText(topic);
    //   updateTopic();
    // }
  }

} // namespace rviz_plugins
// void CustomPanel::updateLabel()
// {
//   // Update your visual display here based on the user's input from line_edit_
// }


PLUGINLIB_EXPORT_CLASS(rviz_plugins::LogPanels, rviz_common::Panel)
