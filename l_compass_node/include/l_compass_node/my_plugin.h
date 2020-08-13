#ifndef local_compass__my_plugin_H
#define local_compass__my_plugin_H
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_my_plugin.h>
#include <QWidget>
#include <QtWidgets>
#include <QMainWindow>
#include "qcgaugewidget.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <QLabel>
namespace local_compass {

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void newDataCallback(const std_msgs::Float64& msg);
  virtual void newDataCallback1(const std_msgs::Int32& msg);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private slots:
    void on_horizontalSlider_valueChanged(int value);
    void heading_position();
private:
  Ui::lcompass ui_;
  QWidget* widget_;

  //QLabel *lcom = new QLabel(this);
  QcGaugeWidget * mCompassGauge;
  QcNeedleItem *mCompassNeedle;
  ros::Subscriber lneedleSub_;
  ros::Subscriber rssiSub_;
};
} // namespace
#endif // my_namespace__my_plugin_H

