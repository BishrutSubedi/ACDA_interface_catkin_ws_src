#include "r_compass_node/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include<r_compass_node/qcgaugewidget.h>
#include <stdlib.h>
#include <sstream>
#include <QDial>

namespace remote_compass {

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("rcompass");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();

    widget_ = new QWidget();
    widget_->setWindowTitle("rcompass");
    ui_.setupUi(widget_);
    mCompassGauge = new QcGaugeWidget(ui_.cwidget);


    mCompassGauge->addBackground(99);
    QcBackgroundItem *bkg1 = mCompassGauge->addBackground(92);
    bkg1->clearrColors();
    bkg1->addColor(0.1,Qt::black);
    bkg1->addColor(1.0,Qt::white);

    QcBackgroundItem *bkg2 = mCompassGauge->addBackground(88);
    bkg2->clearrColors();
    bkg2->addColor(0.1,Qt::white);
    bkg2->addColor(1.0,Qt::black);

    QcLabelItem *w = mCompassGauge->addLabel(80);
    w->setText("W");
    w->setAngle(0);
    w->setColor(Qt::white);

    QcLabelItem *n = mCompassGauge->addLabel(80);
    n->setText("N");
    n->setAngle(90);
    n->setColor(Qt::white);

    QcLabelItem *e = mCompassGauge->addLabel(80);
    e->setText("E");
    e->setAngle(180);
    e->setColor(Qt::white);

    QcLabelItem *s = mCompassGauge->addLabel(80);
    s->setText("S");
    s->setAngle(270);
    s->setColor(Qt::white);

    QcDegreesItem *deg = mCompassGauge->addDegrees(70);
    deg->setStep(10);
    deg->setMaxDegree(360);
    deg->setMinDegree(0);
    deg->setMaxValue(360);
    deg->setMinValue(0);
    deg->setColor(Qt::white);

//ADD NEEDLE 
    mCompassNeedle = mCompassGauge->addNeedle(60);
    mCompassNeedle->setNeedle(QcNeedleItem::CompassNeedle);
    mCompassNeedle->setValueRange(0,360);
    mCompassNeedle->setMaxDegree(360);
    mCompassNeedle->setMinDegree(0);
    mCompassGauge->addBackground(7);
    mCompassGauge->addGlass(40);


  // add widget to the user interface
  context.addWidget(widget_);
  rneedleSub_ = getNodeHandle().subscribe ("/remote_com", 1, &remote_compass::MyPlugin::newDataCallback, this);
 // rssiSub_ = getNodeHandle().subscribe ("/remote_rssi", 1, &remote_compass::MyPlugin::newDataCallback1, this);

}

void MyPlugin::  on_horizontalSlider_valueChanged(int value){

   mCompassNeedle->setCurrentValue(value);

}

void MyPlugin::newDataCallback(const std_msgs::Float64& msg)
{
  
  mCompassNeedle-> setCurrentValue(msg.data);
	ui_.lineEdit_2->setText(QString::number(msg.data));
 
}

void MyPlugin::newDataCallback1(const std_msgs::Int32& msg)
{
  

  ui_.lineEdit->setText(QString::number(msg.data));
}


void MyPlugin::heading_position(){

  //ui_.doubleSpinBox->setValue(double(mCompassNeedle->mCurrentValue));
	

}






void MyPlugin::shutdownPlugin()
{
  // TODO unregister all publishers here
	rneedleSub_.shutdown();

}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
PLUGINLIB_DECLARE_CLASS(remote_compass, MyPlugin, remote_compass::MyPlugin, rqt_gui_cpp::Plugin)
