#include "px4_gcs_rqt/gcs_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QColorDialog>
#include <QVariantMap>
#include <QTreeWidgetItem>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <ros/service.h>
#include <ros/param.h>
#include <ros/topic.h>

#include "ctrller_msgs/CommandInt6.h"

// #include <mavros_msgs/CommandInt.h>

#include "ui_gcs_plugin_6dof.h"

// #include "px4_gcs_rqt/service_caller.h"

namespace px4_gcs_rqt {

    GcsPlugin::GcsPlugin()
        : rqt_gui_cpp::Plugin()
        , ui_(new Ui::GcsPluginWidget)
        , widget_(0)
    {
        // Constructor is called first before initPlugin function, needless to say.

        // give QObjects reasonable names
        setObjectName("GcsPlugin");

        nh_ = new ros::NodeHandle();
        pwm_pub_ = nh_->advertise<ctrller_msgs::Pwm>("/gcs_rqt/pwm",1);
    }

    GcsPlugin::~GcsPlugin()
    {
        delete nh_;
    }

    void GcsPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        ROS_INFO("Init px4_gcs_rqt plugin");
        // access standalone command line arguments
        QStringList argv = context.argv();
        // create QWidget
        widget_ = new QWidget();
        // extend the widget with all attributes and children from UI file
        ui_->setupUi(widget_);
        // add widget to the user interface
        context.addWidget(widget_);

        connect(ui_->btnArming, SIGNAL(clicked()), this, SLOT(on_btnArming_clicked()));
        connect(ui_->btnCtrl_start, SIGNAL(clicked()), this, SLOT(on_btnCtrlStart_clicked()));
        connect(ui_->btnTransitToFF, SIGNAL(clicked()), this, SLOT(on_btnTransitToFF_clicked()));
        
        connect(ui_->btnUsePlanner, SIGNAL(clicked()), this, SLOT(on_btnUsePlanner_clicked()));
        connect(ui_->btnTransitToPerch, SIGNAL(clicked()), this, SLOT(on_btnTransitToPerch_clicked()));

        connect(ui_->btnDown, SIGNAL(clicked()), this, SLOT(on_btnDown_clicked()));
        connect(ui_->btnUp, SIGNAL(clicked()), this, SLOT(on_btnUp_clicked()));
        connect(ui_->btnFront, SIGNAL(clicked()), this, SLOT(on_btnFront_clicked()));
        connect(ui_->btnBack, SIGNAL(clicked()), this, SLOT(on_btnBack_clicked()));
        connect(ui_->btnLeft, SIGNAL(clicked()), this, SLOT(on_btnLeft_clicked()));
        connect(ui_->btnRight, SIGNAL(clicked()), this, SLOT(on_btnRight_clicked()));
        connect(ui_->btnRoll_plus, SIGNAL(clicked()), this, SLOT(on_btnRollPlus_clicked()));
        connect(ui_->btnRoll_minus, SIGNAL(clicked()), this, SLOT(on_btnRollMinus_clicked()));
        connect(ui_->btnPitch_plus, SIGNAL(clicked()), this, SLOT(on_btnPitchPlus_clicked()));
        connect(ui_->btnPitch_minus, SIGNAL(clicked()), this, SLOT(on_btnPitchMinus_clicked()));
        connect(ui_->btnYaw_minus, SIGNAL(clicked()), this, SLOT(on_btnYawMinus_clicked()));
        connect(ui_->btnYaw_plus, SIGNAL(clicked()), this, SLOT(on_btnYawPlus_clicked()));

        connect(ui_->verticalSlider, SIGNAL(sliderMoved(int)), this, SLOT(receive_sliderValue(int)));
    }

    void GcsPlugin::shutdownPlugin()
    {
        // TODO unregister all publishers here
    }

    void GcsPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
    {
        // TODO save intrinsic configuration, usually using:
        // instance_settings.setValue(k, v)
    }

    void GcsPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
    {
        // TODO restore intrinsic configuration, usually using:
        // v = instance_settings.value(k)
    }

    void GcsPlugin::on_btnArming_clicked()
    {
        arming_flag_ = !arming_flag_;
        if(arming_flag_ == true)
            ROS_INFO("ARMED");
        else
            ROS_INFO("DISARMED");
        std_srvs::SetBool srv;
        srv.request.data = arming_flag_;
        ros::service::call<std_srvs::SetBool>("/agent1/ctrl_alloc/arming", srv);
        ros::service::call<std_srvs::SetBool>("/agent2/ctrl_alloc/arming", srv);
    }

    void GcsPlugin::on_btnCtrlStart_clicked()
    {
        ctrl_start_flag_ = !ctrl_start_flag_;
        if(ctrl_start_flag_ == true)
            ROS_INFO("ctrl start");
        else
            ROS_INFO("ctrl stop");

        std_srvs::SetBool srv;
        srv.request.data = ctrl_start_flag_;
        ros::service::call<std_srvs::SetBool>("/agent1/coop_ctrller/start", srv);
        ros::service::call<std_srvs::SetBool>("/agent2/coop_ctrller/start", srv);
    }

     void GcsPlugin::on_btnTransitToFF_clicked()
    {
        ROS_INFO_STREAM("Transit_to_FF clicked.");
        std_srvs::SetBool srv;
        srv.request.data = true;
        ros::service::call<std_srvs::SetBool>("/ctrller/transitTo_ff", srv);
    }

    void GcsPlugin::on_btnUsePlanner_clicked()
    {
        useTrajectory_flag_ = !useTrajectory_flag_;
        if( useTrajectory_flag_ == true)
            ROS_INFO_STREAM("usePlanner clicked. useTrajectory_flag: TRUE");
        else
            ROS_INFO_STREAM("usePlanner clicked. useTrajectory_flag: FALSE");

        std_srvs::SetBool srv;
        srv.request.data = useTrajectory_flag_;
        ros::service::call<std_srvs::SetBool>("/ref_planner/use_ext_sp", srv);
    }

    void GcsPlugin::on_btnTransitToPerch_clicked()
    {
        ROS_INFO_STREAM("Transit_to_perch clicked.");

        std_srvs::SetBool srv;
        srv.request.data = true;
        ros::service::call<std_srvs::SetBool>("/ctrller/transitTo_perch", srv); // TODO - temporary service name
    }

    void GcsPlugin::on_btnUp_clicked()
    {
        ROS_INFO("[Teleop] Up clicked");
        move_setpoint(clicked_btn::UPDOWN,true);
    }

    void GcsPlugin::on_btnDown_clicked()
    {
        ROS_INFO("[Teleop] Down clicked");
        move_setpoint(clicked_btn::UPDOWN,false);
    }

    void GcsPlugin::on_btnFront_clicked()
    {
        ROS_INFO("[Teleop] Front clicked");
        move_setpoint(clicked_btn::FRONTBACK,true);
    }

    void GcsPlugin::on_btnBack_clicked()
    {
        ROS_INFO("[Teleop] Back clicked");
        move_setpoint(clicked_btn::FRONTBACK,false);
    }

    void GcsPlugin::on_btnLeft_clicked()
    {
        ROS_INFO("[Teleop] Left clicked");
        move_setpoint(clicked_btn::LEFTRIGHT,true);
    }

    void GcsPlugin::on_btnRight_clicked()
    {
        ROS_INFO("[Teleop] Right clicked");
        move_setpoint(clicked_btn::LEFTRIGHT,false);
    }

    void GcsPlugin::on_btnRollPlus_clicked()
    {
        ROS_INFO("[Teleop] Roll Plus clicked");
        move_setpoint(clicked_btn::ROLL_PM,true);
    }

    void GcsPlugin::on_btnRollMinus_clicked()
    {
        ROS_INFO("[Teleop] Roll Minus clicked");
        move_setpoint(clicked_btn::ROLL_PM,false);
    }

    void GcsPlugin::on_btnPitchPlus_clicked()
    {
        ROS_INFO("[Teleop] Pitch Plus clicked");
        move_setpoint(clicked_btn::PITCH_PM,true);
    }

    void GcsPlugin::on_btnPitchMinus_clicked()
    {
        ROS_INFO("[Teleop] Pitch Minus clicked");
        move_setpoint(clicked_btn::PITCH_PM,false);
    }

    void GcsPlugin::on_btnYawPlus_clicked()
    {
        ROS_INFO("[Teleop] Yaw Plus clicked");
        move_setpoint(clicked_btn::YAW_PM,true);
    }

    void GcsPlugin::on_btnYawMinus_clicked()
    {
        ROS_INFO("[Teleop] Yaw Minus clicked");
        move_setpoint(clicked_btn::YAW_PM,false);
    }

    void GcsPlugin::receive_sliderValue(int pwm)
    {
        pwm_msg_.header.stamp = ros::Time::now();
        pwm_msg_.pwm.resize(1);
        pwm_msg_.pwm[0] = (double) pwm;

        pwm_pub_.publish(pwm_msg_);
    }

    void GcsPlugin::move_setpoint(int idx, bool increase)
	{
		ctrller_msgs::CommandInt6 cmd;
		cmd.request.param1 = 0.0; // x
		cmd.request.param2 = 0.0; // y
		cmd.request.param3 = 0.0; // z
		cmd.request.param4 = 0.0; // roll
        cmd.request.param5 = 0.0; // pitch
        cmd.request.param6 = 0.0; // yaw

		switch( idx )
		{
			case clicked_btn::FRONTBACK:
				increase ? cmd.request.param1 = 0.1 : cmd.request.param1 = -0.1;
				break;
			case clicked_btn::LEFTRIGHT:
				increase ? cmd.request.param2 = 0.1 : cmd.request.param2 = -0.1;
				break;
			case clicked_btn::UPDOWN:
				increase ? cmd.request.param3 = 0.1 : cmd.request.param3 = -0.1;
				break;
			case clicked_btn::ROLL_PM:
				increase ? cmd.request.param4 = 0.1 : cmd.request.param4 = -0.1;
				break;
            case clicked_btn::PITCH_PM:
				increase ? cmd.request.param5 = 0.1 : cmd.request.param5 = -0.1;
				break;
            case clicked_btn::YAW_PM:
				increase ? cmd.request.param6 = 0.1 : cmd.request.param6 = -0.1;
				break;
			default:
				break;
		}
		ros::service::call<ctrller_msgs::CommandInt6>("/ref_planner/move_setpoint", cmd);
	}
} // namespace

// Deprecated
// See: http://wiki.ros.org/pluginlib#pluginlib.2Fpluginlib_groovy.Simplified_Export_Macro
//PLUGINLIB_DECLARE_CLASS(px4_gcs_rqt, GcsPlugin, px4_gcs_rqt::GcsPlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(px4_gcs_rqt::GcsPlugin, rqt_gui_cpp::Plugin)