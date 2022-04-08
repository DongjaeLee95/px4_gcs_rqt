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

#include <mavros_msgs/CommandInt.h>
#include <mavros_msgs/State.h>

#include "ui_gcs_plugin.h"

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
        // connect(ui_->btnDisarming, SIGNAL(clicked()), this, SLOT(on_btnDisarming_clicked()));
        connect(ui_->btnOffboard, SIGNAL(clicked()), this, SLOT(on_btnOffboard_clicked()));
        // connect(ui_->btnManual, SIGNAL(clicked()), this, SLOT(on_btnManual_clicked()));
        connect(ui_->btnTrajectory, SIGNAL(clicked()), this, SLOT(on_btnTrajectory_clicked()));
        connect(ui_->btnManipulator, SIGNAL(clicked()), this, SLOT(on_btnManipulator_clicked()));

        connect(ui_->btnDown, SIGNAL(clicked()), this, SLOT(on_btnDown_clicked()));
        connect(ui_->btnUp, SIGNAL(clicked()), this, SLOT(on_btnUp_clicked()));
        connect(ui_->btnFront, SIGNAL(clicked()), this, SLOT(on_btnFront_clicked()));
        connect(ui_->btnBack, SIGNAL(clicked()), this, SLOT(on_btnBack_clicked()));
        connect(ui_->btnLeft, SIGNAL(clicked()), this, SLOT(on_btnLeft_clicked()));
        connect(ui_->btnRight, SIGNAL(clicked()), this, SLOT(on_btnRight_clicked()));
        connect(ui_->btnCw, SIGNAL(clicked()), this, SLOT(on_btnCw_clicked()));
        connect(ui_->btnCcw, SIGNAL(clicked()), this, SLOT(on_btnCcw_clicked()));

        // connect(ui_->treeDrones, SIGNAL(itemSelectionChanged()), 
        //         this, SLOT(on_selection_changed()));

        // updateDroneTree();
    }

    // void GcsPlugin::updateDroneTree()
    // {
    //     // https://stackoverflow.com/questions/26785675/ros-get-current-available-topic-in-code-not-command
    //     // Use XML-RPC ROS Master API to get the topic names
    //     // Then filter for topics containing pose (which belongs to a turtle)
    //     ros::master::V_TopicInfo master_topics;
    //     ros::master::getTopics(master_topics);

    //     ros::NodeHandle nh = getNodeHandle();
    //     for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
    //     {
    //         const ros::master::TopicInfo& info = *it;
    //         ROS_INFO_STREAM("topic_" << it - master_topics.begin() << ": " << info.name);
    //         QString topic_name = QString::fromStdString(info.name);
    //         if (topic_name.contains(QString("/state")))
    //         {
    //             QStringList topic_name_parts = topic_name.split(QRegExp("\\/"), QString::SkipEmptyParts);
    //             std::string drone_name = topic_name_parts[0].toStdString();
    //             ROS_INFO("topic_name_part 0: %s", drone_name.c_str());
                
    //             // Wait for a single pose message to arrive on the turtlesim::Pose topic
                
    //             mavros_msgs::StateConstPtr state = ros::topic::waitForMessage<mavros_msgs::State>(topic_name.toStdString());
    //             bool isArmed = state->armed;
    //             bool isOffboard = (state->mode == "OFFBOARD");

    //             ROS_INFO("State received: Arming: %f, Offboard: %f", isArmed, isOffboard);

    //             // Create new turtle in turtle map
    //             // Note: assume that the pen is toggled on
    //             QSharedPointer<Drone> drone = QSharedPointer<Drone>(new drone(drone_name, isArmed, isOffboard));
    //             drones_[QString::fromStdString(drone_name)] = drone;
    //         }
    //     }

    //     // Insert the turtles into the QTreeWidget
    //     for (auto drone : drones_)
    //     {
    //         ui_->treeDrones->insertTopLevelItem(0, drone->toTreeItem(ui_->treeDrones));
    //     }
    // }

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

    /*bool hasConfiguration() const
    {
        return true;
    }

    void triggerConfiguration()
    {
        // Usually used to open a dialog to offer the user a set of configuration
    }*/

    void GcsPlugin::on_btnArming_clicked()
    {
        ROS_INFO("Arming clicked");
        std_srvs::SetBool srv;
        srv.request.data = true;
        ros::service::call<std_srvs::SetBool>("/commander/arm", srv);
    }

    void GcsPlugin::on_btnOffboard_clicked()
    {
        ROS_INFO("Offboard clicked");
        std_srvs::SetBool srv;
        srv.request.data = true;
        ros::service::call<std_srvs::SetBool>("/commander/flight_mode", srv);
    }

    void GcsPlugin::on_btnTrajectory_clicked()
    {
        useTrajectory_flag_ = !useTrajectory_flag_;
        if( useTrajectory_flag_ == true)
            ROS_INFO_STREAM("Trajectory clicked. useTrajectory_flag: TRUE");
        else
            ROS_INFO_STREAM("Trajectory clicked. useTrajectory_flag: FALSE");

        std_srvs::SetBool srv;
        srv.request.data = true;
        ros::service::call<std_srvs::SetBool>("/commander/useTrajectory", srv); // TODO - temporary service name
    }

    void GcsPlugin::on_btnManipulator_clicked()
    {
        moveManipulator_flag_ = !moveManipulator_flag_;
        if( moveManipulator_flag_ == true)
            ROS_INFO_STREAM("Manipulator clicked. moveManipulator_flag: TRUE");
        else
            ROS_INFO_STREAM("Manipulator clicked. moveManipulator_flag: FALSE");

        std_srvs::SetBool srv;
        srv.request.data = true;
        ros::service::call<std_srvs::SetBool>("/commander/useManipulator", srv); // TODO - temporary service name
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

    void GcsPlugin::on_btnCw_clicked()
    {
        ROS_INFO("[Teleop] Cw clicked");
        move_setpoint(clicked_btn::CWCCW,false);
    }

    void GcsPlugin::on_btnCcw_clicked()
    {
        ROS_INFO("[Teleop] Ccw clicked");
        move_setpoint(clicked_btn::CWCCW,true);
    }

    void GcsPlugin::move_setpoint(int idx, bool increase)
	{
		mavros_msgs::CommandInt cmd;
		cmd.request.param1 = 0.0;
		cmd.request.param2 = 0.0;
		cmd.request.param3 = 0.0;
		cmd.request.param4 = 0.0;

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
			case clicked_btn::CWCCW:
				increase ? cmd.request.param4 = 0.1 : cmd.request.param4 = -0.1;
				break;
			default:
				break;
		}
		ros::service::call<mavros_msgs::CommandInt>("/commander/move_setpoint", cmd);
	}

    // QVariantMap GcsPlugin::teleport(std::string strServiceName)
    // {
    //     service_caller_dialog_ = QSharedPointer<ServiceCaller>(new ServiceCaller(widget_, strServiceName));

    //     QString qstrTurtleName;
    //     QVariantMap request;
    //     bool ok = service_caller_dialog_->exec() == QDialog::Accepted;
    //     if (ok)
    //     {
    //         ROS_DEBUG("accepted");
    //         request = service_caller_dialog_->getRequest();
    //         qstrTurtleName = request["name"].toString();
    //     }
    //     else
    //     {
    //         ROS_DEBUG("ServiceCaller Dialog closed");
    //         return QVariantMap();
    //     }

    //     return request;
    // }

    // void GcsPlugin::on_selection_changed()
    // {
    //     //auto current = m_pUi->treeTurtles->currentItem(); // TODO use member list if multiple turtles are selected
    //     // Get list of selected turtles
    //     auto selected_items = ui_->treeDrones->selectedItems();
    //     selected_drones_.clear();
    //     if (selected_items.empty())
    //     {
    //         return;
    //     }
    //     //m_strSelectedTurtle = current->text(0).toStdString();
    //     QString drone_name;
    //     std::stringstream ss;
    //     for (auto item : selected_items)
    //     {
    //         drone_name = item->text(0);
    //         selected_drones_.push_back(drone_name);
    //         ss << str(drone_name) << " ";
            
    //     }
    //     ROS_INFO("Selected %s", ss.str().c_str());
        
    // }


} // namespace

// Deprecated
// See: http://wiki.ros.org/pluginlib#pluginlib.2Fpluginlib_groovy.Simplified_Export_Macro
//PLUGINLIB_DECLARE_CLASS(px4_gcs_rqt, GcsPlugin, px4_gcs_rqt::GcsPlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(px4_gcs_rqt::GcsPlugin, rqt_gui_cpp::Plugin)