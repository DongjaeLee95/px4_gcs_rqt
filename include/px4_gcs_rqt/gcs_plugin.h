#ifndef px4_gcs_rqt__gcs_plugin_H
#define px4_gcs_rqt__gcs_plugin_H


#include <rqt_gui_cpp/plugin.h>

#include <QWidget>
#include <QSharedPointer>
#include <QVariantMap>

#include <ros/ros.h>
#include "ctrller_msgs/Pwm.h"
// #include <px4_gcs_rqt/drone.h>


class QListWidgetItem;

namespace Ui {
    class GcsPluginWidget;
}

namespace px4_gcs_rqt {

    class ServiceCaller;

    class GcsPlugin
        : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
    public:
        /**
         * @brief Construct a new Turtle Plugin object
         * 
         * @details All qt signal to slot connections are done here.
         */
        GcsPlugin();
        ~GcsPlugin();

        /**
         * @brief Setup the plugin and the signal and slot connections.
         * 
         * @param context 
         */
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

        // Comment in to signal that the plugin has a way to configure it
        //bool hasConfiguration() const;
        //void triggerConfiguration();
    private:
        ros::NodeHandle* nh_;
        ros::Publisher pwm_pub_;
        ctrller_msgs::Pwm pwm_msg_;

        enum clicked_btn{ UPDOWN = 0, FRONTBACK = 1, LEFTRIGHT = 2, 
                          ROLL_PM = 3, PITCH_PM = 4, YAW_PM = 5 }; // front, back
        
        bool useTrajectory_flag_ = false;
        bool perching_flag_ = false;

        /// Pointer to reference the main ui widgets of the turtle_plugin.ui
        Ui::GcsPluginWidget* ui_;
        /// The main widget that will be used to setup the ui_
        QWidget* widget_;

        /// Pointer to the ServiceCaller class dialog
        QSharedPointer<ServiceCaller> service_caller_dialog_;       

        inline std::string str(QString qstr) { return qstr.toStdString(); };

    private slots:
        void on_btnArming_clicked();
        void on_btnDisarming_clicked();
        void on_btnTrajectory_clicked();
        void on_btnPerching_clicked();
        
        void on_btnUp_clicked();
        void on_btnDown_clicked();
        void on_btnFront_clicked();
        void on_btnBack_clicked();
        void on_btnLeft_clicked();
        void on_btnRight_clicked();
        void on_btnRollPlus_clicked();
        void on_btnRollMinus_clicked();
        void on_btnPitchPlus_clicked();
        void on_btnPitchMinus_clicked();
        void on_btnYawMinus_clicked();
        void on_btnYawPlus_clicked();

        void receive_sliderValue(int);

        void move_setpoint(int idx, bool increase);
    };

} // namespace
#endif // px4_gcs_rqt__gcs_plugin_H