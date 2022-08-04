#ifndef px4_gcs_rqt__gcs_plugin_H
#define px4_gcs_rqt__gcs_plugin_H


#include <rqt_gui_cpp/plugin.h>

#include <QWidget>
#include <QSharedPointer>
#include <QVariantMap>

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
        enum clicked_btn{ UPDOWN = 0, FRONTBACK = 1, LEFTRIGHT = 2, CWCCW = 3, PITCH_PM = 4 }; // front, back
        
        bool useTrajectory_flag_ = false;
        bool moveManipulator_flag_ = false;

        /// Pointer to reference the main ui widgets of the turtle_plugin.ui
        Ui::GcsPluginWidget* ui_;
        /// The main widget that will be used to setup the ui_
        QWidget* widget_;

        /// Pointer to the ServiceCaller class dialog
        QSharedPointer<ServiceCaller> service_caller_dialog_;       

        /// Storing the currently selected turtles present in the treeTrutles QTreeWdiget.
        // QVector<QString> selected_drones_;

        /// Vector to keep track of all turtles (keep turtles on the heap using vector of shared pointers)
        // QMap<QString, QSharedPointer<Drone> > drones_;


        /**
         * @brief Teleport selected turtle(s)
         * 
         * @details teleport is used for both slots, on_btnTeleportAbs_clicked and 
         * on_btnTeleportRel_clicked. It will create a new ServiceCaller widget,
         * which allows to enter the coordinates where the turtle should be teleported.
         * Depending on the pressed button ('Teleport Abs' or 'Teleport Rel'), either
         * the '/teleport_absoulte' or '/teleport_relative' service will be executed with the
         * provided coordinates.
         * 
         * @param teleport_type Can be one of /teleport_absolute or /teleport_relative
         */
        // QVariantMap teleport(std::string teleport_type);

        // void updateDroneTree();


        inline std::string str(QString qstr) { return qstr.toStdString(); };

    private slots:
        void on_btnArming_clicked();
        void on_btnDisarming_clicked();
        void on_btnTrajectory_clicked();
        void on_btnManipulator_clicked();
        
        void on_btnUp_clicked();
        void on_btnDown_clicked();
        void on_btnFront_clicked();
        void on_btnBack_clicked();
        void on_btnLeft_clicked();
        void on_btnRight_clicked();
        void on_btnCw_clicked();
        void on_btnCcw_clicked();
        void on_btnPitchPlus_clicked();
        void on_btnPitchMinus_clicked();

        void move_setpoint(int idx, bool increase);

        /**
         * @brief Keeps track of the slected turtles
         * 
         * @details When selecting different turtles in the QTreeWidget,
         * the member selected_drones_ storing the selected turtles is updated.
         * 
         */
        // void on_selection_changed();
    };

} // namespace
#endif // px4_gcs_rqt__gcs_plugin_H