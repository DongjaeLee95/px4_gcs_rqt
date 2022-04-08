#ifndef px4_gcs_rqt__drone_H
#define px4_gcs_rqt__drone_H

#include <ros/ros.h>

#include <QTreeWidgetItem>
#include <std_msgs/Bool.h>

#include <mavros_msgs/State.h>

typedef mavros_msgs::State State;

class Drone
{
public:
    Drone(std::string name);
    Drone(std::string name, bool isArmed);
    Drone(std::string name, bool isArmed, bool isOffboard);

    QTreeWidgetItem* toTreeItem(QTreeWidget* parent);

    std::string name_;
    bool isArmed_;
    bool isOffboard_;

};

#endif // px4_gcs_rqt__drone_H