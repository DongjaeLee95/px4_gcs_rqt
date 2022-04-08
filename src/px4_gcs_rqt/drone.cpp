#include <px4_gcs_rqt/drone.h>

Drone::Drone(std::string name)
{
    name_ = name;

    isArmed_ = false;
    isOffboard_ = false;
}

Drone::Drone(std::string name, bool isArmed)
{
    name_ = name;

    isArmed_ = isArmed;
    isOffboard_ = false;
}

Drone::Drone(std::string name, bool isArmed, bool isOffboard)
{
    name_ = name;

    isArmed_ = isArmed;
    isOffboard_ = isOffboard;
}

QTreeWidgetItem* Drone::toTreeItem(QTreeWidget* parent)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(parent);
    item->setText(0, QString::fromStdString(name_));
    item->setText(1, QString::number(isArmed_));
    item->setText(2, QString::number(isOffboard_));
    return item;
}