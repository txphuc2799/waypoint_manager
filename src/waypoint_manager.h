#ifndef _WAYPOINT_MANAGER_PLUGIN_H_
#define _WAYPOINT_MANAGER_PLUGIN_H_

#include <rviz/panel.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QTimer>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/Odometry.h>

namespace waypoint_manager
{
class WaypointManagerPlugin : public rviz::Panel
{
Q_OBJECT
public:
    WaypointManagerPlugin(QWidget *parent = 0);

public Q_SLOTS:
    void saveWaypoints();
    void loadWaypoints();
    void addWaypoint();
    void deleteWaypoint();
    void clearAllWaypoints();
    void getFixedPose();
    void publishMarkers();

protected:
    QVBoxLayout* layout;
    QPushButton* save_button;
    QPushButton* load_button;
    QPushButton* add_button;
    QPushButton* delete_button;
    QPushButton* clear_button;
    QPushButton* get_fixed_pose_button;

    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber amcl_pose_sub_;
    QTimer* timer_;

    std::vector<geometry_msgs::Pose> waypoints_;
    std::vector<std::string> waypoint_names_;
    geometry_msgs::Pose fixed_pose_;

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void deleteWaypointAtIndex(int index);
};
} // end namespace waypoint_manager

#endif // _WAYPOINT_MANAGER_PLUGIN_H_
