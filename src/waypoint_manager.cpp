#include "waypoint_manager.h"
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/default_plugin/tools/pose_tool.h>
#include <algorithm>

namespace waypoint_manager
{
WaypointManagerPlugin::WaypointManagerPlugin(QWidget *parent)
  : rviz::Panel(parent)
{
    layout = new QVBoxLayout;

    save_button = new QPushButton("Save Waypoints");
    load_button = new QPushButton("Load Waypoints");
    add_button = new QPushButton("Add Waypoint");
    delete_button = new QPushButton("Delete Waypoints");
    clear_button = new QPushButton("Clear All Waypoints");
    get_fixed_pose_button = new QPushButton("Get Fixed Pose");

    layout->addWidget(save_button);
    layout->addWidget(load_button);
    layout->addWidget(add_button);
    layout->addWidget(delete_button);
    layout->addWidget(clear_button);
    layout->addWidget(get_fixed_pose_button);

    setLayout(layout);

    connect(save_button, SIGNAL(clicked()), this, SLOT(saveWaypoints()));
    connect(load_button, SIGNAL(clicked()), this, SLOT(loadWaypoints()));
    connect(add_button, SIGNAL(clicked()), this, SLOT(addWaypoint()));
    connect(delete_button, SIGNAL(clicked()), this, SLOT(deleteWaypoint()));
    connect(clear_button, SIGNAL(clicked()), this, SLOT(clearAllWaypoints()));
    connect(get_fixed_pose_button, SIGNAL(clicked()), this, SLOT(getFixedPose()));

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_markers", 10);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &WaypointManagerPlugin::goalCallback, this);
    amcl_pose_sub_ = nh_.subscribe("/amcl_pose", 10, &WaypointManagerPlugin::amclPoseCallback, this);

    timer_ = new QTimer(this);
    connect(timer_, SIGNAL(timeout()), this, SLOT(publishMarkers()));
    timer_->start(100);
}

void WaypointManagerPlugin::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    bool added = false;
    for (size_t i = 0; i < waypoints_.size(); ++i)
    {
        if (waypoint_names_[i].empty())
        {
            waypoints_[i] = msg->pose;
            waypoint_names_[i] = "WAYPOINT " + std::to_string(i);
            added = true;
            break;
        }
    }

    if (!added)
    {
        waypoints_.push_back(msg->pose);
        std::string waypoint_name = "WAYPOINT " + std::to_string(waypoints_.size() - 1);
        waypoint_names_.push_back(waypoint_name);
    }
    publishMarkers();
}

void WaypointManagerPlugin::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    fixed_pose_ = msg->pose.pose;
}

void WaypointManagerPlugin::getFixedPose()
{
    ros::spinOnce();

    bool ok;
    QString text = QInputDialog::getText(this, tr("Input Pose Name"), tr("Pose Name:"), QLineEdit::Normal, "", &ok);
    if (ok && !text.isEmpty())
    {
        std::string name = text.toStdString();
        waypoints_.push_back(fixed_pose_);
        waypoint_names_.push_back(name);

        saveWaypoints();
    }
    waypoints_.clear();
    waypoint_names_.clear();
}

void WaypointManagerPlugin::saveWaypoints()
{
    bool has_waypoint = false;
    for (const auto& name : waypoint_names_)
    {
        if (!name.empty())
        {
            has_waypoint = true;
            break;
        }
    }

    if (!has_waypoint)
    {
        QMessageBox::warning(this, tr("Save Waypoints"), tr("No waypoints to save."));
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Waypoints"), "", tr("YAML Files (*.yaml);;All Files (*)"));

    if (fileName.isEmpty())
        return;

    if (!fileName.endsWith(".yaml", Qt::CaseInsensitive))
    {
        fileName += ".yaml";
    }

    YAML::Node doc;
    std::ifstream fin(fileName.toStdString());
    if (fin.good())
    {
        doc = YAML::Load(fin);
        fin.close();
    }

    bool ok;
    QString parent_name;
    while (true)
    {
        parent_name = QInputDialog::getText(this, tr("Waypoint Header Name"), tr("Enter parent waypoint name:"), QLineEdit::Normal, "", &ok);
        if (!ok || parent_name.isEmpty())
        {
            QMessageBox::warning(this, tr("Save Waypoints"), tr("Waypoint header name is required."));
            return;
        }

        if (doc[parent_name.toStdString()])
        {
            QMessageBox::StandardButton reply;
            reply = QMessageBox::question(this, tr("Header Name Exist"), tr("Waypoint header name already exists. Do you want to replace it?"), QMessageBox::Yes|QMessageBox::No);
            if (reply == QMessageBox::Yes)
            {
                doc.remove(parent_name.toStdString());
                break;
            }
            else
            {
                continue;
            }
        }
        else
        {
            break;
        }
    }

    YAML::Node parent_node;
    for (size_t i = 0; i < waypoints_.size(); ++i)
    {
        if (!waypoint_names_[i].empty())
        {
            YAML::Node waypoint_node;
            waypoint_node["position"].push_back(waypoints_[i].position.x);
            waypoint_node["position"].push_back(waypoints_[i].position.y);
            waypoint_node["position"].push_back(waypoints_[i].position.z);
            waypoint_node["orientation"].push_back(waypoints_[i].orientation.x);
            waypoint_node["orientation"].push_back(waypoints_[i].orientation.y);
            waypoint_node["orientation"].push_back(waypoints_[i].orientation.z);
            waypoint_node["orientation"].push_back(waypoints_[i].orientation.w);
            parent_node[waypoint_names_[i]] = waypoint_node;
        }
    }

    doc[parent_name.toStdString()] = parent_node;

    std::ofstream fout(fileName.toStdString());
    fout << doc;
    fout.close();
}

void WaypointManagerPlugin::loadWaypoints()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Load Waypoints"), "", tr("YAML Files (*.yaml);;Text Files (*.text);;All Files (*)"));

    if (fileName.isEmpty())
        return;

    if (!fileName.endsWith(".yaml", Qt::CaseInsensitive) && !fileName.endsWith(".text", Qt::CaseInsensitive))
    {
        QMessageBox::warning(this, tr("Load Waypoints"), tr("The selected file is not a .yaml or .text file."));
        return;
    }

    std::ifstream fin(fileName.toStdString());
    if (!fin.good())
    {
        QMessageBox::warning(this, tr("Load Waypoints"), tr("Failed to open the file."));
        return;
    }

    YAML::Node doc = YAML::Load(fin);
    fin.close();

    if (doc.size() == 0)
    {
        QMessageBox::warning(this, tr("Load Waypoints"), tr("The file is empty."));
        return;
    }

    std::string parent_name;
    if (doc.size() == 1)
    {
        parent_name = doc.begin()->first.as<std::string>();
    }
    else
    {
        bool ok;
        QString parent_name_qs = QInputDialog::getText(this, tr("Parent Waypoint Name"), tr("Enter parent waypoint name to load:"), QLineEdit::Normal, "", &ok);
        if (!ok || parent_name_qs.isEmpty())
        {
            QMessageBox::warning(this, tr("Load Waypoints"), tr("Parent waypoint name is required."));
            return;
        }
        parent_name = parent_name_qs.toStdString();
    }

    if (!doc[parent_name])
    {
        QMessageBox::warning(this, tr("Load Waypoints"), tr("Parent waypoint name not found in the file."));
        return;
    }

    YAML::Node waypoints_node = doc[parent_name];
    if (!waypoints_node.IsMap())
    {
        QMessageBox::warning(this, tr("Load Waypoints"), tr("Invalid file format: parent node should be a map."));
        return;
    }

    waypoints_.clear();
    waypoint_names_.clear();

    for (YAML::const_iterator it = waypoints_node.begin(); it != waypoints_node.end(); ++it)
    {
        std::string child_name = it->first.as<std::string>();
        YAML::Node waypoint_node = it->second;

        if (!waypoint_node["position"].IsSequence() || !waypoint_node["orientation"].IsSequence() ||
            waypoint_node["position"].size() != 3 || waypoint_node["orientation"].size() != 4)
        {
            QMessageBox::warning(this, tr("Load Waypoints"), tr("Invalid file format: waypoint node should contain position (3 elements) and orientation (4 elements)."));
            return;
        }

        geometry_msgs::Pose waypoint;
        waypoint.position.x = waypoint_node["position"][0].as<double>();
        waypoint.position.y = waypoint_node["position"][1].as<double>();
        waypoint.position.z = waypoint_node["position"][2].as<double>();
        waypoint.orientation.x = waypoint_node["orientation"][0].as<double>();
        waypoint.orientation.y = waypoint_node["orientation"][1].as<double>();
        waypoint.orientation.z = waypoint_node["orientation"][2].as<double>();
        waypoint.orientation.w = waypoint_node["orientation"][3].as<double>();

        waypoints_.push_back(waypoint);
        waypoint_names_.push_back(child_name);
    }
    publishMarkers();
}

void WaypointManagerPlugin::addWaypoint()
{
    ros::spinOnce();

    waypoints_.push_back(fixed_pose_);
    std::string waypoint_name = "WAYPOINT " + std::to_string(waypoints_.size() - 1);  // In hoa
    waypoint_names_.push_back(waypoint_name);

    publishMarkers();
}

void WaypointManagerPlugin::deleteWaypoint()
{
    bool has_waypoint = false;
    for (const auto& name : waypoint_names_)
    {
        if (!name.empty())
        {
            has_waypoint = true;
            break;
        }
    }

    if (!has_waypoint)
    {
        QMessageBox::warning(this, tr("Delete Waypoints"), tr("No waypoints to delete."));
        return;
    }

    bool ok;
    int index = QInputDialog::getInt(this, tr("Delete Waypoint"), tr("Waypoint Index:"), 0, 0, waypoints_.size() - 1, 1, &ok);
    if (ok)
    {
        deleteWaypointAtIndex(index);
        publishMarkers();
    }
}

void WaypointManagerPlugin::clearAllWaypoints()
{
    visualization_msgs::MarkerArray markers;
    int id = 0;

    for (size_t i = 0; i < waypoints_.size(); ++i)
    {
        if (!waypoint_names_[i].empty())
        {
            // Marker for mesh
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = "map";
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.id = id++;
            delete_marker.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(delete_marker);

            // Marker for text
            delete_marker.id = id++;
            markers.markers.push_back(delete_marker);

            // Marker for arrow
            delete_marker.id = id++;
            markers.markers.push_back(delete_marker);
        }
    }
    // Clear all waypoints
    marker_pub_.publish(markers);

    waypoints_.clear();
    waypoint_names_.clear();
}

void WaypointManagerPlugin::deleteWaypointAtIndex(int index)
{
    if (index >= 0 && index < waypoints_.size())
    {
        waypoint_names_[index] = "";
    }
}

void WaypointManagerPlugin::publishMarkers()
{
    visualization_msgs::MarkerArray markers;
    int id = 0;
    for (size_t i = 0; i < waypoints_.size(); ++i)
    {
        if (!waypoint_names_[i].empty())
        {
            // Marker for waypoint
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = id++;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = waypoints_[i];
            marker.scale.x = 0.001;
            marker.scale.y = 0.001;
            marker.scale.z = 0.001;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.mesh_resource = "package://waypoint_manager/mesh/generic_flag.stl";
            markers.markers.push_back(marker);

            // Marker for text
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = ros::Time::now();
            text_marker.id = id++;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose = waypoints_[i];
            text_marker.pose.position.z += 0.3; // Offset text above the waypoint marker
            text_marker.scale.z = 0.15; // Height of the text
            text_marker.color.a = 1.0;
            text_marker.color.r = 0.0;
            text_marker.color.g = 0.0;
            text_marker.color.b = 1.0;
            text_marker.text = waypoint_names_[i];
            markers.markers.push_back(text_marker);

            // Marker for arrow
            visualization_msgs::Marker arrow_marker;
            arrow_marker.header.frame_id = "map";
            arrow_marker.header.stamp = ros::Time::now();
            arrow_marker.id = id++;
            arrow_marker.type = visualization_msgs::Marker::ARROW;
            arrow_marker.action = visualization_msgs::Marker::ADD;
            arrow_marker.pose = waypoints_[i];
            arrow_marker.scale.x = 0.4; // Length of the arrow
            arrow_marker.scale.y = 0.03; // Width of the arrow
            arrow_marker.scale.z = 0.03; // Height of the arrow
            arrow_marker.color.a = 1.0;
            arrow_marker.color.r = 1.0;
            arrow_marker.color.g = 0.0;
            arrow_marker.color.b = 0.0;
            markers.markers.push_back(arrow_marker);
        }
        else
        {
            // Create empty markers to delete existing markers in RViz
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = "map";
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.id = id++;
            delete_marker.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(delete_marker);

            delete_marker.id = id++;
            markers.markers.push_back(delete_marker);

            delete_marker.id = id++;
            markers.markers.push_back(delete_marker);
        }
    }
    marker_pub_.publish(markers);
}

} // end namespace waypoint_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(waypoint_manager::WaypointManagerPlugin, rviz::Panel)
