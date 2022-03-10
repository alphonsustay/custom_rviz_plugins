#include <stdio.h>
#include "robot_ui_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QGroupBox>
#include <QGridLayout>
#include <QFileDialog>
#include <QFile>
#include <tf/transform_datatypes.h>
#include "visualization_msgs/Marker.h"
#include <yaml-cpp/yaml.h>

namespace custom_rviz_plugin
{
    RobotUIPanel::RobotUIPanel(QWidget* parent) : rviz::Panel(parent)
    {
        _heartbeat = new QTimer(this);
        _heartbeat->start(500);
        create_layout();
        initialize_subscribers();
        initialize_qt_connections();
    }

    void RobotUIPanel::create_layout()
    {
        QGridLayout* UI_panel_layout = new QGridLayout;

        QGroupBox* add_waypoint_gb = new QGroupBox("Add Waypoint");
        QGridLayout* add_waypoint_layout = new QGridLayout();
        // QFont _font_format = QFont("Source Code Pro", 10, QFont::Bold);

        QLabel* _new_waypoint_name_label = new QLabel("Name:");
        _new_waypoint_name_editor = new QLineEdit;
        // _set_waypoint_button = new QPushButton("Set Pose");

        add_waypoint_layout->addWidget(_new_waypoint_name_label, 0, 0, 1, 1);
        add_waypoint_layout->addWidget(_new_waypoint_name_editor, 0, 1, 1, 3);
        // add_waypoint_layout->addWidget(_set_waypoint_button, 0, 3, 1, 1);

        QLabel* _new_waypoint_info_label = new QLabel("Current Waypoint Information");
        QLabel* _x_coord_label = new QLabel("X: ");
        QLabel* _y_coord_label = new QLabel("Y: ");
        QLabel* _theta_coord_label = new QLabel("Theta: ");
        _x_coord_value = new QLabel("NULL");
        _y_coord_value = new QLabel("NULL");
        _theta_coord_value = new QLabel("NULL");

        add_waypoint_layout->addWidget(_new_waypoint_info_label, 1, 0, 1, 4);
        add_waypoint_layout->addWidget(_x_coord_label, 2, 0, 1, 1);
        add_waypoint_layout->addWidget(_x_coord_value, 2, 1, 1, 1);
        add_waypoint_layout->addWidget(_y_coord_label, 3, 0, 1, 1);
        add_waypoint_layout->addWidget(_y_coord_value, 3, 1, 1, 1);
        add_waypoint_layout->addWidget(_theta_coord_label, 4, 0, 1, 1);
        add_waypoint_layout->addWidget(_theta_coord_value, 4, 1, 1, 1);

        _add_waypoint_button = new QPushButton("Add");
        _clear_waypoint_button = new QPushButton("Clear");
        _status_info = new QLabel("");
        add_waypoint_layout->addWidget(_add_waypoint_button, 5, 0, 1, 2);
        add_waypoint_layout->addWidget(_clear_waypoint_button, 5, 2, 1, 2);
        add_waypoint_layout->addWidget(_status_info, 6, 0, 1, 4);

        add_waypoint_gb->setLayout(add_waypoint_layout);

////////////////////////////////////////////////////////////////////////////////

        QGroupBox* goto_waypoint_gb = new QGroupBox("Go To Waypoint");
        QGridLayout* goto_waypoint_layout = new QGridLayout();

        QLabel* _sel_waypoint_name_label = new QLabel("Waypoint: ");
        _sel_waypoint_selector = new QComboBox;
        _sel_waypoint_selector->setEditable(true);
        _show_waypoint_button = new QPushButton("Show");

        goto_waypoint_layout->addWidget(_sel_waypoint_name_label, 0, 0, 1, 1);
        goto_waypoint_layout->addWidget(_sel_waypoint_selector, 0, 1, 1, 2);
        goto_waypoint_layout->addWidget(_show_waypoint_button, 0, 3, 1, 1);

        QLabel* _sel_x_coord_label = new QLabel("X: ");
        QLabel* _sel_y_coord_label = new QLabel("Y: ");
        QLabel* _sel_theta_coord_label = new QLabel("Theta: ");
        _sel_x_coord_value = new QLabel("NULL");
        _sel_y_coord_value = new QLabel("NULL");
        _sel_theta_coord_value = new QLabel("NULL");

        goto_waypoint_layout->addWidget(_sel_x_coord_label, 1, 0, 1, 1);
        goto_waypoint_layout->addWidget(_sel_x_coord_value, 1, 1, 1, 1);
        goto_waypoint_layout->addWidget(_sel_y_coord_label, 2, 0, 1, 1);
        goto_waypoint_layout->addWidget(_sel_y_coord_value, 2, 1, 1, 1);
        goto_waypoint_layout->addWidget(_sel_theta_coord_label, 3, 0, 1, 1);
        goto_waypoint_layout->addWidget(_sel_theta_coord_value, 3, 1, 1, 1);

        _goto_waypoint_button = new QPushButton("Send to Waypoint");
        _cancel_button = new QPushButton("Cancel");
        goto_waypoint_layout->addWidget(_goto_waypoint_button, 4, 0, 1, 2);
        goto_waypoint_layout->addWidget(_cancel_button, 4, 2, 1, 2);

        _nav_status_info = new QLabel("");
        goto_waypoint_layout->addWidget(_nav_status_info, 5, 0, 1, 4);

        goto_waypoint_gb->setLayout(goto_waypoint_layout);
        
/////////////////////////////////////////////////////////////////////////////////

        QGroupBox* waypoint_man_gb = new QGroupBox("Waypoint Management");
        QGridLayout* waypoint_man_layout = new QGridLayout();

        _load_button = new QPushButton("Load Config");
        _save_button = new QPushButton("Save Config");
        waypoint_man_layout->addWidget(_load_button, 0, 0, 1, 2);
        waypoint_man_layout->addWidget(_save_button, 0, 2, 1, 2);

        waypoint_man_gb->setLayout(waypoint_man_layout);
        // TODO Load and save waypoint from and to files
        // TODO Delete waypoints from existing list

        UI_panel_layout->addWidget(add_waypoint_gb, 0, 0, 7, 4);
        UI_panel_layout->addWidget(goto_waypoint_gb, 7, 0, 5, 4);
        UI_panel_layout->addWidget(waypoint_man_gb, 12, 0 , 1, 4);
        setLayout(UI_panel_layout);
    }

    void RobotUIPanel::initialize_qt_connections()
    {
        connect(_add_waypoint_button, SIGNAL(clicked()), this,
            SLOT(add_waypoint_request()));

        connect(_clear_waypoint_button, SIGNAL(clicked()), this,
            SLOT(clear_current_waypoint()));

        // connect(_set_waypoint_button, SIGNAL(clicked()), this,
        //     SLOT(init_add_pose_tool()));

        to_show = false;
        connect(_show_waypoint_button, SIGNAL(clicked()), this,
            SLOT(update_to_show_status()));

        connect(_goto_waypoint_button, SIGNAL(clicked()), this,
            SLOT(send_to_waypoint()));

        connect(_cancel_button, SIGNAL(clicked()), this,
            SLOT(cancel_task()));

        connect(_load_button, SIGNAL(clicked()), this,
            SLOT(load_waypoints()));

        connect(_save_button, SIGNAL(clicked()), this,
            SLOT(export_waypoints()));

        connect(_heartbeat, SIGNAL(timeout()), this,
            SLOT(show_sel_waypoint()));

        connect(_heartbeat, SIGNAL(timeout()), this,
            SLOT(update_sel_waypoint_info()));
    }

    void RobotUIPanel::initialize_subscribers()
    {
        position_sub_ = nh_.subscribe("clicked_pose", 1000, &RobotUIPanel::clickPoseCallback, this);
        waypoint_vis_pub_ = nh_.advertise<visualization_msgs::Marker> ("visualization_marker", 1000);
    }

    void RobotUIPanel::clickPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
    {
        ROS_INFO("Clicked Pose Caught by RobotUIPanel");
        holding_pose_.position.x = msg.pose.pose.position.x;
        holding_pose_.position.y = msg.pose.pose.position.y;
        holding_pose_.orientation = msg.pose.pose.orientation;

        tf::Quaternion q(holding_pose_.orientation.x,
                         holding_pose_.orientation.y,
                         holding_pose_.orientation.z,
                         holding_pose_.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        _x_coord_value->setText(QString::fromStdString(std::to_string(holding_pose_.position.x)));
        _y_coord_value->setText(QString::fromStdString(std::to_string(holding_pose_.position.y)));
        _theta_coord_value->setText(QString::fromStdString(std::to_string(yaw)));
        Q_EMIT configChanged();
    }

    void RobotUIPanel::add_waypoint_request()
    {
        std::string wp_name = _new_waypoint_name_editor->text().toStdString();
        if (wp_name.empty())
        {
            ROS_ERROR("Missing Waypoint Name!!!");
            _status_info->setText(QString("Missing Waypoint Name!"));
            Q_EMIT configChanged();
            return;
        }
        if (!holding_pose_.position.x)
        {
            ROS_ERROR("Missing Waypoint Data!!!");
            _status_info->setText(QString("Missing Waypoint Data!"));
            Q_EMIT configChanged();
            return;
        }

        // TODO: waypoint name check
        known_waypoints_.insert(std::pair<std::string, geometry_msgs::Pose>(wp_name, holding_pose_));
        update_waypoint_selector();
        _new_waypoint_name_editor->clear();
        clear_current_waypoint();
        ROS_INFO("Successfully Added!");
        _status_info->setText(QString("Successfully Added!"));
    }

    void RobotUIPanel::clear_current_waypoint()
    {
        geometry_msgs::Pose pose;
        holding_pose_ = pose;
        _x_coord_value->setText(QString("NULL"));
        _y_coord_value->setText(QString("NULL"));
        _theta_coord_value->setText(QString("NULL"));
        ROS_INFO("Stored Data Cleared!");
        _status_info->setText(QString("Stored Data Cleared!"));
        Q_EMIT configChanged();
    }

    // TODO
    // void RobotUIPanel::init_add_pose_tool()
    // {
    //     add_pose.onInitialize();
    //     ROS_INFO("Trying the add add_pose_tool");
    // }

    void RobotUIPanel::update_to_show_status()
    {
        if (to_show)
        {
            to_show = false;
            _show_waypoint_button->setText(QString("Show"));
            visualization_msgs::Marker waypoint;
            waypoint.header.frame_id = "map";
            waypoint.action = 3;
            waypoint_vis_pub_.publish(waypoint);
        }
        else 
        {
            to_show = true;
            _show_waypoint_button->setText(QString("Unshow"));
            show_sel_waypoint();
        }
    }

    void RobotUIPanel::update_waypoint_selector()
    {
        std::unordered_map<std::string, geometry_msgs::Pose>::iterator itr;
        _sel_waypoint_selector->clear();
        for (itr = known_waypoints_.begin(); itr != known_waypoints_.end(); itr++)
        {
            _sel_waypoint_selector->addItem(QString((itr->first).c_str()));
        }
        _sel_waypoint_selector->addItem(QString(""));
    }

    void RobotUIPanel::show_sel_waypoint()
    {
        if (to_show)
        {
            std::string sel_waypoint = _sel_waypoint_selector->currentText().toStdString();
            visualization_msgs::Marker waypoint;
            waypoint.header.frame_id = "map";
            if (sel_waypoint == "")
            {
                waypoint.action = 3;
                waypoint_vis_pub_.publish(waypoint);
            }
            else
            {
                waypoint.type = 0;
                waypoint.action = 0;
                waypoint.pose = known_waypoints_[sel_waypoint];
                waypoint.scale.x = 0.25;
                waypoint.scale.y = 0.25;
                waypoint.scale.z = 0.25;
                waypoint.color.a = 1.0;
                waypoint.color.r = 0.0;
                waypoint.color.g = 0.0;
                waypoint.color.b = 1.0;
                waypoint_vis_pub_.publish(waypoint);
            }
        }
    }

    void RobotUIPanel::update_sel_waypoint_info()
    {
        std::string sel_waypoint = _sel_waypoint_selector->currentText().toStdString();
        if (sel_waypoint == "")
        {
            _sel_x_coord_value->setText(QString("NULL"));
            _sel_y_coord_value->setText(QString("NULL"));
            _sel_theta_coord_value->setText(QString("NULL"));
        }
        else 
        {
            geometry_msgs::Pose waypoint_pose = known_waypoints_[sel_waypoint];
            tf::Quaternion q(waypoint_pose.orientation.x,
                            waypoint_pose.orientation.y,
                            waypoint_pose.orientation.z,
                            waypoint_pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            _sel_x_coord_value->setText(QString::fromStdString(std::to_string(waypoint_pose.position.x)));
            _sel_y_coord_value->setText(QString::fromStdString(std::to_string(waypoint_pose.position.y)));
            _sel_theta_coord_value->setText(QString::fromStdString(std::to_string(yaw)));
        }
        Q_EMIT configChanged();
    }

    void RobotUIPanel::send_to_waypoint()
    {
        int wait_attempt_lmt = 2;
        int count = 0;
        while (!_robot_client.waitForServer(ros::Duration(5.0))) 
        {
            ROS_INFO("Unable to contact move_base server");
            if (count == wait_attempt_lmt)
            {
                ROS_INFO("Timeout reached, Send Task Failed");
                _nav_status_info->setText(QString("Timeout reached, Send Task Failed"));
                Q_EMIT configChanged();
                return;
            }
            else
            {
                std::string status_text = "Attempting to connect with move_base server (" + std::to_string(count) + "/" + std::to_string(wait_attempt_lmt + 1) + ")";
                _nav_status_info->setText(QString::fromStdString(status_text));
                count++;
                Q_EMIT configChanged();
            }
        }
        std::string goal_name = _sel_waypoint_selector->currentText().toStdString();
        geometry_msgs::Pose goal_pose = known_waypoints_[goal_name];
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = goal_pose;

        _robot_client.sendGoal(goal);
    }

    void RobotUIPanel::cancel_task()
    {
        int wait_attempt_lmt = 2;
        int count = 0;
        while (!_robot_client.waitForServer(ros::Duration(5.0))) 
        {
            ROS_INFO("Unable to contact move_base server");
            if (count == wait_attempt_lmt)
            {
                ROS_INFO("Timeout reached, Send Task Failed");
                return;
            }
            else
            {
                count++;
            }
        }
        ros::Time current_time = ros::Time::now();
        _robot_client.cancelGoalsAtAndBeforeTime(current_time);
        ROS_INFO("Current Task has been cancelled");
        _nav_status_info->setText(QString("Current Task has been cancelled"));
        Q_EMIT configChanged();
    }

    void RobotUIPanel::load_waypoints()
    {
        QString filename = QFileDialog::getOpenFileName(this,
            tr("Load Waypoint Configuration File"), "", tr("*.yaml"));

        if (filename.isEmpty())
        {
            ROS_INFO("No file selected!!!");
            return;
        }
        
        param_file_path = filename.toStdString();
        ROS_INFO("File Selected: %s", param_file_path.c_str());

        YAML::Node waypoints = YAML::LoadFile(param_file_path)["waypoints"];
        ROS_INFO("Clearing current known waypoints");
        known_waypoints_.clear();
        for (YAML::const_iterator it = waypoints.begin(); it!=waypoints.end(); it++)
        {

            std::cout << it->first.as<std::string>() << std::endl;
            std::cout << (it->second)["X"].as<std::string>() << std::endl;
            geometry_msgs::Pose coord;
            std::string name = it->first.as<std::string>();
            coord.position.x = (it->second)["X"].as<double>();
            coord.position.y = (it->second)["Y"].as<double>();
            coord.orientation.x = (it->second)["qx"].as<double>();
            coord.orientation.y = (it->second)["qy"].as<double>();
            coord.orientation.z = (it->second)["qz"].as<double>();
            coord.orientation.w = (it->second)["qw"].as<double>();
            known_waypoints_.insert(std::pair<std::string, geometry_msgs::Pose>(name, coord));
        }

        ROS_INFO("Waypoints Loaded");
        update_waypoint_selector();
    }

    void RobotUIPanel::export_waypoints()
    {
        QString filename = QFileDialog::getSaveFileName(this,
            tr("Save Waypoint Configuration File"), "", tr("*.yaml"));

        if (filename.isEmpty())
        {
            ROS_INFO("Invalid filename, configuration not saved!");
            return;
        }
        
        YAML::Emitter output;
        YAML::Node waypoints;
        std::unordered_map<std::string, geometry_msgs::Pose>::iterator itr;
        for (itr = known_waypoints_.begin(); itr != known_waypoints_.end(); itr++)
        {
            
            YAML::Node pose;
            pose["X"] = std::to_string(((itr->second).position.x));
            pose["Y"] = std::to_string(((itr->second).position.y));
            pose["qx"] = std::to_string(((itr->second).orientation.x));
            pose["qy"] = std::to_string(((itr->second).orientation.y));
            pose["qz"] = std::to_string(((itr->second).orientation.z));
            pose["qw"] = std::to_string(((itr->second).orientation.w));
            waypoints["waypoints"][(itr->first).c_str()] = pose;
        }

        output << waypoints;
        std::string save_filename = filename.toStdString();
        if(save_filename.substr(save_filename.find_last_of(".") + 1) != "yaml")
        {
            save_filename = save_filename + ".yaml";
        }
        std::ofstream fout(save_filename);
        fout << output.c_str();
        ROS_INFO("%s file saved", save_filename.c_str());
    }

    void RobotUIPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    void RobotUIPanel::load(const rviz::Config& config)
    {
        rviz::Panel::load(config);
    }

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(custom_rviz_plugin::RobotUIPanel, rviz::Panel)