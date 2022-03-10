#ifndef CUSTOM_RVIZ_PLUGIN_ROBOT_UI_PANEL_HPP
#define CUSTOM_RVIZ_PLUGIN_ROBOT_UI_PANEL_HPP

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
    #include <QObject> 
    #include <ros/ros.h>    
    #include "rviz/panel.h"
    #include "add_pose_tool.hpp"
    #include "geometry_msgs/Pose.h"
    #include <geometry_msgs/PoseWithCovarianceStamped.h>
    #include <thread>
    #include <QLabel>
    #include <QPushButton>
    #include <QLineEdit>
    #include <QComboBox>
    #include <QTimer>
    #include "move_base_msgs/MoveBaseAction.h"
    #include "actionlib/client/simple_action_client.h"
    #include "unordered_map"
#endif

namespace custom_rviz_plugin
{
    class RobotUIPanel : public rviz::Panel
    {
        Q_OBJECT
        public:
            // QWidget subclass constructors usually take a parent widget
            // parameter (which usually defaults to 0).  At the same time,
            // pluginlib::ClassLoader creates instances by calling the default
            // constructor (with no arguments).  Taking the parameter and giving
            // a default of 0 lets the default constructor work and also lets
            // someone using the class for something else to pass in a parent
            // widget as they normally would with Qt.
            RobotUIPanel(QWidget* parent = 0);

            // Now we declare overrides of rviz::Panel functions for saving and
            // loading data from the config file.  Here the data is the
            // topic name.
            virtual void load(const rviz::Config& config);
            virtual void save(rviz::Config config) const;


        public Q_SLOTS:
            void add_waypoint_request();
            void clear_current_waypoint();
            // void init_add_pose_tool();
            

        protected Q_SLOTS:
            void update_waypoint_selector();
            void show_sel_waypoint();
            void update_to_show_status();
            void update_sel_waypoint_info();
            void send_to_waypoint();
            void cancel_task();
            void load_waypoints();
            void export_waypoints();

        protected:
            ros::NodeHandle nh_;
            ros::Subscriber position_sub_;
            ros::Publisher  waypoint_vis_pub_;
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _robot_client{"move_base", true};
            
        private:
            void create_layout();
            void initialize_subscribers();
            void initialize_qt_connections();
            void clickPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg);
            // rviz::AddPoseTool add_pose;
            std::unordered_map<std::string, geometry_msgs::Pose> known_waypoints_;
            geometry_msgs::Pose holding_pose_;
            bool to_show;
            std::string param_file_path;
            
            QTimer* _heartbeat;
            QLineEdit* _new_waypoint_name_editor;
            QLabel* _x_coord_value;
            QLabel* _y_coord_value;
            QLabel* _theta_coord_value;
            QLabel* _sel_x_coord_value;
            QLabel* _sel_y_coord_value;
            QLabel* _sel_theta_coord_value;
            QLabel* _status_info;
            QLabel* _nav_status_info;
            QPushButton* _add_waypoint_button;
            QPushButton* _clear_waypoint_button;
            QPushButton* _show_waypoint_button;
            QPushButton* _goto_waypoint_button;
            QPushButton* _cancel_button;
            QPushButton* _load_button;
            QPushButton* _save_button;
            // QPushButton* _set_waypoint_button;
            QComboBox* _sel_waypoint_selector;
    };
}
#endif // CUSTOM_RVIZ_PLUGIN_ROBOT_UI_PANEL_HPP