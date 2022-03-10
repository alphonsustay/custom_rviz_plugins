#ifndef CUSTOM_RVIZ_PLUGIN_ADD_POSE_TOOL_HPP
#define CUSTOM_RVIZ_PLUGIN_ADD_POSE_TOOL_HPP

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
    #include <QObject> 
    #include <ros/ros.h>    
    #include "rviz/default_plugin/tools/pose_tool.h"
#endif

namespace rviz
{
    class Arrow;
    class DisplayContext;
    class StringProperty;
    class AddPoseTool : public PoseTool
    {
        Q_OBJECT
        public:
            AddPoseTool();
            virtual ~AddPoseTool() {}
            virtual void onInitialize();
        
        protected:
            virtual void onPoseSet(double x, double y, double theta);
        
        private Q_SLOTS:
            void updateTopic();

        private:
            ros::NodeHandle nh_;
            ros::Publisher pub_;
            StringProperty* topic_property_;
    }; 
}

#endif // CUSTOM_RVIZ_PLUGIN_ADD_POSE_TOOL_HPP