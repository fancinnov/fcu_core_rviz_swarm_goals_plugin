#ifndef MULTI_NAVI_GOAL_PANEL_H
#define MULTI_NAVI_GOAL_PANEL_H


#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <rviz/panel.h>

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>
#include <QGridLayout>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

namespace navi_multi_goals_pub_rviz_plugin {

    typedef enum {
        ReadyToGoal,
        ExecutingGoal
    } path_track_flag;

    class MultiNaviGoalsPanel : public rviz::Panel {
    Q_OBJECT
    public:
        explicit MultiNaviGoalsPanel(QWidget *parent = 0);

    public Q_SLOTS:

        void setMaxNumGoal(const QString &maxNumGoal);

        void writePose001(geometry_msgs::Pose pose);
        void writePose002(geometry_msgs::Pose pose);
        void writePose003(geometry_msgs::Pose pose);
        void writePose004(geometry_msgs::Pose pose);
        void writePose005(geometry_msgs::Pose pose);
        void writePose006(geometry_msgs::Pose pose);
        void markPose();
        void deleteMark();
        void markWall(const QString &wall_startx, const QString &wall_starty, const QString &wall_endx,const QString &wall_endy);
        

    protected Q_SLOTS:
        void updateWall();
        void updateMaxNumGoal();             // update max number of goal
        void computeGlobalOffset(double dx, double dy, double& out_dx, double& out_dy);
        void initPoseTable();               // initialize the pose table

        void updatePoseTable();             // update the pose table

        void highlightTableRow(QTableWidget* table, int targetRow);
        bool isDroneAtGoal(int drone_id, const geometry_msgs::PoseArray& goals, int curGoalIdx); 
        void NaviControl();   
        void StopControl();
        void startNavi();                   // start navigate for the first pose

        void refreshPoseArrayTable(QTableWidget* table, const geometry_msgs::PoseArray& pose_array);
        void deleteGoalPoint();
        void deleteAllMark();

        void goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose);  //goal count sub callback function

        void statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses); //status sub callback function

        void checkCycle001();
        void checkCycle002();
        void checkCycle003();
        void checkCycle004();
        void checkCycle005();
        void checkCycle006();

        void completeNavi();               //after the first pose, continue to navigate the rest of poses
        void cycleNavi();

        bool checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list);  // check whether arrived the goal

        void startSpin(); // spin for sub
        void odom_global001_handler(const nav_msgs::Odometry::ConstPtr& odom);
        void odom_global002_handler(const nav_msgs::Odometry::ConstPtr& odom);
        void odom_global003_handler(const nav_msgs::Odometry::ConstPtr& odom);
        void odom_global004_handler(const nav_msgs::Odometry::ConstPtr& odom);
        void odom_global005_handler(const nav_msgs::Odometry::ConstPtr& odom);
        void odom_global006_handler(const nav_msgs::Odometry::ConstPtr& odom);
    protected:
        QLineEdit *position_threshold_editor_, *default_z_editor;
        QLineEdit *drone_id_editor_, *wall_startpointx_editor_, *wall_startpointy_editor_, *wall_endpointx_editor_, *wall_endpointy_editor_, *output_maxNumGoal_editor_;
        QPushButton *output_maxNumGoal_button_, *output_reset_button_, *output_startNavi_button_, *output_stopNavi_button_, *output_delete_button_;
        QTableWidget *poseArray_table_001, *poseArray_table_002, *poseArray_table_003, *poseArray_table_004, *poseArray_table_005, *poseArray_table_006;
        QCheckBox *cycle_checkbox_001, *cycle_checkbox_002, *cycle_checkbox_003, *cycle_checkbox_004, *cycle_checkbox_005, *cycle_checkbox_006;

        QString output_maxNumGoal_, output_wall_start_x, output_wall_start_y, output_wall_end_x, output_wall_end_y;

        // The ROS node handle.
        ros::NodeHandle nh_;
        ros::Publisher goal_pub_001, goal_pub_002, goal_pub_003, goal_pub_004, goal_pub_005, goal_pub_006, 
                        marker_pub_001, marker_pub_002, marker_pub_003, marker_pub_004, marker_pub_005, marker_pub_006,
                        wall_pub_, cmd_pub;
        ros::Subscriber goal_sub_, odom001, odom002, odom003, odom004, odom005, odom006;
        float pos_odom_001_x=0.0f; float pos_odom_001_y=0.0f; float pos_odom_001_z=0.0f; float pos_odom_001_roll;float pos_odom_001_pitch;float pos_odom_001_yaw;
        float pos_odom_002_x=0.0f; float pos_odom_002_y=0.0f; float pos_odom_002_z=0.0f; float pos_odom_002_roll;float pos_odom_002_pitch;float pos_odom_002_yaw;
        float pos_odom_003_x=0.0f; float pos_odom_003_y=0.0f; float pos_odom_003_z=0.0f; float pos_odom_003_roll;float pos_odom_003_pitch;float pos_odom_003_yaw;
        float pos_odom_004_x=0.0f; float pos_odom_004_y=0.0f; float pos_odom_004_z=0.0f; float pos_odom_004_roll;float pos_odom_004_pitch;float pos_odom_004_yaw;
        float pos_odom_005_x=0.0f; float pos_odom_005_y=0.0f; float pos_odom_005_z=0.0f; float pos_odom_005_roll;float pos_odom_005_pitch;float pos_odom_005_yaw;
        float pos_odom_006_x=0.0f; float pos_odom_006_y=0.0f; float pos_odom_006_z=0.0f; float pos_odom_006_roll;float pos_odom_006_pitch;float pos_odom_006_yaw;

        int maxNumGoal_;
        float wall_start_x=0.0,wall_start_y=0.0,wall_end_x=0.0,wall_end_y=0.0;
        int curGoalIdx_001 = 0, curGoalIdx_002 = 0, curGoalIdx_003 = 0, curGoalIdx_004 = 0, curGoalIdx_005 = 0, curGoalIdx_006 = 0,
            cycleCnt_001 = 0, cycleCnt_002 = 0, cycleCnt_003 = 0, cycleCnt_004 = 0, cycleCnt_005 = 0, cycleCnt_006 = 0;
        bool permit_001 = false, permit_002 = false, permit_003 = false, permit_004 = false, permit_005 = false, permit_006 = false, 
             cycle_001 = false, cycle_002 = false, cycle_003 = false, cycle_004 = false, cycle_005 = false, cycle_006 = false,
                arrived_ = false;
        geometry_msgs::PoseArray pose_array_001,pose_array_002,pose_array_003,pose_array_004,pose_array_005,pose_array_006;
        bool init_wall=false;

            
        path_track_flag path_track_status_001 = ReadyToGoal;
        path_track_flag path_track_status_002 = ReadyToGoal;
        path_track_flag path_track_status_003 = ReadyToGoal;
        path_track_flag path_track_status_004 = ReadyToGoal;
        path_track_flag path_track_status_005 = ReadyToGoal;
        path_track_flag path_track_status_006 = ReadyToGoal;

        QPushButton *button_arm, *button_disarm, *button_takeoff, *button_land, *button_trackfront, *button_trackdown, *button_stoptrack;
        QPushButton *button_movefront, *button_moveback, *button_moveleft, *button_moveright, *button_turnleft, *button_turnright;
    };

} // end namespace navi-multi-goals-pub-rviz-plugin

#endif // MULTI_NAVI_GOAL_PANEL_H
