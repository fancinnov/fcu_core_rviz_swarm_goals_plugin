#include <cstdio>

#include <ros/console.h>

#include <fstream>
#include <sstream>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QColor>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>
#include <QKeyEvent>
#include <QPointF>
#include <QRegularExpression>
#include <QDebug>
#include <QMessageBox>
#include <QtWidgets/QTableWidgetItem>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <cmath>
#include <vector>
#include "../mavlink/common/mavlink.h"

#include "multi_navi_goal_panel.h"

namespace navi_multi_goals_pub_rviz_plugin {

    int selected_row = -1;

    class MyTableWidget : public QTableWidget {

    public:
        MyTableWidget(QWidget *parent = nullptr) : QTableWidget(parent) {
            // 连接点击信号
            connect(this, &QTableWidget::cellDoubleClicked, this, &MyTableWidget::cellDoubleClicked);
        }
    protected:
        void keyPressEvent(QKeyEvent *event) override {
            QTableWidgetItem *currentItem = QTableWidget::currentItem();
            QString currentText = currentItem->text();
            if (event->key() == Qt::Key_Backspace) {
                if (!currentText.isEmpty()) {
                    currentItem->setText("");
                }
                return;
            }
            if ((event->key() >= Qt::Key_0 && event->key() <= Qt::Key_9) || 
            event->key() == Qt::Key_Period || event->key() == Qt::Key_Minus) {
                QString keyText = event->text();
                 // 检查小数点重复
                if (keyText == "." && currentText.contains(".")) {
                    return;
                }
                // 检查负号位置
                if (keyText == "-" && !currentText.isEmpty()) {
                    return;
                }
                if (currentItem) {
                    currentText += QString(event->text());
                    currentItem->setText(currentText);
                }
            }
        }
    private Q_SLOTS:
        void cellDoubleClicked(int row, int column) {
            selected_row = row;
        }
    };

    MultiNaviGoalsPanel::MultiNaviGoalsPanel(QWidget *parent)
            : rviz::Panel(parent), nh_(), maxNumGoal_(10) {

        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal_temp", 100, boost::bind(&MultiNaviGoalsPanel::goalCntCB, this, _1));
        odom001=nh_.subscribe<nav_msgs::Odometry>("/odom_global_001", 100, boost::bind(&MultiNaviGoalsPanel::odom_global001_handler, this, _1));
        odom002=nh_.subscribe<nav_msgs::Odometry>("/odom_global_002", 100, boost::bind(&MultiNaviGoalsPanel::odom_global002_handler, this, _1));
        odom003=nh_.subscribe<nav_msgs::Odometry>("/odom_global_003", 100, boost::bind(&MultiNaviGoalsPanel::odom_global003_handler, this, _1));
        odom004=nh_.subscribe<nav_msgs::Odometry>("/odom_global_004", 100, boost::bind(&MultiNaviGoalsPanel::odom_global004_handler, this, _1));
        odom005=nh_.subscribe<nav_msgs::Odometry>("/odom_global_005", 100, boost::bind(&MultiNaviGoalsPanel::odom_global005_handler, this, _1));
        odom006=nh_.subscribe<nav_msgs::Odometry>("/odom_global_006", 100, boost::bind(&MultiNaviGoalsPanel::odom_global006_handler, this, _1));
        goal_pub_001 = nh_.advertise<geometry_msgs::PoseStamped>("fcu_mission/goal_001", 100);
        goal_pub_002 = nh_.advertise<geometry_msgs::PoseStamped>("fcu_mission/goal_002", 100);
        goal_pub_003 = nh_.advertise<geometry_msgs::PoseStamped>("fcu_mission/goal_003", 100);
        goal_pub_004 = nh_.advertise<geometry_msgs::PoseStamped>("fcu_mission/goal_004", 100);
        goal_pub_005 = nh_.advertise<geometry_msgs::PoseStamped>("fcu_mission/goal_005", 100);
        goal_pub_006 = nh_.advertise<geometry_msgs::PoseStamped>("fcu_mission/goal_006", 100);

        marker_pub_001 = nh_.advertise<visualization_msgs::Marker>("visualization_marker_001", 100);
        marker_pub_002 = nh_.advertise<visualization_msgs::Marker>("visualization_marker_002", 100);
        marker_pub_003 = nh_.advertise<visualization_msgs::Marker>("visualization_marker_003", 100);
        marker_pub_004 = nh_.advertise<visualization_msgs::Marker>("visualization_marker_004", 100);
        marker_pub_005 = nh_.advertise<visualization_msgs::Marker>("visualization_marker_005", 100);
        marker_pub_006 = nh_.advertise<visualization_msgs::Marker>("visualization_marker_006", 100);

        cmd_pub = nh_.advertise<std_msgs::Int16>("fcu_command/command",100);

        wall_pub_ = nh_.advertise<visualization_msgs::Marker>("wall_marker", 1);
        QVBoxLayout *root_layout = new QVBoxLayout;

        QGridLayout* grid_layout = new QGridLayout;

        button_arm = new QPushButton("解锁");
        button_disarm = new QPushButton("锁定");
        button_takeoff = new QPushButton("起飞");
        button_land = new QPushButton("降落");

        button_trackfront = new QPushButton("前视追踪");
        button_trackdown = new QPushButton("下视追踪");
        button_stoptrack = new QPushButton("停止追踪");

        button_movefront = new QPushButton("前进");
        button_moveback = new QPushButton("后退");
        button_moveleft = new QPushButton("左移");
        button_moveright = new QPushButton("右移");

        button_turnleft = new QPushButton("左转");
        button_turnright = new QPushButton("右转");
        

        grid_layout->addWidget(button_arm, 0, 1);
        grid_layout->addWidget(button_disarm, 0, 2);
        grid_layout->addWidget(button_takeoff, 0, 3);
        grid_layout->addWidget(button_land, 0, 4);

        grid_layout->addWidget(button_trackfront, 1, 1);
        grid_layout->addWidget(button_trackdown, 1, 2);
        grid_layout->addWidget(button_stoptrack, 1, 3);

        grid_layout->addWidget(button_movefront, 2, 1);
        grid_layout->addWidget(button_moveback, 2, 2);
        grid_layout->addWidget(button_moveleft, 2, 3);
        grid_layout->addWidget(button_moveright, 2, 4);

        grid_layout->addWidget(button_turnleft, 3, 1);
        grid_layout->addWidget(button_turnright, 3, 2);

        root_layout->addLayout(grid_layout);

        connect(button_arm, &QPushButton::clicked, [this]() {
            std_msgs::Int16 msg;
            msg.data = 1;
            cmd_pub.publish(msg);
        });

        connect(button_disarm, &QPushButton::clicked, [this]() {
            std_msgs::Int16 msg;
            msg.data = 2;
            cmd_pub.publish(msg);
        });

        connect(button_takeoff, &QPushButton::clicked, [this]() {
            std_msgs::Int16 msg;
            msg.data = 3;
            cmd_pub.publish(msg);
        });

        connect(button_land, &QPushButton::clicked, [this]() {
            std_msgs::Int16 msg;
            msg.data = 4;
            cmd_pub.publish(msg);
        });

        connect(button_trackfront, &QPushButton::clicked, [this]() {
            std_msgs::Int16 msg;
            msg.data = 1011;
            cmd_pub.publish(msg);
        });

        connect(button_trackdown, &QPushButton::clicked, [this]() {
            std_msgs::Int16 msg;
            msg.data = 1012;
            cmd_pub.publish(msg);
        });

        connect(button_stoptrack, &QPushButton::clicked, [this]() {
            std_msgs::Int16 msg;
            msg.data = 1013;
            cmd_pub.publish(msg);
        });

        connect(button_movefront, &QPushButton::clicked, [this]() {
            if(drone_id_editor_->text()=="1"){
                if (pose_array_001.poses.size() < maxNumGoal_) {
                    if (pose_array_001.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_001.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_001.poses.push_back(new_pose);

                    pose_array_001.poses[pose_array_001.poses.size() - 1] = new_pose;

                    // if (path_track_status_001 == ReadyToGoal)
                    //     curGoalIdx_001 = curGoalIdx_001 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_001.header;
                    goal.pose = new_pose;

                    goal_pub_001.publish(goal);

                    // 写入（传入 Pose）
                    writePose001(new_pose);  // 确保 writePose001 接收的是 geometry_msgs::Pose
                    
                    // permit_001 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="2"){
                if (pose_array_002.poses.size() < maxNumGoal_) {
                    if (pose_array_002.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_002.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_002.poses.push_back(new_pose);

                    pose_array_002.poses[pose_array_002.poses.size() - 1] = new_pose;

                    // if (path_track_status_002 == ReadyToGoal)
                    //     curGoalIdx_002 = curGoalIdx_002 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_002.header;
                    goal.pose = new_pose;

                    goal_pub_002.publish(goal);

                    // 写入（传入 Pose）
                    writePose002(new_pose);  // 确保 writePose002 接收的是 geometry_msgs::Pose
                    
                    // permit_002 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="3"){
                if (pose_array_003.poses.size() < maxNumGoal_) {
                    if (pose_array_003.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_003.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_003.poses.push_back(new_pose);

                    pose_array_003.poses[pose_array_003.poses.size() - 1] = new_pose;

                    // if (path_track_status_003 == ReadyToGoal)
                    //     curGoalIdx_003 = curGoalIdx_003 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_003.header;
                    goal.pose = new_pose;

                    goal_pub_003.publish(goal);

                    // 写入（传入 Pose）
                    writePose003(new_pose);  // 确保 writePose003 接收的是 geometry_msgs::Pose
                    
                    // permit_003 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="4"){
                if (pose_array_004.poses.size() < maxNumGoal_) {
                    if (pose_array_004.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_004.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_004.poses.push_back(new_pose);

                    pose_array_004.poses[pose_array_004.poses.size() - 1] = new_pose;

                    // if (path_track_status_004 == ReadyToGoal)
                    //     curGoalIdx_004 = curGoalIdx_004 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_004.header;
                    goal.pose = new_pose;

                    goal_pub_004.publish(goal);

                    // 写入（传入 Pose）
                    writePose004(new_pose);  // 确保 writePose004 接收的是 geometry_msgs::Pose
                    
                    // permit_004 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="5"){
                if (pose_array_005.poses.size() < maxNumGoal_) {
                    if (pose_array_005.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_005.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_005.poses.push_back(new_pose);

                    pose_array_005.poses[pose_array_005.poses.size() - 1] = new_pose;

                    // if (path_track_status_005 == ReadyToGoal)
                    //     curGoalIdx_005 = curGoalIdx_005 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_005.header;
                    goal.pose = new_pose;

                    goal_pub_005.publish(goal);

                    // 写入（传入 Pose）
                    writePose005(new_pose);  // 确保 writePose005 接收的是 geometry_msgs::Pose
                    
                    // permit_005 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="6"){
                if (pose_array_006.poses.size() < maxNumGoal_) {
                    if (pose_array_006.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_006.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_006.poses.push_back(new_pose);

                    pose_array_006.poses[pose_array_006.poses.size() - 1] = new_pose;

                    // if (path_track_status_006 == ReadyToGoal)
                    //     curGoalIdx_006 = curGoalIdx_006 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_006.header;
                    goal.pose = new_pose;

                    goal_pub_006.publish(goal);

                    // 写入（传入 Pose）
                    writePose006(new_pose);  // 确保 writePose006 接收的是 geometry_msgs::Pose
                    
                    // permit_006 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
        });

        connect(button_moveback, &QPushButton::clicked, [this]() {
            if(drone_id_editor_->text()=="1"){
                if (pose_array_001.poses.size() < maxNumGoal_) {
                    if (pose_array_001.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_001.poses.back();

                    // 创建新位姿，x - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(-0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_001.poses.push_back(new_pose);

                    pose_array_001.poses[pose_array_001.poses.size() - 1] = new_pose;

                    // if (path_track_status_001 == ReadyToGoal)
                    //     curGoalIdx_001 = curGoalIdx_001 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_001.header;
                    goal.pose = new_pose;

                    goal_pub_001.publish(goal);

                    // 写入（传入 Pose）
                    writePose001(new_pose);  // 确保 writePose001 接收的是 geometry_msgs::Pose
                    
                    // permit_001 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="2"){
                if (pose_array_002.poses.size() < maxNumGoal_) {
                    if (pose_array_002.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_002.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(-0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_002.poses.push_back(new_pose);

                    pose_array_002.poses[pose_array_002.poses.size() - 1] = new_pose;

                    // if (path_track_status_002 == ReadyToGoal)
                    //     curGoalIdx_002 = curGoalIdx_002 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_002.header;
                    goal.pose = new_pose;

                    goal_pub_002.publish(goal);

                    // 写入（传入 Pose）
                    writePose002(new_pose);  // 确保 writePose002 接收的是 geometry_msgs::Pose
                    
                    // permit_002 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="3"){
                if (pose_array_003.poses.size() < maxNumGoal_) {
                    if (pose_array_003.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_003.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(-0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_003.poses.push_back(new_pose);

                    pose_array_003.poses[pose_array_003.poses.size() - 1] = new_pose;

                    // if (path_track_status_003 == ReadyToGoal)
                    //     curGoalIdx_003 = curGoalIdx_003 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_003.header;
                    goal.pose = new_pose;

                    goal_pub_003.publish(goal);

                    // 写入（传入 Pose）
                    writePose003(new_pose);  // 确保 writePose003 接收的是 geometry_msgs::Pose
                    
                    // permit_003 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="4"){
                if (pose_array_004.poses.size() < maxNumGoal_) {
                    if (pose_array_004.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_004.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(-0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_004.poses.push_back(new_pose);

                    pose_array_004.poses[pose_array_004.poses.size() - 1] = new_pose;

                    // if (path_track_status_004 == ReadyToGoal)
                    //     curGoalIdx_004 = curGoalIdx_004 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_004.header;
                    goal.pose = new_pose;

                    goal_pub_004.publish(goal);

                    // 写入（传入 Pose）
                    writePose004(new_pose);  // 确保 writePose004 接收的是 geometry_msgs::Pose
                    
                    // permit_004 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="5"){
                if (pose_array_005.poses.size() < maxNumGoal_) {
                    if (pose_array_005.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_005.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(-0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_005.poses.push_back(new_pose);

                    pose_array_005.poses[pose_array_005.poses.size() - 1] = new_pose;

                    // if (path_track_status_005 == ReadyToGoal)
                    //     curGoalIdx_005 = curGoalIdx_005 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_005.header;
                    goal.pose = new_pose;

                    goal_pub_005.publish(goal);

                    // 写入（传入 Pose）
                    writePose005(new_pose);  // 确保 writePose005 接收的是 geometry_msgs::Pose
                    
                    // permit_005 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="6"){
                if (pose_array_006.poses.size() < maxNumGoal_) {
                    if (pose_array_006.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_006.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(-0.5, 0.0, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_006.poses.push_back(new_pose);

                    pose_array_006.poses[pose_array_006.poses.size() - 1] = new_pose;

                    // if (path_track_status_006 == ReadyToGoal)
                    //     curGoalIdx_006 = curGoalIdx_006 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_006.header;
                    goal.pose = new_pose;

                    goal_pub_006.publish(goal);

                    // 写入（传入 Pose）
                    writePose006(new_pose);  // 确保 writePose006 接收的是 geometry_msgs::Pose
                    
                    // permit_006 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
        });

        connect(button_moveleft, &QPushButton::clicked, [this]() {
            if(drone_id_editor_->text()=="1"){
                if (pose_array_001.poses.size() < maxNumGoal_) {
                    if (pose_array_001.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_001.poses.back();

                    // 创建新位姿，y + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, 0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_001.poses.push_back(new_pose);

                    pose_array_001.poses[pose_array_001.poses.size() - 1] = new_pose;

                    // if (path_track_status_001 == ReadyToGoal)
                    //     curGoalIdx_001 = curGoalIdx_001 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_001.header;
                    goal.pose = new_pose;

                    goal_pub_001.publish(goal);

                    // 写入（传入 Pose）
                    writePose001(new_pose);  // 确保 writePose001 接收的是 geometry_msgs::Pose
                    
                    // permit_001 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="2"){
                if (pose_array_002.poses.size() < maxNumGoal_) {
                    if (pose_array_002.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_002.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, 0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_002.poses.push_back(new_pose);

                    pose_array_002.poses[pose_array_002.poses.size() - 1] = new_pose;

                    // if (path_track_status_002 == ReadyToGoal)
                    //     curGoalIdx_002 = curGoalIdx_002 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_002.header;
                    goal.pose = new_pose;

                    goal_pub_002.publish(goal);

                    // 写入（传入 Pose）
                    writePose002(new_pose);  // 确保 writePose002 接收的是 geometry_msgs::Pose
                    
                    // permit_002 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="3"){
                if (pose_array_003.poses.size() < maxNumGoal_) {
                    if (pose_array_003.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_003.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, 0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_003.poses.push_back(new_pose);

                    pose_array_003.poses[pose_array_003.poses.size() - 1] = new_pose;

                    // if (path_track_status_003 == ReadyToGoal)
                    //     curGoalIdx_003 = curGoalIdx_003 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_003.header;
                    goal.pose = new_pose;

                    goal_pub_003.publish(goal);

                    // 写入（传入 Pose）
                    writePose003(new_pose);  // 确保 writePose003 接收的是 geometry_msgs::Pose
                    
                    // permit_003 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="4"){
                if (pose_array_004.poses.size() < maxNumGoal_) {
                    if (pose_array_004.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_004.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, 0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_004.poses.push_back(new_pose);

                    pose_array_004.poses[pose_array_004.poses.size() - 1] = new_pose;

                    // if (path_track_status_004 == ReadyToGoal)
                    //     curGoalIdx_004 = curGoalIdx_004 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_004.header;
                    goal.pose = new_pose;

                    goal_pub_004.publish(goal);

                    // 写入（传入 Pose）
                    writePose004(new_pose);  // 确保 writePose004 接收的是 geometry_msgs::Pose
                    
                    // permit_004 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="5"){
                if (pose_array_005.poses.size() < maxNumGoal_) {
                    if (pose_array_005.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_005.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, 0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_005.poses.push_back(new_pose);

                    pose_array_005.poses[pose_array_005.poses.size() - 1] = new_pose;

                    // if (path_track_status_005 == ReadyToGoal)
                    //     curGoalIdx_005 = curGoalIdx_005 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_005.header;
                    goal.pose = new_pose;

                    goal_pub_005.publish(goal);

                    // 写入（传入 Pose）
                    writePose005(new_pose);  // 确保 writePose005 接收的是 geometry_msgs::Pose
                    
                    // permit_005 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="6"){
                if (pose_array_006.poses.size() < maxNumGoal_) {
                    if (pose_array_006.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_006.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, 0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_006.poses.push_back(new_pose);

                    pose_array_006.poses[pose_array_006.poses.size() - 1] = new_pose;

                    // if (path_track_status_006 == ReadyToGoal)
                    //     curGoalIdx_006 = curGoalIdx_006 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_006.header;
                    goal.pose = new_pose;

                    goal_pub_006.publish(goal);

                    // 写入（传入 Pose）
                    writePose006(new_pose);  // 确保 writePose006 接收的是 geometry_msgs::Pose
                    
                    // permit_006 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
        });

        connect(button_moveright, &QPushButton::clicked, [this]() {
            if(drone_id_editor_->text()=="1"){
                if (pose_array_001.poses.size() < maxNumGoal_) {
                    if (pose_array_001.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_001.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, -0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_001.poses.push_back(new_pose);

                    pose_array_001.poses[pose_array_001.poses.size() - 1] = new_pose;

                    // if (path_track_status_001 == ReadyToGoal)
                    //     curGoalIdx_001 = curGoalIdx_001 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_001.header;
                    goal.pose = new_pose;

                    goal_pub_001.publish(goal);

                    // 写入（传入 Pose）
                    writePose001(new_pose);  // 确保 writePose001 接收的是 geometry_msgs::Pose
                    
                    // permit_001 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="2"){
                if (pose_array_002.poses.size() < maxNumGoal_) {
                    if (pose_array_002.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_002.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, -0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_002.poses.push_back(new_pose);

                    pose_array_002.poses[pose_array_002.poses.size() - 1] = new_pose;

                    // if (path_track_status_002 == ReadyToGoal)
                    //     curGoalIdx_002 = curGoalIdx_002 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_002.header;
                    goal.pose = new_pose;

                    goal_pub_002.publish(goal);

                    // 写入（传入 Pose）
                    writePose002(new_pose);  // 确保 writePose002 接收的是 geometry_msgs::Pose
                    
                    // permit_002 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="3"){
                if (pose_array_003.poses.size() < maxNumGoal_) {
                    if (pose_array_003.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_003.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, -0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_003.poses.push_back(new_pose);

                    pose_array_003.poses[pose_array_003.poses.size() - 1] = new_pose;

                    // if (path_track_status_003 == ReadyToGoal)
                    //     curGoalIdx_003 = curGoalIdx_003 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_003.header;
                    goal.pose = new_pose;

                    goal_pub_003.publish(goal);

                    // 写入（传入 Pose）
                    writePose003(new_pose);  // 确保 writePose003 接收的是 geometry_msgs::Pose
                    
                    // permit_003 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="4"){
                if (pose_array_004.poses.size() < maxNumGoal_) {
                    if (pose_array_004.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_004.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, -0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_004.poses.push_back(new_pose);

                    pose_array_004.poses[pose_array_004.poses.size() - 1] = new_pose;

                    // if (path_track_status_004 == ReadyToGoal)
                    //     curGoalIdx_004 = curGoalIdx_004 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_004.header;
                    goal.pose = new_pose;

                    goal_pub_004.publish(goal);

                    // 写入（传入 Pose）
                    writePose004(new_pose);  // 确保 writePose004 接收的是 geometry_msgs::Pose
                    
                    // permit_004 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="5"){
                if (pose_array_005.poses.size() < maxNumGoal_) {
                    if (pose_array_005.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_005.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, -0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_005.poses.push_back(new_pose);

                    pose_array_005.poses[pose_array_005.poses.size() - 1] = new_pose;

                    // if (path_track_status_005 == ReadyToGoal)
                    //     curGoalIdx_005 = curGoalIdx_005 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_005.header;
                    goal.pose = new_pose;

                    goal_pub_005.publish(goal);

                    // 写入（传入 Pose）
                    writePose005(new_pose);  // 确保 writePose005 接收的是 geometry_msgs::Pose
                    
                    // permit_005 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="6"){
                if (pose_array_006.poses.size() < maxNumGoal_) {
                    if (pose_array_006.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_006.poses.back();

                    // 创建新位姿，x + 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double offset_x, offset_y;
                    computeGlobalOffset(0.0, -0.5, offset_x, offset_y);
                    new_pose.position.x += offset_x;
                    new_pose.position.y += offset_y;

                    // 添加到 PoseArray
                    // pose_array_006.poses.push_back(new_pose);

                    pose_array_006.poses[pose_array_006.poses.size() - 1] = new_pose;

                    // if (path_track_status_006 == ReadyToGoal)
                    //     curGoalIdx_006 = curGoalIdx_006 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_006.header;
                    goal.pose = new_pose;

                    goal_pub_006.publish(goal);

                    // 写入（传入 Pose）
                    writePose006(new_pose);  // 确保 writePose006 接收的是 geometry_msgs::Pose
                    
                    // permit_006 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
        });

        connect(button_turnleft, &QPushButton::clicked, [this]() {
            if(drone_id_editor_->text()=="1"){
                if (pose_array_001.poses.size() < maxNumGoal_) {
                    if (pose_array_001.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_001.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = 0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_001_roll,pos_odom_001_pitch,pos_odom_001_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_001.poses.push_back(new_pose);

                    pose_array_001.poses[pose_array_001.poses.size() - 1] = new_pose;

                    // if (path_track_status_001 == ReadyToGoal)
                    //     curGoalIdx_001 = curGoalIdx_001 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_001.header;
                    goal.pose = new_pose;

                    goal_pub_001.publish(goal);

                    // 写入（传入 Pose）
                    writePose001(new_pose);  // 确保 writePose001 接收的是 geometry_msgs::Pose
                    
                    // permit_001 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="2"){
                if (pose_array_002.poses.size() < maxNumGoal_) {
                    if (pose_array_002.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_002.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = 0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_002_roll,pos_odom_002_pitch,pos_odom_002_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_002.poses.push_back(new_pose);

                    pose_array_002.poses[pose_array_002.poses.size() - 1] = new_pose;

                    // if (path_track_status_002 == ReadyToGoal)
                    //     curGoalIdx_002 = curGoalIdx_002 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_002.header;
                    goal.pose = new_pose;

                    goal_pub_002.publish(goal);

                    // 写入（传入 Pose）
                    writePose002(new_pose);  // 确保 writePose002 接收的是 geometry_msgs::Pose
                    
                    // permit_002 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="3"){
                if (pose_array_003.poses.size() < maxNumGoal_) {
                    if (pose_array_003.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_003.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = 0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_003_roll,pos_odom_003_pitch,pos_odom_003_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_003.poses.push_back(new_pose);

                    pose_array_003.poses[pose_array_003.poses.size() - 1] = new_pose;

                    // if (path_track_status_003 == ReadyToGoal)
                    //     curGoalIdx_003 = curGoalIdx_003 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_003.header;
                    goal.pose = new_pose;

                    goal_pub_003.publish(goal);

                    // 写入（传入 Pose）
                    writePose003(new_pose);  // 确保 writePose003 接收的是 geometry_msgs::Pose
                    
                    // permit_003 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="4"){
                if (pose_array_004.poses.size() < maxNumGoal_) {
                    if (pose_array_004.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_004.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = 0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_004_roll,pos_odom_004_pitch,pos_odom_004_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_004.poses.push_back(new_pose);

                    pose_array_004.poses[pose_array_004.poses.size() - 1] = new_pose;

                    // if (path_track_status_004 == ReadyToGoal)
                    //     curGoalIdx_004 = curGoalIdx_004 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_004.header;
                    goal.pose = new_pose;

                    goal_pub_004.publish(goal);

                    // 写入（传入 Pose）
                    writePose004(new_pose);  // 确保 writePose004 接收的是 geometry_msgs::Pose
                    
                    // permit_004 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="5"){
                if (pose_array_005.poses.size() < maxNumGoal_) {
                    if (pose_array_005.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_005.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = 0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_005_roll,pos_odom_005_pitch,pos_odom_005_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_005.poses.push_back(new_pose);

                    pose_array_005.poses[pose_array_005.poses.size() - 1] = new_pose;

                    // if (path_track_status_005 == ReadyToGoal)
                    //     curGoalIdx_005 = curGoalIdx_005 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_005.header;
                    goal.pose = new_pose;

                    goal_pub_005.publish(goal);

                    // 写入（传入 Pose）
                    writePose005(new_pose);  // 确保 writePose005 接收的是 geometry_msgs::Pose
                    
                    // permit_005 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="6"){
                if (pose_array_006.poses.size() < maxNumGoal_) {
                    if (pose_array_006.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_006.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = 0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_006_roll,pos_odom_006_pitch,pos_odom_006_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_006.poses.push_back(new_pose);

                    pose_array_006.poses[pose_array_006.poses.size() - 1] = new_pose;

                    // if (path_track_status_006 == ReadyToGoal)
                    //     curGoalIdx_006 = curGoalIdx_006 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_006.header;
                    goal.pose = new_pose;

                    goal_pub_006.publish(goal);

                    // 写入（传入 Pose）
                    writePose006(new_pose);  // 确保 writePose006 接收的是 geometry_msgs::Pose
                    
                    // permit_006 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
        });

        connect(button_turnright, &QPushButton::clicked, [this]() {
            if(drone_id_editor_->text()=="1"){
                if (pose_array_001.poses.size() < maxNumGoal_) {
                    if (pose_array_001.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_001.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = -0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_001_roll,pos_odom_001_pitch,pos_odom_001_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_001.poses.push_back(new_pose);

                    pose_array_001.poses[pose_array_001.poses.size() - 1] = new_pose;

                    // if (path_track_status_001 == ReadyToGoal)
                    //     curGoalIdx_001 = curGoalIdx_001 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_001.header;
                    goal.pose = new_pose;

                    goal_pub_001.publish(goal);

                    // 写入（传入 Pose）
                    writePose001(new_pose);  // 确保 writePose001 接收的是 geometry_msgs::Pose
                    
                    // permit_001 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="2"){
                if (pose_array_002.poses.size() < maxNumGoal_) {
                    if (pose_array_002.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_002.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = -0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_002_roll,pos_odom_002_pitch,pos_odom_002_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_002.poses.push_back(new_pose);

                    pose_array_002.poses[pose_array_002.poses.size() - 1] = new_pose;

                    // if (path_track_status_002 == ReadyToGoal)
                    //     curGoalIdx_002 = curGoalIdx_002 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_002.header;
                    goal.pose = new_pose;

                    goal_pub_002.publish(goal);

                    // 写入（传入 Pose）
                    writePose002(new_pose);  // 确保 writePose002 接收的是 geometry_msgs::Pose
                    
                    // permit_002 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="3"){
                if (pose_array_003.poses.size() < maxNumGoal_) {
                    if (pose_array_003.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_003.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = -0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_003_roll,pos_odom_003_pitch,pos_odom_003_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_003.poses.push_back(new_pose);

                    pose_array_003.poses[pose_array_003.poses.size() - 1] = new_pose;

                    // if (path_track_status_003 == ReadyToGoal)
                    //     curGoalIdx_003 = curGoalIdx_003 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_003.header;
                    goal.pose = new_pose;

                    goal_pub_003.publish(goal);

                    // 写入（传入 Pose）
                    writePose003(new_pose);  // 确保 writePose003 接收的是 geometry_msgs::Pose
                    
                    // permit_003 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="4"){
                if (pose_array_004.poses.size() < maxNumGoal_) {
                    if (pose_array_004.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_004.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = -0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_004_roll,pos_odom_004_pitch,pos_odom_004_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_004.poses.push_back(new_pose);

                    pose_array_004.poses[pose_array_004.poses.size() - 1] = new_pose;

                    // if (path_track_status_004 == ReadyToGoal)
                    //     curGoalIdx_004 = curGoalIdx_004 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_004.header;
                    goal.pose = new_pose;

                    goal_pub_004.publish(goal);

                    // 写入（传入 Pose）
                    writePose004(new_pose);  // 确保 writePose004 接收的是 geometry_msgs::Pose
                    
                    // permit_004 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="5"){
                if (pose_array_005.poses.size() < maxNumGoal_) {
                    if (pose_array_005.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_005.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = -0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_005_roll,pos_odom_005_pitch,pos_odom_005_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_005.poses.push_back(new_pose);

                    pose_array_005.poses[pose_array_005.poses.size() - 1] = new_pose;

                    // if (path_track_status_005 == ReadyToGoal)
                    //     curGoalIdx_005 = curGoalIdx_005 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_005.header;
                    goal.pose = new_pose;

                    goal_pub_005.publish(goal);

                    // 写入（传入 Pose）
                    writePose005(new_pose);  // 确保 writePose005 接收的是 geometry_msgs::Pose
                    
                    // permit_005 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
            if(drone_id_editor_->text()=="6"){
                if (pose_array_006.poses.size() < maxNumGoal_) {
                    if (pose_array_006.poses.empty()) {
                        ROS_WARN("No initial goal pose for drone 1. Please set a starting pose first.");
                        return; // 或弹出提示框
                    }
                    // 获取最后一个 Pose（注意：是 geometry_msgs::Pose）
                    geometry_msgs::Pose last_pose = pose_array_006.poses.back();

                    // 创建新位姿，y - 0.5
                    geometry_msgs::Pose new_pose = last_pose;
                    double delta_yaw = -0.3;
                    float quaternion[4];
                    mavlink_euler_to_quaternion(pos_odom_006_roll,pos_odom_006_pitch,pos_odom_006_yaw + delta_yaw,quaternion);

                    new_pose.orientation.w = quaternion[0];
                    new_pose.orientation.x = quaternion[1];
                    new_pose.orientation.y = quaternion[2];
                    new_pose.orientation.z = quaternion[3];

                    // 添加到 PoseArray
                    // pose_array_006.poses.push_back(new_pose);

                    pose_array_006.poses[pose_array_006.poses.size() - 1] = new_pose;

                    // if (path_track_status_006 == ReadyToGoal)
                    //     curGoalIdx_006 = curGoalIdx_006 - 1;

                    geometry_msgs::PoseStamped goal;

                    goal.header = pose_array_006.header;
                    goal.pose = new_pose;

                    goal_pub_006.publish(goal);

                    // 写入（传入 Pose）
                    writePose006(new_pose);  // 确保 writePose006 接收的是 geometry_msgs::Pose
                    
                    // permit_006 = true;
                } else {
                    ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
                }
            }
        });

        QHBoxLayout *wall_layout = new QHBoxLayout;
        wall_layout->addWidget(new QLabel("虚拟围墙(m)"));
        wall_startpointx_editor_ = new QLineEdit;
        wall_startpointx_editor_->setText("0");
        wall_startpointx_editor_->setPlaceholderText("x1");
        wall_startpointy_editor_ = new QLineEdit;
        wall_startpointy_editor_->setText("0");
        wall_startpointy_editor_->setPlaceholderText("y1");
        wall_layout->addWidget(wall_startpointx_editor_);
        wall_layout->addWidget(wall_startpointy_editor_);
        wall_endpointx_editor_ = new QLineEdit;
        wall_endpointx_editor_->setText("5.7");
        wall_endpointx_editor_->setPlaceholderText("x2");
        wall_endpointy_editor_ = new QLineEdit;
        wall_endpointy_editor_->setText("-11.4");
        wall_endpointy_editor_->setPlaceholderText("y2");
        wall_layout->addWidget(wall_endpointx_editor_);
        wall_layout->addWidget(wall_endpointy_editor_);
        root_layout->addLayout(wall_layout);

        QHBoxLayout *drone_layout = new QHBoxLayout;
        drone_layout->addWidget(new QLabel("当前目标飞机id"));
        drone_id_editor_ = new QLineEdit;
        drone_id_editor_->setText("1");
        drone_layout->addWidget(drone_id_editor_);
        root_layout->addLayout(drone_layout);

        // create a panel about "maxNumGoal"
        QHBoxLayout *maxNumGoal_layout = new QHBoxLayout;
        maxNumGoal_layout->addWidget(new QLabel("目标最大数量"));
        output_maxNumGoal_editor_ = new QLineEdit;
        output_maxNumGoal_editor_->setText("10");
        maxNumGoal_layout->addWidget(output_maxNumGoal_editor_);
        output_maxNumGoal_button_ = new QPushButton("确定");
        maxNumGoal_layout->addWidget(output_maxNumGoal_button_);
        root_layout->addLayout(maxNumGoal_layout);

        QHBoxLayout *position_threshold_layout = new QHBoxLayout;
        position_threshold_layout->addWidget(new QLabel("允许最大偏差阈值(m)"));
        position_threshold_editor_ = new QLineEdit;
        position_threshold_editor_->setText("0.1");
        position_threshold_layout->addWidget(position_threshold_editor_);
        root_layout->addLayout(position_threshold_layout);

        QHBoxLayout *default_z_layout = new QHBoxLayout;
        default_z_layout->addWidget(new QLabel("默认z高度(m)"));
        default_z_editor = new QLineEdit;
        default_z_editor->setText("1");
        default_z_layout->addWidget(default_z_editor);
        root_layout->addLayout(default_z_layout);

        cycle_checkbox_001 = new QCheckBox("1号机轨迹点循环");
        // creat a QTable to contain the poseArray
        poseArray_table_001 = new MyTableWidget;
        poseArray_table_001->setEditTriggers(QAbstractItemView::AnyKeyPressed | QAbstractItemView::DoubleClicked); 

        cycle_checkbox_002 = new QCheckBox("2号机轨迹点循环");
        // creat a QTable to contain the poseArray
        poseArray_table_002 = new MyTableWidget;
        poseArray_table_002->setEditTriggers(QAbstractItemView::AnyKeyPressed | QAbstractItemView::DoubleClicked);

        cycle_checkbox_003 = new QCheckBox("3号机轨迹点循环");
        // creat a QTable to contain the poseArray
        poseArray_table_003 = new MyTableWidget;
        poseArray_table_003->setEditTriggers(QAbstractItemView::AnyKeyPressed | QAbstractItemView::DoubleClicked); 

        cycle_checkbox_004 = new QCheckBox("4号机轨迹点循环");
        // creat a QTable to contain the poseArray
        poseArray_table_004 = new MyTableWidget;
        poseArray_table_004->setEditTriggers(QAbstractItemView::AnyKeyPressed | QAbstractItemView::DoubleClicked); 

        cycle_checkbox_005 = new QCheckBox("5号机轨迹点循环");
        // creat a QTable to contain the poseArray
        poseArray_table_005 = new MyTableWidget;
        poseArray_table_005->setEditTriggers(QAbstractItemView::AnyKeyPressed | QAbstractItemView::DoubleClicked); 

        cycle_checkbox_006 = new QCheckBox("6号机轨迹点循环");
        // creat a QTable to contain the poseArray
        poseArray_table_006 = new MyTableWidget;
        poseArray_table_006->setEditTriggers(QAbstractItemView::AnyKeyPressed | QAbstractItemView::DoubleClicked); 

        initPoseTable();
        root_layout->addWidget(cycle_checkbox_001);
        root_layout->addWidget(poseArray_table_001);
        root_layout->addWidget(cycle_checkbox_002);
        root_layout->addWidget(poseArray_table_002);
        root_layout->addWidget(cycle_checkbox_003);
        root_layout->addWidget(poseArray_table_003);
        root_layout->addWidget(cycle_checkbox_004);
        root_layout->addWidget(poseArray_table_004);
        root_layout->addWidget(cycle_checkbox_005);
        root_layout->addWidget(poseArray_table_005);
        root_layout->addWidget(cycle_checkbox_006);
        root_layout->addWidget(poseArray_table_006);

        //create a manipulate layout
        QHBoxLayout *manipulate_layout = new QHBoxLayout;
        output_reset_button_ = new QPushButton("清除全部目标点");
        manipulate_layout->addWidget(output_reset_button_);
        output_delete_button_ = new QPushButton("清除单个目标点");
        manipulate_layout->addWidget(output_delete_button_);
        output_startNavi_button_ = new QPushButton("执行轨迹");
        manipulate_layout->addWidget(output_startNavi_button_);
        output_stopNavi_button_ = new QPushButton("停止轨迹");
        manipulate_layout->addWidget(output_stopNavi_button_);
        root_layout->addLayout(manipulate_layout);

        setLayout(root_layout);
        // set a Qtimer to start a spin for subscriptions
        QTimer *output_timer = new QTimer(this);
        output_timer->start(200);

        // 设置信号与槽的连接
        connect(output_maxNumGoal_button_, SIGNAL(clicked()), this,
                SLOT(updateMaxNumGoal()));
        connect(output_maxNumGoal_button_, SIGNAL(clicked()), this,
                SLOT(updatePoseTable()));
        connect(output_reset_button_, SIGNAL(clicked()), this, SLOT(initPoseTable()));
        connect(output_delete_button_, SIGNAL(clicked()), this, SLOT(deleteGoalPoint()));
        connect(output_startNavi_button_, SIGNAL(clicked()), this, SLOT(NaviControl()));
        connect(output_stopNavi_button_, SIGNAL(clicked()), this, SLOT(StopControl()));
        connect(cycle_checkbox_001, SIGNAL(clicked(bool)), this, SLOT(checkCycle001()));
        connect(cycle_checkbox_002, SIGNAL(clicked(bool)), this, SLOT(checkCycle002()));
        connect(cycle_checkbox_003, SIGNAL(clicked(bool)), this, SLOT(checkCycle003()));
        connect(cycle_checkbox_004, SIGNAL(clicked(bool)), this, SLOT(checkCycle004()));
        connect(cycle_checkbox_005, SIGNAL(clicked(bool)), this, SLOT(checkCycle005()));
        connect(cycle_checkbox_006, SIGNAL(clicked(bool)), this, SLOT(checkCycle006()));
        connect(output_timer, SIGNAL(timeout()), this, SLOT(startSpin()));
    }

    void MultiNaviGoalsPanel::updateWall() {
        markWall(wall_startpointx_editor_->text(),wall_startpointy_editor_->text(),wall_endpointx_editor_->text(),wall_endpointy_editor_->text());
    }

    void MultiNaviGoalsPanel::updateMaxNumGoal() {
        setMaxNumGoal(output_maxNumGoal_editor_->text());
    }

// set up the maximum number of goals
    void MultiNaviGoalsPanel::setMaxNumGoal(const QString &new_maxNumGoal) {
        // 检查maxNumGoal是否发生改变.
        if (new_maxNumGoal != output_maxNumGoal_) {
            output_maxNumGoal_ = new_maxNumGoal;

            // 如果命名为空，不发布任何信息
            if (output_maxNumGoal_ == "") {
                nh_.setParam("maxNumGoal_", 1);
                maxNumGoal_ = 1;
            } else {
//                velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>(output_maxNumGoal_.toStdString(), 1);
                nh_.setParam("maxNumGoal_", output_maxNumGoal_.toInt());
                maxNumGoal_ = output_maxNumGoal_.toInt();
            }
            Q_EMIT configChanged();
        }
    }

    void MultiNaviGoalsPanel::computeGlobalOffset(double dx, double dy, double& out_dx, double& out_dy) {
        out_dx = dx * std::cos(pos_odom_001_yaw) - dy * std::sin(pos_odom_001_yaw);
        out_dy = dx * std::sin(pos_odom_001_yaw) + dy * std::cos(pos_odom_001_yaw);
    }

    // initialize the table of pose
    void MultiNaviGoalsPanel::initPoseTable() {
        deleteAllMark();
        ROS_INFO("Initialize");
        curGoalIdx_001 = 0, cycleCnt_001 = 0;
        curGoalIdx_002 = 0, cycleCnt_002 = 0;
        curGoalIdx_003 = 0, cycleCnt_003 = 0;
        curGoalIdx_004 = 0, cycleCnt_004 = 0;
        curGoalIdx_005 = 0, cycleCnt_005 = 0;
        curGoalIdx_006 = 0, cycleCnt_006 = 0;
        permit_001 = false, cycle_001 = false;
        permit_002 = false, cycle_002 = false;
        permit_003 = false, cycle_003 = false;
        permit_004 = false, cycle_004 = false;
        permit_005 = false, cycle_005 = false;
        permit_006 = false, cycle_006 = false;
        QStringList pose_header;
        pose_header << "x(m)" << "y(m)" << "z(m)" << "yaw(deg)";
        pose_array_001.poses.clear();
        poseArray_table_001->clear();
        poseArray_table_001->setRowCount(maxNumGoal_);
        poseArray_table_001->setColumnCount(4);
        poseArray_table_001->setEditTriggers(QAbstractItemView::NoEditTriggers);
        poseArray_table_001->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        poseArray_table_001->setHorizontalHeaderLabels(pose_header);
        cycle_checkbox_001->setCheckState(Qt::Unchecked);

        pose_array_002.poses.clear();
        poseArray_table_002->clear();
        poseArray_table_002->setRowCount(maxNumGoal_);
        poseArray_table_002->setColumnCount(4);
        poseArray_table_002->setEditTriggers(QAbstractItemView::NoEditTriggers);
        poseArray_table_002->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        poseArray_table_002->setHorizontalHeaderLabels(pose_header);
        cycle_checkbox_002->setCheckState(Qt::Unchecked);

        pose_array_003.poses.clear();
        poseArray_table_003->clear();
        poseArray_table_003->setRowCount(maxNumGoal_);
        poseArray_table_003->setColumnCount(4);
        poseArray_table_003->setEditTriggers(QAbstractItemView::NoEditTriggers);
        poseArray_table_003->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        poseArray_table_003->setHorizontalHeaderLabels(pose_header);
        cycle_checkbox_003->setCheckState(Qt::Unchecked);

        pose_array_004.poses.clear();
        poseArray_table_004->clear();
        poseArray_table_004->setRowCount(maxNumGoal_);
        poseArray_table_004->setColumnCount(4);
        poseArray_table_004->setEditTriggers(QAbstractItemView::NoEditTriggers);
        poseArray_table_004->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        poseArray_table_004->setHorizontalHeaderLabels(pose_header);
        cycle_checkbox_004->setCheckState(Qt::Unchecked);

        pose_array_005.poses.clear();
        poseArray_table_005->clear();
        poseArray_table_005->setRowCount(maxNumGoal_);
        poseArray_table_005->setColumnCount(4);
        poseArray_table_005->setEditTriggers(QAbstractItemView::NoEditTriggers);
        poseArray_table_005->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        poseArray_table_005->setHorizontalHeaderLabels(pose_header);
        cycle_checkbox_005->setCheckState(Qt::Unchecked);

        pose_array_006.poses.clear();
        poseArray_table_006->clear();
        poseArray_table_006->setRowCount(maxNumGoal_);
        poseArray_table_006->setColumnCount(4);
        poseArray_table_006->setEditTriggers(QAbstractItemView::NoEditTriggers);
        poseArray_table_006->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        poseArray_table_006->setHorizontalHeaderLabels(pose_header);
        cycle_checkbox_006->setCheckState(Qt::Unchecked);
    }

    void MultiNaviGoalsPanel::deleteAllMark() {
        for(int i=0; i<maxNumGoal_; i++){
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_001";
            marker_delete.id=i;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_001.publish(marker_delete);
            marker_delete.ns="navi_point_number_001";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_001.publish(marker_delete);
        }
        for(int i=0; i<maxNumGoal_; i++){
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_002";
            marker_delete.id=i;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_002.publish(marker_delete);
            marker_delete.ns="navi_point_number_002";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_002.publish(marker_delete);
        }
        for(int i=0; i<maxNumGoal_; i++){
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_003";
            marker_delete.id=i;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_003.publish(marker_delete);
            marker_delete.ns="navi_point_number_003";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_002.publish(marker_delete);
        }
        for(int i=0; i<maxNumGoal_; i++){
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_004";
            marker_delete.id=i;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_004.publish(marker_delete);
            marker_delete.ns="navi_point_number_004";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_004.publish(marker_delete);
        }
        for(int i=0; i<maxNumGoal_; i++){
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_005";
            marker_delete.id=i;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_005.publish(marker_delete);
            marker_delete.ns="navi_point_number_005";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_005.publish(marker_delete);
        }
        for(int i=0; i<maxNumGoal_; i++){
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_006";
            marker_delete.id=i;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_006.publish(marker_delete);
            marker_delete.ns="navi_point_number_006";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_006.publish(marker_delete);
        }
    }
    
    // delete marks in the map
    void MultiNaviGoalsPanel::deleteMark() {
        if (!pose_array_001.poses.empty()) {
            for(int i=pose_array_001.poses.size(); i<=maxNumGoal_; i++){
                visualization_msgs::Marker marker_delete;
                marker_delete.ns="navi_point_arrow_001";
                marker_delete.id=i;
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_001.publish(marker_delete);
                marker_delete.ns="navi_point_number_001";
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_001.publish(marker_delete);
            }
        }
        if (!pose_array_002.poses.empty()) {
            for(int i=pose_array_002.poses.size(); i<=maxNumGoal_; i++){
                visualization_msgs::Marker marker_delete;
                marker_delete.ns="navi_point_arrow_002";
                marker_delete.id=i;
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_002.publish(marker_delete);
                marker_delete.ns="navi_point_number_002";
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_002.publish(marker_delete);
            }
        }
        if (!pose_array_003.poses.empty()) {
            for(int i=pose_array_003.poses.size(); i<=maxNumGoal_; i++){
                visualization_msgs::Marker marker_delete;
                marker_delete.ns="navi_point_arrow_003";
                marker_delete.id=i;
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_003.publish(marker_delete);
                marker_delete.ns="navi_point_number_003";
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_002.publish(marker_delete);
            }
        }
        if (!pose_array_004.poses.empty()) {
            for(int i=pose_array_004.poses.size(); i<=maxNumGoal_; i++){
                visualization_msgs::Marker marker_delete;
                marker_delete.ns="navi_point_arrow_004";
                marker_delete.id=i;
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_004.publish(marker_delete);
                marker_delete.ns="navi_point_number_004";
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_004.publish(marker_delete);
            }
        }
        if (!pose_array_005.poses.empty()) {
            for(int i=pose_array_005.poses.size(); i<=maxNumGoal_; i++){
                visualization_msgs::Marker marker_delete;
                marker_delete.ns="navi_point_arrow_005";
                marker_delete.id=i;
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_005.publish(marker_delete);
                marker_delete.ns="navi_point_number_005";
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_005.publish(marker_delete);
            }
        }
        if (!pose_array_006.poses.empty()) {
            for(int i=pose_array_006.poses.size(); i<=maxNumGoal_; i++){
                visualization_msgs::Marker marker_delete;
                marker_delete.ns="navi_point_arrow_006";
                marker_delete.id=i;
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_006.publish(marker_delete);
                marker_delete.ns="navi_point_number_006";
                marker_delete.action = visualization_msgs::Marker::DELETE;
                marker_pub_006.publish(marker_delete);
            }
        }
    }

    //update the table of pose
    void MultiNaviGoalsPanel::updatePoseTable() {
        QStringList pose_header;
        pose_header << "x" << "y" << "z" << "yaw";
        poseArray_table_001->setRowCount(maxNumGoal_);
        poseArray_table_001->setHorizontalHeaderLabels(pose_header);
        poseArray_table_001->show();

        poseArray_table_002->setRowCount(maxNumGoal_);
        poseArray_table_002->setHorizontalHeaderLabels(pose_header);
        poseArray_table_002->show();

        poseArray_table_003->setRowCount(maxNumGoal_);
        poseArray_table_003->setHorizontalHeaderLabels(pose_header);
        poseArray_table_003->show();

        poseArray_table_004->setRowCount(maxNumGoal_);
        poseArray_table_004->setHorizontalHeaderLabels(pose_header);
        poseArray_table_004->show();

        poseArray_table_005->setRowCount(maxNumGoal_);
        poseArray_table_005->setHorizontalHeaderLabels(pose_header);
        poseArray_table_005->show();

        poseArray_table_006->setRowCount(maxNumGoal_);
        poseArray_table_006->setHorizontalHeaderLabels(pose_header);
        poseArray_table_006->show();
    }

    // call back function for counting goals
    void MultiNaviGoalsPanel::goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose) {
        if(drone_id_editor_->text()=="1"){
            if (pose_array_001.poses.size() < maxNumGoal_) {
                pose_array_001.poses.push_back(pose->pose);
                pose_array_001.poses[pose_array_001.poses.size() - 1].position.z = default_z_editor->text().toDouble();
                pose_array_001.header.frame_id = pose->header.frame_id;
                writePose001(pose->pose);
            } else {
                ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
            }
        }else if(drone_id_editor_->text()=="2"){
            if (pose_array_002.poses.size() < maxNumGoal_) {
                pose_array_002.poses.push_back(pose->pose);
                pose_array_002.poses[pose_array_002.poses.size() - 1].position.z = default_z_editor->text().toDouble();
                pose_array_002.header.frame_id = pose->header.frame_id;
                writePose002(pose->pose);
            } else {
                ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
            }
        }else if(drone_id_editor_->text()=="3"){
            if (pose_array_003.poses.size() < maxNumGoal_) {
                pose_array_003.poses.push_back(pose->pose);
                pose_array_003.poses[pose_array_003.poses.size() - 1].position.z = default_z_editor->text().toDouble();
                pose_array_003.header.frame_id = pose->header.frame_id;
                writePose003(pose->pose);
            } else {
                ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
            }
        }else if(drone_id_editor_->text()=="4"){
            if (pose_array_004.poses.size() < maxNumGoal_) {
                pose_array_004.poses.push_back(pose->pose);
                pose_array_004.poses[pose_array_004.poses.size() - 1].position.z = default_z_editor->text().toDouble();
                pose_array_004.header.frame_id = pose->header.frame_id;
                writePose004(pose->pose);
            } else {
                ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
            }
        }else if(drone_id_editor_->text()=="5"){
            if (pose_array_005.poses.size() < maxNumGoal_) {
                pose_array_005.poses.push_back(pose->pose);
                pose_array_005.poses[pose_array_005.poses.size() - 1].position.z = default_z_editor->text().toDouble();
                pose_array_005.header.frame_id = pose->header.frame_id;
                writePose005(pose->pose);
            } else {
                ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
            }
        }else if(drone_id_editor_->text()=="6"){
            if (pose_array_006.poses.size() < maxNumGoal_) {
                pose_array_006.poses.push_back(pose->pose);
                pose_array_006.poses[pose_array_006.poses.size() - 1].position.z = default_z_editor->text().toDouble();
                pose_array_006.header.frame_id = pose->header.frame_id;
                writePose006(pose->pose);
            } else {
                ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
            }
        }
    }

    // write the poses into the table
    void MultiNaviGoalsPanel::writePose001(geometry_msgs::Pose pose) {
        poseArray_table_001->setItem(pose_array_001.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_001->setItem(pose_array_001.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_001->setItem(pose_array_001.poses.size() - 1, 2,
                                new QTableWidgetItem(QString::number(default_z_editor->text().toDouble(), 'f', 2)));
        poseArray_table_001->setItem(pose_array_001.poses.size() - 1, 3,
                                new QTableWidgetItem(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    void MultiNaviGoalsPanel::writePose002(geometry_msgs::Pose pose) {
        poseArray_table_002->setItem(pose_array_002.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_002->setItem(pose_array_002.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_002->setItem(pose_array_002.poses.size() - 1, 2,
                                new QTableWidgetItem(QString::number(default_z_editor->text().toDouble(), 'f', 2)));
        poseArray_table_002->setItem(pose_array_002.poses.size() - 1, 3,
                                new QTableWidgetItem(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    void MultiNaviGoalsPanel::writePose003(geometry_msgs::Pose pose) {
        poseArray_table_003->setItem(pose_array_003.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_003->setItem(pose_array_003.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_003->setItem(pose_array_003.poses.size() - 1, 2,
                                new QTableWidgetItem(QString::number(default_z_editor->text().toDouble(), 'f', 2)));
        poseArray_table_003->setItem(pose_array_003.poses.size() - 1, 3,
                                new QTableWidgetItem(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    void MultiNaviGoalsPanel::writePose004(geometry_msgs::Pose pose) {
        poseArray_table_004->setItem(pose_array_004.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_004->setItem(pose_array_004.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_004->setItem(pose_array_004.poses.size() - 1, 2,
                                new QTableWidgetItem(QString::number(default_z_editor->text().toDouble(), 'f', 2)));
        poseArray_table_004->setItem(pose_array_004.poses.size() - 1, 3,
                                new QTableWidgetItem(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    void MultiNaviGoalsPanel::writePose005(geometry_msgs::Pose pose) {
        poseArray_table_005->setItem(pose_array_005.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_005->setItem(pose_array_005.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_005->setItem(pose_array_005.poses.size() - 1, 2,
                                new QTableWidgetItem(QString::number(default_z_editor->text().toDouble(), 'f', 2)));
        poseArray_table_005->setItem(pose_array_005.poses.size() - 1, 3,
                                new QTableWidgetItem(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    void MultiNaviGoalsPanel::writePose006(geometry_msgs::Pose pose) {
        poseArray_table_006->setItem(pose_array_006.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_006->setItem(pose_array_006.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_006->setItem(pose_array_006.poses.size() - 1, 2,
                                new QTableWidgetItem(QString::number(default_z_editor->text().toDouble(), 'f', 2)));
        poseArray_table_006->setItem(pose_array_006.poses.size() - 1, 3,
                                new QTableWidgetItem(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    // when setting a Navi Goal, it will set a mark on the map
    void MultiNaviGoalsPanel::markPose() {
        visualization_msgs::Marker arrow;
        visualization_msgs::Marker number;
        arrow.header.frame_id = number.header.frame_id = "map";
        arrow.action = number.action = visualization_msgs::Marker::ADD;
        arrow.type = visualization_msgs::Marker::SPHERE;
        number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        arrow.scale.x = 0.1;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        number.scale.z = 0.25;
        if(pose_array_001.poses.size()>0){
            for(int i=0; i<pose_array_001.poses.size(); i++){
                arrow.pose = number.pose = pose_array_001.poses[i];
                arrow.pose.position.z = 0;
                number.pose.position.z = 0;
                number.pose.position.z += 0.5;
                arrow.ns = "navi_point_arrow_001";
                number.ns = "navi_point_number_001";
                arrow.color.r = number.color.r = 1.0f;
                arrow.color.g = number.color.g = 0.2f;
                arrow.color.b = number.color.b = 0.2f;
                arrow.color.a = number.color.a = 1.0f;
                arrow.id = number.id = i;
                number.text = std::to_string(i+1);
                marker_pub_001.publish(arrow);
                marker_pub_001.publish(number);
            }
        }
        if(pose_array_002.poses.size()>0){
            for(int i=0; i<pose_array_002.poses.size(); i++){
                arrow.pose = number.pose = pose_array_002.poses[i];
                arrow.pose.position.z = 0;
                number.pose.position.z = 0;
                number.pose.position.z += 0.5;
                arrow.ns = "navi_point_arrow_002";
                number.ns = "navi_point_number_002";
                arrow.color.r = number.color.r = 1.0f;
                arrow.color.g = number.color.g = 0.5f;
                arrow.color.b = number.color.b = 1.0f;
                arrow.color.a = number.color.a = 1.0f;
                arrow.id = number.id = i;
                number.text = std::to_string(i+1);
                marker_pub_002.publish(arrow);
                marker_pub_002.publish(number);
            }
        }
        if(pose_array_003.poses.size()>0){
            for(int i=0; i<pose_array_003.poses.size(); i++){
                arrow.pose = number.pose = pose_array_003.poses[i];
                arrow.pose.position.z = 0;
                number.pose.position.z = 0;
                number.pose.position.z += 0.5;
                arrow.ns = "navi_point_arrow_003";
                number.ns = "navi_point_number_003";
                arrow.color.r = number.color.r = 0.5f;
                arrow.color.g = number.color.g = 1.0f;
                arrow.color.b = number.color.b = 0.5f;
                arrow.color.a = number.color.a = 1.0f;
                arrow.id = number.id = i;
                number.text = std::to_string(i+1);
                marker_pub_003.publish(arrow);
                marker_pub_003.publish(number);
            }
        }
        if(pose_array_004.poses.size()>0){
            for(int i=0; i<pose_array_004.poses.size(); i++){
                arrow.pose = number.pose = pose_array_004.poses[i];
                arrow.pose.position.z = 0;
                number.pose.position.z = 0;
                number.pose.position.z += 0.5;
                arrow.ns = "navi_point_arrow_004";
                number.ns = "navi_point_number_004";
                arrow.color.r = number.color.r = 0.5f;
                arrow.color.g = number.color.g = 0.5f;
                arrow.color.b = number.color.b = 1.0f;
                arrow.color.a = number.color.a = 1.0f;
                arrow.id = number.id = i;
                number.text = std::to_string(i+1);
                marker_pub_004.publish(arrow);
                marker_pub_004.publish(number);
            }
        }
        if(pose_array_005.poses.size()>0){
            for(int i=0; i<pose_array_005.poses.size(); i++){
                arrow.pose = number.pose = pose_array_005.poses[i];
                arrow.pose.position.z = 0;
                number.pose.position.z = 0;
                number.pose.position.z += 0.5;
                arrow.ns = "navi_point_arrow_005";
                number.ns = "navi_point_number_005";
                arrow.color.r = number.color.r = 0.2f;
                arrow.color.g = number.color.g = 0.2f;
                arrow.color.b = number.color.b = 0.2f;
                arrow.color.a = number.color.a = 1.0f;
                arrow.id = number.id = i;
                number.text = std::to_string(i+1);
                marker_pub_005.publish(arrow);
                marker_pub_005.publish(number);
            }
        }
        if(pose_array_006.poses.size()>0){
            for(int i=0; i<pose_array_006.poses.size(); i++){
                arrow.pose = number.pose = pose_array_006.poses[i];
                arrow.pose.position.z = 0;
                number.pose.position.z = 0;
                number.pose.position.z += 0.5;
                arrow.ns = "navi_point_arrow_006";
                number.ns = "navi_point_number_006";
                arrow.color.r = number.color.r = 0.2f;
                arrow.color.g = number.color.g = 1.0f;
                arrow.color.b = number.color.b = 1.0f;
                arrow.color.a = number.color.a = 1.0f;
                arrow.id = number.id = i;
                number.text = std::to_string(i+1);
                marker_pub_006.publish(arrow);
                marker_pub_006.publish(number);
            }
        }
    }

    // check whether it is in the cycling situation
    void MultiNaviGoalsPanel::checkCycle001() {
        cycle_001 = cycle_checkbox_001->isChecked();
    }

    void MultiNaviGoalsPanel::checkCycle002() {
        cycle_002 = cycle_checkbox_002->isChecked();
    }

    void MultiNaviGoalsPanel::checkCycle003() {
        cycle_003 = cycle_checkbox_003->isChecked();
    }

    void MultiNaviGoalsPanel::checkCycle004() {
        cycle_004 = cycle_checkbox_004->isChecked();
    }

    void MultiNaviGoalsPanel::checkCycle005() {
        cycle_005 = cycle_checkbox_005->isChecked();
    }

    void MultiNaviGoalsPanel::checkCycle006() {
        cycle_006 = cycle_checkbox_006->isChecked();
    }

    void MultiNaviGoalsPanel::odom_global001_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        pos_odom_001_x=(float)odom->pose.pose.position.x;//位置点为FLU坐标
        pos_odom_001_y=(float)odom->pose.pose.position.y;
        pos_odom_001_z=(float)odom->pose.pose.position.z;
        float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
        mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_001_roll, &pos_odom_001_pitch, &pos_odom_001_yaw);
    }

    void MultiNaviGoalsPanel::odom_global002_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        pos_odom_002_x=(float)odom->pose.pose.position.x;//位置点为FLU坐标
        pos_odom_002_y=(float)odom->pose.pose.position.y;
        pos_odom_002_z=(float)odom->pose.pose.position.z;
        float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
        mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_002_roll, &pos_odom_002_pitch, &pos_odom_002_yaw);
    }

    void MultiNaviGoalsPanel::odom_global003_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        pos_odom_003_x=(float)odom->pose.pose.position.x;//位置点为FLU坐标
        pos_odom_003_y=(float)odom->pose.pose.position.y;
        pos_odom_003_z=(float)odom->pose.pose.position.z;
        float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
        mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_003_roll, &pos_odom_003_pitch, &pos_odom_003_yaw);
    }

    void MultiNaviGoalsPanel::odom_global004_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        pos_odom_004_x=(float)odom->pose.pose.position.x;//位置点为FLU坐标
        pos_odom_004_y=(float)odom->pose.pose.position.y;
        pos_odom_004_z=(float)odom->pose.pose.position.z;
        float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
        mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_004_roll, &pos_odom_004_pitch, &pos_odom_004_yaw);
    }

    void MultiNaviGoalsPanel::odom_global005_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        pos_odom_005_x=(float)odom->pose.pose.position.x;//位置点为FLU坐标
        pos_odom_005_y=(float)odom->pose.pose.position.y;
        pos_odom_005_z=(float)odom->pose.pose.position.z;
        float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
        mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_005_roll, &pos_odom_005_pitch, &pos_odom_005_yaw);
    }

    void MultiNaviGoalsPanel::odom_global006_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        pos_odom_006_x=(float)odom->pose.pose.position.x;//位置点为FLU坐标
        pos_odom_006_y=(float)odom->pose.pose.position.y;
        pos_odom_006_z=(float)odom->pose.pose.position.z;
        float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
        mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_006_roll, &pos_odom_006_pitch, &pos_odom_006_yaw);
    }

    void MultiNaviGoalsPanel::highlightTableRow(QTableWidget* table, int targetRow)
    {
        if (!table || targetRow < 0 || targetRow >= table->rowCount()) {
            // 行号无效时直接返回
            return;
        }

        const QColor highlightedColor(204, 255, 204); // 浅绿色 (R,G,B)
        const QColor defaultColor(Qt::white);         // 白色

        // 遍历所有行
        for (int row = 0; row < table->rowCount(); ++row) {
            for (int col = 0; col < table->columnCount(); ++col) {
                QTableWidgetItem* item = table->item(row, col);
                if (item) { // 确保 item 存在
                    if (row == targetRow) {
                        // 设置目标行的颜色为浅绿色
                        item->setBackground(highlightedColor);
                    } else {
                        // 其他行保持白色
                        item->setBackground(defaultColor);
                    }
                }
            }
        }
    }

    bool MultiNaviGoalsPanel::isDroneAtGoal(
        int drone_id,
        const geometry_msgs::PoseArray& goals,  // ← 改这里：去掉 vector<>
        int curGoalIdx)
    {
        // 1. 检查 drone_id 是否有效（你用 ID>=1，说明 ID 从 1 开始）
        if (drone_id < 1) {
            return false;
        }

        // 2. 检查 curGoalIdx 是否有效
        if (curGoalIdx < 0 || static_cast<size_t>(curGoalIdx) >= goals.poses.size()) {
            return false;
        }

        double x, y, z;

        // 3. 获取当前无人机的位置（目前只支持 drone_id == 1）
        if (drone_id == 1) {
            x = pos_odom_001_x;
            y = pos_odom_001_y;
            z = pos_odom_001_z;
        }

        if (drone_id == 2) {
            x = pos_odom_002_x;
            y = pos_odom_002_y;
            z = pos_odom_002_z;
        }

        if (drone_id == 3) {
            x = pos_odom_003_x;
            y = pos_odom_003_y;
            z = pos_odom_003_z;
        }

        if (drone_id == 4) {
            x = pos_odom_004_x;
            y = pos_odom_004_y;
            z = pos_odom_004_z;
        }

        if (drone_id == 5) {
            x = pos_odom_005_x;
            y = pos_odom_005_y;
            z = pos_odom_005_z;
        }

        if (drone_id == 6) {
            x = pos_odom_006_x;
            y = pos_odom_006_y;
            z = pos_odom_006_z;
        }

        // 4. 获取目标点（goals 是 PoseArray，goals.poses 是 vector<Pose>）
        const auto& goal_pose = goals.poses[curGoalIdx]; // ← 这里是 Pose
        double gx = goal_pose.position.x;                // ← 注意：goal 是 Pose，不是 Point！
        double gy = goal_pose.position.y;
        double gz = goal_pose.position.z;

        // 5. 计算欧氏距离
        double dx = x - gx;
        double dy = y - gy;
        double dz = z - gz;
        double distance = std::sqrt(dx*dx + dy*dy);

        // 6. 判断是否在阈值内

        bool result = false;

        result = (distance <= position_threshold_editor_->text().toDouble());

        if (result == true) {
            ROS_ERROR("Drone is at goal");
        }

        return result;
    }


    void MultiNaviGoalsPanel::NaviControl() { 
        if (permit_001 == false) { 
            permit_001 = true;
        } else {
            permit_001 = false;
        }
        if (permit_002 == false) { 
            permit_002 = true;
        } else {
            permit_002 = false;
        }
        if (permit_003 == false) { 
            permit_003 = true;
        } else {
            permit_003 = false;
        }
        if (permit_004 == false) { 
            permit_004 = true;
        } else {
            permit_004 = false;
        }
        if (permit_005 == false) { 
            permit_005 = true;
        } else {
            permit_005 = false;
        }
        if (permit_006 == false) { 
            permit_006 = true;
        } else {
            permit_006 = false;
        }
        ROS_ERROR("curIndex_001: %d",curGoalIdx_001);
    }

    void MultiNaviGoalsPanel::StopControl() { 
        permit_001 = false;
        permit_002 = false;
        permit_003 = false;
        permit_004 = false;
        permit_005 = false;
        permit_006 = false;
    }
    
    // start to navigate, and only command the first goal
    void MultiNaviGoalsPanel::startNavi() {
        if (permit_001 == true && !pose_array_001.poses.empty() && curGoalIdx_001 < maxNumGoal_) {
            if (cycle_001 == true)
            {
                curGoalIdx_001 = curGoalIdx_001 % pose_array_001.poses.size();
            }
            geometry_msgs::PoseStamped goal;
            if (path_track_status_001 == ReadyToGoal)
            {
                goal.header = pose_array_001.header;
                goal.pose = pose_array_001.poses.at(curGoalIdx_001);
                goal_pub_001.publish(goal);
                highlightTableRow(poseArray_table_001,curGoalIdx_001);
                path_track_status_001 = ExecutingGoal;
            }
            if (path_track_status_001 == ExecutingGoal)
            {
                if (isDroneAtGoal(1, pose_array_001, curGoalIdx_001) == true)
                {
                    curGoalIdx_001 += 1;
                    if (curGoalIdx_001 == pose_array_001.poses.size() && cycle_001 == false)
                    {
                        // curGoalIdx_001 = 0;
                        permit_001 = false;
                    }
                    path_track_status_001 = ReadyToGoal;
                }
            }
        } else {
            // ROS_ERROR("Waiting For Command");
        }

        if (permit_002 == true && !pose_array_002.poses.empty() && curGoalIdx_002 < maxNumGoal_) {
            if (cycle_002 == true)
            {
                curGoalIdx_002 = curGoalIdx_002 % pose_array_002.poses.size();
            }
            geometry_msgs::PoseStamped goal;
            if (path_track_status_002 == ReadyToGoal)
            {
                goal.header = pose_array_002.header;
                goal.pose = pose_array_002.poses.at(curGoalIdx_002);
                goal_pub_002.publish(goal);
                highlightTableRow(poseArray_table_002,curGoalIdx_002);
                path_track_status_002 = ExecutingGoal;
            }
            if (path_track_status_002 == ExecutingGoal)
            {
                if (isDroneAtGoal(2, pose_array_002, curGoalIdx_002) == true)
                {
                    curGoalIdx_002 += 1;
                    if (curGoalIdx_002 == pose_array_002.poses.size() && cycle_002 == false)
                    {
                        // curGoalIdx_002 = 0;
                        permit_002 = false;
                    }
                    path_track_status_002 = ReadyToGoal;
                }
            }
        } else {
            // ROS_ERROR("Waiting For Command");
        }

        if (permit_003 == true && !pose_array_003.poses.empty() && curGoalIdx_003 < maxNumGoal_) {
            if (cycle_003 == true)
            {
                curGoalIdx_003 = curGoalIdx_003 % pose_array_003.poses.size();
            }
            geometry_msgs::PoseStamped goal;
            if (path_track_status_003 == ReadyToGoal)
            {
                goal.header = pose_array_003.header;
                goal.pose = pose_array_003.poses.at(curGoalIdx_003);
                goal_pub_003.publish(goal);
                highlightTableRow(poseArray_table_003,curGoalIdx_003);
                path_track_status_003 = ExecutingGoal;
            }
            if (path_track_status_003 == ExecutingGoal)
            {
                if (isDroneAtGoal(3, pose_array_003, curGoalIdx_003) == true)
                {
                    curGoalIdx_003 += 1;
                    if (curGoalIdx_003 == pose_array_003.poses.size() && cycle_003 == false)
                    {
                        // curGoalIdx_003 = 0;
                        permit_003 = false;
                    }
                    path_track_status_003 = ReadyToGoal;
                }
            }
        } else {
            // ROS_ERROR("Waiting For Command");
        }

        if (permit_004 == true && !pose_array_004.poses.empty() && curGoalIdx_004 < maxNumGoal_) {
            if (cycle_004 == true)
            {
                curGoalIdx_004 = curGoalIdx_004 % pose_array_004.poses.size();
            }
            geometry_msgs::PoseStamped goal;
            if (path_track_status_004 == ReadyToGoal)
            {
                goal.header = pose_array_004.header;
                goal.pose = pose_array_004.poses.at(curGoalIdx_004);
                goal_pub_004.publish(goal);
                highlightTableRow(poseArray_table_004,curGoalIdx_004);
                path_track_status_004 = ExecutingGoal;
            }
            if (path_track_status_004 == ExecutingGoal)
            {
                if (isDroneAtGoal(4, pose_array_004, curGoalIdx_004) == true)
                {
                    curGoalIdx_004 += 1;
                    if (curGoalIdx_004 == pose_array_004.poses.size() && cycle_004 == false)
                    {
                        // curGoalIdx_004 = 0;
                        permit_004 = false;
                    }
                    path_track_status_004 = ReadyToGoal;
                }
            }
        } else {
            // ROS_ERROR("Waiting For Command");
        }

        if (permit_005 == true && !pose_array_005.poses.empty() && curGoalIdx_005 < maxNumGoal_) {
            if (cycle_005 == true)
            {
                curGoalIdx_005 = curGoalIdx_005 % pose_array_005.poses.size();
            }
            geometry_msgs::PoseStamped goal;
            if (path_track_status_005 == ReadyToGoal)
            {
                goal.header = pose_array_005.header;
                goal.pose = pose_array_005.poses.at(curGoalIdx_005);
                goal_pub_005.publish(goal);
                highlightTableRow(poseArray_table_005,curGoalIdx_005);
                path_track_status_005 = ExecutingGoal;
            }
            if (path_track_status_005 == ExecutingGoal)
            {
                if (isDroneAtGoal(5, pose_array_005, curGoalIdx_005) == true)
                {
                    curGoalIdx_005 += 1;
                    if (curGoalIdx_005 == pose_array_005.poses.size() && cycle_005 == false)
                    {
                        // curGoalIdx_005 = 0;
                        permit_005 = false;
                    }
                    path_track_status_005 = ReadyToGoal;
                }
            }
        } else {
            // ROS_ERROR("Waiting For Command");
        }

        if (permit_006 == true && !pose_array_006.poses.empty() && curGoalIdx_006 < maxNumGoal_) {
            if (cycle_006 == true)
            {
                curGoalIdx_006 = curGoalIdx_006 % pose_array_006.poses.size();
            }
            geometry_msgs::PoseStamped goal;
            if (path_track_status_006 == ReadyToGoal)
            {
                goal.header = pose_array_006.header;
                goal.pose = pose_array_006.poses.at(curGoalIdx_006);
                goal_pub_006.publish(goal);
                highlightTableRow(poseArray_table_006,curGoalIdx_006);
                path_track_status_006 = ExecutingGoal;
            }
            if (path_track_status_006 == ExecutingGoal)
            {
                if (isDroneAtGoal(6, pose_array_006, curGoalIdx_006) == true)
                {
                    curGoalIdx_006 += 1;
                    if (curGoalIdx_006 == pose_array_006.poses.size() && cycle_006 == false)
                    {
                        // curGoalIdx_006 = 0;
                        permit_006 = false;
                    }
                    path_track_status_006 = ReadyToGoal;
                }
            }
        } else {
            // ROS_ERROR("Waiting For Command");
        }
    }

    void MultiNaviGoalsPanel::refreshPoseArrayTable(QTableWidget* table, const geometry_msgs::PoseArray& pose_array)
    {
        if (!table) return;

        // 清空现有内容（保留表头）
        table->clearContents();

        int rowCount = static_cast<int>(pose_array.poses.size());
        poseArray_table_006->setRowCount(maxNumGoal_);

        for (size_t i = 0; i < pose_array.poses.size(); ++i) {
            const auto& pose = pose_array.poses[i];

            double x = pose.position.x;
            double y = pose.position.y;
            double z = pose.position.z;
            double yaw_deg = tf::getYaw(pose.orientation) * 180.0 / 3.14159265358979323846;

            // 设置每一列
            table->setItem(static_cast<int>(i), 0,
                new QTableWidgetItem(QString::number(x, 'f', 2)));
            table->setItem(static_cast<int>(i), 1,
                new QTableWidgetItem(QString::number(y, 'f', 2)));
            table->setItem(static_cast<int>(i), 2,
                new QTableWidgetItem(QString::number(z, 'f', 2)));
            table->setItem(static_cast<int>(i), 3,
                new QTableWidgetItem(QString::number(yaw_deg, 'f', 2)));
        }
    }

    // cancel the current command
    void MultiNaviGoalsPanel::deleteGoalPoint() {
        if(drone_id_editor_->text()=="1"&&pose_array_001.poses.size()>0){
            // if (!pose_array_001.poses.empty()) {
            //     pose_array_001.poses.pop_back();
            // }
            if (pose_array_001.poses.size()>selected_row)
            {
                pose_array_001.poses.erase(pose_array_001.poses.begin() + selected_row);
            }
            permit_001 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_001";
            marker_delete.id=selected_row;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_001.publish(marker_delete);
            marker_delete.ns="navi_point_number_001";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_001.publish(marker_delete);
            for (int col = 0; col < poseArray_table_001->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_001->item(selected_row, col);
                if (item) {
                    item->setText("");
                }
            }
            refreshPoseArrayTable(poseArray_table_001,pose_array_001);
            selected_row = -1;
            path_track_status_001 = ReadyToGoal;
        }else if(drone_id_editor_->text()=="2"&&pose_array_002.poses.size()>0){
            // if (!pose_array_002.poses.empty()) {
            //     pose_array_002.poses.pop_back();
            // }
            if (pose_array_002.poses.size()>selected_row)
            {
                pose_array_002.poses.erase(pose_array_002.poses.begin() + selected_row);
            }
            permit_002 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_002";
            marker_delete.id=selected_row;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_002.publish(marker_delete);
            marker_delete.ns="navi_point_number_002";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_002.publish(marker_delete);
            for (int col = 0; col < poseArray_table_002->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_002->item(selected_row, col);
                if (item) {
                    item->setText("");
                }
            }
            refreshPoseArrayTable(poseArray_table_002,pose_array_002);
            selected_row = -1;
            path_track_status_002 = ReadyToGoal;
        }else if(drone_id_editor_->text()=="3"&&pose_array_003.poses.size()>0){
            // if (!pose_array_003.poses.empty()) {
            //     pose_array_003.poses.pop_back();
            // }
            if (pose_array_003.poses.size()>selected_row)
            {
                pose_array_003.poses.erase(pose_array_003.poses.begin() + selected_row);
            }
            permit_003 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_003";
            marker_delete.id=selected_row;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_003.publish(marker_delete);
            marker_delete.ns="navi_point_number_003";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_003.publish(marker_delete);
            for (int col = 0; col < poseArray_table_003->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_003->item(selected_row, col);
                if (item) {
                    item->setText("");
                }
            }
            refreshPoseArrayTable(poseArray_table_003,pose_array_003);
            selected_row = -1;
            path_track_status_003 = ReadyToGoal;
        }else if(drone_id_editor_->text()=="4"&&pose_array_002.poses.size()>0){
            // if (!pose_array_004.poses.empty()) {
            //     pose_array_004.poses.pop_back();
            // }
            if (pose_array_004.poses.size()>selected_row)
            {
                pose_array_004.poses.erase(pose_array_004.poses.begin() + selected_row);
            }
            permit_004 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_004";
            marker_delete.id=selected_row;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_004.publish(marker_delete);
            marker_delete.ns="navi_point_number_004";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_004.publish(marker_delete);
            for (int col = 0; col < poseArray_table_004->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_004->item(selected_row, col);
                if (item) {
                    item->setText("");
                }
            }
            refreshPoseArrayTable(poseArray_table_004,pose_array_004);
            selected_row = -1;
            path_track_status_004 = ReadyToGoal;
        }else if(drone_id_editor_->text()=="5"&&pose_array_005.poses.size()>0){
            // if (!pose_array_005.poses.empty()) {
            //     pose_array_005.poses.pop_back();
            // }
            if (pose_array_005.poses.size()>selected_row)
            {
                pose_array_005.poses.erase(pose_array_005.poses.begin() + selected_row);
            }
            permit_005 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_005";
            marker_delete.id=selected_row;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_005.publish(marker_delete);
            marker_delete.ns="navi_point_number_005";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_005.publish(marker_delete);
            for (int col = 0; col < poseArray_table_005->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_005->item(selected_row, col);
                if (item) {
                    item->setText("");
                }
            }
            refreshPoseArrayTable(poseArray_table_005,pose_array_005);
            selected_row = -1;
            path_track_status_005 = ReadyToGoal;
        }else if(drone_id_editor_->text()=="6"&&pose_array_006.poses.size()>0){
            // if (!pose_array_006.poses.empty()) {
            //     pose_array_006.poses.pop_back();
            // }
            if (pose_array_006.poses.size()>selected_row)
            {
                pose_array_006.poses.erase(pose_array_006.poses.begin() + selected_row);
            }
            permit_006 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_006";
            marker_delete.id=selected_row;
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_006.publish(marker_delete);
            marker_delete.ns="navi_point_number_006";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_006.publish(marker_delete);
            for (int col = 0; col < poseArray_table_006->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_006->item(selected_row, col);
                if (item) {
                    item->setText("");
                }
            }
            refreshPoseArrayTable(poseArray_table_006,pose_array_006);
            selected_row = -1;
            path_track_status_006 = ReadyToGoal;
        }
        
    }

    void MultiNaviGoalsPanel::markWall(const QString &wall_startx, const QString &wall_starty, const QString &wall_endx,const QString &wall_endy){
        if (wall_startx != output_wall_start_x || wall_starty != output_wall_start_y || wall_endx != output_wall_end_x || wall_endy != output_wall_end_y) {
            
            output_wall_start_x = wall_startx;
            output_wall_start_y = wall_starty;
            output_wall_end_x = wall_endx;
            output_wall_end_y = wall_endy;

            // 如果命名为空，不发布任何信息
            if (output_wall_start_x == "" || output_wall_start_y == "" || output_wall_end_x == "" || output_wall_end_y == "") {
                wall_start_x = 0.0;
                wall_start_y = 0.0;
                wall_end_x = 0.0;
                wall_end_y = 0.0;
            } else {
                wall_start_x = output_wall_start_x.toFloat();
                wall_start_y = output_wall_start_y.toFloat();
                wall_end_x = output_wall_end_x.toFloat();
                wall_end_y = output_wall_end_y.toFloat();
            }
            Q_EMIT configChanged();
        }
        visualization_msgs::Marker wall;
        wall.header.frame_id = "map";
        wall.ns = "point_wall";
        wall.action = visualization_msgs::Marker::ADD;
        wall.type = visualization_msgs::Marker::CUBE;
        wall.color.r = 0.0f;
        wall.color.g = 1.0f;
        wall.color.b = 0.0f;
        wall.color.a = 0.2;
        wall.pose.position.x = (wall_start_x + wall_end_x)/2;
        wall.pose.position.y = wall_start_y;
        wall.pose.position.z = 1.0;
        wall.pose.orientation.w=0;
        wall.pose.orientation.x=0;
        wall.pose.orientation.y=0;
        wall.pose.orientation.z=1.0;
        wall.scale.x = fabs(wall_start_x - wall_end_x);
        wall.scale.y = 0.2;
        wall.scale.z = 2.0;
        wall.id = 1;
        wall_pub_.publish(wall);

        wall.pose.position.x = wall_start_x;
        wall.pose.position.y = (wall_start_y + wall_end_y)/2;
        wall.pose.position.z = 1.0;
        wall.pose.orientation.w=0;
        wall.pose.orientation.x=0;
        wall.pose.orientation.y=0;
        wall.pose.orientation.z=1.0;
        wall.scale.x = 0.2;
        wall.scale.y = fabs(wall_start_y - wall_end_y);
        wall.scale.z = 2.0;
        wall.id = 2;
        wall_pub_.publish(wall);

        wall.pose.position.x = (wall_start_x + wall_end_x)/2;
        wall.pose.position.y = wall_end_y;
        wall.pose.position.z = 1.0;
        wall.pose.orientation.w=0;
        wall.pose.orientation.x=0;
        wall.pose.orientation.y=0;
        wall.pose.orientation.z=1.0;
        wall.scale.x = fabs(wall_start_x - wall_end_x);
        wall.scale.y = 0.2;
        wall.scale.z = 2.0;
        wall.id = 3;
        wall_pub_.publish(wall);

        wall.pose.position.x = wall_end_x;
        wall.pose.position.y = (wall_start_y + wall_end_y)/2;
        wall.pose.position.z = 1.0;
        wall.pose.orientation.w=0;
        wall.pose.orientation.x=0;
        wall.pose.orientation.y=0;
        wall.pose.orientation.z=1.0;
        wall.scale.x = 0.2;
        wall.scale.y = fabs(wall_start_y - wall_end_y);
        wall.scale.z = 2.0;
        wall.id = 4;
        wall_pub_.publish(wall);
    }

// spin for subscribing
    void MultiNaviGoalsPanel::startSpin() {
        if (ros::ok()) {
            updateWall();
            deleteMark();
            markPose();
            startNavi();
            ros::spinOnce();
        }
    }

} // end namespace navi-multi-goals-pub-rviz-plugin

// 声明此类是一个rviz的插件

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::MultiNaviGoalsPanel, rviz::Panel)

