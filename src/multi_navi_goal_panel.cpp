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
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>
#include <QKeyEvent>


#include "multi_navi_goal_panel.h"

namespace navi_multi_goals_pub_rviz_plugin {

    class MyTableWidget : public QTableWidget {
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
    };


    MultiNaviGoalsPanel::MultiNaviGoalsPanel(QWidget *parent)
            : rviz::Panel(parent), nh_(), maxNumGoal_(10) {

        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal_temp", 100, boost::bind(&MultiNaviGoalsPanel::goalCntCB, this, _1));
        ros::Subscriber odom001=nh_.subscribe<nav_msgs::Odometry>("odom_global_001", 100, boost::bind(&MultiNaviGoalsPanel::odom_global001_handler, this, _1));
        ros::Subscriber odom002=nh_.subscribe<nav_msgs::Odometry>("odom_global_002", 100, boost::bind(&MultiNaviGoalsPanel::odom_global002_handler, this, _1));
        ros::Subscriber odom003=nh_.subscribe<nav_msgs::Odometry>("odom_global_003", 100, boost::bind(&MultiNaviGoalsPanel::odom_global003_handler, this, _1));
        ros::Subscriber odom004=nh_.subscribe<nav_msgs::Odometry>("odom_global_004", 100, boost::bind(&MultiNaviGoalsPanel::odom_global004_handler, this, _1));
        ros::Subscriber odom005=nh_.subscribe<nav_msgs::Odometry>("odom_global_005", 100, boost::bind(&MultiNaviGoalsPanel::odom_global005_handler, this, _1));
        ros::Subscriber odom006=nh_.subscribe<nav_msgs::Odometry>("odom_global_006", 100, boost::bind(&MultiNaviGoalsPanel::odom_global006_handler, this, _1));
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

        wall_pub_ = nh_.advertise<visualization_msgs::Marker>("wall_marker", 1);
        QVBoxLayout *root_layout = new QVBoxLayout;

        QHBoxLayout *wall_layout = new QHBoxLayout;
        wall_layout->addWidget(new QLabel("虚拟围墙长度(m)"));
        wall_length_editor_ = new QLineEdit;
        wall_length_editor_->setText("5.0");
        wall_layout->addWidget(wall_length_editor_);
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

        //creat a manipulate layout
        QHBoxLayout *manipulate_layout = new QHBoxLayout;
        output_reset_button_ = new QPushButton("清除全部目标点");
        manipulate_layout->addWidget(output_reset_button_);
        output_delete_button_ = new QPushButton("清除单个目标点");
        manipulate_layout->addWidget(output_delete_button_);
        output_startNavi_button_ = new QPushButton("执行轨迹");
        manipulate_layout->addWidget(output_startNavi_button_);
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
        connect(output_startNavi_button_, SIGNAL(clicked()), this, SLOT(startNavi()));
        connect(cycle_checkbox_001, SIGNAL(clicked(bool)), this, SLOT(checkCycle001()));
        connect(cycle_checkbox_002, SIGNAL(clicked(bool)), this, SLOT(checkCycle002()));
        connect(cycle_checkbox_003, SIGNAL(clicked(bool)), this, SLOT(checkCycle003()));
        connect(cycle_checkbox_004, SIGNAL(clicked(bool)), this, SLOT(checkCycle004()));
        connect(cycle_checkbox_005, SIGNAL(clicked(bool)), this, SLOT(checkCycle005()));
        connect(cycle_checkbox_006, SIGNAL(clicked(bool)), this, SLOT(checkCycle006()));
        connect(output_timer, SIGNAL(timeout()), this, SLOT(startSpin()));

    }

    void MultiNaviGoalsPanel::updateWall() {
        markWall(wall_length_editor_->text());
    }

// 更新maxNumGoal命名
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

    // initialize the table of pose
    void MultiNaviGoalsPanel::initPoseTable() {
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
                pose_array_001.header.frame_id = pose->header.frame_id;
                writePose001(pose->pose);
            } else {
                ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
            }
        }else if(drone_id_editor_->text()=="2"){
            if (pose_array_002.poses.size() < maxNumGoal_) {
                pose_array_002.poses.push_back(pose->pose);
                pose_array_002.header.frame_id = pose->header.frame_id;
                writePose002(pose->pose);
            } else {
                ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
            }
        }else if(drone_id_editor_->text()=="3"){
            if (pose_array_003.poses.size() < maxNumGoal_) {
                pose_array_003.poses.push_back(pose->pose);
                pose_array_003.header.frame_id = pose->header.frame_id;
                writePose003(pose->pose);
            } else {
                ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
            }
        }else if(drone_id_editor_->text()=="4"){
            if (pose_array_004.poses.size() < maxNumGoal_) {
                pose_array_004.poses.push_back(pose->pose);
                pose_array_004.header.frame_id = pose->header.frame_id;
                writePose004(pose->pose);
            } else {
                ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
            }
        }else if(drone_id_editor_->text()=="5"){
            if (pose_array_005.poses.size() < maxNumGoal_) {
                pose_array_005.poses.push_back(pose->pose);
                pose_array_005.header.frame_id = pose->header.frame_id;
                writePose005(pose->pose);
            } else {
                ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
            }
        }else if(drone_id_editor_->text()=="6"){
            if (pose_array_006.poses.size() < maxNumGoal_) {
                pose_array_006.poses.push_back(pose->pose);
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
                                new QTableWidgetItem(QString::number(1.0, 'f', 2)));
        poseArray_table_001->setItem(pose_array_001.poses.size() - 1, 3,
                                new QTableWidgetItem(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    void MultiNaviGoalsPanel::writePose002(geometry_msgs::Pose pose) {
        poseArray_table_002->setItem(pose_array_002.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_002->setItem(pose_array_002.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_002->setItem(pose_array_002.poses.size() - 1, 2,
                                new QTableWidgetItem(QString::number(1.0, 'f', 2)));
        poseArray_table_002->setItem(pose_array_002.poses.size() - 1, 3,
                                new QTableWidgetItem(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    void MultiNaviGoalsPanel::writePose003(geometry_msgs::Pose pose) {
        poseArray_table_003->setItem(pose_array_003.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_003->setItem(pose_array_003.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_003->setItem(pose_array_003.poses.size() - 1, 2,
                                new QTableWidgetItem(QString::number(1.0, 'f', 2)));
        poseArray_table_003->setItem(pose_array_003.poses.size() - 1, 3,
                                new QTableWidgetItem(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    void MultiNaviGoalsPanel::writePose004(geometry_msgs::Pose pose) {
        poseArray_table_004->setItem(pose_array_004.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_004->setItem(pose_array_004.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_004->setItem(pose_array_004.poses.size() - 1, 2,
                                new QTableWidgetItem(QString::number(1.0, 'f', 2)));
        poseArray_table_004->setItem(pose_array_004.poses.size() - 1, 3,
                                new QTableWidgetItem(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    void MultiNaviGoalsPanel::writePose005(geometry_msgs::Pose pose) {
        poseArray_table_005->setItem(pose_array_005.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_005->setItem(pose_array_005.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_005->setItem(pose_array_005.poses.size() - 1, 2,
                                new QTableWidgetItem(QString::number(1.0, 'f', 2)));
        poseArray_table_005->setItem(pose_array_005.poses.size() - 1, 3,
                                new QTableWidgetItem(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    void MultiNaviGoalsPanel::writePose006(geometry_msgs::Pose pose) {
        poseArray_table_006->setItem(pose_array_006.poses.size() - 1, 0,
                                new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_006->setItem(pose_array_006.poses.size() - 1, 1,
                                new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_006->setItem(pose_array_006.poses.size() - 1, 2,
                                new QTableWidgetItem(QString::number(1.0, 'f', 2)));
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
    }

    void MultiNaviGoalsPanel::odom_global002_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        pos_odom_002_x=(float)odom->pose.pose.position.x;//位置点为FLU坐标
        pos_odom_002_y=(float)odom->pose.pose.position.y;
        pos_odom_002_z=(float)odom->pose.pose.position.z;
    }

    void MultiNaviGoalsPanel::odom_global003_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        pos_odom_003_x=(float)odom->pose.pose.position.x;//位置点为FLU坐标
        pos_odom_003_y=(float)odom->pose.pose.position.y;
        pos_odom_003_z=(float)odom->pose.pose.position.z;
    }

    void MultiNaviGoalsPanel::odom_global004_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        pos_odom_004_x=(float)odom->pose.pose.position.x;//位置点为FLU坐标
        pos_odom_004_y=(float)odom->pose.pose.position.y;
        pos_odom_004_z=(float)odom->pose.pose.position.z;
    }

    void MultiNaviGoalsPanel::odom_global005_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        pos_odom_005_x=(float)odom->pose.pose.position.x;//位置点为FLU坐标
        pos_odom_005_y=(float)odom->pose.pose.position.y;
        pos_odom_005_z=(float)odom->pose.pose.position.z;
    }

    void MultiNaviGoalsPanel::odom_global006_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        pos_odom_006_x=(float)odom->pose.pose.position.x;//位置点为FLU坐标
        pos_odom_006_y=(float)odom->pose.pose.position.y;
        pos_odom_006_z=(float)odom->pose.pose.position.z;
    }

    // start to navigate, and only command the first goal
    void MultiNaviGoalsPanel::startNavi() {
        if (!pose_array_001.poses.empty() && curGoalIdx_001 < maxNumGoal_) {
            curGoalIdx_001 = curGoalIdx_001 % pose_array_001.poses.size();
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_001.header;
            goal.pose = pose_array_001.poses.at(curGoalIdx_001);
            goal_pub_001.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_001 + 1);
            poseArray_table_001->item(curGoalIdx_001, 0)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_001->item(curGoalIdx_001, 1)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_001->item(curGoalIdx_001, 2)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_001->item(curGoalIdx_001, 3)->setBackgroundColor(QColor(200, 80, 0));
            curGoalIdx_001 += 1;
            permit_001 = true;
            cycle_001 = cycle_checkbox_001->isChecked();
        } else {
            ROS_ERROR("Something Wrong");
        }

        if (!pose_array_002.poses.empty() && curGoalIdx_002 < maxNumGoal_) {
            curGoalIdx_002 = curGoalIdx_002 % pose_array_002.poses.size();
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_002.header;
            goal.pose = pose_array_002.poses.at(curGoalIdx_002);
            goal_pub_002.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_002 + 1);
            poseArray_table_002->item(curGoalIdx_002, 0)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_002->item(curGoalIdx_002, 1)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_002->item(curGoalIdx_002, 2)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_002->item(curGoalIdx_002, 3)->setBackgroundColor(QColor(200, 80, 0));
            curGoalIdx_002 += 1;
            permit_002 = true;
            cycle_002 = cycle_checkbox_002->isChecked();
        } else {
            ROS_ERROR("Something Wrong");
        }

        if (!pose_array_003.poses.empty() && curGoalIdx_003 < maxNumGoal_) {
            curGoalIdx_003 = curGoalIdx_003 % pose_array_003.poses.size();
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_003.header;
            goal.pose = pose_array_003.poses.at(curGoalIdx_003);
            goal_pub_003.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_003 + 1);
            poseArray_table_003->item(curGoalIdx_003, 0)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_003->item(curGoalIdx_003, 1)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_003->item(curGoalIdx_003, 2)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_003->item(curGoalIdx_003, 3)->setBackgroundColor(QColor(200, 80, 0));
            curGoalIdx_003 += 1;
            permit_003 = true;
            cycle_003 = cycle_checkbox_003->isChecked();
        } else {
            ROS_ERROR("Something Wrong");
        }

        if (!pose_array_004.poses.empty() && curGoalIdx_004 < maxNumGoal_) {
            curGoalIdx_004 = curGoalIdx_004 % pose_array_004.poses.size();
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_004.header;
            goal.pose = pose_array_004.poses.at(curGoalIdx_004);
            goal_pub_004.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_004 + 1);
            poseArray_table_004->item(curGoalIdx_004, 0)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_004->item(curGoalIdx_004, 1)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_004->item(curGoalIdx_004, 2)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_004->item(curGoalIdx_004, 3)->setBackgroundColor(QColor(200, 80, 0));
            curGoalIdx_004 += 1;
            permit_004 = true;
            cycle_004 = cycle_checkbox_004->isChecked();
        } else {
            ROS_ERROR("Something Wrong");
        }

        if (!pose_array_005.poses.empty() && curGoalIdx_005 < maxNumGoal_) {
            curGoalIdx_005 = curGoalIdx_005 % pose_array_005.poses.size();
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_005.header;
            goal.pose = pose_array_005.poses.at(curGoalIdx_005);
            goal_pub_005.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_005 + 1);
            poseArray_table_005->item(curGoalIdx_005, 0)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_005->item(curGoalIdx_005, 1)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_005->item(curGoalIdx_005, 2)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_005->item(curGoalIdx_005, 3)->setBackgroundColor(QColor(200, 80, 0));
            curGoalIdx_005 += 1;
            permit_005 = true;
            cycle_005 = cycle_checkbox_005->isChecked();
        } else {
            ROS_ERROR("Something Wrong");
        }

        if (!pose_array_006.poses.empty() && curGoalIdx_006 < maxNumGoal_) {
            curGoalIdx_006 = curGoalIdx_006 % pose_array_006.poses.size();
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_006.header;
            goal.pose = pose_array_006.poses.at(curGoalIdx_006);
            goal_pub_006.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_006 + 1);
            poseArray_table_006->item(curGoalIdx_006, 0)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_006->item(curGoalIdx_006, 1)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_006->item(curGoalIdx_006, 2)->setBackgroundColor(QColor(200, 80, 0));
            poseArray_table_006->item(curGoalIdx_006, 3)->setBackgroundColor(QColor(200, 80, 0));
            curGoalIdx_006 += 1;
            permit_006 = true;
            cycle_006 = cycle_checkbox_006->isChecked();
        } else {
            ROS_ERROR("Something Wrong");
        }
    }

    // cancel the current command
    void MultiNaviGoalsPanel::deleteGoalPoint() {
        if(drone_id_editor_->text()=="1"&&pose_array_001.poses.size()>0){
            if (!pose_array_001.poses.empty()) {
                pose_array_001.poses.pop_back();
            }
            permit_001 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_001";
            marker_delete.id=pose_array_001.poses.size();
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_001.publish(marker_delete);
            marker_delete.ns="navi_point_number_001";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_001.publish(marker_delete);
            for (int col = 0; col < poseArray_table_001->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_001->item(pose_array_001.poses.size(), col);
                if (item) {
                    item->setText("");
                }
            }
        }else if(drone_id_editor_->text()=="2"&&pose_array_002.poses.size()>0){
            if (!pose_array_002.poses.empty()) {
                pose_array_002.poses.pop_back();
            }
            permit_002 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_002";
            marker_delete.id=pose_array_002.poses.size();
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_002.publish(marker_delete);
            marker_delete.ns="navi_point_number_002";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_002.publish(marker_delete);
            for (int col = 0; col < poseArray_table_002->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_002->item(pose_array_002.poses.size(), col);
                if (item) {
                    item->setText("");
                }
            }
        }else if(drone_id_editor_->text()=="3"&&pose_array_003.poses.size()>0){
            if (!pose_array_003.poses.empty()) {
                pose_array_003.poses.pop_back();
            }
            permit_003 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_003";
            marker_delete.id=pose_array_003.poses.size();
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_003.publish(marker_delete);
            marker_delete.ns="navi_point_number_003";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_003.publish(marker_delete);
            for (int col = 0; col < poseArray_table_003->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_003->item(pose_array_003.poses.size(), col);
                if (item) {
                    item->setText("");
                }
            }
        }else if(drone_id_editor_->text()=="4"&&pose_array_002.poses.size()>0){
            if (!pose_array_004.poses.empty()) {
                pose_array_004.poses.pop_back();
            }
            permit_004 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_004";
            marker_delete.id=pose_array_004.poses.size();
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_004.publish(marker_delete);
            marker_delete.ns="navi_point_number_004";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_004.publish(marker_delete);
            for (int col = 0; col < poseArray_table_004->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_004->item(pose_array_004.poses.size(), col);
                if (item) {
                    item->setText("");
                }
            }
        }else if(drone_id_editor_->text()=="5"&&pose_array_005.poses.size()>0){
            if (!pose_array_005.poses.empty()) {
                pose_array_005.poses.pop_back();
            }
            permit_005 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_005";
            marker_delete.id=pose_array_005.poses.size();
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_005.publish(marker_delete);
            marker_delete.ns="navi_point_number_005";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_005.publish(marker_delete);
            for (int col = 0; col < poseArray_table_005->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_005->item(pose_array_005.poses.size(), col);
                if (item) {
                    item->setText("");
                }
            }
        }else if(drone_id_editor_->text()=="6"&&pose_array_006.poses.size()>0){
            if (!pose_array_006.poses.empty()) {
                pose_array_006.poses.pop_back();
            }
            permit_006 = false;
            visualization_msgs::Marker marker_delete;
            marker_delete.ns="navi_point_arrow_006";
            marker_delete.id=pose_array_006.poses.size();
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_006.publish(marker_delete);
            marker_delete.ns="navi_point_number_006";
            marker_delete.action = visualization_msgs::Marker::DELETE;
            marker_pub_006.publish(marker_delete);
            for (int col = 0; col < poseArray_table_006->columnCount(); ++col) {
                QTableWidgetItem *item = poseArray_table_006->item(pose_array_006.poses.size(), col);
                if (item) {
                    item->setText("");
                }
            }
        }
        
    }

    void MultiNaviGoalsPanel::markWall(const QString &wall_length){
        if (wall_length != output_wall_length) {
            output_wall_length = wall_length;

            // 如果命名为空，不发布任何信息
            if (output_wall_length == "") {
                nh_.setParam("wall_length_", 5.0);
                wall_length_ = 5.0;
            } else {
                nh_.setParam("wall_length_", output_wall_length.toFloat());
                wall_length_ = output_wall_length.toFloat();
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
        wall.pose.position.x = 0.0;
        wall.pose.position.y = wall_length_/2;
        wall.pose.position.z = 1.0;
        wall.pose.orientation.w=0;
        wall.pose.orientation.x=0;
        wall.pose.orientation.y=0;
        wall.pose.orientation.z=1.0;
        wall.scale.x = wall_length_;
        wall.scale.y = 0.2;
        wall.scale.z = 2.0;
        wall.id = 1;
        wall_pub_.publish(wall);

        wall.pose.position.x = -wall_length_/2;
        wall.pose.position.y = 0.0;
        wall.pose.position.z = 1.0;
        wall.pose.orientation.w=0;
        wall.pose.orientation.x=0;
        wall.pose.orientation.y=0;
        wall.pose.orientation.z=1.0;
        wall.scale.x = 0.2;
        wall.scale.y = wall_length_;
        wall.scale.z = 2.0;
        wall.id = 2;
        wall_pub_.publish(wall);

        wall.pose.position.x = 0.0;
        wall.pose.position.y = -wall_length_/2;
        wall.pose.position.z = 1.0;
        wall.pose.orientation.w=0;
        wall.pose.orientation.x=0;
        wall.pose.orientation.y=0;
        wall.pose.orientation.z=1.0;
        wall.scale.x = wall_length_;
        wall.scale.y = 0.2;
        wall.scale.z = 2.0;
        wall.id = 3;
        wall_pub_.publish(wall);

        wall.pose.position.x = wall_length_/2;
        wall.pose.position.y = 0.0;
        wall.pose.position.z = 1.0;
        wall.pose.orientation.w=0;
        wall.pose.orientation.x=0;
        wall.pose.orientation.y=0;
        wall.pose.orientation.z=1.0;
        wall.scale.x = 0.2;
        wall.scale.y = wall_length_;
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
            ros::spinOnce();
        }
    }

} // end namespace navi-multi-goals-pub-rviz-plugin

// 声明此类是一个rviz的插件

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::MultiNaviGoalsPanel, rviz::Panel)

