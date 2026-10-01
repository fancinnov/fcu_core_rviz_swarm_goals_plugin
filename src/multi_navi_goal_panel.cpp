#include "multi_navi_goal_panel.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

#include <boost/bind.hpp>

#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

#include <QCheckBox>
#include <QAbstractItemView>
#include <QColor>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

namespace navi_multi_goals_pub_rviz_plugin {

constexpr double MultiNaviGoalsPanel::kDefaultPositionToleranceM;
constexpr double MultiNaviGoalsPanel::kDefaultYawToleranceDeg;
constexpr double MultiNaviGoalsPanel::kDefaultDwellSeconds;
constexpr double MultiNaviGoalsPanel::kOdomStaleSeconds;
constexpr double MultiNaviGoalsPanel::kDefaultMoveStepM;
constexpr double MultiNaviGoalsPanel::kDefaultTurnStepDeg;

namespace {
constexpr double kPi = 3.14159265358979323846;

// Applies to all drones: true checks XY only, ignoring Z and yaw;
// false checks XYZ distance and yaw tolerance.
bool g_ignore_z_for_arrival = true;

QString topicFor(const QString& prefix, std::size_t index) {
  return QString("%1_%2").arg(prefix).arg(static_cast<int>(index + 1), 3, 10, QChar('0'));
}

double normalizeAngle(double radians) {
  while (radians > kPi) radians -= 2.0 * kPi;
  while (radians < -kPi) radians += 2.0 * kPi;
  return radians;
}
}  // namespace

MultiNaviGoalsPanel::MultiNaviGoalsPanel(QWidget* parent) : rviz::Panel(parent) {
  initializeRosInterfaces();
  buildUi();
  // RViz restores a fixed geometry from its .rviz file. Run after the panel is
  // attached to the main window so Qt can maximize against the current screen.
  QTimer::singleShot(0, this, [this]() { maximizeRvizWindow(); });
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &MultiNaviGoalsPanel::onTimer);
  timer_->start(100);
}

void MultiNaviGoalsPanel::maximizeRvizWindow() {
  QWidget* rviz_window = window();
  if (rviz_window) rviz_window->showMaximized();
}

void MultiNaviGoalsPanel::initializeRosInterfaces() {
  rviz_goal_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "move_base_simple/goal_temp", 10, &MultiNaviGoalsPanel::addWaypointFromRviz, this);
  command_publisher_ = nh_.advertise<std_msgs::Int16>("fcu_command/command", 10);
  wall_publisher_ = nh_.advertise<visualization_msgs::Marker>("wall_marker", 1, true);

  for (std::size_t i = 0; i < kMaxDrones; ++i) {
    DroneState& drone = drones_[i];
    drone.waypoints.header.frame_id = "map";
    drone.goal_publisher = nh_.advertise<geometry_msgs::PoseStamped>(
        topicFor("fcu_mission/goal", i).toStdString(), 10);
    drone.marker_publisher = nh_.advertise<visualization_msgs::Marker>(
        topicFor("visualization_marker", i).toStdString(), 10, true);
    drone.odom_subscriber = nh_.subscribe<nav_msgs::Odometry>(
        topicFor("/odom_global", i).toStdString(), 10,
        boost::bind(&MultiNaviGoalsPanel::odometryCallback, this, _1, i));
  }
}

void MultiNaviGoalsPanel::buildUi() {
  QVBoxLayout* root = new QVBoxLayout;

  QGridLayout* command_layout = new QGridLayout;
  struct CommandButton { const char* text; int command; int row; int column; };
  const CommandButton commands[] = {
      {"解锁", 1, 0, 0}, {"锁定", 2, 0, 1}, {"起飞", 3, 0, 2}, {"降落", 4, 0, 3},
      {"前视追踪", 1011, 1, 0}, {"下视追踪", 1012, 1, 1}, {"自由追踪", 1015, 1, 2}, {"停止追踪", 1013, 1, 3}};
  for (const CommandButton& definition : commands) {
    QPushButton* button = new QPushButton(tr(definition.text));
    connect(button, &QPushButton::clicked, this,
            [this, definition]() { publishCommand(definition.command); });
    command_layout->addWidget(button, definition.row, definition.column);
  }
  root->addLayout(command_layout);

  // Two parameter groups per row: label/control | label/control.
  QGridLayout* settings = new QGridLayout;
  settings->setColumnStretch(1, 1);
  settings->setColumnStretch(3, 1);
  const auto add_setting = [settings](int row, int group, const QString& label, QWidget* control) {
    const int column = group * 2;
    settings->addWidget(new QLabel(label), row, column);
    settings->addWidget(control, row, column + 1);
  };
  active_drone_spin_ = new QSpinBox;
  active_drone_spin_->setRange(1, static_cast<int>(kMaxDrones));
  active_drone_spin_->setValue(active_drone_count_);
  add_setting(0, 0, tr("活动无人机数量"), active_drone_spin_);
  connect(active_drone_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &MultiNaviGoalsPanel::setActiveDroneCount);

  max_goals_spin_ = new QSpinBox;
  max_goals_spin_->setRange(1, kHardMaxGoals);
  max_goals_spin_->setValue(max_goals_);
  add_setting(0, 1, tr("每机最大航点数"), max_goals_spin_);
  connect(max_goals_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &MultiNaviGoalsPanel::setMaxGoals);

  default_z_spin_ = new QDoubleSpinBox;
  default_z_spin_->setRange(-100.0, 100.0);
  default_z_spin_->setDecimals(2);
  default_z_spin_->setValue(1.0);
  add_setting(1, 0, tr("默认高度 (m)"), default_z_spin_);

  move_step_spin_ = new QDoubleSpinBox;
  move_step_spin_->setRange(0.01, 100.0);
  move_step_spin_->setDecimals(2);
  move_step_spin_->setValue(kDefaultMoveStepM);
  add_setting(1, 1, tr("相对移动步长 (m)"), move_step_spin_);

  turn_step_spin_ = new QDoubleSpinBox;
  turn_step_spin_->setRange(1.0, 180.0);
  turn_step_spin_->setDecimals(1);
  turn_step_spin_->setValue(kDefaultTurnStepDeg);
  add_setting(2, 0, tr("转向步长 (°)"), turn_step_spin_);

  position_tolerance_spin_ = new QDoubleSpinBox;
  position_tolerance_spin_->setRange(0.01, 10.0);
  position_tolerance_spin_->setDecimals(2);
  position_tolerance_spin_->setValue(kDefaultPositionToleranceM);
  add_setting(2, 1, tr("到点距离阈值 (m)"), position_tolerance_spin_);

  yaw_tolerance_spin_ = new QDoubleSpinBox;
  yaw_tolerance_spin_->setRange(1.0, 180.0);
  yaw_tolerance_spin_->setDecimals(1);
  yaw_tolerance_spin_->setValue(kDefaultYawToleranceDeg);
  add_setting(3, 0, tr("到点偏航阈值 (°)"), yaw_tolerance_spin_);

  dwell_spin_ = new QDoubleSpinBox;
  dwell_spin_->setRange(0.0, 30.0);
  dwell_spin_->setDecimals(1);
  dwell_spin_->setValue(kDefaultDwellSeconds);
  add_setting(3, 1, tr("到点悬停时间 (s)"), dwell_spin_);
  root->addLayout(settings);

  drone_tabs_ = new QTabWidget;
  for (std::size_t i = 0; i < kMaxDrones; ++i) buildDroneTab(i);
  root->addWidget(drone_tabs_);

  QGroupBox* waypoint_group = new QGroupBox(tr("航点编辑"));
  QGridLayout* waypoint_layout = new QGridLayout(waypoint_group);
  const struct { const char* text; void (MultiNaviGoalsPanel::*slot)(); int column; } waypoint_actions[] = {
      {"添加空航点", &MultiNaviGoalsPanel::addEmptyWaypoint, 0},
      {"插入航点", &MultiNaviGoalsPanel::insertWaypoint, 1},
      {"删除航点", &MultiNaviGoalsPanel::deleteWaypoint, 2},
      {"上移", &MultiNaviGoalsPanel::moveWaypointUp, 3},
      {"下移", &MultiNaviGoalsPanel::moveWaypointDown, 4}};
  for (const auto& action : waypoint_actions) {
    QPushButton* button = new QPushButton(tr(action.text));
    connect(button, &QPushButton::clicked, this, action.slot);
    waypoint_layout->addWidget(button, 0, action.column);
  }
  root->addWidget(waypoint_group);

  QGroupBox* relative_group = new QGroupBox(tr("相对控制"));
  QGridLayout* relative_layout = new QGridLayout(relative_group);
  const auto add_relative_button = [this, relative_layout](const QString& text, int row, int column,
                                                            double forward, double left, double vertical) {
    QPushButton* button = new QPushButton(text);
    connect(button, &QPushButton::clicked, this, [this, forward, left, vertical]() {
      const double step = move_step_spin_->value();
      addRelativeWaypoint(forward * step, left * step, vertical * step);
    });
    relative_layout->addWidget(button, row, column);
  };
  add_relative_button(tr("上升"), 0, 0, 0.0, 0.0, 1.0);
  add_relative_button(tr("前进"), 0, 1, 1.0, 0.0, 0.0);
  add_relative_button(tr("下降"), 0, 2, 0.0, 0.0, -1.0);
  add_relative_button(tr("左移"), 1, 0, 0.0, 1.0, 0.0);
  add_relative_button(tr("后退"), 1, 1, -1.0, 0.0, 0.0);
  add_relative_button(tr("右移"), 1, 2, 0.0, -1.0, 0.0);
  QPushButton* turn_left = new QPushButton(tr("左转"));
  QPushButton* turn_right = new QPushButton(tr("右转"));
  connect(turn_left, &QPushButton::clicked, this, [this]() { rotateWaypoint(turn_step_spin_->value()); });
  connect(turn_right, &QPushButton::clicked, this, [this]() { rotateWaypoint(-turn_step_spin_->value()); });
  relative_layout->addWidget(turn_left, 2, 0);
  relative_layout->addWidget(turn_right, 2, 2);
  root->addWidget(relative_group);

  QGridLayout* mission_layout = new QGridLayout;
  QPushButton* start = new QPushButton(tr("执行轨迹"));
  QPushButton* step = new QPushButton(tr("单步执行"));
  QPushButton* stop = new QPushButton(tr("停止轨迹"));
  QPushButton* save = new QPushButton(tr("保存目标点"));
  QPushButton* load = new QPushButton(tr("加载目标点"));
  connect(start, &QPushButton::clicked, this, &MultiNaviGoalsPanel::startMission);
  connect(step, &QPushButton::clicked, this, &MultiNaviGoalsPanel::stepMission);
  connect(stop, &QPushButton::clicked, this, &MultiNaviGoalsPanel::stopMission);
  connect(save, &QPushButton::clicked, this, &MultiNaviGoalsPanel::saveGoals);
  connect(load, &QPushButton::clicked, this, &MultiNaviGoalsPanel::loadGoals);
  mission_layout->addWidget(start, 0, 0); mission_layout->addWidget(step, 0, 1);
  mission_layout->addWidget(stop, 0, 2); mission_layout->addWidget(save, 0, 3);
  mission_layout->addWidget(load, 0, 4);
  root->addLayout(mission_layout);

  // Two coordinate inputs per row: start X/Y, then end X/Y.
  QGridLayout* wall = new QGridLayout;
  wall->setColumnStretch(1, 1);
  wall->setColumnStretch(3, 1);
  wall_start_x_edit_ = new QLineEdit("0"); wall_start_y_edit_ = new QLineEdit("0");
  wall_end_x_edit_ = new QLineEdit("0"); wall_end_y_edit_ = new QLineEdit("0");
  wall->addWidget(new QLabel(tr("墙体起点 X")), 0, 0);
  wall->addWidget(wall_start_x_edit_, 0, 1);
  wall->addWidget(new QLabel(tr("墙体起点 Y")), 0, 2);
  wall->addWidget(wall_start_y_edit_, 0, 3);
  wall->addWidget(new QLabel(tr("墙体终点 X")), 1, 0);
  wall->addWidget(wall_end_x_edit_, 1, 1);
  wall->addWidget(new QLabel(tr("墙体终点 Y")), 1, 2);
  wall->addWidget(wall_end_y_edit_, 1, 3);
  root->addLayout(wall);
  for (QLineEdit* edit : {wall_start_x_edit_, wall_start_y_edit_, wall_end_x_edit_, wall_end_y_edit_})
    connect(edit, &QLineEdit::editingFinished, this, &MultiNaviGoalsPanel::updateWall);

  setLayout(root);
  refreshAllTabs();
}

void MultiNaviGoalsPanel::buildDroneTab(std::size_t index) {
  QWidget* tab = new QWidget;
  QVBoxLayout* layout = new QVBoxLayout(tab);
  status_labels_[index] = new QLabel;
  layout->addWidget(status_labels_[index]);
  loop_buttons_[index] = new QPushButton(tr("循环执行: 关"));
  loop_buttons_[index]->setCheckable(true);
  connect(loop_buttons_[index], &QPushButton::toggled, this, [this, index](bool checked) {
    drones_[index].loop = checked;
    loop_buttons_[index]->setText(checked ? tr("循环执行: 开") : tr("循环执行: 关"));
  });
  layout->addWidget(loop_buttons_[index]);
  QTableWidget* table = new QTableWidget;
  table->setColumnCount(4);
  table->setHorizontalHeaderLabels({tr("X (m)"), tr("Y (m)"), tr("Z (m)"), tr("Yaw (°)")});
  table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->setDragDropMode(QAbstractItemView::InternalMove);
  table->setDefaultDropAction(Qt::MoveAction);
  connect(table, &QTableWidget::cellChanged, this, &MultiNaviGoalsPanel::onTableChanged);
  waypoint_tables_[index] = table;
  layout->addWidget(table);
  drone_tabs_->addTab(tab, tr("无人机 %1").arg(index + 1));
}

void MultiNaviGoalsPanel::setActiveDroneCount(int count) {
  const int previous_count = active_drone_count_;
  active_drone_count_ = count;
  // QTabWidget::setTabVisible() is not available in every Qt5 version shipped with Noetic.
  // Disabled tabs preserve their fixed external drone/topic index while keeping Qt 5.12 compatibility.
  for (int i = 0; i < static_cast<int>(kMaxDrones); ++i) drone_tabs_->setTabEnabled(i, i < count);
  for (int i = count; i < previous_count; ++i) deleteMarkers(i);
  for (int i = previous_count; i < count; ++i) drones_[i].markers_dirty = true;
  if (drone_tabs_->currentIndex() >= count) drone_tabs_->setCurrentIndex(0);
}

void MultiNaviGoalsPanel::setMaxGoals(int count) {
  max_goals_ = count;
  for (std::size_t i = 0; i < kMaxDrones; ++i) {
    DroneState& drone = drones_[i];
    if (static_cast<int>(drone.waypoints.poses.size()) > max_goals_) drone.waypoints.poses.resize(max_goals_);
    drone.markers_dirty = true;
  }
  refreshAllTabs();
}

void MultiNaviGoalsPanel::addWaypointFromRviz(const geometry_msgs::PoseStamped::ConstPtr& pose) {
  const int index = currentDroneIndex();
  if (index < 0) return;
  geometry_msgs::Pose waypoint = pose->pose;
  waypoint.position.z = default_z_spin_->value();
  if (appendWaypoint(static_cast<std::size_t>(index), waypoint)) refreshDroneTab(index);
}

bool MultiNaviGoalsPanel::appendWaypoint(std::size_t index, const geometry_msgs::Pose& pose) {
  DroneState& drone = drones_[index];
  if (static_cast<int>(drone.waypoints.poses.size()) >= max_goals_) {
    showWarning(tr("已达到每机最大航点数 %1。").arg(max_goals_));
    return false;
  }
  drone.waypoints.poses.push_back(pose);
  drone.waypoints.header.frame_id = "map";
  drone.markers_dirty = true;
  resetMission(drone);
  return true;
}

void MultiNaviGoalsPanel::addEmptyWaypoint() {
  const int index = currentDroneIndex();
  if (index < 0) return;
  const DroneState& drone = drones_[index];
  geometry_msgs::Pose pose = drone.waypoints.poses.empty()
      ? makePose(0.0, 0.0, default_z_spin_->value(), 0.0) : drone.waypoints.poses.back();
  if (appendWaypoint(index, pose)) refreshDroneTab(index);
}

void MultiNaviGoalsPanel::insertWaypoint() {
  const int index = currentDroneIndex(); const int row = selectedRow();
  if (index < 0 || row < 0) { addEmptyWaypoint(); return; }
  DroneState& drone = drones_[index];
  if (static_cast<int>(drone.waypoints.poses.size()) >= max_goals_) { showWarning(tr("已达到最大航点数。")); return; }
  geometry_msgs::Pose pose = drone.waypoints.poses[std::min<std::size_t>(row, drone.waypoints.poses.size() - 1)];
  drone.waypoints.poses.insert(drone.waypoints.poses.begin() + row, pose);
  drone.markers_dirty = true; resetMission(drone); refreshDroneTab(index);
}

void MultiNaviGoalsPanel::deleteWaypoint() {
  const int index = currentDroneIndex(); const int row = selectedRow();
  if (index < 0 || row < 0 || row >= static_cast<int>(drones_[index].waypoints.poses.size())) return;
  DroneState& drone = drones_[index];
  drone.waypoints.poses.erase(drone.waypoints.poses.begin() + row);
  drone.markers_dirty = true; resetMission(drone); refreshDroneTab(index);
}

void MultiNaviGoalsPanel::moveWaypointUp() {
  const int index = currentDroneIndex(); const int row = selectedRow();
  if (index < 0 || row <= 0) return;
  DroneState& drone = drones_[index];
  std::swap(drone.waypoints.poses[row], drone.waypoints.poses[row - 1]);
  drone.markers_dirty = true; resetMission(drone); refreshDroneTab(index); waypoint_tables_[index]->selectRow(row - 1);
}

void MultiNaviGoalsPanel::moveWaypointDown() {
  const int index = currentDroneIndex(); const int row = selectedRow();
  if (index < 0 || row < 0 || row + 1 >= static_cast<int>(drones_[index].waypoints.poses.size())) return;
  DroneState& drone = drones_[index];
  std::swap(drone.waypoints.poses[row], drone.waypoints.poses[row + 1]);
  drone.markers_dirty = true; resetMission(drone); refreshDroneTab(index); waypoint_tables_[index]->selectRow(row + 1);
}

void MultiNaviGoalsPanel::addRelativeWaypoint(double forward, double left, double vertical) {
  const int index = currentDroneIndex();
  if (index < 0 || drones_[index].waypoints.poses.empty()) { showWarning(tr("请先添加一个初始航点。")); return; }
  geometry_msgs::Pose pose = drones_[index].waypoints.poses.back();
  const double yaw = selectedYawRadians(index);
  pose.position.x += forward * std::cos(yaw) - left * std::sin(yaw);
  pose.position.y += forward * std::sin(yaw) + left * std::cos(yaw);
  pose.position.z += vertical;
  if (appendWaypoint(index, pose)) refreshDroneTab(index);
}

void MultiNaviGoalsPanel::rotateWaypoint(double degrees) {
  const int index = currentDroneIndex();
  if (index < 0 || drones_[index].waypoints.poses.empty()) { showWarning(tr("请先添加一个初始航点。")); return; }
  geometry_msgs::Pose pose = drones_[index].waypoints.poses.back();
  pose = makePose(pose.position.x, pose.position.y, pose.position.z,
                  tf::getYaw(pose.orientation) + degrees * kPi / 180.0);
  if (appendWaypoint(index, pose)) refreshDroneTab(index);
}

void MultiNaviGoalsPanel::onTableChanged(int row, int) {
  if (refreshing_table_) return;
  const int index = currentDroneIndex();
  if (index >= 0) setWaypointFromTable(index, row);
}

void MultiNaviGoalsPanel::setWaypointFromTable(std::size_t index, int row) {
  QTableWidget* table = waypoint_tables_[index];
  DroneState& drone = drones_[index];
  if (row < 0 || row >= static_cast<int>(drone.waypoints.poses.size())) return;
  double values[4];
  for (int column = 0; column < 4; ++column) {
    QTableWidgetItem* item = table->item(row, column);
    bool ok = false; values[column] = item ? item->text().toDouble(&ok) : 0.0;
    if (!ok || !std::isfinite(values[column])) { refreshDroneTab(index); showWarning(tr("航点必须为有效数字。")); return; }
  }
  drone.waypoints.poses[row] = makePose(values[0], values[1], values[2], values[3] * kPi / 180.0);
  drone.markers_dirty = true; resetMission(drone); updateStatus(index);
}

void MultiNaviGoalsPanel::startMission() {
  const ros::Time now = ros::Time::now();
  for (int i = 0; i < active_drone_count_; ++i) {
    DroneState& drone = drones_[i];
    resetMission(drone);
    if (drone.waypoints.poses.empty()) continue;
    drone.mission_state = MissionState::Executing;
    if (hasFreshOdometry(drone, now)) dispatchCurrentWaypoint(i);
    else drone.error = tr("等待有效里程计");
    updateStatus(i);
  }
}

void MultiNaviGoalsPanel::stepMission() {
  for (int i = 0; i < active_drone_count_; ++i) {
    DroneState& drone = drones_[i];
    if (drone.waypoints.poses.empty()) continue;

    // A completed non-looping route starts over on the next manual step.
    if (drone.current_waypoint >= drone.waypoints.poses.size()) {
      drone.current_waypoint = 0;
    }
    drone.mission_state = MissionState::Idle;
    dispatchCurrentWaypoint(i);
    ++drone.current_waypoint;

    if (drone.current_waypoint == drone.waypoints.poses.size()) {
      if (drone.loop) {
        drone.current_waypoint = 0;
      } else {
        drone.mission_state = MissionState::Completed;
      }
    }
    updateStatus(i);
  }
}

void MultiNaviGoalsPanel::stopMission() {
  for (int i = 0; i < active_drone_count_; ++i) { resetMission(drones_[i]); updateStatus(i); }
}

void MultiNaviGoalsPanel::dispatchCurrentWaypoint(std::size_t index) {
  DroneState& drone = drones_[index];
  if (drone.current_waypoint >= drone.waypoints.poses.size()) return;
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();
  goal.pose = drone.waypoints.poses[drone.current_waypoint];
  drone.goal_publisher.publish(goal);
  drone.waypoint_sent_at = goal.header.stamp;
  drone.within_tolerance_since = ros::Time();
  drone.error.clear();
}

void MultiNaviGoalsPanel::advanceMission(std::size_t index, const ros::Time& now) {
  DroneState& drone = drones_[index];
  if (drone.mission_state != MissionState::Executing) return;
  if (!hasFreshOdometry(drone, now)) { drone.error = tr("里程计不可用或已过期"); updateStatus(index); return; }
  if (drone.waypoint_sent_at.isZero()) dispatchCurrentWaypoint(index);
  const geometry_msgs::Pose& waypoint = drone.waypoints.poses[drone.current_waypoint];
  if (!isAtWaypoint(drone, waypoint)) { drone.within_tolerance_since = ros::Time(); return; }
  if (drone.within_tolerance_since.isZero()) { drone.within_tolerance_since = now; return; }
  if ((now - drone.within_tolerance_since).toSec() < dwell_spin_->value()) return;
  ++drone.current_waypoint;
  if (drone.current_waypoint == drone.waypoints.poses.size()) {
    if (drone.loop) drone.current_waypoint = 0;
    else { drone.mission_state = MissionState::Completed; updateStatus(index); return; }
  }
  dispatchCurrentWaypoint(index); refreshDroneTab(index);
}

bool MultiNaviGoalsPanel::hasFreshOdometry(const DroneState& drone, const ros::Time& now) const {
  return !drone.odom_stamp.isZero() && (now - drone.odom_stamp).toSec() <= kOdomStaleSeconds;
}

bool MultiNaviGoalsPanel::isAtWaypoint(const DroneState& drone, const geometry_msgs::Pose& waypoint) const {
  const geometry_msgs::Pose& actual = drone.odometry.pose.pose;
  const double dx = actual.position.x - waypoint.position.x;
  const double dy = actual.position.y - waypoint.position.y;
  const double dz = g_ignore_z_for_arrival ? 0.0 : actual.position.z - waypoint.position.z;
  const double distance = std::hypot(std::hypot(dx, dy), dz);
  if (g_ignore_z_for_arrival) return distance <= position_tolerance_spin_->value();
  const double yaw_error = std::abs(normalizeAngle(tf::getYaw(actual.orientation) - tf::getYaw(waypoint.orientation)));
  return distance <= position_tolerance_spin_->value() &&
         yaw_error <= yaw_tolerance_spin_->value() * kPi / 180.0;
}

void MultiNaviGoalsPanel::resetMission(DroneState& drone, MissionState state) {
  drone.mission_state = state; drone.current_waypoint = 0; drone.waypoint_sent_at = ros::Time();
  drone.within_tolerance_since = ros::Time(); drone.error.clear();
}

void MultiNaviGoalsPanel::odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry, std::size_t index) {
  drones_[index].odometry = *odometry;
  drones_[index].odom_stamp = ros::Time::now();
}

void MultiNaviGoalsPanel::onTimer() {
  ros::spinOnce();
  const ros::Time now = ros::Time::now();
  for (int i = 0; i < active_drone_count_; ++i) {
    advanceMission(i, now);
    if (drones_[i].markers_dirty) publishMarkers(i);
    updateStatus(i);
  }
  if (wall_dirty_) publishWallMarker();
}

void MultiNaviGoalsPanel::refreshDroneTab(std::size_t index) {
  QTableWidget* table = waypoint_tables_[index];
  refreshing_table_ = true;
  table->clearContents(); table->setRowCount(static_cast<int>(drones_[index].waypoints.poses.size()));
  for (std::size_t row = 0; row < drones_[index].waypoints.poses.size(); ++row) {
    const geometry_msgs::Pose& pose = drones_[index].waypoints.poses[row];
    const double values[] = {pose.position.x, pose.position.y, pose.position.z, tf::getYaw(pose.orientation) * 180.0 / kPi};
    for (int column = 0; column < 4; ++column)
      table->setItem(static_cast<int>(row), column, new QTableWidgetItem(QString::number(values[column], 'f', 3)));
    if (drones_[index].mission_state == MissionState::Executing && row == drones_[index].current_waypoint)
      for (int column = 0; column < 4; ++column) table->item(static_cast<int>(row), column)->setBackground(QColor(204, 255, 204));
  }
  refreshing_table_ = false; updateStatus(index);
}

void MultiNaviGoalsPanel::refreshAllTabs() { for (std::size_t i = 0; i < kMaxDrones; ++i) refreshDroneTab(i); }

QString MultiNaviGoalsPanel::missionStateText(const DroneState& drone, const ros::Time& now) const {
  switch (drone.mission_state) {
    case MissionState::Idle: return tr("空闲");
    case MissionState::Executing: return hasFreshOdometry(drone, now) ? tr("执行中") : tr("等待里程计");
    case MissionState::Completed: return tr("已完成");
    case MissionState::Error: return tr("错误");
  }
  return tr("未知");
}

void MultiNaviGoalsPanel::updateStatus(std::size_t index) {
  const DroneState& drone = drones_[index]; const ros::Time now = ros::Time::now();
  const geometry_msgs::Point& p = drone.odometry.pose.pose.position;
  status_labels_[index]->setText(tr("状态: %1 | 航点: %2/%3 | 位置: (%4, %5, %6)%7")
      .arg(missionStateText(drone, now)).arg(drone.current_waypoint + (drone.waypoints.poses.empty() ? 0 : 1))
      .arg(drone.waypoints.poses.size()).arg(p.x, 0, 'f', 2).arg(p.y, 0, 'f', 2).arg(p.z, 0, 'f', 2)
      .arg(drone.error.isEmpty() ? QString() : tr(" | %1").arg(drone.error)));
}

void MultiNaviGoalsPanel::publishMarkers(std::size_t index) {
  DroneState& drone = drones_[index];
  deleteMarkers(index);
  const QColor colors[] = {QColor(255, 51, 51), QColor(255, 128, 255), QColor(128, 255, 128),
                           QColor(128, 128, 255), QColor(51, 51, 51), QColor(51, 255, 255)};
  const QColor color = colors[index];
  for (std::size_t i = 0; i < drone.waypoints.poses.size(); ++i) {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map"; arrow.header.stamp = ros::Time::now();
    arrow.ns = topicFor("navi_point_arrow", index).toStdString(); arrow.id = static_cast<int>(i);
    arrow.type = visualization_msgs::Marker::ARROW; arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose = drone.waypoints.poses[i]; arrow.scale.x = 0.35; arrow.scale.y = 0.08; arrow.scale.z = 0.08;
    arrow.color.r = color.redF(); arrow.color.g = color.greenF(); arrow.color.b = color.blueF(); arrow.color.a = 1.0;
    drone.marker_publisher.publish(arrow);
    visualization_msgs::Marker text = arrow;
    text.ns = topicFor("navi_point_number", index).toStdString(); text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.pose.position.z += 0.35; text.scale.z = 0.25; text.text = std::to_string(i + 1);
    drone.marker_publisher.publish(text);
  }
  drone.markers_dirty = false;
}

void MultiNaviGoalsPanel::deleteMarkers(std::size_t index) {
  DroneState& drone = drones_[index];
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.action = visualization_msgs::Marker::DELETEALL;
  drone.marker_publisher.publish(marker);
}

void MultiNaviGoalsPanel::updateWall() { wall_dirty_ = true; }

void MultiNaviGoalsPanel::publishWallMarker() {
  bool ok[4]; const double x1 = wall_start_x_edit_->text().toDouble(&ok[0]); const double y1 = wall_start_y_edit_->text().toDouble(&ok[1]);
  const double x2 = wall_end_x_edit_->text().toDouble(&ok[2]); const double y2 = wall_end_y_edit_->text().toDouble(&ok[3]);
  if (!(ok[0] && ok[1] && ok[2] && ok[3])) { showWarning(tr("墙体坐标必须为有效数字。")); wall_dirty_ = false; return; }
  const struct Segment { double x; double y; double sx; double sy; int id; } segments[] = {
      {(x1 + x2) / 2.0, y1, std::abs(x2 - x1), 0.2, 1}, {x1, (y1 + y2) / 2.0, 0.2, std::abs(y2 - y1), 2},
      {(x1 + x2) / 2.0, y2, std::abs(x2 - x1), 0.2, 3}, {x2, (y1 + y2) / 2.0, 0.2, std::abs(y2 - y1), 4}};
  for (const Segment& segment : segments) {
    visualization_msgs::Marker wall; wall.header.frame_id = "map"; wall.header.stamp = ros::Time::now();
    wall.ns = "point_wall"; wall.id = segment.id; wall.action = visualization_msgs::Marker::ADD; wall.type = visualization_msgs::Marker::CUBE;
    wall.pose.orientation.w = 1.0; wall.pose.position.x = segment.x; wall.pose.position.y = segment.y; wall.pose.position.z = 1.0;
    wall.scale.x = segment.sx; wall.scale.y = segment.sy; wall.scale.z = 2.0; wall.color.g = 1.0; wall.color.a = 0.2;
    wall_publisher_.publish(wall);
  }
  wall_dirty_ = false;
}

void MultiNaviGoalsPanel::saveGoals() {
  const QString file_name = QFileDialog::getSaveFileName(this, tr("保存目标点"), QDir::homePath() + "/multi_goals.txt", tr("文本文件 (*.txt)"));
  if (file_name.isEmpty()) return;
  std::ofstream file(file_name.toStdString());
  if (!file) { showWarning(tr("无法写入文件：%1").arg(file_name)); return; }
  for (std::size_t i = 0; i < kMaxDrones; ++i) {
    const auto& poses = drones_[i].waypoints.poses; file << "DRONE_" << std::setw(3) << std::setfill('0') << i + 1 << " " << poses.size() << "\n";
    for (const auto& pose : poses) file << std::fixed << std::setprecision(6) << pose.position.x << " " << pose.position.y << " " << pose.position.z << " " << tf::getYaw(pose.orientation) * 180.0 / kPi << "\n";
    file << "\n";
  }
}

void MultiNaviGoalsPanel::loadGoals() {
  const QString file_name = QFileDialog::getOpenFileName(this, tr("加载目标点"), QDir::homePath(), tr("文本文件 (*.txt)"));
  if (file_name.isEmpty()) return;
  std::ifstream file(file_name.toStdString());
  if (!file) { showWarning(tr("无法读取文件：%1").arg(file_name)); return; }
  for (std::size_t i = 0; i < kMaxDrones; ++i) { drones_[i].waypoints.poses.clear(); resetMission(drones_[i]); drones_[i].markers_dirty = true; }
  std::string name; int count = 0;
  while (file >> name >> count) {
    if (name.rfind("DRONE_", 0) != 0 || count < 0) { showWarning(tr("航点文件格式无效。")); return; }
    int index = -1; try { index = std::stoi(name.substr(6)) - 1; } catch (...) { showWarning(tr("航点文件格式无效。")); return; }
    for (int row = 0; row < count; ++row) {
      double x, y, z, yaw; if (!(file >> x >> y >> z >> yaw)) { showWarning(tr("航点数据不完整。")); return; }
      if (index >= 0 && index < static_cast<int>(kMaxDrones) && static_cast<int>(drones_[index].waypoints.poses.size()) < max_goals_)
        drones_[index].waypoints.poses.push_back(makePose(x, y, z, yaw * kPi / 180.0));
    }
  }
  refreshAllTabs();
}

void MultiNaviGoalsPanel::publishCommand(int command) { std_msgs::Int16 message; message.data = command; command_publisher_.publish(message); }

geometry_msgs::Pose MultiNaviGoalsPanel::makePose(double x, double y, double z, double yaw_radians) const {
  geometry_msgs::Pose pose; pose.position.x = x; pose.position.y = y; pose.position.z = z;
  pose.orientation = tf::createQuaternionMsgFromYaw(normalizeAngle(yaw_radians)); return pose;
}

double MultiNaviGoalsPanel::selectedYawRadians(std::size_t index) const {
  const DroneState& drone = drones_[index];
  return drone.odom_stamp.isZero() ? tf::getYaw(drone.waypoints.poses.back().orientation) : tf::getYaw(drone.odometry.pose.pose.orientation);
}

int MultiNaviGoalsPanel::currentDroneIndex() const {
  const int index = drone_tabs_ ? drone_tabs_->currentIndex() : -1;
  return index >= 0 && index < active_drone_count_ ? index : -1;
}

int MultiNaviGoalsPanel::selectedRow() const { const int index = currentDroneIndex(); return index < 0 ? -1 : waypoint_tables_[index]->currentRow(); }

void MultiNaviGoalsPanel::showWarning(const QString& message) { ROS_WARN_STREAM(message.toStdString()); }

}  // namespace navi_multi_goals_pub_rviz_plugin

PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::MultiNaviGoalsPanel, rviz::Panel)
