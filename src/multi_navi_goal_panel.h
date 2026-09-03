#ifndef MULTI_NAVI_GOAL_PANEL_H
#define MULTI_NAVI_GOAL_PANEL_H

#include <array>
#include <vector>

#include <QString>

#include <ros/ros.h>
#include <rviz/panel.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QTabWidget;
class QTimer;

namespace navi_multi_goals_pub_rviz_plugin {

class MultiNaviGoalsPanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit MultiNaviGoalsPanel(QWidget* parent = nullptr);

 private:
  void setActiveDroneCount(int count);
  void setMaxGoals(int count);
  void addWaypointFromRviz(const geometry_msgs::PoseStamped::ConstPtr& pose);
  void addRelativeWaypoint(double forward, double left, double vertical = 0.0);
  void rotateWaypoint(double degrees);
  void addEmptyWaypoint();
  void insertWaypoint();
  void deleteWaypoint();
  void moveWaypointUp();
  void moveWaypointDown();
  void onTableChanged(int row, int column);
  void startMission();
  void stepMission();
  void stopMission();
  void saveGoals();
  void loadGoals();
  void publishCommand(int command);
  void updateWall();
  void onTimer();

 private:
  static constexpr std::size_t kMaxDrones = 6;
  static constexpr int kDefaultMaxGoals = 50;
  static constexpr int kHardMaxGoals = 500;
  static constexpr double kDefaultPositionToleranceM = 0.1;
  static constexpr double kDefaultYawToleranceDeg = 10.0;
  static constexpr double kDefaultDwellSeconds = 0.5;
  static constexpr double kOdomStaleSeconds = 2.0;
  static constexpr double kWaypointTimeoutSeconds = 60.0;
  static constexpr double kDefaultMoveStepM = 0.5;
  static constexpr double kDefaultTurnStepDeg = 15.0;

  enum class MissionState { Idle, Executing, Completed, Error };

  struct DroneState {
    geometry_msgs::PoseArray waypoints;
    ros::Publisher goal_publisher;
    ros::Publisher marker_publisher;
    ros::Subscriber odom_subscriber;
    nav_msgs::Odometry odometry;
    ros::Time odom_stamp;
    MissionState mission_state{MissionState::Idle};
    std::size_t current_waypoint{0};
    ros::Time waypoint_sent_at;
    ros::Time within_tolerance_since;
    bool loop{false};
    bool markers_dirty{true};
    QString error;
  };

  void initializeRosInterfaces();
  void buildUi();
  void maximizeRvizWindow();
  void buildDroneTab(std::size_t index);
  void refreshDroneTab(std::size_t index);
  void refreshAllTabs();
  void updateStatus(std::size_t index);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry, std::size_t index);
  void dispatchCurrentWaypoint(std::size_t index);
  void advanceMission(std::size_t index, const ros::Time& now);
  bool hasFreshOdometry(const DroneState& drone, const ros::Time& now) const;
  bool isAtWaypoint(const DroneState& drone, const geometry_msgs::Pose& waypoint) const;
  void resetMission(DroneState& drone, MissionState state = MissionState::Idle);
  void publishMarkers(std::size_t index);
  void deleteMarkers(std::size_t index);
  void publishWallMarker();
  void setWaypointFromTable(std::size_t drone_index, int row);
  bool appendWaypoint(std::size_t index, const geometry_msgs::Pose& pose);
  geometry_msgs::Pose makePose(double x, double y, double z, double yaw_radians) const;
  double selectedYawRadians(std::size_t index) const;
  int currentDroneIndex() const;
  int selectedRow() const;
  QString missionStateText(const DroneState& drone, const ros::Time& now) const;
  void showWarning(const QString& message);

  ros::NodeHandle nh_;
  ros::Subscriber rviz_goal_subscriber_;
  ros::Publisher command_publisher_;
  ros::Publisher wall_publisher_;
  std::array<DroneState, kMaxDrones> drones_;

  int active_drone_count_{6};
  int max_goals_{kDefaultMaxGoals};
  bool refreshing_table_{false};
  bool wall_dirty_{true};

  QTabWidget* drone_tabs_{nullptr};
  QSpinBox* active_drone_spin_{nullptr};
  QSpinBox* max_goals_spin_{nullptr};
  QDoubleSpinBox* default_z_spin_{nullptr};
  QDoubleSpinBox* move_step_spin_{nullptr};
  QDoubleSpinBox* turn_step_spin_{nullptr};
  QDoubleSpinBox* position_tolerance_spin_{nullptr};
  QDoubleSpinBox* yaw_tolerance_spin_{nullptr};
  QDoubleSpinBox* dwell_spin_{nullptr};
  QLineEdit* wall_start_x_edit_{nullptr};
  QLineEdit* wall_start_y_edit_{nullptr};
  QLineEdit* wall_end_x_edit_{nullptr};
  QLineEdit* wall_end_y_edit_{nullptr};
  std::array<QTableWidget*, kMaxDrones> waypoint_tables_{};
  std::array<QLabel*, kMaxDrones> status_labels_{};
  std::array<QPushButton*, kMaxDrones> loop_buttons_{};
  QTimer* timer_{nullptr};
};

}  // namespace navi_multi_goals_pub_rviz_plugin

#endif  // MULTI_NAVI_GOAL_PANEL_H
