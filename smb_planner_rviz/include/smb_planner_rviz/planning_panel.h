/**
  * @author Luca Bartolomei, V4RL
  * @brief  Rviz panel to select global goal, trigger the global planner and
  *         start the local planner
  * @date   14.06.2019
  */

#pragma once

#ifndef Q_MOC_RUN
#include "smb_planner_rviz/edit_button.h"
#include "smb_planner_rviz/planning_interactive_markers.h"
#include "smb_planner_rviz/pose_widget.h"
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

class QLineEdit;
class QCheckBox;
namespace smb_planner_rviz {

class PlanningPanel : public rviz::Panel {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  explicit PlanningPanel(QWidget *parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;
  virtual void onInitialize();

  // All the settings to manage pose <-> edit mapping.
  void registerPoseWidget(PoseWidget *widget);
  void registerEditButton(EditButton *button);

  // Callback from ROS when the pose updates:
  void updateInteractiveMarkerPose(const geometry_msgs::Pose &pose);
  // And when we get robot odometry:
  void stateEstimateCallback(const geometry_msgs::PoseStamped &state_msg);

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  void startEditing(const std::string &id);
  void finishEditing(const std::string &id);
  void widgetPoseUpdated(const std::string &id, geometry_msgs::Pose &pose);
  void callGlobalPlannerService();
  void callLocalPlannerService();

protected:
  // Set up the layout, only called by the constructor.
  void createLayout();
  void setNamespace(const QString &new_namespace);
  void setPlannerName(const QString &new_planner_name);
  void setOdometryTopic(const QString &new_odometry_topic);

  // ROS Stuff:
  ros::NodeHandle nh_;
  ros::Subscriber odometry_sub_;

  // QT stuff:
  PoseWidget *start_pose_widget_;
  PoseWidget *goal_pose_widget_;
  QPushButton *planner_service_button_;
  QPushButton *start_local_planner_button_;

  // Keep track of all the pose <-> button widgets as they're related:
  std::map<std::string, PoseWidget *> pose_widget_map_;
  std::map<std::string, EditButton *> edit_button_map_;
  // ROS state:
  PlanningInteractiveMarkers interactive_markers_;

  // QT state:
  QString namespace_;
  QString planner_name_;
  QString odometry_topic_;
  QString local_planner_srv_name_;
  QString global_planner_srv_name_;

  // Other state:
  std::string currently_editing_;
};

} // end namespace smb_planner_rviz
