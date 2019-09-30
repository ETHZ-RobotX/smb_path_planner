/**
  * @author Luca Bartolomei, V4RL
  * @date   14.06.2019
  */

#include <functional>
#include <stdio.h>
#include <thread>

#include <QCheckBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/Twist.h>
#include <ros/names.h>
#include <ros/package.h>
#include <rviz/visualization_manager.h>
#include <smb_planner_msgs/PlannerService.h>
#include <std_srvs/Empty.h>
#include <yaml-cpp/yaml.h>

#include "smb_planner_rviz/edit_button.h"
#include "smb_planner_rviz/planning_panel.h"
#include "smb_planner_rviz/pose_widget.h"

namespace smb_planner_rviz {

PlanningPanel::PlanningPanel(QWidget *parent)
    : rviz::Panel(parent), nh_(ros::NodeHandle()), interactive_markers_(nh_) {
  try {
    createLayout();
  } catch(const YAML::ParserException& e) {
  	ROS_ERROR("[Planning Panel] YAML Exception: %s", e.what());
  }
}

void PlanningPanel::onInitialize() {
  interactive_markers_.initialize();
  interactive_markers_.setPoseUpdatedCallback(
      std::bind(&PlanningPanel::updateInteractiveMarkerPose, this,
                std::placeholders::_1));

  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  // Initialize all the markers.
  for (const auto &kv : pose_widget_map_) {
    geometry_msgs::Pose pose;
    kv.second->getPose(&pose);
    interactive_markers_.enableMarker(kv.first, pose);
  }
}

void PlanningPanel::createLayout() {
  QGridLayout *topic_layout = new QGridLayout;
  // Input the namespace.
  std::string filename(ros::package::getPath("smb_planner_common") +
      "/cfg/topics.yaml");
  YAML::Node lconf = YAML::LoadFile(filename);
  ROS_INFO_STREAM("[Planning Panel] Reading from " << filename);

  try {
		odometry_topic_ = QString::fromStdString(
		    lconf["stateEstimatorMsgName"].as<std::string>());
		namespace_ = QString::fromStdString("");
		global_planner_srv_name_ = QString::fromStdString(
		    lconf["globalPlanner/plannerServiceName"].as<std::string>());
		local_planner_srv_name_ = QString::fromStdString(
		    lconf["localPlanner/triggerServiceName"].as<std::string>());
  } catch(const YAML::ParserException& e) {
  	ROS_ERROR("[Planning Panel] YAML Exception: %s", e.what());
  	return;
  }

  odometry_sub_ = nh_.subscribe(namespace_.toStdString() + "/" +
                                    odometry_topic_.toStdString(),
                                1, &PlanningPanel::stateEstimateCallback, this);

  // Start and goal poses.
  QGridLayout *start_goal_layout = new QGridLayout;

  // Minimums...
  start_goal_layout->setColumnMinimumWidth(0, 50);
  start_goal_layout->setColumnMinimumWidth(1, 245);
  start_goal_layout->setColumnMinimumWidth(2, 80);
  start_goal_layout->setRowMinimumHeight(0, 55);
  start_goal_layout->setRowMinimumHeight(1, 55);
  start_goal_layout->setColumnStretch(0, 1);
  start_goal_layout->setColumnStretch(1, 9);
  start_goal_layout->setColumnStretch(2, 3);

  start_pose_widget_ = new PoseWidget("");
  goal_pose_widget_ = new PoseWidget("goal");
  EditButton *goal_edit_button = new EditButton("goal");
  registerPoseWidget(start_pose_widget_);
  registerPoseWidget(goal_pose_widget_);
  registerEditButton(goal_edit_button);

  start_goal_layout->addWidget(new QLabel("Current State:"), 0, 0,
                               Qt::AlignTop);
  start_goal_layout->addWidget(start_pose_widget_, 0, 1);
  start_goal_layout->addWidget(new QLabel("Goal:"), 1, 0, Qt::AlignTop);
  start_goal_layout->addWidget(goal_pose_widget_, 1, 1);
  start_goal_layout->addWidget(goal_edit_button, 1, 2);

  // Planner services and publications.
  QGridLayout *service_layout = new QGridLayout;
  planner_service_button_ = new QPushButton("Global Planner Service");
  start_local_planner_button_ = new QPushButton("Start Local Planner");
  service_layout->addWidget(planner_service_button_, 0, 0);
  service_layout->addWidget(start_local_planner_button_, 0, 1);

  // First the names, then the start/goal, then service buttons.
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(start_goal_layout);
  layout->addLayout(service_layout);
  setLayout(layout);

  // Hook up connections.
  connect(planner_service_button_, SIGNAL(released()), this,
          SLOT(callGlobalPlannerService()));
  connect(start_local_planner_button_, SIGNAL(released()), this,
          SLOT(callLocalPlannerService()));
}

// Set the topic name we are publishing to.
void PlanningPanel::setNamespace(const QString &new_namespace) {
  ROS_DEBUG_STREAM("Setting namespace from: " << namespace_.toStdString()
                                              << " to "
                                              << new_namespace.toStdString());
  // Only take action if the name has changed.
  if (new_namespace != namespace_) {
    namespace_ = new_namespace;
    Q_EMIT configChanged();

    std::string error;
    if (ros::names::validate(namespace_.toStdString(), error)) {
      odometry_sub_ = nh_.subscribe(
          namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
          &PlanningPanel::stateEstimateCallback, this);
    }
  }
}

// Set the topic name we are publishing to.
void PlanningPanel::setPlannerName(const QString &new_planner_name) {
  // Only take action if the name has changed.
  if (new_planner_name != planner_name_) {
    planner_name_ = new_planner_name;
    Q_EMIT configChanged();
  }
}

// Set the topic name we are publishing to.
void PlanningPanel::setOdometryTopic(const QString &new_odometry_topic) {
  // Only take action if the name has changed.
  if (new_odometry_topic != odometry_topic_) {
    odometry_topic_ = new_odometry_topic;
    Q_EMIT configChanged();

    std::string error;
    if (ros::names::validate(namespace_.toStdString(), error)) {
      odometry_sub_ = nh_.subscribe(
          namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
          &PlanningPanel::stateEstimateCallback, this);
    }
  }
}

void PlanningPanel::startEditing(const std::string &id) {
  // Make sure nothing else is being edited.
  if (!currently_editing_.empty()) {
    auto search = edit_button_map_.find(currently_editing_);
    if (search != edit_button_map_.end()) {
      search->second->finishEditing();
    }
  }
  currently_editing_ = id;
  // Get the current pose:
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end()) {
    return;
  }
  // Update fixed frame (may have changed since last time):
  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  geometry_msgs::Pose pose;
  search->second->getPose(&pose);
  interactive_markers_.enableSetPoseMarker(pose);
  interactive_markers_.disableMarker(id);
}

void PlanningPanel::finishEditing(const std::string &id) {
  if (currently_editing_ == id) {
    currently_editing_.clear();
    interactive_markers_.disableSetPoseMarker();
  }
  auto search = pose_widget_map_.find(id);
  if (search == pose_widget_map_.end()) {
    return;
  }
  ros::spinOnce();
  geometry_msgs::Pose pose;
  search->second->getPose(&pose);
  interactive_markers_.enableMarker(id, pose);
}

void PlanningPanel::registerPoseWidget(PoseWidget *widget) {
  pose_widget_map_[widget->id()] = widget;
  connect(widget,
          SIGNAL(poseUpdated(const std::string &, geometry_msgs::Pose &)), this,
          SLOT(widgetPoseUpdated(const std::string &, geometry_msgs::Pose &)));
}

void PlanningPanel::registerEditButton(EditButton *button) {
  edit_button_map_[button->id()] = button;
  connect(button, SIGNAL(startedEditing(const std::string &)), this,
          SLOT(startEditing(const std::string &)));
  connect(button, SIGNAL(finishedEditing(const std::string &)), this,
          SLOT(finishEditing(const std::string &)));
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlanningPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz::Config &config) {
  rviz::Panel::load(config);
  QString topic;
  QString ns;
}

void PlanningPanel::updateInteractiveMarkerPose(
    const geometry_msgs::Pose &pose) {
  if (currently_editing_.empty()) {
    return;
  }
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end()) {
    return;
  }
  search->second->setPose(pose);
}

void PlanningPanel::widgetPoseUpdated(const std::string &id,
                                      geometry_msgs::Pose &pose) {
  if (currently_editing_ == id) {
    interactive_markers_.setPose(pose);
  }
  interactive_markers_.updateMarkerPose(id, pose);
}

void PlanningPanel::callGlobalPlannerService() {
  std::string service_name =
      namespace_.toStdString() + global_planner_srv_name_.toStdString();
  geometry_msgs::PoseStamped start_pose, goal_pose;

  start_pose_widget_->getPose(&start_pose.pose);
  goal_pose_widget_->getPose(&goal_pose.pose);

  std::thread t([service_name, start_pose, goal_pose] {
    smb_planner_msgs::PlannerService req;
    req.request.goal_pose = goal_pose;

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        ROS_WARN_STREAM("Couldn't call service: " << service_name);
      }
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("Service Exception: " << e.what());
    }
  });
  t.detach();
}

void PlanningPanel::callLocalPlannerService() {
  std_srvs::Empty req;
  std::string service_name =
      namespace_.toStdString() + local_planner_srv_name_.toStdString();
  try {
    if (!ros::service::call(service_name, req)) {
      ROS_WARN_STREAM("Couldn't call service: " << service_name);
    }
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("Service Exception: " << e.what());
  }
}

void PlanningPanel::stateEstimateCallback(
    const geometry_msgs::PoseStamped &state_msg) {
  ROS_INFO_ONCE("Got pose callback.");
  pose_widget_map_[""]->setPose(state_msg.pose);
  interactive_markers_.updateMarkerPose("", state_msg.pose);
}

} // namespace smb_planner_rviz

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(smb_planner_rviz::PlanningPanel, rviz::Panel)
