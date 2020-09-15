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

#include <ros/names.h>
#include <ros/package.h>
#include <rviz/visualization_manager.h>
#include <std_srvs/Empty.h>

#include "smb_planner_rviz/edit_button.h"
#include "smb_planner_rviz/planning_panel.h"
#include "smb_planner_rviz/pose_widget.h"

namespace smb_navigation_rviz
{

PlanningPanel::PlanningPanel(QWidget* parent)
    : rviz::Panel(parent), nh_(ros::NodeHandle()), interactive_markers_(nh_)
{
  createLayout();
}

void PlanningPanel::onInitialize()
{
  interactive_markers_.initialize();
  interactive_markers_.setPoseUpdatedCallback(
      std::bind(&PlanningPanel::updateInteractiveMarkerPose, this,
                std::placeholders::_1));

  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  // Initialize all the markers.
  for (const auto& kv : pose_widget_map_)
  {
    geometry_msgs::Pose pose;
    kv.second->getPose(&pose);
    interactive_markers_.enableMarker(kv.first, pose);
  }
}

void PlanningPanel::createLayout()
{
  QGridLayout* topic_layout = new QGridLayout;
  topic_layout->addWidget(new QLabel("Frame ID:"), 0, 0);
  frame_id_editor_ = new QLineEdit;
  frame_id_editor_->setText("world");
  topic_layout->addWidget(frame_id_editor_, 0, 1);

  topic_layout->addWidget(new QLabel("Odometry topic:"), 1, 0);
  odometry_topic_editor_ = new QLineEdit;
  odometry_topic_editor_->setText("/odometry");
  topic_layout->addWidget(odometry_topic_editor_, 1, 1);

  // Goal poses.
  QGridLayout* start_goal_layout = new QGridLayout;

  // Minimums...
  start_goal_layout->setColumnMinimumWidth(0, 50);
  start_goal_layout->setColumnMinimumWidth(1, 245);
  start_goal_layout->setColumnMinimumWidth(2, 80);
  start_goal_layout->setRowMinimumHeight(0, 55);
  start_goal_layout->setRowMinimumHeight(1, 55);
  start_goal_layout->setColumnStretch(0, 1);
  start_goal_layout->setColumnStretch(1, 9);
  start_goal_layout->setColumnStretch(2, 3);

  start_pose_widget_ = new PoseWidget("start");
  goal_pose_widget_ = new PoseWidget("goal");
  EditButton* goal_edit_button = new EditButton("goal");
  registerPoseWidget(start_pose_widget_);
  registerPoseWidget(goal_pose_widget_);
  registerEditButton(goal_edit_button);

  start_goal_layout->addWidget(new QLabel("Start:"), 0, 0, Qt::AlignTop);
  start_goal_layout->addWidget(start_pose_widget_, 0, 1);
  start_goal_layout->addWidget(new QLabel("Goal:"), 1, 0, Qt::AlignTop);
  start_goal_layout->addWidget(goal_pose_widget_, 1, 1);
  start_goal_layout->addWidget(goal_edit_button, 1, 2);

  // Planner services and publications.
  QGridLayout* service_layout = new QGridLayout;
  planner_service_button_ = new QPushButton("Start Planning");
  service_layout->addWidget(planner_service_button_, 0, 0);

  // First the names, then the start/goal, then service buttons.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(start_goal_layout);
  layout->addLayout(service_layout);
  setLayout(layout);

  // Set up the ROS communication - with the initial values
  odometry_topic_ = odometry_topic_editor_->text();
  frame_id_ = frame_id_editor_->text();

  odometry_sub_ = nh_.subscribe(odometry_topic_.toStdString(), 1,
                                &PlanningPanel::odometryCallback, this);
  move_base_sub_ = nh_.subscribe("/move_base_simple/goal", 1,
                                 &PlanningPanel::moveBaseGoalCallback, this);
  move_base_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);

  // Hook up connections.
  connect(frame_id_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateFrameId()));
  connect(odometry_topic_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateOdometryTopic()));
  connect(planner_service_button_, SIGNAL(released()), this,
          SLOT(callMoveBasePlanner()));
}

void PlanningPanel::updateFrameId()
{
  // Only take action if the name has changed.
  QString new_frame_id = frame_id_editor_->text();
  if (new_frame_id != frame_id_)
  {
    frame_id_ = new_frame_id;
    interactive_markers_.setFrameId(frame_id_.toStdString());
    Q_EMIT configChanged();
  }
}

void PlanningPanel::updateOdometryTopic()
{
  QString new_odometry_topic = odometry_topic_editor_->text();

  // Only take action if the name has changed.
  if (new_odometry_topic != odometry_topic_)
  {
    odometry_topic_ = new_odometry_topic;
    Q_EMIT configChanged();

    odometry_sub_ = nh_.subscribe(odometry_topic_.toStdString(), 1,
                                  &PlanningPanel::odometryCallback, this);
  }
}

void PlanningPanel::startEditing(const std::string& id)
{
  // Make sure nothing else is being edited.
  if (!currently_editing_.empty())
  {
    auto search = edit_button_map_.find(currently_editing_);
    if (search != edit_button_map_.end())
    {
      search->second->finishEditing();
    }
  }
  currently_editing_ = id;
  // Get the current pose:
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end())
  {
    return;
  }
  // Update fixed frame (may have changed since last time):
  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  geometry_msgs::Pose pose;
  search->second->getPose(&pose);
  interactive_markers_.enableSetPoseMarker(pose);
  interactive_markers_.disableMarker(id);
}

void PlanningPanel::finishEditing(const std::string& id)
{
  if (currently_editing_ == id)
  {
    currently_editing_.clear();
    interactive_markers_.disableSetPoseMarker();
  }
  auto search = pose_widget_map_.find(id);
  if (search == pose_widget_map_.end())
  {
    return;
  }
  ros::spinOnce();
  geometry_msgs::Pose pose;
  search->second->getPose(&pose);
  interactive_markers_.enableMarker(id, pose);
}

void PlanningPanel::registerPoseWidget(PoseWidget* widget)
{
  pose_widget_map_[widget->id()] = widget;
  connect(widget, SIGNAL(poseUpdated(const std::string&, geometry_msgs::Pose&)),
          this,
          SLOT(widgetPoseUpdated(const std::string&, geometry_msgs::Pose&)));
}

void PlanningPanel::registerEditButton(EditButton* button)
{
  edit_button_map_[button->id()] = button;
  connect(button, SIGNAL(startedEditing(const std::string&)), this,
          SLOT(startEditing(const std::string&)));
  connect(button, SIGNAL(finishedEditing(const std::string&)), this,
          SLOT(finishEditing(const std::string&)));
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlanningPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString topic;
  QString ns;
}

void PlanningPanel::updateInteractiveMarkerPose(const geometry_msgs::Pose& pose)
{
  if (currently_editing_.empty())
  {
    return;
  }
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end())
  {
    return;
  }
  search->second->setPose(pose);
}

void PlanningPanel::widgetPoseUpdated(const std::string& id,
                                      geometry_msgs::Pose& pose)
{
  if (currently_editing_ == id)
  {
    interactive_markers_.setPose(pose);
  }
  interactive_markers_.updateMarkerPose(id, pose);
}

void PlanningPanel::odometryCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO_ONCE("Got odometry callback.");
  pose_widget_map_["start"]->setPose(msg.pose.pose);
}

void PlanningPanel::moveBaseGoalCallback(const geometry_msgs::PoseStamped& msg)
{
  goal_pose_widget_->setPose(msg.pose);
}

void PlanningPanel::callMoveBasePlanner()
{
  if (move_base_pub_.getNumSubscribers() == 0)
  {
    ROS_ERROR("No subscribers to move base goal publisher!");
    return;
  }

  geometry_msgs::PoseStamped goal_pose;
  goal_pose_widget_->getPose(&goal_pose.pose);

  // Update the header
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.header.seq = 0;
  goal_pose.header.frame_id = frame_id_.toStdString();

  move_base_pub_.publish(goal_pose);
  ROS_INFO("Published goal pose to move base");
}

} // namespace smb_navigation_rviz

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(smb_navigation_rviz::PlanningPanel, rviz::Panel)
