/*
 * @brief Pose widget class to show a table in the planning panel
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: 14.06.2019
 */

#pragma once

#ifndef Q_MOC_RUN
#include <QItemDelegate>
#include <QLineEdit>
#include <QStringList>
#include <QTableWidget>
#endif

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

class QLineEdit;
namespace smb_navigation_rviz
{

// This is a little widget that allows pose input.
class PoseWidget : public QWidget
{
  Q_OBJECT
public:
  explicit PoseWidget(const std::string& id, QWidget* parent = 0);

  std::string id() const { return id_; }
  void setId(const std::string& id) { id_ = id; }

  void getPose(geometry_msgs::Pose* pose) const;
  void setPose(const geometry_msgs::Pose& pose);

  virtual QSize sizeHint() const { return table_widget_->sizeHint(); }

Q_SIGNALS:
  void poseUpdated(const std::string& id, geometry_msgs::Pose& pose);

public Q_SLOTS:
  void itemChanged(QTableWidgetItem* item);

protected:
  // Set up the layout, only called by the constructor.
  void createTable();

  // QT stuff:
  QTableWidget* table_widget_;

  // QT state:
  QStringList table_headers_;

  // Other state:
  // This is the ID that binds the button to the pose widget.
  std::string id_;
};

class DoubleTableDelegate : public QItemDelegate
{
public:
  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const;
};

} // end namespace smb_navigation_rviz
