/*
 * ompl_typed.h
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: 14.06.2019
 */

#include <QApplication>
#include <QDesktopWidget>
#include <QHeaderView>
#include <QTableView>
#include <QTableWidget>

#include "smb_planner_rviz/pose_widget.h"

namespace smb_navigation_rviz
{

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

PoseWidget::PoseWidget(const std::string& id, QWidget* parent)
    : QWidget(parent), id_(id)
{
  createTable();
}

void PoseWidget::createTable()
{
  table_widget_ = new QTableWidget(this);
  table_widget_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  table_widget_->setRowCount(1);
  table_widget_->setColumnCount(3);
  table_headers_ << "x [m]"
                 << "y [m]" << QString::fromUtf8("yaw [°]");
  table_widget_->setHorizontalHeaderLabels(table_headers_);
  table_widget_->verticalHeader()->setVisible(false);
  table_widget_->setShowGrid(true);
  table_widget_->setItemDelegate(new DoubleTableDelegate);

  table_widget_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  table_widget_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  table_widget_->resizeColumnsToContents();
  // From:
  // https://stackoverflow.com/questions/8766633/how-to-determine-the-correct-size-of-a-qtablewidget
  table_widget_->setFixedSize(table_widget_->horizontalHeader()->length(),
                              table_widget_->verticalHeader()->length() +
                                  table_widget_->horizontalHeader()->height());

  for (int i = 0; i < 3; i++)
  {
    table_widget_->setItem(0, i, new QTableWidgetItem("0.00"));
    table_widget_->item(0, i)->setTextAlignment(Qt::AlignRight |
                                                Qt::AlignVCenter);
  }

  connect(table_widget_, SIGNAL(itemChanged(QTableWidgetItem*)), this,
          SLOT(itemChanged(QTableWidgetItem*)));
}

void PoseWidget::getPose(geometry_msgs::Pose* point) const
{
  point->position.x = table_widget_->item(0, 0)->text().toDouble();
  point->position.y = table_widget_->item(0, 1)->text().toDouble();
  point->position.z = 0.5; // TODO set right height
  point->orientation = tf::createQuaternionMsgFromYaw(
      table_widget_->item(0, 2)->text().toDouble() * kDegToRad);
}

void PoseWidget::setPose(const geometry_msgs::Pose& pose)
{
  table_widget_->blockSignals(true);
  table_widget_->item(0, 0)->setText(QString::number(pose.position.x, 'f', 2));
  table_widget_->item(0, 1)->setText(QString::number(pose.position.y, 'f', 2));
  table_widget_->item(0, 2)->setText(
      QString::number(tf::getYaw(pose.orientation) * kRadToDeg, 'f', 2));
  table_widget_->blockSignals(false);

  geometry_msgs::Pose pose_copy(pose);
  Q_EMIT poseUpdated(id_, pose_copy);
}

void PoseWidget::itemChanged(QTableWidgetItem* item)
{
  geometry_msgs::Pose point;
  getPose(&point);
  Q_EMIT poseUpdated(id_, point);
}

QWidget* DoubleTableDelegate::createEditor(QWidget* parent,
                                           const QStyleOptionViewItem& option,
                                           const QModelIndex& index) const
{
  // From:
  // https://stackoverflow.com/questions/22708623/qtablewidget-only-numbers-permitted
  QLineEdit* line_edit = new QLineEdit(parent);
  // Set validator
  QDoubleValidator* validator = new QDoubleValidator(line_edit);
  line_edit->setValidator(validator);
  line_edit->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  return line_edit;
}

} // namespace smb_navigation_rviz
