#ifndef RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H
#define RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H

#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <rviz/tool.h>

namespace Ogre
{
class SceneNode;
}  // namespace Ogre

namespace rviz
{
class StringProperty;
class IntProperty;
class ColorProperty;
class BoolProperty;
class FloatProperty;
}  // namespace rviz

namespace rviz_tool_cursor
{
class ToolCursor : public rviz::Tool
{
  Q_OBJECT
public:
  ToolCursor();
  ~ToolCursor();

  virtual void onInitialize() override;
  virtual void activate() override;
  virtual void deactivate() override;
  virtual int processMouseEvent(rviz::ViewportMouseEvent& event) override;

public Q_SLOTS:
  void updateTopic();
  void updateToolVisualization();

  void updateSelectionVisual();
  void updatePtsColor();
  void updatePtsSize();
  void updateLinesColor();

protected:
  virtual Ogre::MovableObject* createToolVisualization();
  static void updateMaterialColor(Ogre::MaterialPtr material, const QColor& color,
                                  const bool override_self_illumination = true);

  QCursor std_cursor_;
  QCursor hit_cursor_;

  Ogre::SceneNode* cursor_node_{ nullptr };
  Ogre::SceneNode* selection_node_{ nullptr };
  Ogre::MovableObject* cursor_object_{ nullptr };

  Ogre::ManualObject* pts_vis_{ nullptr };
  Ogre::ManualObject* lines_vis_{ nullptr };
  Ogre::MaterialPtr pts_material_;
  Ogre::MaterialPtr lines_material_;

  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  ros::Publisher point_pub_;
  ros::Publisher pose_array_pub_;
  geometry_msgs::PoseArray selection_;

  rviz::StringProperty* pose_topic_property_;
  rviz::StringProperty* point_topic_property_;
  rviz::StringProperty* pose_array_topic_property_;
  rviz::IntProperty* patch_size_property_;

  // Selection
  rviz::BoolProperty* show_points_property_;
  rviz::BoolProperty* show_lines_property_;
  rviz::BoolProperty* lasso_mode_property_;
  rviz::BoolProperty* close_loop_property_;
  rviz::ColorProperty* pt_color_property_;
  rviz::ColorProperty* line_color_property_;
  rviz::FloatProperty* pt_size_property_;
};

}  // namespace rviz_tool_cursor

#endif  // RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H
