#ifndef RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H
#define RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <rviz/tool.h>

namespace Ogre
{
class SceneNode;
class MovableObject;
}  // namespace Ogre

namespace rviz
{
class StringProperty;
class IntProperty;
class ColorProperty;
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

  virtual void updateToolVisualization() = 0;

  void updatePatchSize();

protected:
  virtual Ogre::MovableObject* createToolVisualization() = 0;

  QCursor std_cursor_;

  QCursor hit_cursor_;

  Ogre::SceneNode* cursor_node_;

  Ogre::MovableObject* movable_obj_;

  ros::NodeHandle nh_;

  ros::Publisher pose_pub_;

  ros::Publisher point_pub_;

  rviz::StringProperty* pose_topic_property_;

  rviz::StringProperty* point_topic_property_;

  rviz::IntProperty* patch_size_property_;

  rviz::ColorProperty* color_property_;
};

}  // namespace rviz_tool_cursor

#endif  // RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H
