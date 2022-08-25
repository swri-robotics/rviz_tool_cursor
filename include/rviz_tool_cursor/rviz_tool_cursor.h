#ifndef RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H
#define RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H

#include <OgreMaterial.h>
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
  void updateToolVisualization();

protected:
  virtual std::string getObjectName() const
  {
    return "tool_cursor";
  }
  virtual Ogre::MovableObject* createToolVisualization();
  static void updateMaterialColor(Ogre::MaterialPtr material, const QColor& color,
                                  const bool override_self_illumination = true);

  QCursor std_cursor_;
  QCursor hit_cursor_;

  Ogre::SceneNode* cursor_node_;
  Ogre::MovableObject* cursor_object_;

  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  ros::Publisher point_pub_;

  rviz::StringProperty* pose_topic_property_;
  rviz::StringProperty* point_topic_property_;
  rviz::IntProperty* patch_size_property_;
};

}  // namespace rviz_tool_cursor

#endif  // RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H
