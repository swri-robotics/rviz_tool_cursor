#ifndef RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H
#define RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H

#include <rviz/tool.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

namespace Ogre
{
  class SceneNode;
  class ManualObject;
}

namespace rviz
{
  class StringProperty;
  class IntProperty;
  class ColorProperty;
}

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

  void updatePatchSize();

protected:

  virtual Ogre::ManualObject* createToolVisualization() = 0;

  QCursor std_cursor_;

  QCursor hit_cursor_;

  Ogre::SceneNode* cursor_node_;

  const std::string ogre_object_name_ = "tool";

  ros::NodeHandle nh_;

  ros::Publisher pub_;

  rviz::StringProperty* topic_property_;

  rviz::IntProperty* patch_size_property_;

  rviz::ColorProperty* color_property_;
};

} // namespace rviz_tool_cursor

#endif // RVIZ_TOOL_CURSOR_RVIZ_TOOL_CURSOR_H
