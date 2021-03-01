#pragma once

#include <rviz_common/tool.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace Ogre
{
  class SceneNode;
  class MovableObject;
}

namespace rviz_common
{
  class RenderPanel;
}

namespace rviz_common
{
namespace properties
{
  class StringProperty;
  class IntProperty;
  class ColorProperty;
  class FloatProperty;
}
}

namespace rviz_tool_cursor
{

class ToolCursor : public rviz_common::Tool
{
Q_OBJECT
public:

  ToolCursor();

  ~ToolCursor();

  virtual void onInitialize() override;

  virtual void activate() override;

  virtual void deactivate() override;

  virtual int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

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

  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;

  rviz_common::properties::StringProperty* topic_property_;

  rviz_common::properties::IntProperty* patch_size_property_;

  rviz_common::properties::ColorProperty* color_property_;
};

} // namespace rviz_tool_cursor
