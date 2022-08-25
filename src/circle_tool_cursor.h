#ifndef RVIZ_TOOL_CURSOR_CIRCLE_TOOL_CURSOR_H
#define RVIZ_TOOL_CURSOR_CIRCLE_TOOL_CURSOR_H

#include <rviz_tool_cursor/rviz_tool_cursor.h>

namespace rviz
{
class FloatProperty;
}

namespace rviz_tool_cursor
{
class CircleToolCursor : public ToolCursor
{
  Q_OBJECT
public:
  CircleToolCursor();
  virtual ~CircleToolCursor() override;

protected:
  virtual Ogre::MovableObject* createToolVisualization() override;

  rviz::FloatProperty* radius_property_;
  rviz::ColorProperty* color_property_;
};

}  // namespace rviz_tool_cursor

#endif  // RVIZ_TOOL_CURSOR_CIRCLE_TOOL_CURSOR_H
