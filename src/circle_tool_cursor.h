#ifndef RVIZ_TOOL_CURSOR_CIRCLE_TOOL_CURSOR_H
#define RVIZ_TOOL_CURSOR_CIRCLE_TOOL_CURSOR_H

#include <rviz_tool_cursor/rviz_tool_cursor.h>

namespace rviz
{
  class FloatProperty;
}

namespace Ogre
{
  class ManualObject;
}

namespace rviz_tool_cursor
{

class CircleToolCursor : public ToolCursor
{
Q_OBJECT
public:

  CircleToolCursor();

  virtual ~CircleToolCursor();

public Q_SLOTS:

  virtual void updateToolVisualization() override;

protected:

  virtual Ogre::MovableObject* createToolVisualization() override;

  rviz::FloatProperty* radius_property_;

  const std::string object_name_ = "circle_tool_cursor";
};

} // namespace rviz_tool_cursor

#endif // RVIZ_TOOL_CURSOR_CIRCLE_TOOL_CURSOR_H
