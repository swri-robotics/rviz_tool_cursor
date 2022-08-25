#ifndef MESH_TOOL_CURSOR_H
#define MESH_TOOL_CURSOR_H

#include <rviz_tool_cursor/rviz_tool_cursor.h>

namespace rviz_tool_cursor
{
class MeshToolCursor : public ToolCursor
{
  Q_OBJECT
public:
  MeshToolCursor();
  virtual ~MeshToolCursor() override;

public Q_SLOTS:
  void updateColor();

protected:
  virtual Ogre::MovableObject* createToolVisualization() override;

  rviz::StringProperty* mesh_file_;
  rviz::ColorProperty* color_property_;
  Ogre::MaterialPtr material_;
};

}  // namespace rviz_tool_cursor

#endif  // MESH_TOOL_CURSOR_H
