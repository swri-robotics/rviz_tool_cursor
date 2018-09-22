#ifndef MESH_TOOL_CURSOR_H
#define MESH_TOOL_CURSOR_H

#include <rviz_tool_cursor/rviz_tool_cursor.h>
#include <OgreMaterial.h>

namespace Ogre
{
  class Entity;
}

namespace rviz_tool_cursor
{

class MeshToolCursor : public ToolCursor
{
Q_OBJECT
public:

  MeshToolCursor();

  virtual ~MeshToolCursor();

public Q_SLOTS:

  virtual void updateToolVisualization() override;

protected:

  virtual Ogre::MovableObject* createToolVisualization() override;

  rviz::StringProperty* mesh_file_;

  const std::string object_name_ = "mesh_tool_cursor";

  Ogre::MaterialPtr material_;
};

} // namespace rviz_tool_cursor

#endif // MESH_TOOL_CURSOR_H
