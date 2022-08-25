#ifndef MESH_TOOL_CURSOR_H
#define MESH_TOOL_CURSOR_H

#include <OgreMaterial.h>
#include <rviz_tool_cursor/rviz_tool_cursor.h>

namespace rviz_tool_cursor
{
class MeshToolCursor : public ToolCursor
{
  Q_OBJECT
public:
  MeshToolCursor();
  virtual ~MeshToolCursor() override;

protected:
  virtual Ogre::MovableObject* createToolVisualization() override;
  std::string getObjectName() const override
  {
    return "mesh_tool_cursor";
  }

  rviz::StringProperty* mesh_file_;
  Ogre::MaterialPtr material_;
};

}  // namespace rviz_tool_cursor

#endif  // MESH_TOOL_CURSOR_H
