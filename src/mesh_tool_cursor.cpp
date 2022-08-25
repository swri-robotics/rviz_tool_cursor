#include "mesh_tool_cursor.h"

#include <OgreEntity.h>
#include <OgreMesh.h>
#include <OgreMovableObject.h>
#include <OgreSceneManager.h>
#include <OgreSubEntity.h>
#include <rviz/mesh_loader.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/string_property.h>

const static std::string COLOR_NAME = "mesh_cursor_tool_color";
const static std::string DEFAULT_MESH_RESOURCE = "package://rviz_tool_cursor/resources/default.stl";

namespace rviz_tool_cursor
{
MeshToolCursor::MeshToolCursor()
{
  shortcut_key_ = 'm';

  mesh_file_ = new rviz::StringProperty("Mesh Filename", QString(DEFAULT_MESH_RESOURCE.c_str()),
                                        "The mesh resource to display as a cursor", getPropertyContainer(),
                                        SLOT(updateToolVisualization()), this);

  material_ = Ogre::MaterialManager::getSingletonPtr()->create(COLOR_NAME,
                                                               Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  const Ogre::ColourValue& color = color_property_->getOgreColor();
  material_->getTechnique(0)->getPass(0)->setDiffuse(color.r, color.g, color.b, 1.0);
  material_->getTechnique(0)->getPass(0)->setAmbient(color.r, color.g, color.b);
}

MeshToolCursor::~MeshToolCursor()
{
  Ogre::MaterialManager::getSingletonPtr()->remove(COLOR_NAME);
}

Ogre::MovableObject* MeshToolCursor::createToolVisualization()
{
  // Attempt to load the mesh
  Ogre::MeshPtr mesh = rviz::loadMeshFromResource(mesh_file_->getStdString());
  if (mesh.isNull())
  {
    ROS_WARN("Loading default mesh...");

    // Load a default mesh
    mesh = rviz::loadMeshFromResource(DEFAULT_MESH_RESOURCE);
  }

  Ogre::Entity* entity = scene_manager_->createEntity(getObjectName(), mesh);
  for (unsigned i = 0; i < entity->getNumSubEntities(); ++i)
  {
    Ogre::SubEntity* sub = entity->getSubEntity(i);
    sub->setMaterial(material_);
  }

  return entity;
}

}  // namespace rviz_tool_cursor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_tool_cursor::MeshToolCursor, rviz::Tool)
