#include <OgreEntity.h>
#include <OgreSubEntity.h>
#include <OgreMaterialManager.h>
#include <OgreMovableObject.h>
#include <OgreMesh.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include <pluginlib/class_list_macros.h>

#include "mesh_tool_cursor.h"
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/color_property.hpp>

#include <rviz_rendering/mesh_loader.hpp>

const static std::string COLOR_NAME = "mesh_cursor_tool_color";
const static std::string DEFAULT_MESH_RESOURCE = "package://rviz_tool_cursor/resources/default.stl";

namespace rviz_tool_cursor
{

MeshToolCursor::MeshToolCursor()
{
  mesh_file_ = new rviz_common::properties::StringProperty("Mesh Filename", QString(DEFAULT_MESH_RESOURCE.c_str()),
                                         "The mesh resource to display as a cursor",
                                         getPropertyContainer(), SLOT(updateToolVisualization()), this);

  material_ = Ogre::MaterialManager::getSingletonPtr()->create(COLOR_NAME, "rviz_rendering");

  const Ogre::ColourValue& color = color_property_->getOgreColor();
  material_->getTechnique(0)->getPass(0)->setDiffuse(color.r, color.g, color.b, 1.0);
  material_->getTechnique(0)->getPass(0)->setAmbient(color.r, color.g, color.b);
}

MeshToolCursor::~MeshToolCursor()
{
  if(cursor_node_ != nullptr)
  {
    cursor_node_->detachObject(object_name_);
    scene_manager_->destroyEntity(object_name_);
    scene_manager_->destroySceneNode(cursor_node_);
  }
  Ogre::MaterialManager::getSingletonPtr()->remove(COLOR_NAME, "rviz_rendering");
}

Ogre::MovableObject* MeshToolCursor::createToolVisualization()
{
  // Attempt to load the mesh
  Ogre::MeshPtr mesh = rviz_rendering::loadMeshFromResource(mesh_file_->getStdString());
  if(mesh.isNull())
  {
//    ROS_WARN("Loading default mesh...");

    // Load a default mesh
    mesh = rviz_rendering::loadMeshFromResource(DEFAULT_MESH_RESOURCE);
  }

  Ogre::Entity* entity = scene_manager_->createEntity(object_name_, mesh);
  for(unsigned i = 0; i < entity->getNumSubEntities(); ++i)
  {
    Ogre::SubEntity* sub = entity->getSubEntity(i);
    sub->setMaterial(material_);
  }

  return entity;
}

void MeshToolCursor::updateToolVisualization()
{
  const Ogre::ColourValue& color = color_property_->getOgreColor();
  material_->getTechnique(0)->getPass(0)->setDiffuse(color.r, color.g, color.b, 1.0);
  material_->getTechnique(0)->getPass(0)->setAmbient(color.r, color.g, color.b);

  // Remove and destroy the first tool visualization from the scene node
  scene_manager_->destroyEntity(object_name_);

  // Create a new tool visualization
  movable_obj_ = createToolVisualization();

  // Attach the new tool visualization to the scene node
  cursor_node_->attachObject(movable_obj_);
  cursor_node_->setVisible(false);
}

} // namespace rviz_tool_cursor

PLUGINLIB_EXPORT_CLASS(rviz_tool_cursor::MeshToolCursor, rviz_common::Tool)
