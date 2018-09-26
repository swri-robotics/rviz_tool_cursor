#include <OgreManualObject.h>
#include <OgreMovableObject.h>
#include <OgreSceneManager.h>

#include <pluginlib/class_list_macros.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

#include "circle_tool_cursor.h"

const static unsigned POINTS = 1000;

namespace rviz_tool_cursor
{

CircleToolCursor::CircleToolCursor()
  : ToolCursor()
{
  radius_property_ = new rviz::FloatProperty("Tool Radius", 0.210f,
                                             "The radius of the tool circle display",
                                             getPropertyContainer(), SLOT(updateToolVisualization()), this);
}

CircleToolCursor::~CircleToolCursor()
{
  if(cursor_node_ != nullptr)
  {
    cursor_node_->detachObject(object_name_);
    scene_manager_->destroyManualObject(object_name_);
    scene_manager_->destroySceneNode(cursor_node_);
  }
}

Ogre::MovableObject* CircleToolCursor::createToolVisualization()
{  
  Ogre::ManualObject* manual = scene_manager_->createManualObject(object_name_);

  // Set the type of manual object
  manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

  // Loop over an arbitrary number of points in the circle
  const float& radius = radius_property_->getFloat();
  const Ogre::ColourValue& color = color_property_->getOgreColor();

  for(unsigned i = 0; i < POINTS; ++i)
  {
    float angle = static_cast<float>(i) / static_cast<float>(POINTS) * 2.0f * M_PI;
    float x = radius * std::cos(angle);
    float y = radius * std::sin(angle);

    manual->position(x, y, 0.0f);
    manual->colour(color);
    manual->index(i);
  }

  // Add a line strip from the last index to the first
  manual->index(0);
  manual->end();

  return manual;
}

void CircleToolCursor::updateToolVisualization()
{
  // Remove and destroy the first tool visualization from the scene node
  cursor_node_->detachObject(object_name_);
  scene_manager_->destroyManualObject(object_name_);

  // Create a new tool visualization
  movable_obj_ = createToolVisualization();

  // Attach the new tool visualization to the scene node
  cursor_node_->attachObject(movable_obj_);
  cursor_node_->setVisible(false);
}

} // namespace rviz_tool_cursor

PLUGINLIB_EXPORT_CLASS(rviz_tool_cursor::CircleToolCursor, rviz::Tool)
