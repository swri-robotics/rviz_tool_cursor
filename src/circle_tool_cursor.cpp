#include "circle_tool_cursor.h"

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

const static unsigned POINTS = 1000;

namespace rviz_tool_cursor
{
CircleToolCursor::CircleToolCursor() : ToolCursor()
{
  radius_property_ = new rviz::FloatProperty("Tool Radius", 0.210f, "The radius of the tool circle display",
                                             getPropertyContainer(), SLOT(updateToolVisualization()), this);
}

CircleToolCursor::~CircleToolCursor()
{
}

Ogre::MovableObject* CircleToolCursor::createToolVisualization()
{
  Ogre::ManualObject* manual = scene_manager_->createManualObject(getObjectName());

  // Set the type of manual object
  manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

  // Loop over an arbitrary number of points in the circle
  const float& radius = radius_property_->getFloat();
  const Ogre::ColourValue& color = color_property_->getOgreColor();

  for (unsigned i = 0; i < POINTS; ++i)
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

}  // namespace rviz_tool_cursor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_tool_cursor::CircleToolCursor, rviz::Tool)
