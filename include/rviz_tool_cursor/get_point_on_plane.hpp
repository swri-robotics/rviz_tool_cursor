#pragma once

#include <OgrePrerequisites.h>

namespace rviz_rendering
{
/** @brief Given a viewport and an x,y position in window-pixel coordinates,
 *  find the point on a plane directly behind it, if any.
 * @return true if the intersection exists, false if it does not. */
bool getPointOnPlaneFromWindowXY(Ogre::Viewport* viewport,
                                 Ogre::Plane& plane,
                                 int window_x,
                                 int window_y,
                                 Ogre::Vector3& intersection_out);

}
