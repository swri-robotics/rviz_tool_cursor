#include <OgreRay.h>
#include <OgrePlane.h>
#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_tool_cursor/get_point_on_plane.hpp"

namespace rviz_rendering
{
/** Given a viewport and an x,y position in window-pixel coordinates,
 *  find the point on a plane directly behind it, if any.
 * @return true if the intersection exists, false if it does not. */
bool getPointOnPlaneFromWindowXY(Ogre::Viewport* viewport,
                                 Ogre::Plane& plane,
                                 int window_x,
                                 int window_y,
                                 Ogre::Vector3& intersection_out)
{
  int width = viewport->getActualWidth();
  int height = viewport->getActualHeight();

  Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay((float)window_x / (float)width,
                                                                      (float)window_y / (float)height);
  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(plane);
  if (!intersection.first)
  {
    return false;
  }
  intersection_out = mouse_ray.getPoint(intersection.second);

  return true;
}
}  // namespace rviz_rendering
