#include <geometry_msgs/PoseStamped.h>

#include <OgreMovableObject.h>
#include <OgreSceneManager.h>

#include <rviz/display_context.h>
#include <rviz/load_resource.h>
#include <rviz/geometry.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/selection/selection_manager.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/string_property.h>

#include <rviz_tool_cursor/rviz_tool_cursor.h>

#include <Eigen/Dense>

#include <pcl/common/pca.h>

namespace
{

Eigen::Matrix3f createMatrix(const Eigen::Vector3f& norm)
{
  Eigen::Matrix3f mat (Eigen::Matrix3f::Identity());
  mat.col(2) = norm;

  // Create plane from point normal
  Eigen::Hyperplane<float, 3> plane (norm, Eigen::Vector3f(0, 0, 0));

  // If the normal and global x-axis are not closely aligned
  if (std::abs(norm.dot(Eigen::Vector3f::UnitX())) < 0.90f)
  {
    // Project the global x-axis onto the plane to generate the x-axis
    Eigen::Vector3f x_axis = plane.projection(Eigen::Vector3f::UnitX()).normalized();
    mat.col(0) = x_axis;
    mat.col(1) = norm.cross(x_axis);
  }
  else
  {
    // Project the global y-axis onto the plane to generate the y-axis
    Eigen::Vector3f y_axis = plane.projection(Eigen::Vector3f::UnitY()).normalized();
    mat.col(0) = y_axis.cross(norm);
    mat.col(1) = y_axis;
  }

  return mat;
}

Ogre::Quaternion estimateNormal(const std::vector<Ogre::Vector3>& points,
                                const Ogre::Vector3& camera_norm)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  for(std::size_t i = 0; i < points.size(); ++i)
  {
    pcl::PointXYZ pt;
    pt.x = points[i].x;
    pt.y = points[i].y;
    pt.z = points[i].z;

    cloud->push_back(pt);
  }

  // Use principal component analysis to the get eigenvectors
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud);
  Eigen::Matrix3f& evecs = pca.getEigenVectors();

  // Get the eigenvector associated with the smallest eigenvalue
  // (should be Z-axis, assuming points are relatively planar)
  Eigen::Vector3f norm = evecs.col(2);
  norm.normalize();

  // The
  Eigen::Vector3f camera_normal;
  camera_normal << camera_norm.x, camera_norm.y, camera_norm.z;
  camera_normal.normalize();

  if(norm.dot(camera_normal) < 0)
  {
    norm *= -1;
  }

  // Create a random orientation matrix with the normal being in the direction of the smalles eigenvector
  Eigen::Matrix3f mat = createMatrix(norm);

  Eigen::Quaternionf q(mat); //Eigen::AngleAxisf(0.0, evecs.col(2)));
  Ogre::Quaternion out;
  out.w = q.w();
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();

  return out;
}

} // namepsace anonymous

namespace rviz_tool_cursor
{

ToolCursor::ToolCursor()
  : rviz::Tool()
{
  shortcut_key_ = 'c';


  topic_property_ = new rviz::StringProperty("Topic", "/selection_point",
                                             "The topic on which to publish points",
                                             getPropertyContainer(), SLOT(updateTopic()), this);

  patch_size_property_ = new rviz::IntProperty("Patch Size", 10,
                                               "The number of pixels with which to estimate the surface normal",
                                               getPropertyContainer());

  color_property_ = new rviz::ColorProperty("Color", QColor(255, 255, 255),
                                            "The color of the tool visualization",
                                            getPropertyContainer(), SLOT(updateToolVisualization()), this);

  updateTopic();
}

ToolCursor::~ToolCursor()
{

}

void ToolCursor::onInitialize()
{
  // Initialize the scene node
  cursor_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create the visual tool object
  Ogre::MovableObject* obj = createToolVisualization();

  // Attach the tool visualization to the scene
  cursor_node_->attachObject(obj);
  cursor_node_->setVisible(false);

  // Set the cursors
  hit_cursor_ = cursor_;
  std_cursor_ = rviz::getDefaultCursor();
}

void ToolCursor::activate()
{
  cursor_node_->setVisible(true);
}

void ToolCursor::deactivate()
{
  cursor_node_->setVisible(false);
}

void ToolCursor::updateTopic()
{
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1, true);
}

int ToolCursor::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  // Get the 3D point in space indicated by the mouse and a patch of points around it
  // with which to estimate the surface normal
  Ogre::Vector3 position;
  std::vector<Ogre::Vector3> points;

  const unsigned patch_size = static_cast<unsigned>(patch_size_property_->getInt());

  // Set the visibility of this node off so the selection manager won't choose a point on our cursor mesh in the point and patch
  cursor_node_->setVisible(false);

  bool got_point = context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, position);
  bool got_patch = context_->getSelectionManager()->get3DPatch(event.viewport, event.x, event.y, patch_size, patch_size, true, points);

  // Revisualize the cursor node
  cursor_node_->setVisible(true);

  if(got_point && got_patch && points.size() > 3)
  {
    // Set the cursor
    rviz::Tool::setCursor(hit_cursor_);

    // Estimate the surface normal from the patch of points
    Ogre::Quaternion q = estimateNormal(points, event.viewport->getCamera()->getDirection());
    cursor_node_->setOrientation(q);
    cursor_node_->setPosition(position);

    if(event.leftUp())
    {
      // Publish a point message upon release of the left mouse button
      geometry_msgs::PoseStamped msg;
      msg.header.frame_id = context_->getFixedFrame().toStdString();
      msg.header.stamp = ros::Time::now();

      msg.pose.position.x = static_cast<double>(position.x);
      msg.pose.position.y = static_cast<double>(position.y);
      msg.pose.position.z = static_cast<double>(position.z);

      msg.pose.orientation.w = static_cast<double>(q.w);
      msg.pose.orientation.x = static_cast<double>(q.x);
      msg.pose.orientation.y = static_cast<double>(q.y);
      msg.pose.orientation.z = static_cast<double>(q.z);

      pub_.publish(msg);
    }
  }
  else
  {
    // Set the standard cursor
    rviz::Tool::setCursor(std_cursor_);

    // Project the tool visualization onto the ground
    Ogre::Plane plane (Ogre::Vector3::UNIT_Z, 0.0f);
    rviz::getPointOnPlaneFromWindowXY(event.viewport, plane, event.x, event.y, position);
    cursor_node_->setOrientation(1.0f, 0.0f, 0.0f, 0.0f);
    cursor_node_->setPosition(position);
  }

  return rviz::Tool::Render;
}

} // namespace rviz_tool_cursor
