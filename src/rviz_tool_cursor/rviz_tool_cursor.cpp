#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <rviz/display_context.h>
#include <rviz/geometry.h>
#include <rviz/load_resource.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz_tool_cursor/rviz_tool_cursor.h>

#include <Eigen/Dense>

namespace
{
Eigen::Matrix3f createMatrix(const Eigen::Vector3f& norm)
{
  Eigen::Matrix3f mat(Eigen::Matrix3f::Identity());
  mat.col(2) = norm;

  // Create plane from point normal
  Eigen::Hyperplane<float, 3> plane(norm, Eigen::Vector3f(0, 0, 0));

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

Ogre::Quaternion estimateNormal(const std::vector<Ogre::Vector3>& points, const Ogre::Vector3& camera_norm)
{
  Eigen::MatrixXf data;
  data.resize(points.size(), 3);

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    data.row(i) = Eigen::Map<const Eigen::Vector3f>(points[i].ptr());
  }

  Eigen::MatrixXf centered = data.rowwise() - data.colwise().mean();

  // Use principal component analysis to the get eigenvectors
  Eigen::MatrixXf cov = centered.transpose() * centered;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);

  // Get the eigenvector associated with the smallest eigenvalue
  // (should be Z-axis, assuming points are relatively planar)
  Eigen::Vector3f norm = eig.eigenvectors().col(0);
  norm.normalize();

  Eigen::Vector3f camera_normal;
  camera_normal << camera_norm.x, camera_norm.y, camera_norm.z;
  camera_normal.normalize();

  if (norm.dot(camera_normal) < 0)
  {
    norm *= -1;
  }

  // Create an arbitrary orientation matrix with the normal being in the direction of the smallest eigenvector
  Eigen::Matrix3f mat = createMatrix(norm);

  Eigen::Quaternionf q(mat);  // Eigen::AngleAxisf(0.0, evecs.col(2)));
  Ogre::Quaternion out;
  out.w = q.w();
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();

  return out;
}

geometry_msgs::Point toMsg(const Ogre::Vector3& position)
{
  geometry_msgs::Point point;
  point.x = static_cast<double>(position.x);
  point.y = static_cast<double>(position.y);
  point.z = static_cast<double>(position.z);
  return point;
}

geometry_msgs::Pose toMsg(const Ogre::Vector3& position, const Ogre::Quaternion& q)
{
  geometry_msgs::Pose pose;

  pose.position.x = static_cast<double>(position.x);
  pose.position.y = static_cast<double>(position.y);
  pose.position.z = static_cast<double>(position.z);

  pose.orientation.w = static_cast<double>(q.w);
  pose.orientation.x = static_cast<double>(q.x);
  pose.orientation.y = static_cast<double>(q.y);
  pose.orientation.z = static_cast<double>(q.z);

  return pose;
}

Ogre::Vector3 fromMsg(const geometry_msgs::Pose& pose)
{
  return Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z);
}

std::vector<Ogre::Vector3> fromMsg(const geometry_msgs::PoseArray& arr)
{
  std::vector<Ogre::Vector3> pts;
  pts.reserve(arr.poses.size());
  std::transform(arr.poses.begin(), arr.poses.end(), std::back_inserter(pts),
                 [](const geometry_msgs::Pose& p) { return fromMsg(p); });
  return pts;
}

}  // namespace

namespace rviz_tool_cursor
{
ToolCursor::ToolCursor() : rviz::Tool()
{
  shortcut_key_ = 't';

  pose_topic_property_ =
      new rviz::StringProperty("Pose Topic", "/selection_point", "The topic on which to publish pose messages",
                               getPropertyContainer(), SLOT(updateTopic()), this);

  point_topic_property_ =
      new rviz::StringProperty("Point Topic", "/tool_cursor_point", "The topic on which to publish point messages",
                               getPropertyContainer(), SLOT(updateTopic()), this);

  pose_array_topic_property_ = new rviz::StringProperty("Pose Array Topic", "/selection_points",
                                                        "The topic on which to publish pose array messages",
                                                        getPropertyContainer(), SLOT(updateTopic()), this);

  patch_size_property_ = new rviz::IntProperty(
      "Patch Size", 10, "The number of pixels with which to estimate the surface normal", getPropertyContainer());

  lasso_mode_property_ = new rviz::BoolProperty("Lasso mode", true, "Toggle between lasso and discrete click mode",
                                                getPropertyContainer(), SLOT(updateSelectionVisual()), this);

  close_loop_property_ =
      new rviz::BoolProperty("Close loop", true, "Close the polygon with a line between the last and first points",
                             getPropertyContainer(), SLOT(updateSelectionVisual()), this);

  show_points_property_ = new rviz::BoolProperty("Show points", false, "Toggle display of selection points",
                                                 getPropertyContainer(), SLOT(updateSelectionVisual()), this);

  show_lines_property_ = new rviz::BoolProperty("Show lines", true, "Toggle display of selection boundary lines",
                                                getPropertyContainer(), SLOT(updateSelectionVisual()), this);

  pt_color_property_ = new rviz::ColorProperty("Point Color", Qt::black, "Color of the points", getPropertyContainer(),
                                               SLOT(updatePtsColor()), this);

  line_color_property_ = new rviz::ColorProperty("Line Color", Qt::black, "Color of the line", getPropertyContainer(),
                                                 SLOT(updateLinesColor()), this);

  pt_size_property_ = new rviz::FloatProperty("Point Size", 5.0, "Size of clicked points", getPropertyContainer(),
                                              SLOT(updatePtsSize()), this);
}

ToolCursor::~ToolCursor()
{
  if (cursor_node_->getParentSceneNode())
    cursor_node_->getParentSceneNode()->removeChild(cursor_node_);
  scene_manager_->destroySceneNode(cursor_node_);

  if (selection_node_->getParentSceneNode())
    selection_node_->getParentSceneNode()->removeChild(selection_node_);
  scene_manager_->destroySceneNode(selection_node_);

  scene_manager_->destroyManualObject(pts_vis_);
  scene_manager_->destroyManualObject(lines_vis_);
  Ogre::MaterialManager::getSingleton().remove(pts_material_->getName());
  Ogre::MaterialManager::getSingleton().remove(lines_material_->getName());

  scene_manager_->destroyMovableObject(cursor_object_);
}

void ToolCursor::onInitialize()
{
  // Initialize the scene node
  cursor_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  selection_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create the visual tool object
  cursor_object_ = createToolVisualization();

  // Attach the tool visualization to the scene
  cursor_node_->attachObject(cursor_object_);
  cursor_node_->setVisible(false);

  // Set the cursors
  hit_cursor_ = cursor_;
  std_cursor_ = rviz::getDefaultCursor();

  // Add the points visualization
  pts_vis_ = scene_manager_->createManualObject();
  selection_node_->attachObject(pts_vis_);

  // Add the lines visualization
  lines_vis_ = scene_manager_->createManualObject();
  selection_node_->attachObject(lines_vis_);

  // Add materials
  static int count = 0;
  // Points
  {
    const std::string name = "points_material_" + std::to_string(count);
    pts_material_ =
        Ogre::MaterialManager::getSingleton().create(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  }
  // Lines
  {
    const std::string name = "lines_material_" + std::to_string(count);
    lines_material_ =
        Ogre::MaterialManager::getSingleton().create(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  }
  ++count;

  selection_.header.frame_id = context_->getFixedFrame().toStdString();

  // Update
  updateTopic();
  updatePtsSize();
  updatePtsColor();
  updateLinesColor();
}

void ToolCursor::activate()
{
  cursor_node_->setVisible(true);
}

void ToolCursor::deactivate()
{
  cursor_node_->setVisible(false);
}

int ToolCursor::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  if (event.rightUp())
    return rviz::Tool::Finished;

  // Get the 3D point in space indicated by the mouse and a patch of points around it
  // with which to estimate the surface normal
  Ogre::Vector3 position;
  std::vector<Ogre::Vector3> points;

  const unsigned patch_size = static_cast<unsigned>(patch_size_property_->getInt());

  // Set the visibility of this node off so the selection manager won't choose a point on our cursor mesh in the point
  // and patch
  cursor_node_->setVisible(false);
  selection_node_->setVisible(false);

  bool got_point = context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, position);
  bool got_patch = context_->getSelectionManager()->get3DPatch(event.viewport, event.x, event.y, patch_size, patch_size,
                                                               true, points);

  // Revisualize the cursor node
  cursor_node_->setVisible(true);
  selection_node_->setVisible(true);

  if (got_point && got_patch && points.size() > 3)
  {
    // Set the cursor
    rviz::Tool::setCursor(hit_cursor_);

    // Estimate the surface normal from the patch of points
    Ogre::Quaternion q = estimateNormal(points, event.viewport->getCamera()->getDirection());
    cursor_node_->setOrientation(q);
    cursor_node_->setPosition(position);

    if (event.leftUp() || (event.left() && lasso_mode_property_->getBool()))
    {
      std_msgs::Header header;
      header.frame_id = context_->getFixedFrame().toStdString();
      header.stamp = ros::Time::now();

      // Publish a point message upon release of the left mouse button
      {
        geometry_msgs::PointStamped point_msg;
        point_msg.header = header;
        point_msg.point = toMsg(position);
        point_pub_.publish(point_msg);
      }

      // Publish a pose message
      {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = header;
        pose_msg.pose = toMsg(position, q);
        pose_pub_.publish(pose_msg);

        // Append to the collection
        selection_.poses.push_back(pose_msg.pose);

        // Update the visual
        updateSelectionVisual();
      }

      // Publish the pose array
      pose_array_pub_.publish(selection_);
    }
  }
  else
  {
    // Set the standard cursor
    rviz::Tool::setCursor(std_cursor_);

    // Project the tool visualization onto the ground
    Ogre::Plane plane(Ogre::Vector3::UNIT_Z, 0.0f);
    rviz::getPointOnPlaneFromWindowXY(event.viewport, plane, event.x, event.y, position);
    cursor_node_->setOrientation(1.0f, 0.0f, 0.0f, 0.0f);
    cursor_node_->setPosition(position);
  }

  if (event.middleUp())
  {
    // Clear the selection
    selection_.poses.clear();
    lines_vis_->clear();
    pts_vis_->clear();
  }

  return rviz::Tool::Render;
}

void ToolCursor::updateTopic()
{
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_property_->getStdString(), 1, true);
  point_pub_ = nh_.advertise<geometry_msgs::PointStamped>(point_topic_property_->getStdString(), 1, true);
  pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>(pose_array_topic_property_->getStdString(), 1, true);
}

void ToolCursor::updateSelectionVisual()
{
  const std::vector<Ogre::Vector3> pts = fromMsg(selection_);

  pts_material_->setPointSize(pt_size_property_->getFloat());

  // Add the points to the display when not in lasso mode
  if (!pts.empty() && show_points_property_->getBool())
  {
    pts_vis_->clear();
    pts_vis_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
    for (const Ogre::Vector3& pt : pts)
    {
      pts_vis_->position(pt);
    }
    pts_vis_->end();

    // Set the custom material
    pts_vis_->setMaterialName(0, pts_material_->getName(), pts_material_->getGroup());
  }

  // Add the polygon lines
  if (pts.size() > 1 && show_lines_property_->getBool())
  {
    lines_vis_->clear();
    lines_vis_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    for (std::size_t i = 0; i < selection_.poses.size() - 1; ++i)
    {
      lines_vis_->position(pts.at(i));
      lines_vis_->position(pts.at(i + 1));
    }

    // Close the polygon
    if (pts.size() > 2 && close_loop_property_->getBool())
    {
      lines_vis_->position(pts.back());
      lines_vis_->position(pts.front());
    }

    lines_vis_->end();

    lines_vis_->setMaterialName(0, lines_material_->getName(), lines_material_->getGroup());
  }
}

void ToolCursor::updatePtsColor()
{
  return updateMaterialColor(pts_material_, pt_color_property_->getColor());
}

void ToolCursor::updateLinesColor()
{
  return updateMaterialColor(lines_material_, line_color_property_->getColor());
}

void ToolCursor::updatePtsSize()
{
  pts_material_->setPointSize(pt_size_property_->getFloat());
}

void ToolCursor::updateMaterialColor(Ogre::MaterialPtr material, const QColor& color,
                                     const bool override_self_illumination)
{
  qreal r, g, b, a;
  color.getRgbF(&r, &g, &b, &a);
  material->setDiffuse(r, g, b, a);
  material->setSpecular(r, g, b, a);
  material->setAmbient(r, g, b);
  if (override_self_illumination)
    material->setSelfIllumination(r, g, b);
}

void ToolCursor::updateToolVisualization()
{
  // Remove and destroy the first tool visualization from the scene node
  scene_manager_->destroyMovableObject(cursor_object_);

  // Create the new tool visualization
  cursor_object_ = createToolVisualization();

  // Attach the new tool visualization to the scene node
  cursor_node_->attachObject(cursor_object_);
  cursor_node_->setVisible(false);
}

Ogre::MovableObject* ToolCursor::createToolVisualization()
{
  return scene_manager_->createManualObject();
}

}  // namespace rviz_tool_cursor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_tool_cursor::ToolCursor, rviz::Tool)
