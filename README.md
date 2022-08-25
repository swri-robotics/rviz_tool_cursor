# Rviz Tool Cursor

![](https://github.com/marip8/rviz_tool_cursor/workflows/CI/badge.svg)
[![license - Apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

An Rviz tool that visualizes a given geometry as the cursor and orients the object normal to objects in the scene. On a click of the left mouse button, the
tool will publish the 3D point and 6D pose of the object on which the cursor rests.
When multiple points are clicked, the tool will display all selection points and lines connecting them and will publish a message containing all the current selection points.

![Mesh tool cursor](docs/mesh_tool_cursor.gif)

## Controls
- Left mouse button: Add a selection point (triggers the publication of the point, pose, and pose array messages)
- Left mouse button hold: Adds selection points when lasso mode is enabled
- Right mouse button: Deactivates the tool
- Center mouse button: Clears the current selection points and boundary

## Implementations
### Tool Cursor
The base class of the tool cursor has the following properties:
- `Pose Topic`: The topic on which to publish the 6D pose in the Rviz environment when the left mouse button is clicked
- `Point Topic`: The topic on which to publish the 3D point (no orientation) in the Rviz environment when the left mouse button is clicked.
    - Note: If you want this to match the output of the `Publish Point` tool, you can remap `/tool_cursor_point` to `/clicked_point`.  Alternatively, you can edit this in the `Panels > Tool Properties` menu.
- `Pose Array Topic`: The topic on which to publish the vector of 6D poses that have been selected in the Rviz environment when the left mouse button is clicked
- `Patch Size`: The number of pixels on a side with which to create a patch used for estimated the surface normal
- `Lasso Mode`: Toggle lasso mode where selection points are constantly acquired when the left mouse button is held down
- `Close Loop`: Toggle the visualization of the line connecting the last point to the first point
- `Show Points`: Toggle the display of the selection points
- `Show Lines`: Toggle the display of the lines connecting the selection points
- `Point Color`: Controls the color of the selection points
- `Line Color`: Controls the color of the selection boundary lines
- `Point Size`: Controls the size (in pixels) of the selection points

### Mesh Tool Cursor
The mesh tool cursor visualizes a specified mesh file (.stl or .ply) as the cursor. The tool has the following additional properties:
- `Mesh Filename`: The filename of the mesh file to be displayed (supports package:// and file:// URIs)
- `Color`: The color of the cursor visualization

### Circle Tool Cursor
The circle tool cursor visualizes a 2D circle of specified radius as the cursor. The tool has the following additional properties:
- `Radius`: The radius of the circle visualization
- `Color`: The color of the cursor visualization

## Customization
In order to create a custom Rviz tool cursor, the `ToolCursor` base class requires the implementation of the virtual function `createToolVisualization()` which defines the cursor visualization:
