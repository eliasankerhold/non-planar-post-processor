# cura-non-planar
A simple non-planar slicer that uses cura.

The basic idea is:
1. Slice the model using Fusion360 and save each slice as an indiviudal stl.
2. Use ray-casting to project the lower surface of the slice onto the xy-plane.
3. Copt and offset the projected points by the layer height in z direction.
4. Reconstruct a mesh from this point cloud.
5. Feed the mesh into cura engine and let it generate GCode.
6. Parse the Gcode to read its coordinates and project it back onto the original (curved) slice.
7. Save the new, curved coordinates into Gcode.
