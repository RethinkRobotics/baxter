# Create a visual PDF file from URDF
rosrun urdfdom urdf_to_graphiz baxter.urdf
rm baxter.gv # this is a mid-step file, not needed
gvfs-open baxter.pdf # open file in ubuntu