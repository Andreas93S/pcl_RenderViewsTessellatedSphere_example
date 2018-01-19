# pcl_RenderViewsTessellatedSphere_example

Example code from the issue posted on https://github.com/PointCloudLibrary/pcl/issues/2188

example.cpp loads and renders view point clouds from a CAD model (pass obj-model as input). After the view-generation, a merged point cloud of all views is created and visualized.

merged_clouds1.jpg illustrates when the error occurs. In this case one view was incorrectly rendered where almost all points lie in the same plane. merged_clouds2.jpg illustrates when no distortion occurs and all view point clouds are correctly rendered.

Run code:

	-Open a terminal and navigate to the build folder.
	-Inside the build folder, type: "cmake ..;make" in the terminal to compile the code 
	-Execute code by typing "./example ex_cad.obj" in the terminal where ex_cad is the CAD model used in the images.  
	
	
	


