

#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <vtkOBJReader.h>


/**
  This program loads and renders view point clouds from a CAD model (pass obj-model as input). After the view-generation, a merged point cloud of all views are created and visualized.   
*/
int
main (int argc, char** argv)
{
	// Load model 
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New ();
	std::vector<int> obj_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".obj");
	if (obj_file_indices.size () != 1)
	{
		pcl::console::print_error ("\nNeed a single input OBJ file to continue.\n");
		return 0;
	}
	vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New ();
    readerQuery->SetFileName (argv[obj_file_indices[0]]);
    readerQuery->Update ();
    polydata = readerQuery->GetOutput ();
    
	int resolution = 100;
	float radius_tessellated_sphere = 0.5;
	int tessellation_level = 0;
	float view_angle = 50;
	
	// Setup and render views
	pcl::apps::RenderViewsTesselatedSphere view_render;	
	view_render.addModelFromPolyData (polydata);
	view_render.setResolution (resolution);
	view_render.setUseVertices (true);
	view_render.setRadiusSphere (radius_tessellated_sphere);
	view_render.setComputeEntropies (false);
	view_render.setTesselationLevel (tessellation_level);
	view_render.setViewAngle (view_angle);
	view_render.generateViews ();	
	
	// Get rendered view-point-clouds and poses
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views;
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
	view_render.getViews (views);
	view_render.getPoses (poses);
	
	// Merge all point clouds to get a complete point cloud of the model by transforming all point clouds back to original pose
	pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < views.size(); i++)
	{	
		pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::transformPointCloud (*views[i], *rotated_cloud, poses[i].inverse().eval());
		
		*merged_cloud += *rotated_cloud;
	}
	
	// View merged point clouds
	pcl::visualization::PCLVisualizer viewer ("Viewer");
	viewer.addPointCloud<pcl::PointXYZ> (merged_cloud);
	viewer.spin ();
	
	return (0);
}
