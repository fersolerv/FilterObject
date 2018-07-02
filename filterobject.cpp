#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/common.h>
#include <vector>
#include <ctime>
#include <pcl/filters/crop_hull.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/vtk.h>
#include <pcl/io/vtk_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/uniform_sampling.h>

//pcl::PointCloud<pcl::PointXYZ>::Ptr extractSubCloud(pcl::PointXYZ min, pcl::PointXYZ
//                                                    max,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> & octree )
//{

//     pcl::PointCloud<pcl::PointXYZ>::Ptr sub_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     std::vector<int> pointIdxBoxSearch;
//     Eigen::Vector3f min_pt ( min.x, min.y, min.z );
//     Eigen::Vector3f max_pt ( max.x, max.y, max.z );
//     octree.boxSearch ( min_pt, max_pt, pointIdxBoxSearch );

//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     pcl::PointIndices::Ptr inliers ( new pcl::PointIndices () );
//     inliers->indices = pointIdxBoxSearch;
//     extract.setInputCloud ( cloud);
//     extract.setIndices ( inliers );
//     extract.setNegative ( false );
//     extract.filter ( *sub_cloud );
//     return sub_cloud;
//}



int
main (int argc, char** argv)
{

    ////VOXEL GRID
//        pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
//        pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

//        // Fill in the cloud data
//        pcl::PCDReader reader;
//        // Replace the path below with the path where you saved your file
//        reader.read ("/home/fernando/PHD/Experiments/pcd/Objects/SportBottle.pcd", *cloud);

//        std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//                  << " data points (" << pcl::getFieldsList (*cloud) << ")." << endl;

//        // Create the filtering object
//        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//        sor.setInputCloud (cloud);
//        sor.setLeafSize (5.25f, 3.162f, 2.646f);
//        //sor.setLeafSize (10.5f, 15.5f, 16.5f);
//        sor.filter (*cloud_filtered);

//        std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//                  << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << endl;

//        pcl::PCDWriter writer;
//        writer.write ("/home/fernando/PHD/Applications/FilterObject/SportBottleCH.pcd", *cloud_filtered,
//                      Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////CONVEX HULL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/fernando/PHD/Experiments/pcd/Objects/SportBottle.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    pcl::ConvexHull<pcl::PointXYZ> cHull;
    //pcl::ConcaveHull<pcl::PointXYZ> cHull;
    pcl::PointCloud<pcl::PointXYZ> cHull_points;
    cHull.setInputCloud(cloud);
    cHull.setDimension(3);
    //cHull.setAlpha(0.7);
    cHull.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    std::vector<pcl::Vertices> polygons;
    cHull.reconstruct(cHull_points, polygons);

    pcl::io::savePCDFileASCII ("/home/fernando/PHD/Applications/FilterObject/SportBottleCH.pcd", cHull_points);
    std::cerr << "Saved " << cHull_points.points.size () << " data points to SportBottleCH.pcd." << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/fernando/PHD/Applications/FilterObject/SportBottleCH.pcd", *cloud_filtered) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    pcl::visualization::PCLVisualizer CHObject ("CHObject");
    CHObject.setBackgroundColor(255,255,255);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> object(cloud, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> chull(cloud_filtered, 255, 0, 0);


    CHObject.addPointCloud(cloud, object,"object");
    CHObject.addPointCloud(cloud_filtered,chull,"chull");

     CHObject.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.4, "object");
     CHObject.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.8, "chull");

    while(!CHObject.wasStopped()){
        CHObject.spinOnce();
    }
    CHObject.close();


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    // Objects for storing the point clouds.
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);

//        // Read a PCD file from disk.
//        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/fernando/PHD/Experiments/pcd/Objects/SportBottle.pcd", *cloud) == -1)
//        {
//            return -1;
//        }

//        // Get the plane model, if present.
//        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
//        segmentation.setInputCloud(cloud);
//        segmentation.setModelType(pcl::SACMODEL_PLANE);
//        segmentation.setMethodType(pcl::SAC_RANSAC);
//        segmentation.setDistanceThreshold(0.01);
//        segmentation.setOptimizeCoefficients(true);
//        pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
//        segmentation.segment(*inlierIndices, *coefficients);

//        if (inlierIndices->indices.size() == 0)
//            std::cout << "Could not find a plane in the scene." << std::endl;
//        else
//        {
//            // Copy the points of the plane to a new cloud.
//            pcl::ExtractIndices<pcl::PointXYZ> extract;
//            extract.setInputCloud(cloud);
//            extract.setIndices(inlierIndices);
//            extract.filter(*plane);

//            // Object for retrieving the convex hull.
//            pcl::ConvexHull<pcl::PointXYZ> hull;
//            hull.setInputCloud(plane);
//            hull.reconstruct(*convexHull);

//            // Visualize the hull.
//            pcl::visualization::CloudViewer viewerPlane("Convex hull");
//            viewerPlane.showCloud(convexHull);
//            while (!viewerPlane.wasStopped())
//            {
//                // Do nothing but wait.
//            }
//        }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///FINDING BORDERS
//        pcl::PCDReader reader;

//        pcl::PointCloud<pcl::Boundary> boundaries;
//        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
//        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
//        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//        reader.read ("/home/fernando/PHD/Experiments/pcd/Objects/SportBottle.pcd", *cloud);

//        std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//                  << " data points (" << pcl::getFieldsList (*cloud) << ")." << endl;

//        normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud));
//        normEst.setRadiusSearch(15);
//        normEst.compute(*normals);

//        boundEst.setInputCloud(cloud);
//        boundEst.setInputNormals(normals);
//        boundEst.setRadiusSearch(15);
//        boundEst.setAngleThreshold(3.1416/4);
//        boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
//        boundEst.compute(boundaries);

//        for(int i = 0; i < cloud->points.size(); i++){
//            if(boundaries[i].boundary_point < 1){
//                cloud->at(i).z = 0;
//            }
//        }

//        pcl::visualization::CloudViewer viewer("PCL Viewer");
//        viewer.showCloud(cloud);
//        while (!viewer.wasStopped());

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///OCTREE
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/fernando/PHD/Experiments/pcd/Objects/MilkBox.pcd", *cloud) == -1) //* load the file
//    {
//        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//        return (-1);
//    }

//    float resolution = 128.0f;

//    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

//    octree.setInputCloud (cloud);
//    octree.addPointsFromInputCloud ();

//    pcl::PointXYZ searchPoint;

//    searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
//    searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
//    searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

//    // Neighbors within voxel search

//    std::vector<int> pointIdxVec;

//    if (octree.voxelSearch (searchPoint, pointIdxVec))
//    {
//        std::cout << "Neighbors within voxel search at (" << searchPoint.x
//                  << " " << searchPoint.y
//                  << " " << searchPoint.z << ")"
//                  << std::endl;

//        for (size_t i = 0; i < pointIdxVec.size (); ++i)
//            std::cout << "    " << cloud->points[pointIdxVec[i]].x
//                      << " " << cloud->points[pointIdxVec[i]].y
//                      << " " << cloud->points[pointIdxVec[i]].z << std::endl;
//    }

//    // K nearest neighbor search

//    int K = 10;

//    std::vector<int> pointIdxNKNSearch;
//    std::vector<float> pointNKNSquaredDistance;

//    std::cout << "K nearest neighbor search at (" << searchPoint.x
//              << " " << searchPoint.y
//              << " " << searchPoint.z
//              << ") with K=" << K << std::endl;

//    if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
//    {
//        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
//            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
//                      << " " << cloud->points[ pointIdxNKNSearch[i] ].y
//                      << " " << cloud->points[ pointIdxNKNSearch[i] ].z
//                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
//    }

//    // Neighbors within radius search

//    std::vector<int> pointIdxRadiusSearch;
//    std::vector<float> pointRadiusSquaredDistance;

//    float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

//    std::cout << "Neighbors within radius search at (" << searchPoint.x
//              << " " << searchPoint.y
//              << " " << searchPoint.z
//              << ") with radius=" << radius << std::endl;


//    if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
//    {
//        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//            std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
//                      << " " << cloud->points[ pointIdxRadiusSearch[i] ].y
//                      << " " << cloud->points[ pointIdxRadiusSearch[i] ].z
//                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
//    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///PASS THRHOUG FILTER
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

//        if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/fernando/PHD/Experiments/pcd/Objects/SportBottle.pcd", *cloud) == -1) //* load the file
//        {
//            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//            return (-1);
//        }


//        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//          coefficients->values.resize (4);
//          coefficients->values[0] = coefficients->values[1] = 1.0;
//          coefficients->values[2] = 1.0;
//          coefficients->values[3] = 1.0;

//          // Create the filtering object
//          pcl::ProjectInliers<pcl::PointXYZ> proj;
//          proj.setModelType (pcl::SACMODEL_CYLINDER);
//          proj.setInputCloud (cloud);
//          proj.setModelCoefficients (coefficients);
//          proj.filter (*cloud_filtered);

//        pcl::visualization::PCLVisualizer CHObject ("CHObject");
//        CHObject.setBackgroundColor(255,255,255);

//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> object(cloud, 0, 0, 255); //Points out the box (blue)
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> chull(cloud_filtered, 255, 0, 0);


//        CHObject.addPointCloud(cloud, object,"object");
//        CHObject.addPointCloud(cloud_filtered,chull,"chull");

//         CHObject.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.4, "object");
//         CHObject.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.8, "chull");

//        while(!CHObject.wasStopped()){
//            CHObject.spinOnce();
//        }
//        CHObject.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///Normal length
    //        pcl::VoxelGrid<pcl::PointXYZ> sor;
    //        sor.setLeafSize(0.1f,0.1f,0.1f);
    //        sor.setSaveLeafLayout(true);

    //        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //      //  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm;
    //       // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    //        if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/fernando/PHD/Experiments/pcd/Objects/SportBottle.pcd", *cloud) == -1) //* load the file
    //        {
    //            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    //            return (-1);
    //        }

    //        // Create the normal estimation class, and pass the input dataset to it
    //        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    //        ne.setInputCloud (cloud);
    //        // Create an empty kdtree representation, and pass it to the normal estimation object.
    //        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    //        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    //        ne.setSearchMethod (tree);
    //        // Output datasets
    //        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //        ne.setRadiusSearch (2);
    //        ne.compute (*cloud_normals);

    //        pcl::visualization::PCLVisualizer PV("window");
    //        PV.setBackgroundColor(0,0,0);
    //        PV.addPointCloud(cloud, "object");
    //        PV.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals, 1,10.0, "normal");
    //        PV.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normal");
    //        PV.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0,0, "normal");

    //             while(!PV.wasStopped()){
    //                 PV.spinOnce();
    //             }
    //            // PV.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///Triangulation
    // Load input file into a PointCloud<T> with an appropriate type
//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//      pcl::PCLPointCloud2 cloud_blob;
//      pcl::io::loadPCDFile ("/home/fernando/PHD/Experiments/pcd/Objects/SportBottle.pcd", cloud_blob);
//      pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
//      //* the data should be available in cloud

//      // Normal estimation*
//      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//      tree->setInputCloud (cloud);
//      n.setInputCloud (cloud);
//      n.setSearchMethod (tree);
//      n.setKSearch (20);
//      n.compute (*normals);
//      //* normals should not contain the point normals + surface curvatures

//      // Concatenate the XYZ and normal fields*
//        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//        pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
//        //* cloud_with_normals = cloud + normals

//        // Create search tree*
//        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//        tree2->setInputCloud (cloud_with_normals);

//        // Initialize objects
//        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//        pcl::PolygonMesh triangles;

//        // Set the maximum distance between connected points (maximum edge length)
//        gp3.setSearchRadius (300);

//        // Set typical values for the parameters
//        gp3.setMu (2.5);
//        gp3.setMaximumNearestNeighbors (100);
//        gp3.setMaximumSurfaceAngle(M_PI/3);
//        gp3.setMinimumAngle(M_PI/18); // 10 degrees
//        gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//        gp3.setNormalConsistency(false);

//        // Get result
//        gp3.setInputCloud (cloud_with_normals);
//        gp3.setSearchMethod (tree2);
//        gp3.reconstruct (triangles);
//         pcl::io::saveVTKFile("//home/fernando/PHD/Applications/FilterObject/SportBottlePartial.vtk", triangles);

        return (0);
}
