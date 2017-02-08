#include <iostream>
#include <fstream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/common_headers.h>
#include <pcl/conversions.h>
#include <pcl/point_types_conversion.h>

#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/crop_box.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/features/vfh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace cv;

#define PI 3.14

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

//Get TF
std::string from_frame = "base_link";
std::string to_frame   = "realsense5";
tf::TransformListener * tf_l;
tf::StampedTransform camera_tf[5];
int Idx = 0;
int numPos;
vector<PointCloudRGB::Ptr> cloud_xyzrgb;
Eigen::Affine3f camera_view;


void getAffineTransform(tf::StampedTransform& TFtransform, Eigen::Affine3f& Afftransform)
{
    Afftransform.translation() << (float)TFtransform.getOrigin()[0], 
                                  (float)TFtransform.getOrigin()[1], 
                                  (float)TFtransform.getOrigin()[2];

    Afftransform.rotate (Eigen::Quaternionf((float)TFtransform.getRotation().getW(),
                        (float)TFtransform.getRotation()[0], 
                        (float)TFtransform.getRotation()[1],
                        (float)TFtransform.getRotation()[2]));
}

void changeRPY(tf::StampedTransform& transform, std::string mode, double change)
{
  double roll, pitch, yaw;
  tf::Matrix3x3 m_rotation(transform.getRotation());
  m_rotation.getRPY(roll,pitch,yaw);
  tf::Quaternion q_rotation;
  change = change*PI/180.0;
  std::cout<<"Changing "<<mode<<" by "<<change<<std::endl;
  std::cout<<"roll : "<<roll<<" pitch : "<<pitch<<" yaw: "<<yaw<<std::endl;
  if(strcmp(mode.c_str(),"roll") == 0)
  {
    roll += change;
    m_rotation.setRPY(roll, pitch, yaw);
    m_rotation.getRotation(q_rotation);

    transform.setRotation(q_rotation);
  }

  else if(strcmp(mode.c_str(),"pitch") == 0)
  {
    pitch += change;
    m_rotation.setRPY(roll, pitch, yaw);
    m_rotation.getRotation(q_rotation);

    transform.setRotation(q_rotation);
  }

  else if(strcmp(mode.c_str(),"yaw") == 0)
  {
    yaw += change;
    m_rotation.setRPY(roll, pitch, yaw);
    m_rotation.getRotation(q_rotation);

    transform.setRotation(q_rotation);
  }
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
    camera_tf[Idx].getOrigin()[0] -= 0.01;
  else if (event.getKeySym () == "R" && event.keyDown ())
    camera_tf[Idx].getOrigin()[0] += 0.01;
  else if (event.getKeySym () == "t" && event.keyDown ())
    camera_tf[Idx].getOrigin()[1] -= 0.01;
  else if (event.getKeySym () == "T" && event.keyDown ())
    camera_tf[Idx].getOrigin()[1] += 0.01;
  else if (event.getKeySym () == "y" && event.keyDown ())
    camera_tf[Idx].getOrigin()[2] -= 0.01;
  else if (event.getKeySym () == "Y" && event.keyDown ())
    camera_tf[Idx].getOrigin()[2] += 0.01;
  else if (event.getKeySym () == "h" && event.keyDown ())
    changeRPY(camera_tf[Idx],"roll",2);
  else if (event.getKeySym () == "H" && event.keyDown ())
    changeRPY(camera_tf[Idx],"roll",-2);
  else if (event.getKeySym () == "j" && event.keyDown ())
    changeRPY(camera_tf[Idx],"pitch",2);
  else if (event.getKeySym () == "J" && event.keyDown ())
    changeRPY(camera_tf[Idx],"pitch",-2);
  else if (event.getKeySym () == "k" && event.keyDown ())
    changeRPY(camera_tf[Idx],"yaw",2);
  else if (event.getKeySym () == "K" && event.keyDown ())
    changeRPY(camera_tf[Idx],"yaw",-2);

  PointCloudRGB::Ptr combinedCloud(new PointCloudRGB);

  // Transform all point clouds and combine
  for(int i=0;i<numPos;i++){
    Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    getAffineTransform(camera_tf[i], transform_1);
    PointCloudRGB::Ptr transformed_cloud (new PointCloudRGB);
    pcl::transformPointCloud (*cloud_xyzrgb[i], *transformed_cloud, transform_1);
    *combinedCloud += *transformed_cloud;
  }

  viewer->removePointCloud("CombinedCloud");
  // viewer->addCoordinateSystem(1.0, camera_view);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(combinedCloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (combinedCloud, rgb, "CombinedCloud"); 
}

int main( int ac, char* av[] ) {
    std::string node_name;

    ros::init(ac, av, node_name);
    ros::NodeHandle main_node_handle;
    tf_l = new tf::TransformListener;

    if(ac > 1)
      numPos = atoi(av[1]);
    else{
      std::cout<<"Number of Mapping positions should be entered !!! "<<std::endl;
      exit(-1);
    }

    for(int i=0;i<numPos;i++)
    {
      std::cout<<"Go to mapping position "<<i+1<<" and Press Enter to continue !!! "<<std::endl;
      sensor_msgs::PointCloud2::ConstPtr msg3 = 
          ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/pointcloud", main_node_handle, ros::Duration(6.0));
      PointCloudRGB::Ptr inputCloud(new PointCloudRGB);
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*msg3,pcl_pc2);
      pcl::fromPCLPointCloud2(pcl_pc2,*inputCloud);

      std::cout<<"Received PointCloud !"<<std::endl;

      //Get tf
      try {
      tf_l->waitForTransform(from_frame, to_frame, ros::Time(0), ros::Duration(5));
      tf_l->lookupTransform(from_frame, to_frame, 
                                          ros::Time(0), camera_tf[i]);
      std::cout<<"TF !"<<std::endl;
      }
      catch (tf::TransformException except) {
          std::cerr << "Found no TF from "
                      << from_frame.c_str() << " to " << to_frame.c_str()
                      << " -> " << except.what();
      }

      cloud_xyzrgb.push_back(inputCloud);
      cin.ignore();
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "CombinedCloud");
    viewer->initCameraParameters ();

    std::cout<<"TF for which mapping position do you want to change : ";
    cin >>Idx;
    std::cout<<std::endl;

    // Add the first cloud as dummy
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_xyzrgb[0]);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_xyzrgb[0], rgb, "CombinedCloud");
    getAffineTransform(camera_tf[0], camera_view);
    // camera_view.rotate (Eigen::Quaternionf(0,1,0,0));
    // viewer->addCoordinateSystem(1.0, camera_view);
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    FILE* fp = fopen("cam.info.txt", "w");
    for(int frame_id=0;frame_id<numPos;frame_id++){
      fprintf(fp, "\n# Camera-to-world extrinsic matrix (camera pose) for frame-%06d\n",frame_id);
      for (int i = 0; i < 3; ++i)
          fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", (float)(camera_tf[frame_id].getBasis()[i][0]), 
                                                                (float)(camera_tf[frame_id].getBasis()[i][1]), 
                                                                (float)(camera_tf[frame_id].getBasis()[i][2]), 
                                                                (float)(camera_tf[frame_id].getOrigin()[i]));
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", 0.0f, 0.0f, 0.0f, 1.0f);
    }
    fclose(fp);

    ros::shutdown();
    return 0;
}
