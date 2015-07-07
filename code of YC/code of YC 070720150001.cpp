#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include <pcl\common\time.h>
#include <pcl\filters\passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/statistical_outlier_removal.h>




#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}
    
    pcl::visualization::CloudViewer viewer;
    
    void
    viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
    {
        viewer.setBackgroundColor (1.0, 0.5, 1.0);
        pcl::PointXYZ o;
        o.x = 1.0;
        o.y = 0;
        o.z = 0;
        viewer.addSphere (o, 0.25, "sphere", 0);
        std::cout << "i only run once" << std::endl;
        
    }
    
    //generate 2D video
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
        
        
        //pcl::PassThrough<pcl::PointXYZRGBA> pass;
        //pass.setInputCloud (cloud);
        //pass.setFilterFieldName ("z");
        //pass.setFilterLimits (0.0, 1.0);
        //pass.filter(*cloud_filtered);
        
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        coefficients->values.resize (4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 1.0;
        coefficients->values[3] = 0;
        
        
        pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (cloud);
        proj.setModelCoefficients (coefficients);
        proj.filter (*cloud_filtered);
        
        
        if (!viewer.wasStopped())
        {
            viewer.showCloud (cloud_filtered);
        }
    }
    
    void run ()
    {
        pcl::Grabber* interface = new pcl::OpenNIGrabber();
        
        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
        
        interface->registerCallback (f);
        
        interface->start ();
        
        while (!viewer.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }
        
        interface->stop ();
    }
    
    
    
    
};

int main ()
{
    
    SimpleOpenNIViewer v;
    v.run ();
    
    
    
    
    return 0;
}