#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include </usr/include/eigen3/Eigen/Dense>
#include <cmath>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#define MAX_CLUSTERS 4
#define PI 3.14159

class LinkTracker
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub[MAX_CLUSTERS];
    ros::Publisher marker_pub[2];
    ros::Publisher config_pub[3];
    tf::Transform tf;

public:
    LinkTracker()
	{
	    ROS_DEBUG("Creating subscribers and publishers");
	    // all publishers and subscribers:
	    cloud_sub = n_.subscribe("mass_box_filter/psy/output", 10,
	    			     &LinkTracker::cloudcb, this);

	    int i = 0;
	    for (i=0; i<MAX_CLUSTERS; i++)
	    {
		std::stringstream ss;
		ss << "/cluster_" << i+1 << "_cloud";
		cloud_pub[i] = n_.advertise<sensor_msgs::PointCloud2>
		    (ss.str(), 1);
	    }

	    tf::StampedTransform t;
	    tf::TransformListener listener;

	    ROS_DEBUG("Looking up transform from kinect to optimization frame");
	    try
	    {
		ros::Time now=ros::Time::now();
		listener.waitForTransform("/camera_depth_optical_frame",
					  "/oriented_optimization_frame",
					  now, ros::Duration(1.0));
		listener.lookupTransform("/oriented_optimization_frame",
					 "/camera_depth_optical_frame",
					 ros::Time(0), t);
		tf = t;	    

	    }
	    catch (tf::TransformException ex)
	    {
		ROS_ERROR("%s", ex.what());
		ros::shutdown();
	    }

        marker_pub[0] = n_.advertise<visualization_msgs::Marker>("int1_marker", 1);
        marker_pub[1] = n_.advertise<visualization_msgs::Marker>("int2_marker", 1);
        
        config_pub[0] = n_.advertise<std_msgs::Float32>("xpos",1);
        config_pub[1] = n_.advertise<std_msgs::Float32>("ang1",1);
        config_pub[2] = n_.advertise<std_msgs::Float32>("ang2",1);

	    return;
	}

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)
	{
	    ROS_DEBUG("Filtered cloud receieved");
	    ros::Time start_time = ros::Time::now();
	    ros::Time tcur = ros::Time::now();

	    sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2 ());
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_project_1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_project_2 (new pcl::PointCloud<pcl::PointXYZ>);
	    Eigen::Vector4f centroid1;
	    Eigen::Vector4f centroid2;

	    // set time stamp and frame id
	    ros::Time tstamp = ros::Time::now();

	    ROS_DEBUG("finished declaring vars : %f", (ros::Time::now()-tcur).toSec());
	    tcur = ros::Time::now();

	    // Convert to pcl
	    ROS_DEBUG("Convert incoming cloud to pcl cloud");
	    pcl::fromROSMsg(*scan, *cloud);
	    ROS_DEBUG("cloud transformed and converted to pcl : %f",
		      (ros::Time::now()-tcur).toSec());
	    tcur = ros::Time::now();

	    ////////////////////////////////////////
        //    STARTING CLUSTER EXTRACTION	  //
        ////////////////////////////////////////
	    ROS_DEBUG("Begin cluster extraction");

        // create a vector for storing the indices of the clusters
        std::vector<pcl::PointIndices> cluster_indices;

        // setup cluster extraction:
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.02); // cm
        ec.setMinClusterSize (50);
        ec.setMaxClusterSize (500);
        ec.setInputCloud (cloud);
        // perform cluster extraction
        ec.extract (cluster_indices);

        if (cluster_indices.size()==2)
        {
            std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin();

            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
              cloud_cluster_1->points.push_back (cloud->points[*pit]); //*

            cloud_cluster_1->width = cloud_cluster_1->points.size ();
            cloud_cluster_1->height = 1;
            cloud_cluster_1->is_dense = true;

            ++it;
            
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
              cloud_cluster_2->points.push_back (cloud->points[*pit]); //*

            cloud_cluster_2->width = cloud_cluster_2->points.size ();
            cloud_cluster_2->height = 1;
            cloud_cluster_2->is_dense = true;

	    	// convert to rosmsg and publish:
	    	ROS_DEBUG("Publishing extracted cloud");
	    	pcl::toROSMsg(*cloud_cluster_1, *ros_cloud);
		    ros_cloud->header.frame_id = "/oriented_optimization_frame";
	    	cloud_pub[0].publish(ros_cloud);

	    	pcl::toROSMsg(*cloud_cluster_2, *ros_cloud);
		    ros_cloud->header.frame_id = "/oriented_optimization_frame";
	    	cloud_pub[1].publish(ros_cloud);

            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	    	ROS_DEBUG("Segmenting cloud");
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            // Optional
            seg.setOptimizeCoefficients (true);

            seg.setModelType (pcl::SACMODEL_LINE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (1.0);

            seg.setInputCloud (cloud_cluster_1);
            seg.segment (*inliers, *coefficients);

            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType (pcl::SACMODEL_LINE);
            proj.setIndices (inliers);
            proj.setInputCloud (cloud_cluster_1);
            proj.setModelCoefficients (coefficients);
            proj.filter (*cloud_project_1);

            double link1coeff[6] = {coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3],coefficients->values[4],coefficients->values[5]};

            seg.setModelType (pcl::SACMODEL_LINE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (1.0);

            seg.setInputCloud (cloud_cluster_2);
            seg.segment (*inliers, *coefficients);

            proj.setModelType (pcl::SACMODEL_LINE);
            proj.setIndices (inliers);
            proj.setInputCloud (cloud_cluster_2);
            proj.setModelCoefficients (coefficients);
            proj.filter (*cloud_project_2);

            double link2coeff[6] = {coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3],coefficients->values[4],coefficients->values[5]};

	    	pcl::toROSMsg(*cloud_project_1, *ros_cloud);
		    ros_cloud->header.frame_id = "/oriented_optimization_frame";
	    	cloud_pub[2].publish(ros_cloud);

	    	pcl::toROSMsg(*cloud_project_2, *ros_cloud);
		    ros_cloud->header.frame_id = "/oriented_optimization_frame";
	    	cloud_pub[3].publish(ros_cloud);


		    // compute centroid for link placement:
		    pcl::compute3DCentroid(*cloud_cluster_1, centroid1);              
	    	pcl::compute3DCentroid(*cloud_cluster_2, centroid2);


            double *linkcoeff[2] = {link1coeff,link2coeff};

            int i,j;
            if(centroid1(1)>centroid2(1)){
                i = 0; j = 1;}
            else{
                i = 1; j = 0;}
            
            double x1 = linkcoeff[i][0];
            double y1 = linkcoeff[i][1];
            double x2 = linkcoeff[i][0]+linkcoeff[i][3];
            double y2 = linkcoeff[i][1]+linkcoeff[i][4];
            double x3 = linkcoeff[j][0];
            double y3 = linkcoeff[j][1];
            double x4 = linkcoeff[j][0]+linkcoeff[j][3];
            double y4 = linkcoeff[j][1]+linkcoeff[j][4];

            double xint2 = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
            double yint2 = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));

            x3 = 1;
            y3 = -0.105;
            x4 = 0;
            y4 = -0.105;

            double xint1 = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
            double yint1 = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));

            double Ax = xint2 - xint1;
            double Ay = yint2 - yint1;
            double Bx = 1;
            double By = 0;

            double ang1 = acos((Ax*Bx+Ay*By)/(sqrt(Ax*Ax+Ay*Ay)));
        
            int ang2sign;
            float xpos;

            if(i==0){
                Bx = centroid2(0)-xint2;  By = centroid2(1)-yint2;
                xpos = centroid1(0);
                if(((xint2-xint1)*(centroid2(1)-yint1)-(yint2-yint1)*(centroid2(0)-xint1))>=0)
                    ang2sign = 1;
                else
                    ang2sign = -1;
            }
            else{
                Bx = centroid1(0)-xint2;  By = centroid1(1)-yint2;
                xpos = centroid2(0);
                if(((xint2-xint1)*(centroid1(1)-yint1)-(yint2-yint1)*(centroid1(0)-xint1))>=0)
                    ang2sign = 1;
                else
                    ang2sign = -1;
            }   

            double ang2 = ang2sign*acos((Ax*Bx+Ay*By)/(sqrt(Ax*Ax+Ay*Ay)*sqrt(Bx*Bx+By*By)));

            if(ang2>3.0)
                ang2 = ang2 - PI;
            else if(ang2<-3.0)
                ang2 = ang2 + PI;       

            std_msgs::Float32 xpos_msg;
            xpos_msg.data = xpos;
            config_pub[0].publish(xpos_msg);

            std_msgs::Float32 ang1_msg;
            ang1_msg.data = ang1;
            config_pub[1].publish(ang1_msg);

            std_msgs::Float32 ang2_msg;
            ang2_msg.data = ang2;
            config_pub[2].publish(ang2_msg);

 
            //Define marker for intersection
            visualization_msgs::Marker marker1;
            marker1.header.frame_id = "/oriented_optimization_frame";
            marker1.header.stamp = ros::Time::now();
            marker1.ns = "basic_shapes";
            marker1.id = 0;
            marker1.type = visualization_msgs::Marker::SPHERE;
            marker1.action = visualization_msgs::Marker::ADD;
            marker1.pose.position.x = xint1;
            marker1.pose.position.y = yint1;
            marker1.pose.position.z = (link1coeff[2]+link2coeff[2])/2;
            marker1.scale.x = 0.01;
            marker1.scale.y = 0.01;
            marker1.scale.z = 0.01;
            marker1.color.r = 0.0f;
            marker1.color.g = 1.0f;
            marker1.color.b = 0.0f;
            marker1.color.a = 1.0;
            marker1.lifetime = ros::Duration();
            marker_pub[0].publish(marker1);

            visualization_msgs::Marker marker2;
            marker2.header.frame_id = "/oriented_optimization_frame";
            marker2.header.stamp = ros::Time::now();
            marker2.ns = "basic_shapes";
            marker2.id = 0;
            marker2.type = visualization_msgs::Marker::SPHERE;
            marker2.action = visualization_msgs::Marker::ADD;
            marker2.pose.position.x = xint2;
            marker2.pose.position.y = yint2;
            marker2.pose.position.z = (link1coeff[2]+link2coeff[2])/2;
            marker2.scale.x = 0.01;
            marker2.scale.y = 0.01;
            marker2.scale.z = 0.01;
            marker2.color.r = 0.0f;
            marker2.color.g = 1.0f;
            marker2.color.b = 0.0f;
            marker2.color.a = 1.0;
            marker2.lifetime = ros::Duration();
            marker_pub[1].publish(marker2);

        }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "link_tracker");

    // turn on debugging
    // log4cxx::LoggerPtr my_logger =
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle n;

    ROS_INFO("Starting link tracker...\n");
    LinkTracker tracker;
  
    ros::spin();
  
    return 0;
}
