#include <stdlib.h>
#include <stdio.h>

#include <vector>
#include <string>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <cgnuplot/CGnuplot.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


class ObstacleMapper {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber pc_sub_;
        ros::Subscriber odom_sub_;
        ros::Publisher arm_target_pub_;
        ros::Subscriber arm_target_sub_;

        tf::TransformListener listener_;

        std::string target_frame, ground_frame, laser_frame;
        double displacement_threshold, plane_distance_threshold;
        double min_x, max_x, min_y, max_y, min_z, max_z, min_d;
        double occ_min_x, occ_max_x;
        double occupancy_resolution;
        double weight_horizontal, weight_vertical;
        double tool_width, tool_height;

        float a,b,c,d; // Parameter of the laser plane

        bool received_odom, first_odom;
        pcl::PointCloud<pcl::PointXYZ> pc;
        tf::Pose last_odom,odom;
        tf::StampedTransform base_tf,hokuyo_tf;
        tf::Transform hokuyo_inv_tf;
        tf::Transform Delta;
        cgnuplot::CGnuplot plot;

        typedef std::multimap<float,tf::Vector3> MapType;
        MapType map;

        cv::Mat_<uint8_t> proj_occupancy;
        float z_offset;

    public:
        ObstacleMapper() : nh_("~") {
            
            nh_.param("target_frame",target_frame,std::string("/VSV/ArmPan"));
            nh_.param("ground_frame",ground_frame,std::string("/VSV/base"));
            nh_.param("laser_frame",laser_frame,std::string("/Hokuyo"));
            nh_.param("displacement_threshold",displacement_threshold,0.1);
            nh_.param("plane_distance_threshold",plane_distance_threshold,1e-2);
            // Minimum length of laser rays
            nh_.param("min_d",min_d,5e-2);
            // Parameters of the points of interest in the laser cloud
            nh_.param("min_x",min_x,-1.);
            nh_.param("max_x",max_x,5.);
            nh_.param("min_y",min_y,-10.);
            nh_.param("max_y",max_y,0.);
            nh_.param("min_z",min_z,0.);
            nh_.param("max_z",max_z,10.);
            // Paramerters of the projected occupancy
            nh_.param("occ_min_x",occ_min_x,-0.5);
            nh_.param("occ_max_x",occ_max_x,1.5);
            // Parameters of the occupancy grid
            nh_.param("occupancy_resolution",occupancy_resolution,0.1);
            nh_.param("weight_vertical",weight_vertical,1.0);
            nh_.param("weight_horizontal",weight_horizontal,3.0);
            nh_.param("tool_width",tool_width,1.5);
            nh_.param("tool_height",tool_height,0.5);


            first_odom = true;
            received_odom = false;
            ros::Duration(1.0).sleep();
            pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("scan",1, &ObstacleMapper::pc_callback,this);
            odom_sub_ = nh_.subscribe("odom",1,&ObstacleMapper::odom_callback,this);
            arm_target_sub_ = nh_.subscribe("position_desired",1,&ObstacleMapper::target_callback,this);
            arm_target_pub_ = nh_.advertise<geometry_msgs::Point>("position_command",1);


            Delta.setIdentity();

            // To move the odometry estimation to the arm base
            ros::Time now = ros::Time::now();
            listener_.waitForTransform(ground_frame,target_frame,now,ros::Duration(3.0));
            listener_.lookupTransform(ground_frame,target_frame,now, base_tf);

            // Now prepare the transformation between laser and the ground
            // frame
            listener_.waitForTransform(laser_frame,target_frame,now,ros::Duration(1.0));
            listener_.lookupTransform(target_frame,laser_frame,now, hokuyo_tf);
            hokuyo_inv_tf = hokuyo_tf.inverse();
            // Compute the equation of the laser plane (ax+by+cz+d=0)
            const tf::Vector3 z_up = hokuyo_tf.getBasis().getColumn(2);
            double x0 = hokuyo_tf.getOrigin().getX();
            double y0 = hokuyo_tf.getOrigin().getY();
            double z0 = hokuyo_tf.getOrigin().getZ();
            a = z_up.getX();
            b = z_up.getY();
            c = z_up.getZ();
            ROS_INFO("Hokuyo reference: (%.3f %.3f %.3f) Z %.3f %.3f %.3f",
                    x0,y0,z0,a,b,c);
            d = -(a*x0+b*y0+c*z0);
            ROS_INFO("Laser plane is %.3fx + %.3fy + %.3fz + %.3f = 0",a,b,c,d);


            // Compute the difference between the target frame and the ground level.
            z_offset = -base_tf.getOrigin().getZ();
            ROS_INFO("Z Offset: %.3f",z_offset);

            // Initialise the projected occupancy window
            proj_occupancy = cv::Mat_<uint8_t>((int)round((max_z-min_z)/occupancy_resolution),
                    (int)round((max_y-min_y)/occupancy_resolution),(uint8_t)0x00);
        }

        void odom_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
            // First project the pose to the target frame (tf)
            // And convert to tf::Transform
            tf::poseMsgToTF(msg->pose, odom);
            odom *= base_tf;
            if (received_odom) {
                // then compute dx, dy, dtheta
                tf::Transform inv = odom.inverse();
                Delta = inv * last_odom * Delta;
            }
            received_odom = true;
            last_odom = odom;
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
            ros::Time t0 = ros::Time::now();
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(target_frame,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(target_frame,msg->header.stamp, temp, msg->header.frame_id, pc, listener_);

            // Compute the displacement since last point cloud
            float ds = hypot(Delta.getOrigin().getX(),Delta.getOrigin().getY());
            if (ds > displacement_threshold) {
                // If the displacement is large enough, apply it to the point
                // cloud in map
                MapType new_map;
                for (MapType::iterator it=map.begin();it!=map.end();it++) {
                    float x,y,z;
                    // Apply the transform delta
                    tf::Vector3 v = Delta * it->second; 
                    x = v.getX(); y = v.getY(); z = v.getZ();
                    // Filter out any point on the laser plane
                    float p = a*x+b*y+c*z+d;
                    if (fabs(p) < plane_distance_threshold) {
                        // ROS_INFO("Discarding point %.3f %.3f %.3f: %.3f (was %.3f %.3f %.3f)",x,y,z,p,it->second.getX(),it->second.getY(),it->second.getZ());
                        // This point is on the laser plane. It will be updated by
                        // this scan
                        continue;
                    }
                    if ((x > min_x) && (x < max_x) && (y<max_y) && (y>min_y)) {
                        new_map.insert(MapType::value_type(x,v));
                    }
                }
                map = new_map;
                // Don't forget to reset the displacement transform
                Delta.setIdentity();

                unsigned int n = pc.size();
                for (unsigned int i=0;i<n;i++) {
                    float x = pc[i].x;
                    float y = pc[i].y;
                    float z = pc[i].z;
                    float d = sqrt(temp[i].x*temp[i].x+temp[i].y*temp[i].y+temp[i].z*temp[i].z);
                    // ROS_INFO("New point p=%.3f",a*x+b*y+c*z+d);
                    if (d < min_d) {
                        // Bogus point, ignore
                        pc[i].x=NAN;
                        pc[i].y=NAN;
                        pc[i].z=NAN;
                        continue;
                    }
                    if ((y>max_y)||(y<min_y)||(z>max_z)) { 
                        // Too far, ignore
                        continue;
                    }
                    map.insert(MapType::value_type(x,tf::Vector3(x,y,z)));
                }
            }
            // Now project the point cloud in the map and the one in pc into an
            // occupancy map, in the y-z plane (normal to the longitudinal
            // axis).
            // Set the all projected occupancy to 0xFF (Free)
            proj_occupancy = 0xFF;
            for (MapType::iterator it=map.begin();it!=map.end();it++) {
                if ((it->first < occ_min_x) || (it->first > occ_max_x)) { // TODO: use upper_bound
                    // We only consider forward collision possibilities
                    continue;
                }
                float y,z;
                const tf::Vector3  & v = it->second; 
                y = v.getY(); z = v.getZ();
                int i = (int)round((y-min_y)/occupancy_resolution);
                int j = (int)round((z-z_offset-min_z)/occupancy_resolution);
                // ROS_INFO("PC: %.3f %.3f -> %d %d",y,z,i,j);
                if ((i<0) || (i>=proj_occupancy.cols) || (j<0) || (j>=proj_occupancy.rows)) {
                    continue;
                }
                proj_occupancy(proj_occupancy.rows-j-1,i) = 0x00;
            }
            unsigned int n = pc.size();
            for (unsigned int i=0;i<n;i++) {
                float d = sqrt(temp[i].x*temp[i].x+temp[i].y*temp[i].y+temp[i].z*temp[i].z);
                if (d < min_d) {
                    continue;
                }
                float x,y,z;
                x = pc[i].x; y = pc[i].y; z = pc[i].z;
                if ((x < occ_min_x) || (x> occ_max_x)) { 
                    continue;
                }
                int i = (int)round((y-min_y)/occupancy_resolution);
                int j = (int)round((z-z_offset-min_z)/occupancy_resolution);
                if ((i<0) || (i>=proj_occupancy.cols) || (j<0) || (j>=proj_occupancy.rows)) {
                    continue;
                }
                proj_occupancy(proj_occupancy.rows-j-1,i) = 0x00;
            }
            int erosion_type = cv::MORPH_RECT ;
            cv::Mat element = cv::getStructuringElement(erosion_type,
                    cv::Size((int)round(tool_width/occupancy_resolution),(int)round(tool_height/occupancy_resolution)),
                    cv::Point((int)round(tool_width/(2*occupancy_resolution)), (int)round(tool_height/(2*occupancy_resolution))));
            cv::erode( proj_occupancy, proj_occupancy, element );
            cv::imshow( "OccGrid", proj_occupancy );
            ros::Time tn = ros::Time::now();
            // ROS_INFO("Point-cloud processing: %d points %f ms",(int)map.size() + n,(tn-t0).toSec()*1000);
            // plot_map();
        }

        void target_callback(const geometry_msgs::PointConstPtr & msg) {
            static bool warning_printed = false;
            if (fabs(msg->y) > 0.1) {
                if (!warning_printed) {
                    ROS_WARN("Obstacle avoidance not tuned for y arm position different from zero");
                }
                warning_printed = true;
                return;
            }
            ros::Time now = ros::Time::now();
            cv::Mat_<float> score(proj_occupancy.size(),128.0);

            int ibest=-1,jbest=-1;
            float bestscore = 0;
            for (int j=0;j<score.rows;j++) {
                for (int i=0;i<score.cols;i++) {
                    if (proj_occupancy(j,i)==0) {
                        score(j,i) = 0;
                    } else {
                        // score(j,i) = 0xFF*exp(-0.5*hypot(5.0*(P.x-i/0.1), 1.0*(P.z-(100.-j)/0.1))/10.);
                        float x=min_y + i*occupancy_resolution, z=min_z + z_offset + (score.rows-j)*occupancy_resolution;
                        float dx = -msg->x-x, dz = msg->z - z;
                        // ROS_INFO("%d %d -> %.2f %.2f -> %.2f %.2f",i,j,x,z,dx,dz);
                        score(j,i) = exp(-hypot(weight_horizontal*dx, weight_vertical*dz)/10);
                        if (score(j,i)>bestscore) {
                            bestscore = score(j,i);
                            ibest = i;
                            jbest = j;
                        }
                    }
                }
            }
            // Minus sign du to change of frame. TODO: see if we can get that
            // through the TF tree
            geometry_msgs::Point P;
            P.x = - (min_y + ibest * occupancy_resolution);
            P.y = 0.0;
            P.z = min_z + z_offset + (score.rows-jbest)*occupancy_resolution;
            arm_target_pub_.publish(P);
            cv::imshow( "Scores", score );
        }
        
        void plot_map() {
            FILE *fp = fopen("/tmp/pc","w");
            for (MapType::const_iterator it=map.begin();it!=map.end();it++) {
                fprintf(fp,"%e %e %e\n",it->second.getX(),it->second.getY(),it->second.getZ());
            }
            unsigned int n = pc.size();
            for (unsigned int i=0;i<n;i++) {
                fprintf(fp,"%e %e %e\n",pc[i].x,pc[i].y,pc[i].z);
            }
            fclose(fp);
            plot.plot("splot [-2:5][-10:0][-1:5] \"/tmp/pc\" u 1:2:3 w p");
            // plot.plot("splot \"/tmp/pc\" u 1:2:3 w p");
        }
};


int main(int argc, char *argv[]) {
    ros::init(argc,argv,"obstacle_mapper");
    cv::namedWindow( "OccGrid", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "Scores", CV_WINDOW_AUTOSIZE );

    ObstacleMapper fm;

    while (ros::ok()) {
        ros::spinOnce();
        cv::waitKey(10);
    }

    return 0;
}
        

