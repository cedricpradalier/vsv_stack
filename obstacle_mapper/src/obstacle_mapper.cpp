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

#include <bsplines/Function.h>
#include <cgnuplot/CGnuplot.h>
#include "obstacle_mapper/ColumnCell.h"
#include "obstacle_mapper/LocalMap.h"

using namespace obstacle_mapper;

class ObstacleMapper {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber pc_sub_;
        ros::Subscriber odom_sub_;

        tf::TransformListener listener_;

        std::string target_frame;

        bool received_odom, first_odom;
        tf::Pose last_odom,odom;
        tf::StampedTransform base_tf,hokuyo_tf;
        tf::Transform hokuyo_inv_tf;
        tf::Transform Delta;
        cgnuplot::CGnuplot plot;

        boost::shared_ptr<LocalMap> map;

        size_t x_size, y_size, z_size;
        float x_origin, y_origin, z_origin;
        float horizontal_resolution, vertical_resolution;
        std::vector<float> laser_height;
        std::vector<LevelType> occupancy;
        float laser_tolerance;
        


    public:
        ObstacleMapper() : nh_("~"), x_size(50), y_size(x_size), z_size(50),  // 10m x 10m x 10m
        x_origin(0), y_origin(0), z_origin(-0.1), 
        horizontal_resolution(5.0/x_size), vertical_resolution(-2*z_origin) {
            
            nh_.param("target_frame",target_frame,std::string("/VSV/mowing"));

            first_odom = true;
            received_odom = false;
            pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("scan",1, &ObstacleMapper::pc_callback,this);
            odom_sub_ = nh_.subscribe("odom",1,&ObstacleMapper::odom_callback,this);

            ros::Duration(0.5).sleep();
            ros::Time now = ros::Time::now();

            Delta.setIdentity();

            // To move the odometry estimation to the arm base
            listener_.waitForTransform("/VSV/base",target_frame,now,ros::Duration(1.0));
            listener_.lookupTransform("/VSV/base",target_frame,now, base_tf);

            // Now prepare the transformation between laser and the ground
            // frame
            listener_.waitForTransform("/Hokuyo",target_frame,now,ros::Duration(1.0));
            listener_.lookupTransform(target_frame,"/Hokuyo",now, hokuyo_tf);
            hokuyo_inv_tf = hokuyo_tf.inverse();
            const tf::Vector3 z_up = hokuyo_tf.getBasis().getColumn(2);
            double x0 = hokuyo_tf.getOrigin().getX();
            double y0 = hokuyo_tf.getOrigin().getY();
            double z0 = hokuyo_tf.getOrigin().getZ();
            double a = z_up.getX();
            double b = z_up.getY();
            double c = z_up.getZ();
            ROS_INFO("Hokuyo reference: (%.3f %.3f %.3f) Z %.3f %.3f %.3f",
                    x0,y0,z0,a,b,c);
            double d = -(a*x0+b*y0+c*z0);
            ROS_INFO("Laser plane is %.3fx + %.3fy + %.3fz + %.3f = 0",a,b,c,d);
            // tolerance: projection of horizontal half cell size to the laser plane, plus 25%)
            laser_tolerance = 1.25 * (horizontal_resolution*fabs(a/c)/2.);

            map.reset(new LocalMap(x_origin,y_origin,z_origin, horizontal_resolution, vertical_resolution, 
                        x_size, y_size, z_size));

            size_t ncells = x_size * y_size;
            laser_height = std::vector<float>(ncells,NAN);
            occupancy = std::vector<LevelType>(ncells,Unknown);
            FILE * fp = fopen("/tmp/laser_height.txt","w");
            for (size_t j=0;j<y_size;j++) {
                size_t line = j*x_size;
                float y = y_origin + j * horizontal_resolution;
                for (size_t i=0;i<x_size;i++) {
                    float x = x_origin + i * horizontal_resolution;
                    float z = (-d - a*x - b*y)/c;
                    if ((z >= z_origin) && (z < z_origin + vertical_resolution * z_size)) {
                        laser_height[line + i] = z;
                    }
                    fprintf(fp,"%e %e %e %e\n",x,y,z,laser_height[line+i]);
                }
            }
            fclose(fp);
        }

        void odom_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
            // First project the pose to the target frame (tf)
            // And probably convert to tf::Transform
            tf::poseMsgToTF(msg->pose, odom);
            // ROS_INFO("Odom : %.4f %.4f %.4f",odom.getOrigin().getX(),odom.getOrigin().getY(),tf::getYaw(odom.getRotation())*180./M_PI);
            odom *= base_tf;
            // ROS_INFO("Odom B: %.4f %.4f %.4f",odom.getOrigin().getX(),odom.getOrigin().getY(),tf::getYaw(odom.getRotation())*180./M_PI);
            if (received_odom) {
                // then compute dx, dy, dtheta
                tf::Transform inv = last_odom.inverse();
                // ROS_INFO("Inv Odom : %.4f %.4f %.4f",inv.getOrigin().getX(),inv.getOrigin().getY(),tf::getYaw(inv.getRotation())*180./M_PI);
                // ROS_INFO("Odom : %.4f %.4f %.4f",odom.getOrigin().getX(),odom.getOrigin().getY(),tf::getYaw(odom.getRotation())*180./M_PI);
                // ROS_INFO("Last Odom : %.4f %.4f %.4f",last_odom.getOrigin().getX(),last_odom.getOrigin().getY(),tf::getYaw(last_odom.getRotation())*180./M_PI);
                Delta = inv * odom * Delta;
            }
            received_odom = true;
            last_odom = odom;
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
            ros::Time t0 = ros::Time::now();
            pcl::PointCloud<pcl::PointXYZ> pc;
            pcl::fromROSMsg(*msg, pc);
            // First prepare interpolation function
            splines::Function f;
            unsigned int n = pc.size();
            for (unsigned int i=0;i<n;i++) {
                float x = pc[i].x;
                float y = pc[i].y;
                float d = hypot(x,y);
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                f.set(atan2(y,x),d);
            }
// #define PLOT_OCC
#ifdef PLOT_OCC
            f.print("/tmp/fd");
            FILE *fp_occ=fopen("/tmp/occ","w"),*fp_free=fopen("/tmp/free","w");
#endif
            for (size_t j=0;j<y_size;j++) {
                size_t line = j*x_size;
                float y = y_origin + j * horizontal_resolution;
                for (size_t i=0;i<x_size;i++) {
                    float x = x_origin + i * horizontal_resolution;
                    float z = laser_height[line+i];
                    if (isnan(z)) continue;
                    tf::Vector3 v(x,y,z);
                    v = hokuyo_inv_tf * v;
                    float d = hypot(v.getY(),v.getX());
                    float theta = atan2(v.getY(),-v.getX()); // Voluntary inversion
                    float d_laser = f(theta);

                    // if ((fabs(y-5) < 1) && (x>4)){
                    //     ROS_INFO("PC: %d %d %.3f %.3f %.3f",(int)j,(int)i,x,y,z);
                    //     ROS_INFO("  : V %.3f %.3f %.3f R %.3f %.3f D %.3f",v.getX(),v.getY(),v.getZ(),theta*180./M_PI,d,d_laser);
                    // }
                    if (fabs(d-d_laser)<laser_tolerance) {
                        occupancy[line+i] = Occupied;
                    } else {
                        occupancy[line+i] = (d<d_laser)?Free:Unknown;
                    } 

#ifdef PLOT_OCC
                    switch (occupancy[line+i]) {
                        case Free: fprintf(fp_free,"%e %e %e\n",x,y,z); break;
                        case Occupied: fprintf(fp_occ,"%e %e %e\n",x,y,z); break;
                        default: break;
                    }
#endif
                }
                map->shift(Delta);
                Delta.setIdentity();
                map->update(laser_height,occupancy);
                map->plot_by_type(Occupied,"/tmp/mocc");
                //plot.plot("splot [0:5][0:10][-0.5:5] \"/tmp/mocc\" u 1:2:3 w p");
            }
#ifdef PLOT_OCC
            fclose(fp_occ);
            fclose(fp_free);
            plot.plot("splot [0:5][0:10][0:10]\"/tmp/laser_height.txt\" u 1:2:4 w d, \"/tmp/occ\" u 1:2:3 w p, \"/tmp/occ\" u 1:2:(0) w p");
#endif
            ros::Time tn = ros::Time::now();
            ROS_INFO("Point-cloud processing: %f ms",(tn-t0).toSec());
            // ros::shutdown();
        }
        
};


int main(int argc, char *argv[]) {
    ros::init(argc,argv,"obstacle_mapper");

    ObstacleMapper fm;

    ros::spin();

    return 0;
}
        

