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

        std::string target_frame;
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
            listener_.waitForTransform("/VSV/base",target_frame,now,ros::Duration(3.0));
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
            a = z_up.getX();
            b = z_up.getY();
            c = z_up.getZ();
            ROS_INFO("Hokuyo reference: (%.3f %.3f %.3f) Z %.3f %.3f %.3f",
                    x0,y0,z0,a,b,c);
            d = -(a*x0+b*y0+c*z0);
            ROS_INFO("Laser plane is %.3fx + %.3fy + %.3fz + %.3f = 0",a,b,c,d);


            z_offset = -base_tf.getOrigin().getZ();
            ROS_INFO("Z Offset: %.3f",z_offset);
            proj_occupancy = cv::Mat_<uint8_t>(100,100,(uint8_t)0x00);
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
                tf::Transform inv = odom.inverse();
                // ROS_INFO("Inv Odom : %.4f %.4f %.4f",inv.getOrigin().getX(),inv.getOrigin().getY(),tf::getYaw(inv.getRotation())*180./M_PI);
                // ROS_INFO("Odom : %.4f %.4f %.4f",odom.getOrigin().getX(),odom.getOrigin().getY(),tf::getYaw(odom.getRotation())*180./M_PI);
                // ROS_INFO("Last Odom : %.4f %.4f %.4f",last_odom.getOrigin().getX(),last_odom.getOrigin().getY(),tf::getYaw(last_odom.getRotation())*180./M_PI);
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

            float ds = hypot(Delta.getOrigin().getX(),Delta.getOrigin().getY());
            // ROS_INFO("Delta %.3f : %.4f %.4f %.4f",ds,Delta.getOrigin().getX(),Delta.getOrigin().getY(),tf::getYaw(Delta.getRotation())*180./M_PI);
            if (ds > 1e-1) {
                MapType new_map;
                for (MapType::iterator it=map.begin();it!=map.end();it++) {
                    float x,y,z;
                    tf::Vector3 v = Delta * it->second; 
                    x = v.getX(); y = v.getY(); z = v.getZ();
                    float p = a*x+b*y+c*z+d;
                    if (fabs(p) < 1e-2) {// TODO: set a param
                        // ROS_INFO("Discarding point %.3f %.3f %.3f: %.3f (was %.3f %.3f %.3f)",x,y,z,p,it->second.getX(),it->second.getY(),it->second.getZ());
                        // This point is on the laser plane. It will be updated by
                        // this scan
                        continue;
                    }
                    if ((x > -1.0) && (x < 5.0) && (y<0) && (y>-10.0)) {// TODO: set a param 
                        new_map.insert(MapType::value_type(x,v));
                    }
                }
                map = new_map;
                Delta.setIdentity();

                unsigned int n = pc.size();
                for (unsigned int i=0;i<n;i++) {
                    float x = pc[i].x;
                    float y = pc[i].y;
                    float z = pc[i].z;
                    float d2 = temp[i].x*temp[i].x+temp[i].y*temp[i].y+temp[i].z*temp[i].z;
                    // ROS_INFO("New point p=%.3f",a*x+b*y+c*z+d);
                    if (d2 < 5e-2) {
                        // Bogus point, ignore
                        pc[i].x=NAN;
                        pc[i].y=NAN;
                        pc[i].z=NAN;
                        continue;
                    }
                    if ((y>0.)||(y<-10.)||(z>10.)) { // TODO: set a param
                        // Too far, ignore
                        continue;
                    }
                    map.insert(MapType::value_type(x,tf::Vector3(x,y,z)));
                }
            }
            proj_occupancy = 0xFF;
            for (MapType::iterator it=map.begin();it!=map.end();it++) {
                if ((it->first < -0.5) || (it->first > 1.5)) { // TODO: set a param, use upper_bound
                    // We only consider forward collision possibilities
                    continue;
                }
                float y,z;
                const tf::Vector3  & v = it->second; 
                y = v.getY(); z = v.getZ();
                int i = (int)round(-y/0.1);
                int j = (int)round((z-z_offset)/0.1);
                // ROS_INFO("PC: %.3f %.3f -> %d %d",y,z,i,j);
                if ((i<0) || (i>=proj_occupancy.cols) || (j<0) || (j>=proj_occupancy.rows)) {
                    continue;
                }
                proj_occupancy(proj_occupancy.rows-j-1,i) = 0x00;
            }
            unsigned int n = pc.size();
            for (unsigned int i=0;i<n;i++) {
                float d2 = temp[i].x*temp[i].x+temp[i].y*temp[i].y+temp[i].z*temp[i].z;
                if (d2 < 5e-2) {
                    continue;
                }
                float x,y,z;
                x = pc[i].x; y = pc[i].y; z = pc[i].z;
                if ((x<-0.5) || (x>1.5)) { // TODO set a param
                    continue;
                }
                int i = (int)round(-y/0.1);
                int j = (int)round((z-z_offset)/0.1);
                if ((i<0) || (i>=proj_occupancy.cols) || (j<0) || (j>=proj_occupancy.rows)) {
                    continue;
                }
                proj_occupancy(proj_occupancy.rows-j-1,i) = 0x00;
            }
            int erosion_type = cv::MORPH_RECT ;
            cv::Mat element = cv::getStructuringElement(erosion_type,
                    cv::Size(15,5), cv::Point( 7, 2));
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
            listener_.waitForTransform(target_frame,"/VSV/ArmPan",now,ros::Duration(1.0));
            geometry_msgs::PointStamped Pin, Pout;
            geometry_msgs::Point P;
            Pin.point = *msg;
            Pin.header.frame_id = "/VSV/ArmPan";
            Pin.header.stamp = now;
            listener_.transformPoint(target_frame, Pin, Pout);
            // ROS_INFO("Target at %.2f %.2f",Pout.point.x,Pout.point.z);
            P = Pout.point;
            cv::Mat_<float> score(proj_occupancy.size(),128.0);

            int ibest=-1,jbest=-1;
            float bestscore = 0;
            for (int j=0;j<score.rows;j++) {
                for (int i=0;i<score.cols;i++) {
                    if (proj_occupancy(j,i)==0) {
                        score(j,i) = 0;
                    } else {
                        // score(j,i) = 0xFF*exp(-0.5*hypot(5.0*(P.x-i/0.1), 1.0*(P.z-(100.-j)/0.1))/10.);
                        float x=i*0.1, z=z_offset + (score.rows-j)*0.1;
                        float dx = P.x-x, dz = P.z - z;
                        // ROS_INFO("%d %d -> %.2f %.2f -> %.2f %.2f",i,j,x,z,dx,dz);
                        score(j,i) = exp(-hypot(3.0*dx, 1.0*dz)/10);
                        if (score(j,i)>bestscore) {
                            bestscore = score(j,i);
                            ibest = i;
                            jbest = j;
                        }
                    }
                }
            }
            P.x = ibest * 0.1;
            P.y = 0.0;
            P.z = z_offset + (score.rows-jbest)*0.1;
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
        

