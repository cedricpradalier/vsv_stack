#include <stdlib.h>
#include <stdio.h>

#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>

#include <bsplines/Function.h>

#include <cgnuplot/CGnuplot.h>



class FloorMapper {
    protected:
        ros::NodeHandle nh_;
        image_transport::Subscriber tsub_;
        image_transport::ImageTransport it_;
        ros::Publisher point_pub_;
        ros::Subscriber odom_sub_;

        tf::TransformListener listener_;

        int search_offset_forward;
        int search_offset_lateral;
        int search_length;
        double projected_floor_size_meter;
        double image_origin_x,image_origin_y;
        std::string target_frame;

        splines::Function line; 
        bool received_odom, first_odom;
        tf::Pose last_odom,odom;
        tf::StampedTransform base_tf;
        cgnuplot::CGnuplot plot;

    public:
        FloorMapper() : nh_("~"), it_(nh_) {
            std::string transport = "raw";
            nh_.param("transport",transport,transport);
            nh_.param("projected_floor_size_meter",projected_floor_size_meter,5.0);
            nh_.param("search_offset_forward_pix",search_offset_forward,0);
            nh_.param("search_offset_lateral_pix",search_offset_lateral,0);
            nh_.param("search_length",search_length,0);
            nh_.param("image_origin_x",image_origin_x,0.0);
            nh_.param("image_origin_y",image_origin_y,0.0);
            nh_.param("target_frame",target_frame,std::string("/VSV/ArmPan"));

            first_odom = true;
            received_odom = false;
            tsub_ = it_.subscribe<FloorMapper>("projection",1, &FloorMapper::image_callback,this,transport);
            point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("detection",1);
            odom_sub_ = nh_.subscribe("odom",1,&FloorMapper::odom_callback,this);

            // To move the odometry estimation to the arm base
            listener_.waitForTransform("/VSV/base",target_frame,ros::Time(0),ros::Duration(1.0));
            listener_.lookupTransform("/VSV/base",target_frame,ros::Time(0), base_tf);

        }

        void odom_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
            // First project the pose to the target frame (tf)
            // And probably convert to tf::Transform
            tf::poseMsgToTF(msg->pose, odom);
            // ROS_INFO("Odom : %.4f %.4f %.4f",odom.getOrigin().getX(),odom.getOrigin().getY(),tf::getYaw(odom.getRotation())*180./M_PI);
            odom *= base_tf;
            // ROS_INFO("Odom B: %.4f %.4f %.4f",odom.getOrigin().getX(),odom.getOrigin().getY(),tf::getYaw(odom.getRotation())*180./M_PI);
            if (received_odom && !line.empty()) {
                // then compute dx, dy, dtheta
                tf::Transform inv = odom.inverse();
                // ROS_INFO("Inv Odom : %.4f %.4f %.4f",inv.getOrigin().getX(),inv.getOrigin().getY(),tf::getYaw(inv.getRotation())*180./M_PI);
                // ROS_INFO("Odom : %.4f %.4f %.4f",odom.getOrigin().getX(),odom.getOrigin().getY(),tf::getYaw(odom.getRotation())*180./M_PI);
                // ROS_INFO("Last Odom : %.4f %.4f %.4f",last_odom.getOrigin().getX(),last_odom.getOrigin().getY(),tf::getYaw(last_odom.getRotation())*180./M_PI);
                tf::Transform Delta = inv * last_odom;
                // ROS_INFO("Odom delta: %.4f %.4f %.4f",Delta.getOrigin().getX(),Delta.getOrigin().getY(),tf::getYaw(Delta.getRotation())*180./M_PI);
                // apply transform to line
                splines::Function transformed_line;
                for (splines::Function::const_iterator it = line.begin();
                        it != line.end(); it ++) {
                    tf::Vector3 v(it->first,-it->second,0.0);
                    v = Delta * v;
                    assert(-v.getY() < 7.0);
                    transformed_line.set(v.getX(),-v.getY());
                }
                // discard any data further than 1m behind
                line = transformed_line.select(-1,line.xmax());
                line.print("/tmp/line");
                plot.plot("plot [-1:1.80][1:3.80] \"/tmp/line\" u 1:2 w lp");
            }
            received_odom = true;
            last_odom = odom;
        }

        void image_callback(const sensor_msgs::ImageConstPtr& msg) {
            cv::Mat img(cv_bridge::toCvShare(msg,"mono8")->image);
            int i,j, jmin, jmax;
            i = search_offset_forward;
            jmin = search_offset_lateral+search_length;
            jmax = search_offset_lateral;
            // printf("Searching on column %d between %d and %d:\n",i,search_offset_lateral,search_offset_lateral+search_length);
            for (j=search_offset_lateral;j<(int)search_offset_lateral+search_length;j++) {
                uint8_t pix = img.at<uint8_t>(j,i);
                // printf("%02X ",pix);
                if (pix < 10) {
                    jmin = std::min(j,jmin);
                    jmax = std::max(j,jmax);
                }
            }
            // printf("\nSolution in [%d,%d]",jmin,jmax);
            if (jmin <= jmax) {
                geometry_msgs::PointStamped Pin,Pout;
                Pin.header.stamp = msg->header.stamp;
                Pin.header.frame_id = msg->header.frame_id;
                Pin.point.z = 0;
                Pin.point.y = image_origin_y+search_offset_forward * projected_floor_size_meter / img.cols;
                Pin.point.x = image_origin_x+(jmax+jmin)/2.0 * projected_floor_size_meter / img.rows;
                listener_.waitForTransform(target_frame,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
                listener_.transformPoint(target_frame,Pin,Pout);
                line.set(Pout.point.y,Pout.point.x);
                Pin.point.y = 0;
                Pin.point.x = line(0);
                point_pub_.publish(Pin);
            }
        }
        
};


int main(int argc, char *argv[]) {
    ros::init(argc,argv,"floor_mapper");

    FloorMapper fm;

    ros::spin();

    return 0;
}
        

