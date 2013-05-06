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





class FloorMapper {
    protected:
        ros::NodeHandle nh_;
        image_transport::Subscriber tsub_;
        image_transport::ImageTransport it_;
        ros::Publisher point_pub_;

        tf::TransformListener listener_;

        int search_offset_forward;
        int search_offset_lateral;
        int search_length;
        double projected_floor_size_meter;
        double image_origin_x,image_origin_y;

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

            tsub_ = it_.subscribe<FloorMapper>("projection",1, &FloorMapper::callback,this,transport);
            point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("detection",1);

        }

        void callback(const sensor_msgs::ImageConstPtr& msg) {
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
                geometry_msgs::PointStamped P;
                P.header.stamp = ros::Time::now();
                P.header.frame_id = msg->header.frame_id;
                P.point.z = 0;
                P.point.y = image_origin_y+search_offset_forward * projected_floor_size_meter / img.cols;
                P.point.x = image_origin_x+(jmax+jmin)/2.0 * projected_floor_size_meter / img.rows;
                point_pub_.publish(P);
            }
        }
        
};


int main(int argc, char *argv[]) {
    ros::init(argc,argv,"floor_mapper");

    FloorMapper fm;

    ros::spin();

    return 0;
}
        

