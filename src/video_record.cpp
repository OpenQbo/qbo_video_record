/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2012 TheCorpora SL
 *
 * This program is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 * Authors: Arturo Bajuelos <arturo@openqbo.com>; 
 *          Sergio Merino <s.merino@openqbo.com>;
 *          etc.
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>

sensor_msgs::CvBridge g_bridge;

int isColor = 1;
int fps     = 30; 
int frameW  = 480; 
int frameH  = 320;
CvVideoWriter *writer=cvCreateVideoWriter("out.avi",CV_FOURCC('P','I','M','1'),fps,cvSize(frameW,frameH),isColor);


void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info)
{
  if (g_bridge.fromImage(*image, "bgr8")) {
    IplImage *image = g_bridge.toIpl();
    if (image) {
      cvWriteFrame(writer, image);
    } else {
      ROS_WARN("Couldn't save image, no data!");
    }
  }
  else
    ROS_ERROR("Unable to convert %s image to bgr8", image->encoding.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "qbo_record_video", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  std::string topic = nh.resolveName("image");
  image_transport::CameraSubscriber sub = it.subscribeCamera(topic, 1, &callback);
  ros::spin();
  cvReleaseVideoWriter(&writer);
  printf("File Closed\n");
}
