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
#include <string>
#include <iostream>

sensor_msgs::CvBridge g_bridge;

std::string defaultDirectory = "/home/qboblue/Pictures/";
std::string defaultTopic="/stereo/left/image_raw";
std::string extension=".avi";
std::string soundRecorder="arecord";
std::string sExtension=".wav";
std::string combinator="mkvmerge -o ";
std::string finalExtension=".mkv";
std::string deleter="rm";
int isColor = 1;
int fps     = 30; 
int frameW  = 480; 
int frameH  = 320;
CvVideoWriter *writer;


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
  //Init ROS
  ros::init(argc, argv, "qbo_record_video", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  std::string topic = nh.resolveName("image");
  std::string directory= nh.resolveName("recordDir");
  if (topic=="/image"){
    topic=defaultTopic;
  }
  if (directory=="/recordDir"){
    directory=defaultDirectory;
  }
  printf("We will record this video topic: '%s' and the destination: %s\n",topic.c_str(), directory.c_str());
  //Set Video file name and directory
  int now=time(0);
  std::stringstream sstr;
  sstr << now;
  std::string filename=sstr.str();
  std::string file=directory+filename+extension;
  const char * f=file.c_str();

  //Set audio command, file name and directory
  std::string sound=soundRecorder+" "+directory+filename+sExtension;
  const char * soundCommand=sound.c_str();

  //Set combining command, filename and directory
  std::string combi=combinator+" "+directory+filename+finalExtension+" "+directory+filename+sExtension+" "+directory+filename+extension;
  const char * combiCommand=combi.c_str();

  //Set remove command
  std::string rm=deleter+" "+directory+filename+sExtension+" "+directory+filename+extension;
  const char * removeCommand=rm.c_str();

  //Define Video file
  writer=cvCreateVideoWriter(f,CV_FOURCC('P','I','M','1'),fps,cvSize(frameW,frameH),isColor);

  //Start recording the sound
  FILE* soundProc = popen(soundCommand, "r");

  //Defining imaging call back
  image_transport::CameraSubscriber sub = it.subscribeCamera(topic, 1, &callback);

  //Ros spin, callback will be used to record images
  ros::spin();

  //Closing the video File
  cvReleaseVideoWriter(&writer);
  
  //Closing record audio command
  pclose(soundProc);
  printf("File Closed\n");
  printf("Starting combining files\n");
 
  //Combining  audio and video command
  FILE* combProc = popen(combiCommand, "r");
  char buffer[1028];
  while (fgets(buffer, 1028, combProc) != NULL)
  {
  }
  pclose(combProc);

  //Removing audio and video files
  FILE* removeProc = popen(removeCommand, "r");
  while (fgets(buffer, 1028, removeProc) != NULL)
  {
  }
  pclose(removeProc);
 
}
