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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <iostream>
#include <signal.h>


std::string defaultDirectory = "/home/qboblue/Videos/";
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
cv::VideoWriter *writer;
FILE* soundProc;
std::string directory;
std::string filename;
pid_t pID;

void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    *writer <<  cv_ptr->image;
}

void endProgram()
{
//  cvReleaseVideoWriter(&writer);

  //Set combining command, filename and directory
  std::string combi=combinator+" "+directory+filename+finalExtension+" "+directory+filename+sExtension+" "+directory+filename+extension;
  const char * combiCommand=combi.c_str();

  //Set remove command
  std::string rm=deleter+" "+directory+filename+sExtension+" "+directory+filename+extension;
  const char * removeCommand=rm.c_str();

  //Closing record audio command
  kill( pID, SIGKILL );
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


void finishRecord(int sig)
{
   ros::shutdown();
   printf("Signal captured\n");
   endProgram();
   printf("Ended Program");
   exit(0);
}

int main(int argc, char** argv)
{
  //Init ROS
  ros::init(argc, argv, "qbo_record_video", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  std::string topic = nh.resolveName("image");
  directory= nh.resolveName("recordDir");
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
  filename=sstr.str();
  std::string file=directory+filename+extension;

  //Define Video file
  writer=new cv::VideoWriter(file, CV_FOURCC('D','I','V','X'), fps, cv::Size(frameW,frameH), true);


  pID = fork();
  if (pID == 0)                // child
  {
      //Set audio command, file name and directory
      std::string sound=soundRecorder+" "+directory+filename+sExtension;
      const char * soundCommand=sound.c_str();
      //Start recording the sound
      soundProc = popen(soundCommand, "r");
      pclose(soundProc);
  }
  else
  {
      //Captur SIGINT
      signal(SIGINT,&finishRecord);

      //Defining imaging call back
      image_transport::CameraSubscriber sub = it.subscribeCamera(topic, 1, &callback);

      //Ros spin, callback will be used to record images
      ros::spin();

      printf("Ending program");
      endProgram();
  }
}
