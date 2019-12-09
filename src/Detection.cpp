/******************************************************************************
 *  MIT License
 *
 *  Copyright (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *******************************************************************************/

/**
 * @file      Detection.hpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Detection class Implementation
 */

#include "../include/irona/Detection.hpp"
#include "../include/irona/IDetection.hpp"


Detection::Detection() {
    tagSub = handler.subscribe("/ironaTags/arucoDetected", 11, &Detection::detectionCallback, this);
    pub = handler.advertise<geometry_msgs::PoseStamped>("boxPoses", 1, true);
}

void Detection::detectionCallback(const std_msgs::Bool::ConstPtr& checkDetect) {
    this->tagDetected = *checkDetect;
    if (detectTag()) {
        publishBoxPoses();
    }
}

bool Detection::detectTag() {
    bool flag;
	try{
		flag = true;
		listener.lookupTransform("/map", "/aruco_marker_frame", 
                                ros::Time(0), transform);

		tagPose.header.frame_id = "map";
		tagPose.header.stamp = ros::Time::now();
		
		tagPose.pose.position.x = transform.getOrigin().x();
		tagPose.pose.position.y = transform.getOrigin().y();
		tagPose.pose.position.z = 0;

		tagPose.pose.orientation.x = 0;
		tagPose.pose.orientation.y = 0;
		tagPose.pose.orientation.z = transform.getRotation().z();
		tagPose.pose.orientation.w = 1;
	}
	catch (const std::exception&){
		flag = false;
	}
	return flag;
}

void Detection::setTagId(int id) {
    ros::param::set("/aruco_single/marker_id", id);
	std::cout << "idhar bhi chal raha\n";
    ros::spinOnce();
}

void Detection::publishBoxPoses() {
    pub.publish(tagPose);
}

Detection::~Detection() {

}



int main(int argc, char* argv[]) {

    ros::init(argc, argv, "detection");
	ROS_INFO_STREAM("Started Detection node");
	std::cout << "chal raha \n";
	IDetection *detect = new Detection();
    detect->setTagId(1);
	ros::spin();
	return 0;
}






