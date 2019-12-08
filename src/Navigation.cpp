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
 * @file      Navigation.hpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Navigation class Implementation
 */


#include "../include/irona/Navigation.hpp"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Navigation::Navigation() {
}


bool Navigation::getToLocation(const move_base_msgs::MoveBaseGoal &goal_pose) {
    // Spinning a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait for the actionlib sever
    while(!ac.waitForServer(ros::Duration(0.5))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("Sending goal");
    ac.sendGoal(goal_pose);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
        return false;

    return true;
}

void Navigation::recieveTagPose() {
    // handler.subscribe()
} 

void Navigation::goalCheckCallback(const move_base_msgs::MoveBaseGoalConstPtr &goal_pose) {
    std::cout << "it's coming here\n";
    goalCheck = true;
}

void Navigation::recieveGoalPose() {
    sub = handler.subscribe("move_base_simple/goal", 10, &Navigation::goalCheckCallback, this);
    // std::cout << "it's coming here\n";
    ros::spin();

}

Navigation::~Navigation() {
        std::cout << "it's coming here destroyed\n";

}


