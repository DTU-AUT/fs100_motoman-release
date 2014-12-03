/*********************************************************************
 *
 * ROS wrapper for continous trajectory point streaming using FS100 controller mh5l robot
 * Author: Asger Winther-Jørgensen (awijor@elektro.dtu.dk)
 *
 * Software License Agreement (LGPL License)
 *
 *  Copyright (c) 2014, Technical University of Denmark
 *  All rights reserved.
 
 *  This program is free software; you can redistribute it and/or modify  
 *  it under the terms of the GNU Lesser General Public License as        
 *  published by the Free Software Foundation; either version 2 of the    
 *  License, or (at your option) any later version.                       
 *                                                                        
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include "mh5l_traj.h"

//assign memory for robot state class
Fs100State *robotState;//("192.168.1.10");
float joint_pos[6];

//assign memory for robot commander class
Fs100Cmd *robotCommander;//("192.168.1.10");

//assign memory for ros subscribers, sevices, etc...
ros::Subscriber *sub_target;
ros::ServiceServer *get_reset_srv;
ros::Publisher *pub_joints;
int retry = 1;

void joint_target_cb(const trajectory_msgs::JointTrajectoryPoint& input){
    float temp_pos[6],temp_vel[6],temp_time;
    for(int i=0;i<6;i++){
        temp_pos[i] = input.positions[i];
        temp_vel[i] = input.velocities[i];
        temp_time = input.time_from_start.toSec();
    }    
    robotCommander->addPointToQueue(temp_pos,temp_vel,temp_time);
}

bool reset_robot_trajectory(fs100_motoman::setReset::Request &req, fs100_motoman::setReset::Response &res){
    bool result;
    retry = 1;
    result = robotCommander->resetTrajectory(&retry);
    res.success = true;
    return true;
}

void *get_data_func(void *arg){
    sensor_msgs::JointState js;
    js.position.resize(6);
    while( *((bool *) arg)){
        while(!robotState->getJointsUpdated(joint_pos));
        for(int i=0;i<6;i++){
            js.position[i] = joint_pos[i];
        }
        js.header.stamp = ros::Time::now();
        pub_joints->publish(js);
    }
    
}



int main(int argc, char **argv){
    char* robot_ip;
    ROS_INFO("argc %d",argc);
    if (argc > 1)
    {
        robot_ip = argv[1];
    }
    else
    {
        ROS_ERROR("invalid number of arguments, expected 1 (robot IP)");
        return 1;
    }



    robotState = new Fs100State (robot_ip);
    robotCommander = new Fs100Cmd (robot_ip);
    //setup ROS
    ros::init(argc, argv, "fs100_motoman");
    ros::NodeHandle nh("~mh5l");
        
    sub_target = new ros::Subscriber;
    *sub_target = nh.subscribe("joint_target", 1, &joint_target_cb); //joint target topic
    
    get_reset_srv = new ros::ServiceServer;
    *get_reset_srv = nh.advertiseService("reset_trajectory",&reset_robot_trajectory); //reset trajectory service
    
    pub_joints = new ros::Publisher;
    *pub_joints = nh.advertise<sensor_msgs::JointState> ("joint_state", 1);
    float pos[6],current_pos[6];
    float vel[6];
    bool run_thread = true;
    pthread_t getthreadID;
    

    
    //create state class instance

    if(robotState->init())
    {
        ROS_ERROR("Could not open socket");
        return 1;
    }
    if(robotState->makeConnect())
    {
        ROS_ERROR("Could not connect to robot controller state server");
        return 1;
    }
    
    
    
    //create command class instance

    if(robotCommander->init())
    {
        ROS_ERROR("Could not open socket");
        return 1;
    }
    if(robotCommander->makeConnect())
    {
        ROS_ERROR("Could not connect to robot controller motion server");
        return 1;
    }
    
    
    
    ros::Duration(2.0).sleep();
    if (robotCommander->start(1)){
        ROS_ERROR("Could not start trajectory on fs100, aborting");
        robotState->pgmClose();
        robotCommander->pgmClose();
        ros::shutdown();
        return 1;
    }

        
    
        
    
    pthread_create(&getthreadID,NULL,get_data_func,&run_thread);
    ROS_INFO("Ready to recieve trajectory points");
    while(nh.ok()){
        ros::spinOnce();
    }
    run_thread = false;
    pthread_join(getthreadID,NULL);
    robotState->pgmClose();
    robotCommander->pgmClose();

    

    ros::shutdown(); 
    return 0;
}

