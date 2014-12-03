/*********************************************************************
 *
 * C++ class for continous trajectory point streaming using FS100 controller mh5l robot
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

#include "fs100_cmd.h"

int Fs100Cmd::init()
{
    /*
    Initializes the class instance, setting up sockect and variables.
    
    */
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = setsockopt( sockfd, IPPROTO_TCP, TCP_NODELAY, (void *)&i, sizeof(i));
    setsockopt( sockfd, IPPROTO_TCP, TCP_QUICKACK, (void *)&i, sizeof(i));
    setsockopt( sockfd, IPPROTO_TCP, TCP_QUICKACK, (void *)&i, sizeof(i));
    if (sockfd < 0) 
        socketError("ERROR opening socket");
    server = gethostbyname(IP);
    if (server == NULL)
    {
        fprintf(stderr,"ERROR, no such host\n");
        return 1;
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    thread_sleep = 1000; //microseconds for thread to sleep

    
    printf("robot commander init done\n");
    seq = 0;
    stop_all = false;
    return 0;
}



int Fs100Cmd::makeConnect()
{
    /*
    Connects to the robot controller motion server.
    */
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    {
        socketError("ERROR connecting");
        return 1;
    }
    return 0;    
}

void *Fs100Cmd::cmdPushThread(void *This)
{
    /*
    Function wrapper for starting thread inside class.
    */
    ((Fs100Cmd *) This)->cmdPush();
}

void Fs100Cmd::cmdPush()
{
    /*
    Sends the next trajectory point in queue to the robot controller. 
    Tries to send until point has been accepted whereafter it is popped from queue.
    */    
    while(run_thread && !stop_all)
    {
        
        int retry = 1;
        ///// Do your sending and stuff here, in this thread...right here.
        while(!cmdList.empty())
        { //executing list as fast as posible
            
            //get next command in queue
            cmd currentCmd = cmdList.front();
            
            
            pushTraj(currentCmd.pos,currentCmd.vel,currentCmd.time,seq);
            
            
            
            seq ++;
            cmdList.pop();
        }
        usleep(thread_sleep); //wait for non empty cmd list

    }

}

int Fs100Cmd::start(int retry)
{
    /*
    Starts the first trajectory by sending start trajectory message to robot controller.
    */
    abs_time = 0.0;
    run_thread = 1;
    char motion_buf[68];
    char motion_reply_buffer[76];
    
    SimpleMsg traj_start, motion_check;
    
    printf("sending motion check message\n");
    /*
    motion_ready(&motion_check);
    serialize(&motion_check,motion_buf,sizeof(motion_buf));
    byte_swap(motion_buf,sizeof(motion_buf));
    n = write(sockfd,motion_buf,sizeof(motion_buf));
    if (n < 0) 
        socket_error("ERROR writing motion check to socket");
    n = recv(sockfd,motion_reply_buffer,sizeof(motion_reply_buffer),0);
    byte_swap(motion_reply_buffer,sizeof(motion_reply_buffer));
    deserialize_motion_reply(motion_reply_buffer,&motion_rpl);
    if (motion_rpl.body.motionReply.result == 0){
        retry = 0;
    }
    else{
        print_error_code_msg(&motion_rpl);
        print_motion_reply(&motion_rpl);
        //return 1;
    }
    printf("seq: %d\n",seq);

    
    printf("\n\nmotion ready sent, reply:\n");

    print_error_code_msg(&motion_rpl);
    
    usleep(2000000); //2 second
    */ //Code not used. Motion check message returns error if servo is off (trajectory not started)
    printf("sending trajecotry start message\n");
    int last_result = 0;
    int last_subcode = 0;
    int result;
    int subcode;
    while(retry && !stop_all)
    {
        trajectoryStart(&traj_start);
        serialize(&traj_start,motion_buf,sizeof(motion_buf));
        byteSwap(motion_buf,sizeof(motion_buf));
        n = write(sockfd,motion_buf,sizeof(motion_buf));
        if (n < 0) 
            socketError("ERROR writing trajectory start to socket");
        n = recv(sockfd,motion_reply_buffer,sizeof(motion_reply_buffer),0);
        byteSwap(motion_reply_buffer,sizeof(motion_reply_buffer));
        deserializeMotionReply(motion_reply_buffer,&motion_rpl);
        result = motion_rpl.body.motionReply.result;
        subcode = motion_rpl.body.motionReply.subcode;
        if (motion_rpl.body.motionReply.result != 0)
        { //no success
            if (last_result != result || last_subcode != subcode)
                printErrorCodeMsg(&motion_rpl);
        }
        else
        {
            retry = 0;
        }
        last_subcode = subcode;
        last_result = result;
    }
    printErrorCodeMsg(&motion_rpl);
    
    
    
    pthread_create(&cmdthreadID,NULL,cmdPushThread,this);
    return 0; 
}

bool Fs100Cmd::pushTraj(float pos[6],float vel[6],float time,int seq)
{
    /*
    Sends the given data as a trajectoryPointFull to robot controller.
    */
    abs_time += time;
    //printf("abs time sent: %f\n",abs_time);
    SimpleMsg traj;
    buildTrajFull(&traj,pos,vel,abs_time,seq);
    serialize(&traj,traj_buffer,sizeof(traj_buffer));
    byteSwap(traj_buffer,sizeof(traj_buffer));
    int retry = 1;
    while(retry && !stop_all)
    {
        write(sockfd,traj_buffer,sizeof(traj_buffer));
        recv(sockfd,motion_reply_buffer,sizeof(motion_reply_buffer),0);
        byteSwap(motion_reply_buffer,sizeof(motion_reply_buffer));
        deserializeMotionReply(motion_reply_buffer,&motion_rpl);
        
        if(motion_rpl.body.motionReply.result == 1); //printf("result: BUSY\n");
        else if (motion_rpl.body.motionReply.result == 0)
        {
            retry = 0;
        }
        else
        {
            printErrorCodeMsg(&motion_rpl);
            retry = 0;
        }

    }
    return true;

}

bool Fs100Cmd::addCmdToQueue(cmd cmd_point)
{
    /*
    Adds trajectory point in cmd_point format to queue.
    */
    cmdList.push(cmd_point);
}

bool Fs100Cmd::addPointToQueue(float pos[6],float vel[6],float time)
{
    /*
    add data to cmd queue.
    */
    cmd temp;
    for(int i=0;i<6;i++)
    {
        temp.pos[i] = pos[i];
        temp.vel[i] = vel[i];
    }
    temp.time = time;
    cmdList.push(temp);
}

bool Fs100Cmd::resetTrajectory(int *retry)
{
    /*
    Resets the trajectory by stopping trajectory, reseting variables and starting new trajectory.
    */
    //stop current trajectory, if any
    printf("reseting buffer\n");
    SimpleMsg traj_stop, m_reply;
    char motion_buf[68];
    char motion_reply_buffer[76];
    trajectoryStop(&traj_stop);
    serialize(&traj_stop,motion_buf,sizeof(motion_buf));
    byteSwap(motion_buf,sizeof(motion_buf));
    n = write(sockfd,motion_buf,sizeof(motion_buf));
    if (n < 0) 
        socketError("ERROR writing trajectory stop to socket");
    n = recv(sockfd,motion_reply_buffer,sizeof(motion_reply_buffer),0);
    byteSwap(motion_reply_buffer,sizeof(motion_reply_buffer));
    deserializeMotionReply(motion_reply_buffer,&m_reply);
    printf("reset:\n");
    printErrorCodeMsg(&m_reply);
    
    // start new trajectory
    SimpleMsg traj_start;
    printf("sending trajecotry start message\n");
    int last_result = 0;
    int last_subcode = 0;
    int result;
    int subcode;
    bool run = true;
    while(*retry && run)
    {
        //printf("alive and stop_all is %d\n",stop_all);
        trajectoryStart(&traj_start);
        serialize(&traj_start,motion_buf,sizeof(motion_buf));
        byteSwap(motion_buf,sizeof(motion_buf));
        n = write(sockfd,motion_buf,sizeof(motion_buf));
        if (n < 0) 
            socketError("ERROR writing trajectory start to socket");
        n = recv(sockfd,motion_reply_buffer,sizeof(motion_reply_buffer),0);
        byteSwap(motion_reply_buffer,sizeof(motion_reply_buffer));
        deserializeMotionReply(motion_reply_buffer,&m_reply);
        
        result = m_reply.body.motionReply.result;
        subcode = m_reply.body.motionReply.subcode;
        if (m_reply.body.motionReply.result != 0)
        { //no success
            if (last_result != result || last_subcode != subcode)
                printErrorCodeMsg(&m_reply);
        }
        else
        {
            last_subcode = subcode;
            last_result = result;
            run = false;
        }

        last_subcode = subcode;
        last_result = result;
    }
    printErrorCodeMsg(&m_reply);
    abs_time = 0.0;
    seq = 0;
    
    
    return true;
}

void Fs100Cmd::socketError(const char *msg)
{
    /*
    print sockect error
    */
    perror(msg);
    return;
}

void Fs100Cmd::byteSwap(char* data,int length)
{ //destructive
    /*
    Swap endian of integer/float array
    */
    char buffer[length];
    memcpy(buffer,data,length);
    for(int i=0;i<length/4;i++)
    { //number of int/float
        for(int k = 0;k<4;k++)
        { //for every byte in int/float
            data[(i*4)+k] = buffer[(i*4)+(3-k)];
            
        }
    }
}

void Fs100Cmd::serialize(SimpleMsg *msg, char *data,int size)
{
    memcpy(data,msg,size);
}



void Fs100Cmd::deserializeMotionReply(char *data,SimpleMsg *msg)
{
    /*
    inteprets serialized SimpleMsg and stores in provided SimpleMsg address.
    */
    int *q = (int*)data;
    //prefix
    msg->prefix.length =  *q; q++;
    //header
    msg->header.msgType = (SmMsgType) *q; q++;
    msg->header.commType =(SmCommType) *q; q++;
    msg->header.replyType = (SmReplyType) *q; q++;
    //body
    msg->body.motionReply.groupNo = *q; q++;
    msg->body.motionReply.sequence = *q; q++;
    msg->body.motionReply.command = *q; q++;
    msg->body.motionReply.result = (SmResultType) *q; q++;
    msg->body.motionReply.subcode = *q; q++;
    for(int i=0;i<10;i++){
        msg->body.motionReply.data[i] = *(float*)q; q++;
    }    
}

void Fs100Cmd::printMotionReply(SimpleMsg *msg)
{
    /*
    For easy printing of motion reply msg to terminal.
    */
    printf("Prefix:\n");
    printf("  length: %d\n",msg->prefix.length);
    printf("header:\n");
    printf("  msgType: %d\n",msg->header.msgType);
    printf("  commType: %d\n",msg->header.commType);
    printf("  replyType: %d\n",msg->header.replyType);
    printf("body:\n");
    printf("  motionReply:\n");
    printf("    groupNo: %d\n",msg->body.motionReply.groupNo);
    printf("    sequence: %d\n",msg->body.motionReply.sequence);
    printf("    command: %d\n",msg->body.motionReply.command);
    printf("    result: %d\n",msg->body.motionReply.result);
    printf("    subcode: %d\n",msg->body.motionReply.subcode);
    printf("    data:\n");
    for(int i=0;i<10;i++){
        printf("      data[%d]: %f\n",i,msg->body.motionReply.data[i]);
    }
}
    
void Fs100Cmd::printTrajFull(SimpleMsg *msg)
{
    /*
    For easy printing of joint trajectory point full msg to terminal.
    */
    printf("Prefix:\n");
    printf("  length: %d\n",msg->prefix.length);
    printf("header:\n");
    printf("  msgType: %d\n",msg->header.msgType);
    printf("  commType: %d\n",msg->header.commType);
    printf("  replyType: %d\n",msg->header.replyType);
    printf("body:\n");
    printf("  jointTrajData:\n");
    printf("    groupNo: %d\n",msg->body.jointTrajData.groupNo);
    printf("    validFields: %d\n",msg->body.jointTrajData.validFields);
    printf("    sequence: %d\n",msg->body.jointTrajData.sequence);
    printf("    time: %f\n",msg->body.jointTrajData.time);
    printf("    pos:\n");
    for(int i=0;i<10;i++)
    {
        printf("      joint[%d]: %f\n",i+1,msg->body.jointTrajData.pos[i]);
    }
    printf("    vel:\n");
    for(int i=0;i<10;i++)
    {
        printf("      joint[%d]: %f\n",i+1,msg->body.jointTrajData.vel[i]);
    }
    printf("    acc:\n");
    for(int i=0;i<10;i++)
    {
        printf("      joint[%d]: %f\n",i+1,msg->body.jointTrajData.acc[i]);
    }
}

void Fs100Cmd::printErrorCodeMsg(SimpleMsg *msg)
{
    /*
    print error code from motion reply. Can be used to gain information about type of error.
    */
    printf("Result code: ");
    int result = msg->body.motionReply.result;
    int subcode = msg->body.motionReply.subcode;
    if (result == ROS_RESULT_SUCCESS || result == ROS_RESULT_TRUE){
        printf("success\n");
    }
    else printf("error!\n");
    printf("  Result type: %d, ",result);
    if(result == ROS_RESULT_SUCCESS) printf("success/true\n");
    else if(result == ROS_RESULT_TRUE) printf("success/true\n");
    else if(result == ROS_RESULT_BUSY) printf("busy\n");
    else if(result == ROS_RESULT_FAILURE) printf("failure/false\n");
    else if(result == ROS_RESULT_FALSE) printf("failure/false\n");
    else if(result == ROS_RESULT_INVALID) printf("invalid\n");
    else if(result == ROS_RESULT_ALARM) printf("alarm\n");
    else if(result == ROS_RESULT_NOT_READY) printf("not ready\n");
    else if(result == ROS_RESULT_MP_FAILURE) printf("mp failure\n");
    if (result != ROS_RESULT_SUCCESS && result!=ROS_RESULT_TRUE)
    {
        printf("  Subcode type: %d, ",subcode);
        if(subcode == ROS_RESULT_INVALID_UNSPECIFIED) printf("invalid unspecified\n");
        else if (subcode == ROS_RESULT_INVALID_MSGSIZE) printf("invalid message size\n");
        else if (subcode == ROS_RESULT_INVALID_MSGHEADER) printf("invalid message header\n");
        else if (subcode == ROS_RESULT_INVALID_MSGTYPE) printf("invalid message type\n");
        else if (subcode == ROS_RESULT_INVALID_GROUPNO) printf("invalid groupno\n");
        else if (subcode == ROS_RESULT_INVALID_SEQUENCE) printf("invalid sequence\n");
        else if (subcode == ROS_RESULT_INVALID_COMMAND) printf("invalid command\n");
        else if (subcode == ROS_RESULT_INVALID_DATA) printf("invalid data\n");
        else if (subcode == ROS_RESULT_INVALID_DATA_START_POS) printf("invalid data start position\n");
        else if (subcode == ROS_RESULT_INVALID_DATA_POSITION) printf("invalid data position\n");
        else if (subcode == ROS_RESULT_INVALID_DATA_SPEED) printf("invalid data speed\n");
        else if (subcode == ROS_RESULT_INVALID_DATA_ACCEL) printf("invalid data acceleration\n");
        else if (subcode == ROS_RESULT_INVALID_DATA_INSUFFICIENT) printf("invalid data insufficient\n");
        else if (subcode == ROS_RESULT_NOT_READY_UNSPECIFIED) printf("not ready unspecified\n");
        else if (subcode == ROS_RESULT_NOT_READY_ALARM) printf("not ready alarm\n");
        else if (subcode == ROS_RESULT_NOT_READY_ERROR) printf("not ready error\n");
        else if (subcode == ROS_RESULT_NOT_READY_ESTOP) printf("not ready emergency stop\n");
        else if (subcode == ROS_RESULT_NOT_READY_NOT_PLAY) printf("not ready not in play\n");
        else if (subcode == ROS_RESULT_NOT_READY_NOT_REMOTE) printf("not ready not in remote\n");
        else if (subcode == ROS_RESULT_NOT_READY_SERVO_OFF) printf("not ready servo power off\n");
        else if (subcode == ROS_RESULT_NOT_READY_HOLD) printf("not ready hold\n");
        else if (subcode == ROS_RESULT_NOT_READY_NOT_STARTED) printf("not ready not started\n");
        else if (subcode == ROS_RESULT_NOT_READY_WAITING_ROS) printf("not ready waiting for ROS\n");
        else if (subcode == ROS_RESULT_NOT_READY_SKILLSEND) printf("not ready skillsend\n");
    }
    else printf("  Subcode type: 0, success\n");
}


int Fs100Cmd::buildTrajPos(SimpleMsg *tm,float pos[6],float time,int seq)
{
    /*
    build trajectory point position (vel and acc = 0) message using provided data.
    */
    // set prefix: length of message excluding the prefix
	tm->prefix.length = sizeof(SmHeader) + sizeof(SmBodyJointTrajPtFull);
	
	// set header information
	tm->header.msgType = ROS_MSG_JOINT_TRAJ_PT_FULL;
	tm->header.commType = ROS_COMM_SERVICE_RESQUEST;
	tm->header.replyType = ROS_REPLY_INVALID;
	
	// set body
	tm->body.jointTrajData.groupNo = 0;
	tm->body.jointTrajData.validFields = 7;
	tm->body.jointTrajData.sequence = seq;
	tm->body.jointTrajData.time = time;
	for(int i=0;i<6;i++)
	{
    	tm->body.jointTrajData.pos[i] = pos[i];
    	tm->body.jointTrajData.vel[i] = (float)0.0;
    	tm->body.jointTrajData.acc[i] = (float)0.0;
	}
	for(int i=6;i<10;i++)
	{
    	tm->body.jointTrajData.pos[i] = (float)0.0;
    	tm->body.jointTrajData.vel[i] = (float)0.0;
    	tm->body.jointTrajData.acc[i] = (float)0.0;
	}

}

int Fs100Cmd::buildTrajFull(SimpleMsg *tm,float pos[6],float vel[6],float time,int seq)
{
    /*
    build trajectory point full message using provided data.
    */
    // set prefix: length of message excluding the prefix
	tm->prefix.length = sizeof(SmHeader) + sizeof(SmBodyJointTrajPtFull);
	
	// set header information
	tm->header.msgType = ROS_MSG_JOINT_TRAJ_PT_FULL;
	tm->header.commType = ROS_COMM_SERVICE_RESQUEST;
	tm->header.replyType = ROS_REPLY_INVALID;
	
	// set body
	tm->body.jointTrajData.groupNo = 0;
	tm->body.jointTrajData.validFields = 7;
	tm->body.jointTrajData.sequence = seq;
	tm->body.jointTrajData.time = time;
	for(int i=0;i<6;i++)
	{
    	tm->body.jointTrajData.pos[i] = pos[i];
    	tm->body.jointTrajData.vel[i] = vel[i];
    	tm->body.jointTrajData.acc[i] = (float)0.0;
	}
	for(int i=6;i<10;i++)
	{
    	tm->body.jointTrajData.pos[i] = (float)0.0;
    	tm->body.jointTrajData.vel[i] = (float)0.0;
    	tm->body.jointTrajData.acc[i] = (float)0.0;
	}

}

int Fs100Cmd::getLength(char* data)
{
    /*
    returns length of provided serialized Simplemsg.
    */
    int length = *(int*) data;
    return length;
}

void Fs100Cmd::motionReady(SimpleMsg *msg)
{
    /*
    Create and return motion check ready Simplemsg.
    */
    //prefix
    msg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoMotionCtrl);
    //header
    msg->header.msgType = ROS_MSG_MOTO_MOTION_CTRL;
    msg->header.commType = ROS_COMM_SERVICE_RESQUEST;
    msg->header.replyType = ROS_REPLY_INVALID;
    //body
    msg->body.motionCtrl.groupNo = 0;
    msg->body.motionCtrl.sequence = 0;
    msg->body.motionCtrl.command = ROS_CMD_CHECK_MOTION_READY;
    for(int i=0;i<10;i++)
    {
        msg->body.motionCtrl.data[i] = (float) 0.0;
    }
}

void Fs100Cmd::trajectoryStart(SimpleMsg *msg)
{
    /*
    Create and return trajectory start Simplemsg.
    */
    //prefix
    msg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoMotionCtrl);
    //header
    msg->header.msgType = ROS_MSG_MOTO_MOTION_CTRL;
    msg->header.commType = ROS_COMM_SERVICE_RESQUEST;
    msg->header.replyType = ROS_REPLY_INVALID;
    //body
    msg->body.motionCtrl.groupNo = 0;
    msg->body.motionCtrl.sequence = 0;
    msg->body.motionCtrl.command = ROS_CMD_START_TRAJ_MODE;
    for(int i=0;i<10;i++)
    {
        msg->body.motionCtrl.data[i] = (float) 0.0;
    }
}

void Fs100Cmd::trajectoryStop(SimpleMsg *msg)
{
    /*
    Create and retyrn trajectory stop Simplemsg.
    */
    //prefix
    msg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoMotionCtrl);
    //header
    msg->header.msgType = ROS_MSG_MOTO_MOTION_CTRL;
    msg->header.commType = ROS_COMM_SERVICE_RESQUEST;
    msg->header.replyType = ROS_REPLY_INVALID;
    //body
    msg->body.motionCtrl.groupNo = 0;
    msg->body.motionCtrl.sequence = 0;
    msg->body.motionCtrl.command = ROS_CMD_STOP_TRAJ_MODE;
    for(int i=0;i<10;i++)
    {
        msg->body.motionCtrl.data[i] = (float) 0.0;
    }
}

void Fs100Cmd::pgmClose()
{
    /*
    Send trajectory stop message, closes socket connection and joins threads.
    */
    stop_all = true;
    if (run_thread == 1)
    {
        run_thread = 0;
        pthread_join(cmdthreadID,NULL);
    }
    SimpleMsg traj_stop;
    char motion_buf[68];
    char motion_reply_buffer[76];
    trajectoryStop(&traj_stop);
    serialize(&traj_stop,motion_buf,sizeof(motion_buf));
    byteSwap(motion_buf,sizeof(motion_buf));
    n = write(sockfd,motion_buf,sizeof(motion_buf));
    if (n < 0) 
         socketError("ERROR writing trajectory stop to socket");

        n = recv(sockfd,motion_reply_buffer,sizeof(motion_reply_buffer),0);
        byteSwap(motion_reply_buffer,sizeof(motion_reply_buffer));
        deserializeMotionReply(motion_reply_buffer,&motion_rpl);
    printf("\n\ntrajectory stop sent, reply:\n");
    printErrorCodeMsg(&motion_rpl);
    
    close(sockfd);
}
