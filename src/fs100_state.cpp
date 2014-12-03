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
#include "fs100_state.h"




int Fs100State::init()
{
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = setsockopt( sockfd, IPPROTO_TCP, TCP_NODELAY, (void *)&i, sizeof(i));
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
    
    pthread_mutex_init(&mut_lock, NULL);
    pos_updated = false;
    printf("robot state init done\n");
    return 0;
}

int Fs100State::makeConnect()
    {
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    {
        socketError("ERROR connecting");
        return 1;
    }
    run_thread = 1;
    pthread_create(&recvthreadID,NULL,recvDataThread,this);
    return 0;
}

void *Fs100State::recvDataThread(void *This)
{
    ((Fs100State *) This)->recvData();
}

void Fs100State::recvData()
{
    //sensor_msgs::JointState js;
    //js.position.resize(6);
    while(run_thread)
    {
        ///// Do your recieving and stuff here, in this thread...right here.
        n = recv(sockfd,raw_data,sizeof(raw_data),0); //expect size? size is 255, eating both messages perhabs?
        if (n < 0) 
            socketError("ERROR reading from socket");
        setsockopt( sockfd, IPPROTO_TCP, TCP_QUICKACK, (void *)&i, sizeof(i)); //force acknowlegde
        byteSwap(raw_data,sizeof(raw_data)); //big-endian to litlle-endian
        //Checking data type (length 40=status; length 144=joint feedback)
        if (getLength(raw_data)==44)
        {
            printf("got status\n");
        }
        else if(getLength(raw_data)==144)
        {
            //semaphore/mutex here!
            pthread_mutex_lock(&mut_lock);                   
            deserializeJointFeedback(raw_data,&joint_data);
            //printf("pos[0]: %f\n",joint_data.body.jointFeedback.pos[0]);
            pthread_mutex_unlock(&mut_lock);
            for(int i=0;i<6;i++)
            {
                //js.position[i] = joint_data.body.jointFeedback.pos[i];
                current_pos[i] = joint_data.body.jointFeedback.pos[i];
                pos_updated = true;
            }
            //js.header.stamp = ros::Time::now();
            //pub_state->publish(js);
            //release semaphore
        }
    }

}

void Fs100State::pgmClose()
{
    run_thread = 0;
    pthread_join(recvthreadID,NULL);
    pthread_mutex_destroy(&mut_lock);
    close(sockfd);
}

void Fs100State::socketError(const char *msg)
{
    perror(msg);
    return;
}

void Fs100State::byteSwap(char* data,int length) //destructive
{ 
    char buffer[length];
    memcpy(buffer,data,length);
    for(int i=0;i<length/4;i++) //number of int/float
    { 
        for(int k = 0;k<4;k++) //for every byte in int/float
        { 
            data[(i*4)+k] = buffer[(i*4)+(3-k)];
            
        }
    }
}

void Fs100State::serialize(SimpleMsg *msg, char *data,int size)
{
    memcpy(data,msg,size);
}

void Fs100State::deserializeJointFeedback(char *data,SimpleMsg *msg)
{
    int *q = (int*)data;
    //prefix
    msg->prefix.length =  *q; q++;
    //header
    msg->header.msgType = (SmMsgType) *q; q++;
    msg->header.commType =(SmCommType) *q; q++;
    msg->header.replyType = (SmReplyType) *q; q++;
    //body
    msg->body.jointFeedback.groupNo = *q; q++;
    msg->body.jointFeedback.validFields = *q; q++;
    //change from integer to float
    msg->body.jointFeedback.time = *(float*)q; q++;
    for(int i=0;i<10;i++)
    {
        msg->body.jointFeedback.pos[i] = *(float*)q; q++;
    }
    for(int i=0;i<10;i++)
    {
        msg->body.jointFeedback.vel[i] = *(float*)q; q++;
    }
    for(int i=0;i<10;i++)
    {
        msg->body.jointFeedback.acc[i] =  *(float*)q; q++;
    }
    
}

void Fs100State::deserializeMotionReply(char *data,SimpleMsg *msg)
{
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
    for(int i=0;i<10;i++)
    {
        msg->body.motionReply.data[i] = *(float*)q; q++;
    }    
}

void Fs100State::printMotionReply(SimpleMsg *msg)
{
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
    printf("    subcide: %d\n",msg->body.motionReply.subcode);
    printf("    data:\n");
    for(int i=0;i<10;i++)
    {
        printf("      data[%d]: %f\n",i,msg->body.motionReply.data[i]);
    }
}
    
void Fs100State::printJointFeedback()
{
    printf("Prefix:\n");
    printf("  length: %d\n",joint_data.prefix.length);
    printf("header:\n");
    printf("  msgType: %d\n",joint_data.header.msgType);
    printf("  commType: %d\n",joint_data.header.commType);
    printf("  replyType: %d\n",joint_data.header.replyType);
    printf("body:\n");
    printf("  jointFeedback:\n");
    printf("    groupNo: %d\n",joint_data.body.jointFeedback.groupNo);
    printf("    validFields: %d\n",joint_data.body.jointFeedback.validFields);
    printf("    time: %f\n",joint_data.body.jointFeedback.time);
    printf("    pos:\n");
    for(int i=0;i<10;i++)
    {
        printf("      joint[%d]: %f\n",i+1,joint_data.body.jointFeedback.pos[i]);
    }
}

int Fs100State::getLength(char* data)
{
    int length = *(int*) data;
    return length;
}

bool Fs100State::getJointsUpdated(float* joints)
{
    
    if (!pos_updated)
    {
        return false;
    }
    //wait for semaphore
    //pthread_mutex_lock(&this->mut_lock);
    pthread_mutex_lock(&mut_lock);
    if (pos_updated)
    {
        for(int i=0;i<6;i++)
        {
            joints[i] = joint_data.body.jointFeedback.pos[i];
        }
        pos_updated = false;
        pthread_mutex_unlock(&mut_lock);
        return true;
    }

    pthread_mutex_unlock(&mut_lock);
    //release semaphore
    //pthread_mutex_unlock(&this->mut_lock);
}


