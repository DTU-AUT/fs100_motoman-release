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
#ifndef FS100CMD_H
#define FS100CMD_H


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> //for time
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netdb.h> 
#include <queue>
#include <pthread.h>

#include "motoman_direct_message.h"

typedef struct{
        float pos[6];
        float vel[6];
        float time;
}cmd;

class Fs100Cmd{
private:
    //private variables and functions
    const char* IP;
    int sockfd, portno, n, i,run_thread;
    volatile int seq;
    unsigned int thread_sleep;
    float abs_time;
    volatile bool stop_all;

    
    
    struct sockaddr_in serv_addr;
    struct hostent *server;
    pthread_t cmdthreadID;
    SimpleMsg joint_data,motion_rpl; //deserialized data in human readable format
    char raw_data[255]; 
    char traj_buffer[152];
    char motion_reply_buffer[76];
    static void *cmdPushThread(void *This);
    std::queue<cmd> cmdList;
    
    
    void cmdPush();
    void socketError(const char *msg);
    void byteSwap(char* data,int length);
    void serialize(SimpleMsg *msg, char *data,int size);
    void deserializeMotionReply(char *data,SimpleMsg *msg);
    int buildTrajPos(SimpleMsg *tm,float pos[6],float time,int seq);
    int buildTrajFull(SimpleMsg *tm,float pos[6],float vel[6],float time,int seq);
    void motionReady(SimpleMsg *msg);
    void trajectoryStart(SimpleMsg *msg);
    void trajectoryStop(SimpleMsg *msg);
    void printErrorCodeMsg(SimpleMsg *msg);
    void printMotionReply(SimpleMsg *msg);
    void printTrajFull(SimpleMsg *msg);
    int getLength(char* data);
    
    

public:
    //public variables and functions
    Fs100Cmd(const char* ip){
    IP = ip;
    portno = 50240;
    i = 1;
    pthread_mutex_t mut_lock = PTHREAD_MUTEX_INITIALIZER;
    }
    int init();
    int makeConnect();
    int start(int retry);

    
    
    bool resetTrajectory(int *retry);
    bool pushTraj(float pos[6],float vel[6],float time,int seq);
    bool addCmdToQueue(cmd cmd_point);
    bool addPointToQueue(float pos[6],float vel[6],float time);
    void pgmClose();
};



#endif //FS100CMD
