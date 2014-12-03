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

#ifndef FS100STATE_H
#define FS100STATE_H


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netdb.h> 
#include <pthread.h>


#include "motoman_direct_message.h"



class Fs100State
{
private:
    //private variables and functions
    
    const char* IP;
    int sockfd, portno, n, i,run_thread;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    pthread_t recvthreadID;
    SimpleMsg joint_data; //deserialized data in human readable format
    bool joint_values_updated;
    char raw_data[148]; 
    
    pthread_mutex_t mut_lock;
    pthread_cond_t mut_cond;
    float current_pos[6];
    bool pos_updated;

    static void *recvDataThread(void *This);
    void recvData();
    void socketError(const char *msg);
    void byteSwap(char* data,int length);
    void serialize(SimpleMsg *msg, char *data,int size);
    void deserializeJointFeedback(char *data,SimpleMsg *msg);
    void deserializeMotionReply(char *data,SimpleMsg *msg);
    void printMotionReply(SimpleMsg *msg);
    void printJointFeedback();
    int getLength(char* data);
    
    

public:
    //public variables and functions
    Fs100State(const char* ip){
    IP = ip;
    portno = 50241;
    i = 1;
    pthread_mutex_t mut_lock = PTHREAD_MUTEX_INITIALIZER;
    }
    int init();
    int makeConnect();
    void pgmClose();
    
    
    bool getJointsUpdated(float* joints);
};



#endif //FS100STATE
