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


//! FS100 robot state class.
/*!
 The FS100 robot state class handles connection with port 50241, which the FS100 uses to send robot joint position data.
*/
class Fs100State
{
private:
    //private variables and functions
    
    const char* IP; /*!< Variable storing ip address of target controller. */
    int sockfd, portno, n, i,run_thread; /*!< Variable for sockect connection. */
    struct sockaddr_in serv_addr; /*!< Struct for socket connection. */
    struct hostent *server; /*!< Struct for socket connection. */
    pthread_t recvthreadID; /*!< Thread ID for data recieving thread.*/
    SimpleMsg joint_data; /*!< Simple message containing joint data. */
    bool joint_values_updated; /*!< Has joint data been updated? */
    char raw_data[148];  /*!< Raw buffer data. */
    
    pthread_mutex_t mut_lock; /*!< Mutex lock. */
    pthread_cond_t mut_cond; /*!< Mutex condition. */
    float current_pos[6]; /*!< Current robot joint position. */
    bool pos_updated; /*!< Is current robot joint position updated? */

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
    //! Constructor.
    /*!
     Constructor for FS100 state class.
    */
    Fs100State(const char* ip){
    IP = ip;
    portno = 50241;
    i = 1;
    pthread_mutex_t mut_lock = PTHREAD_MUTEX_INITIALIZER;
    }
    //! Initiator.
    /*!
     Initiates the sockect connection to the FS100 controller, and initializes internal variables.
    */
    int init();
    
    //! Creates connection.
    /*!
     Establishes socket connection to the FS100 controller, and starts a thread for recieving data over socket.
    */
    int makeConnect();
    
    //! Closes the class.
    /*!
     Closes socket connection and joins all running threads.
    */
    void pgmClose();
    
    //! Get updated joint data.
    /*!
     A function that returns the next updated joint data. Locks on call until new data is available.
     \param joints is a pointer to a double[6] type. Used for returning data.
    */
    bool getJointsUpdated(float* joints);
};



#endif //FS100STATE
