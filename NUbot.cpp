/*! @file NUbot.cpp
    @brief Implementation of top-level NUbot class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <time.h>
#include <errno.h>
using namespace std;

#include "NUbot.h"
#ifdef TARGET_IS_NAOWEBOTS
    #include "NUPlatform/NAOWebots/NAOWebots.h"
#endif
#ifdef TARGET_IS_NAO
    #include "NUPlatform/NAO/NAO.h"
#endif
#ifdef TARGET_IS_CYCLOID
    #include "NUPlatform/Cycloid/Cycloid.h"
#endif

// It is difficult to make threads members of classes, so just keep them close by
static void* runThreadMotion(void* arg);
static void* runThreadVision(void* arg);

NUbot::NUbot(int argc, const char *argv[])
{
    // Construct the right Platform
    #ifdef TARGET_IS_NAOWEBOTS
        platform = new NAOWebots(argc, argv);
    #endif
    #ifdef TARGET_IS_NAO
        platform = new NAO();
    #endif
    #ifdef TARGET_IS_CYCLOID
        platform = new Cycloid();
    #endif
    
    // Construct each enabled module 
    #ifdef USE_VISION
        vision = new Vision();
    #endif
    #ifdef USE_LOCALISATION
        localisation = new Localisation();
    #endif
    #ifdef USE_BEHAVIOUR
        behaviour = new Behaviour();
    #endif
    #ifdef USE_MOTION
        motion = new NUMotion();
    #endif
    #ifdef USE_NETWORK
        network = new Network();
    #endif
    
    createThreads();
}

/*! @brief Create nubot's threads
 
    There is a single real-time thread in which motion runs. A semaphore is set by the body driver which triggers
    the execution of this thread. The fresh sensors data is processed by motion, and commands are sent to the actionators.
 
    There is a single non-real-time thread in which vision, localisation and behaviour run. A semaphore is set by the camera
    driver which triggers the execution of this thread. The new image is processed, and computed actions are sent to motion
    and lcs.
 */

void NUbot::createThreads()
{
    // create threadMotion and its trigger condMotionData
    int err;
    err = pthread_mutex_init(&mutexMotionData, NULL);
    if (err != 0)
        cout << "NUbot::createThreads(). Failed to initialise mutexMotionData errno: " << errno << endl;
    
    err = pthread_cond_init(&condMotionData, NULL);
    if (err != 0)
        cout << "NUbot::createThreads(). Failed to initialise condMotionData errno: " << errno << endl;

#if THREAD_MOTION_PRIORITY > 0
    err = pthread_create(&threadMotion, NULL, runThreadMotion, (void*) this);         // The last parameter is the arguement to the thread
    if (err != 0)
        cout << "NUbot::createThreads(). Failed to create threadMotion! The error code was: " << err << endl;
    
    sched_param param;
    param.sched_priority = THREAD_MOTION_PRIORITY;
    pthread_setschedparam(threadMotion, SCHED_FIFO, &param);            // Note. This will fail (quietly) if the underlying OS doesn't allow real-time
    
    // double check
    int actualpolicy;
    sched_param actualparam;
    pthread_getschedparam(threadMotion, &actualpolicy, &actualparam);
    cout << "NUbot::createThreads(). threadMotion Policy: " << actualpolicy << " Priority: " << actualparam.sched_priority << endl;
#else   
    cout << "NUbot::createThreads(). Warning. Creating threadMotion as non-realtime" << endl;
    int err = pthread_create(&threadMotion, NULL, runThreadMotion, (void*) this);
    if (err != 0)
        cout << "NUbot::createThreads(). Failed to create threadMotion! The error code was: " << err << endl;
#endif

    // create threadVision and its trigger condVisionData
    err = pthread_mutex_init(&mutexVisionData, NULL);
    if (err != 0)
        cout << "NUbot::createThreads(). Failed to initialise mutexVisionData errno: " << errno << endl;
    
    err = pthread_cond_init(&condVisionData, NULL);
    if (err != 0)
        cout << "NUbot::createThreads(). Failed to initialise condVisionData errno: " << errno << endl;
    
#if THREAD_VISION_PRIORITY > 0
    cout << "NUbot::createThreads(). Warning. Creating threadVision as realtime" << endl;
    err = pthread_create(&threadVision, NULL, runThreadVision, (void*) this);         // The last parameter is the arguement to the thread
    
    sched_param param;
    param.sched_priority = THREAD_VISION_PRIORITY;
    pthread_setschedparam(threadVision, SCHED_FIFO, &param);            // Note. This will fail (quietly) if the underlying OS doesn't allow real-time
    
    // double check
    int actualpolicy;
    sched_param actualparam;
    pthread_getschedparam(threadVision, &actualpolicy, &actualparam);
    cout << "NUbot::createThreads(). threadVision Policy: " << actualpolicy << " Priority: " << actualparam.sched_priority << endl;
#else   
    err = pthread_create(&threadVision, NULL, runThreadVision, (void*) this);
#endif
    if (err != 0)
        cout << "NUbot::createThreads(). Failed to create threadVision! The error code was: " << err << endl;
}

NUbot::~NUbot()
{
    delete platform;
    #ifdef USE_VISION
        delete vision;
    #endif
    #ifdef USE_LOCALISATION
        delete localisation;
    #endif
    #ifdef USE_BEHAVIOUR
        delete behaviour;
    #endif
    #ifdef USE_MOTION
        delete motion;
    #endif
    #ifdef USE_NETWORK
        delete network;
    #endif
}

/*! @brief Brief description
    
 Detailed description
 */
void NUbot::run()
{
    // do something smart here!??!
    int count = 0;
    while (true)
    {
        cout << "NUbot::run()" << endl;
        signalMotion();
        if (count%10 == 0)
            signalVision();
        count++;
        usleep(0.1*1e6);
    };
    return;
}

int NUbot::signalMotion()
{
    int err = 0;
    pthread_mutex_lock(&mutexMotionData);
    pthread_cond_signal(&condMotionData);
    pthread_mutex_unlock(&mutexMotionData);
    return err;
}

int NUbot::waitForNewMotionData()
{
    int err = 0;
    pthread_mutex_lock(&mutexMotionData);
    err = pthread_cond_wait(&condMotionData, &mutexMotionData);
    pthread_mutex_unlock(&mutexMotionData);
    return err;
}

int NUbot::signalVision()
{
    int err = 0;
    pthread_mutex_lock(&mutexVisionData);
    pthread_cond_signal(&condVisionData);
    pthread_mutex_unlock(&mutexVisionData);
    return err;
}

int NUbot::waitForNewVisionData()
{
    int err = 0;
    pthread_mutex_lock(&mutexVisionData);
    err = pthread_cond_wait(&condVisionData, &mutexVisionData);
    pthread_mutex_unlock(&mutexVisionData);
    return err;
}

void* runThreadMotion(void* arg)
{
    cout << "NUbot::runThreadMotion: Starting." << endl;
    
    NUbot* nubot = (NUbot*) arg;
    
#ifdef THREAD_MOTION_MONITOR_TIME
    struct timespec pretime, starttime, endtime;
    struct timespec relstarttime, relendtime;
    struct timespec prostarttime, proendtime;
    float runtime, waittime, relruntime, proruntime;       // the run time in ms
#endif
    
    int err;
    do 
    {
#ifdef THREAD_MOTION_MONITOR_TIME
        clock_gettime(CLOCK_REALTIME, &pretime);
#endif
        err = nubot->waitForNewMotionData();
        cout << "NUbot::runThreadMotion. Running" << endl;

#ifdef THREAD_MOTION_MONITOR_TIME
        clock_gettime(CLOCK_REALTIME, &starttime);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &relstarttime);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &prostarttime);
        waittime = (starttime.tv_nsec - pretime.tv_nsec)/1e6 + (starttime.tv_sec - pretime.tv_sec)*1e3;
        if (waittime > 25)
            cout << "JWALKTHREAD: Waittime " << waittime << " ms."<< endl;
#endif
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------

        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef THREAD_MOTION_MONITOR_TIME
        clock_gettime(CLOCK_REALTIME, &endtime);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &relendtime);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &proendtime);
        runtime = (endtime.tv_nsec - starttime.tv_nsec)/1e6 + (endtime.tv_sec - starttime.tv_sec)*1e3;
        relruntime = (relendtime.tv_nsec - relstarttime.tv_nsec)/1e6 + (relendtime.tv_sec - relstarttime.tv_sec)*1e3;
        proruntime = (proendtime.tv_nsec - prostarttime.tv_nsec)/1e6 + (proendtime.tv_sec - prostarttime.tv_sec)*1e3;
        if (runtime > 8)
        {
            cout << "JWALKTHREAD: Jason cycle time error: " << runtime << " ms. Time spent in this thread: " << relruntime << "ms, in this process: " << proruntime << endl;
        }
#endif
    } 
    while (err == 0 | errno != EINTR);
    cout << "NUbot::runThreadMotion. This thread was interrupted by a signal, it will now exit" << endl;
    pthread_exit(NULL);
}

void* runThreadVision(void* arg)
{
    cout << "NUbot::runThreadVision: Starting." << endl;
    
    NUbot* nubot = (NUbot*) arg;
    
#ifdef THREAD_VISION_MONITOR_TIME
    struct timespec pretime, starttime, endtime;
    struct timespec relstarttime, relendtime;
    struct timespec prostarttime, proendtime;
    float runtime, waittime, relruntime, proruntime;       // the run time in ms
#endif
    
    int err;
    do 
    {
#ifdef THREAD_VISION_MONITOR_TIME
        clock_gettime(CLOCK_REALTIME, &pretime);
#endif
        err = nubot->waitForNewVisionData();
        cout << "NUbot::runVisionMotion. Running" << endl;
        
#ifdef THREAD_VISION_MONITOR_TIME
        clock_gettime(CLOCK_REALTIME, &starttime);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &relstarttime);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &prostarttime);
        waittime = (starttime.tv_nsec - pretime.tv_nsec)/1e6 + (starttime.tv_sec - pretime.tv_sec)*1e3;
        if (waittime > 25)
            cout << "JWALKTHREAD: Waittime " << waittime << " ms."<< endl;
#endif
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
        
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef THREAD_VISION_MONITOR_TIME
        clock_gettime(CLOCK_REALTIME, &endtime);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &relendtime);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &proendtime);
        runtime = (endtime.tv_nsec - starttime.tv_nsec)/1e6 + (endtime.tv_sec - starttime.tv_sec)*1e3;
        relruntime = (relendtime.tv_nsec - relstarttime.tv_nsec)/1e6 + (relendtime.tv_sec - relstarttime.tv_sec)*1e3;
        proruntime = (proendtime.tv_nsec - prostarttime.tv_nsec)/1e6 + (proendtime.tv_sec - prostarttime.tv_sec)*1e3;
        if (runtime > 8)
        {
            cout << "JWALKTHREAD: Jason cycle time error: " << runtime << " ms. Time spent in this thread: " << relruntime << "ms, in this process: " << proruntime << endl;
        }
#endif
    } 
    while (err == 0 | errno != EINTR);
    cout << "NUbot::runThreadMotion. This thread was interrupted by a signal, it will now exit" << endl;
    pthread_exit(NULL);
}



// NUbot threads:
// You can't assume that these threads are syncronised, or running at any particular relative rate 
// I need a thread which links BodySensors->NUMotion in a real-time thread.
//      while (true):
//          wait for new data
//          data = nubot->platform->sensors->getData()                // I should not deep copy the data here
//                cmds = nubot->motion->process(data)                       // it is up to motion to decide whether it should deep copy
//        nubot->platform->actionators->process(cmds)
//
// I need a thread which links Image->Vision->Localisation->Behaviour->Motion in a non-real-time thread.
//      while (true):
//          actions.clear()
//          wait for new image
//          image = nubot->platform->camera->getData()
//          data = nubot->platform->sensors->getData()                // I should not deep copy the data here
//                 odometry = nubot->motion->getData()                       // There is no deep copy here either
//      gamectrl, teaminfo = nubot->network->getData()
//      
//          fieldobj = nubot->vision->process(image, data, gamectrl)
//          wm = nubot->localisation->process(fieldobj, teaminfo, odometry, gamectrl, actions)
//                  nubot->behaviour->process(wm, gamectrl, actions)
//          
//          nubot->actionFilter(actions, motionactions, lcsactions)     // I think this is messy, perhaps no actionfilter.
//
//          nubot->motion->process(motionactions)                // I don't think anything needs to be returned here
//          cmds = nubot->lcs->process(lcsactions)
//          nubot->platform->actionators->process(cmds)

// so each thread is passed the nubot, and does it that way