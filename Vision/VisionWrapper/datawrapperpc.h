#ifndef DATAWRAPPERPC_H
#define DATAWRAPPERPC_H

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Kinematics/Horizon.h"

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/segmentedregion.h"
#include "Vision/VisionTools/pccamera.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"

#define GROUP_NAME "/home/shannon/Images/paper"
#define GROUP_EXT ".png"

using namespace std;
//using namespace cv;
using cv::Mat;
using cv::VideoCapture;
using cv::Scalar;
using cv::namedWindow;
using cv::Vec3b;

class DataWrapper
{
    friend class VisionController;
    friend class VisionControlWrapper;

public:
    
    enum DEBUG_ID {
        DBID_IMAGE              = 0,
        DBID_CLASSED_IMAGE       = 1,
        DBID_H_SCANS            = 2,
        DBID_V_SCANS            = 3,
        DBID_SEGMENTS           = 4,
        DBID_MATCHED_SEGMENTS   = 5,
        DBID_HORIZON            = 6,
        DBID_GREENHORIZON_SCANS = 7,
        DBID_GREENHORIZON_FINAL = 8,
        DBID_OBJECT_POINTS      = 9,
        DBID_FILTERED_SEGMENTS  = 10,
        DBID_GOALS              = 11,
        DBID_BEACONS            = 12,
        DBID_BALLS              = 13,
        DBID_OBSTACLES          = 14,
        DBID_LINES              = 15,
        NUMBER_OF_IDS           = 16
    };

    static string getIDName(DEBUG_ID id);

    static DataWrapper* getInstance();

    //! RETRIEVAL METHODS
    NUImage* getFrame();

    bool getCTGVector(vector<float>& ctgvector);    //for transforms
    bool getCTVector(vector<float>& ctvector);    //for transforms
    bool getCameraHeight(float& height);            //for transforms
    bool getCameraPitch(float& pitch);              //for transforms
    bool getBodyPitch(float& pitch);
    
    //! @brief Generates spoofed horizon line.
    const Horizon& getKinematicsHorizon();

    CameraSettings getCameraSettings();

    const LookUpTable& getLUT() const;
        
    //! PUBLISH METHODS
    void publish(const vector<const VisionFieldObject*> &visual_objects);
    void publish(const VisionFieldObject* visual_object);

    void debugRefresh();
    bool debugPublish(const vector<Ball>& data);
    bool debugPublish(const vector<Beacon>& data);
    bool debugPublish(const vector<Goal>& data);
    bool debugPublish(const vector<Obstacle>& data);
    bool debugPublish(const vector<FieldLine>& data);
    bool debugPublish(DEBUG_ID id, const vector<PointType>& data_points);
    bool debugPublish(DEBUG_ID id, const SegmentedRegion& region);
    bool debugPublish(DEBUG_ID id);
    bool debugPublish(DEBUG_ID id, const NUImage *const img);
    
    
private:
    DataWrapper();
    ~DataWrapper();
    //void startImageFileGroup(string filename);
    bool updateFrame();
    bool loadLUTFromFile(const string& fileName);
    int getNumFramesDropped() const {return numFramesDropped;}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return numFramesProcessed;}  //! @brief Returns the number of processed frames since start.
    
    void ycrcb2ycbcr(Mat* img_ycrcb);
    void generateImageFromMat(Mat& frame);

private:
    enum INPUT_METHOD {
        CAMERA,
        STREAM
    };
    
private:
    static const INPUT_METHOD METHOD = STREAM;  //CAMERA, STREAM, FILE

    static DataWrapper* instance;

    NUImage* m_current_image;

    string configname;

    string LUTname;
    LookUpTable LUT;

    Horizon kinematics_horizon;

    PCCamera* m_camera;          //! Used when streaming from camera

    //! Used when reading from strm
    string streamname;
    ifstream imagestrm;

    //! Used when reading from file
    unsigned char* m_yuyv_buffer;
    VideoCapture* capture;
    Mat m_current_image_cv;
    int num_images, cur_image;

    //! Used for debugging
    int debug_window_num;
    map<DEBUG_ID, vector<pair<string, Mat>* > > debug_map;
    pair<string, Mat>* debug_windows;

//    int id_window_map[NUMBER_OF_IDS];
//
//    string* debug_window_name;  //for array
//    Mat* debug_img;             //for array

    //! Used for displaying results
    string results_window_name;
    Mat results_img;
    
    //! Frame info
    double m_timestamp;
    int numFramesDropped;
    int numFramesProcessed;

    
    //NUSensorsData* m_sensor_data;

//! ONLY FOR DEBUGGING - DO NOT RELY ON THE FOLLOWING METHODS OR VARIABLES BEING PRESENT
public:
    void show(Mat* img);
};

#endif // DATAWRAPPERPC_H