/**
   @file CalibrationCore.h
   @brief Server side logic calibration (main)
   @author Manuel Hervas
   @date 08/2014
*/

#ifndef SERVER_CALIB_CORE_H_
#define SERVER_CALIB_CORE_H_

// OpenCV library
#include <opencv2/opencv.hpp>
// Calibration stuff
#include "CameraProjectorCalibration.hpp"
// Singleton
#include "Singleton.h"

namespace argosServer {
  /**
   * Server side main logic
   */
  class CalibrationCore: public Singleton<CalibrationCore> {
    
  public:
    /**
     * Default Constructor
     */
    CalibrationCore();
    
    /**
     * Default Destructor
     */
    virtual ~CalibrationCore();
    
  protected:
    
    // calibration status
    enum CalibState {CAMERA, PROJECTOR_STATIC, PROJECTOR_DYNAMIC, DEMO_AR};
    CalibState currentState;
    
    // Extrinsics parameters CAMERA-PROJECTOR
    CameraProjectorCalibration camProjCalib;
    
    cv::Mat cameraFrame;        // Camera input
    cv::Mat projectorFrame;      //Projector output
    cv::Mat debugFrame;          //debug Frame
    cv::Mat processedImg;
    cv::Mat snapShot;
    
    bool bProjectorRefreshLock;
    // Threshold parameters
    int circleDetectionThreshold;
    
    // Application
    float diffMinBetweenFrames;
    float timeMinBetweenCaptures;
    string currStateString;
    
    //Boards parameters
    int numBoardsFinalCamera;
    int numBoardsFinalProjector;
    int numBoardsBeforeCleaning;
    int numBoardsBeforeDynamicProjection;
    float maxReprojErrorCamera;
    float maxReprojErrorProjectorStatic;
    float maxReprojErrorProjectorDynamic;
    
    // board holding movement
    
    cv::Mat previous;
    cv::Mat diff;
    float diffMean;
    double lastTime;
    
    //---------------------------------------------------------------------------------
    void update();
    void draw();
    void initializeCamera(int dev);
    void setState(CalibState state);
    string getCurrentStateString();
    bool calibrateCamera(const cv::Mat& cameraFrame);
    bool calibrateProjector(const cv::Mat& cameraFrame);
    void processImageForCircleDetection(const cv::Mat& cameraFrame);
    bool timeMinBetweenFrames();
    void drawCameraImagePoints();
    void drawProjectorPattern();
    void drawProjectorFinePattern();
    void toGreyscale(const cv::Mat& inputFrame, cv::Mat& cameraFrame);
    bool calculateDemo(cv::Mat& cameraFrame);
    //--------------------------------------------------------------------------------
    
  };
}

#endif









  
