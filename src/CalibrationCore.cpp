#include "Core.h"
#include "Log.h"
#include "HandDetector.h"
#include "ScriptManager.h"
#include "ConfigManager.h"

namespace argosServer{

  CalibrationCore::CalibrationCore(){
    //- Read configuration file ----
    //Log::info("Reading configuration file... ");
    //ConfigManager::loadConfiguration("data/config.xml");

    bProjectorRefreshLock = true;

    // Threshold parameters
    circleDetectionThreshold = 70;

    // Application
    diffMinBetweenFrames = 4.0;    // maximum amount of movement between successive frames (must be smaller in order to add a board)
    timeMinBetweenCaptures = 2.0;  // minimum time between snapshots (seconds)
  
    //Boards parameters
    numBoardsFinalCamera = 20;
    numBoardsFinalProjector = 20;
    numBoardsBeforeCleaning = 3;
    numBoardsBeforeDynamicProjection = 5;
    maxReprojErrorCamera = 0.20;
    maxReprojErrorProjectorStatic = 0.25;
    maxReprojErrorProjectorDynamic = 0.43;

    projectorFrame.create(600, 800, CV_8UC1);
 
    setState(CAMERA);
    //setState(PROJECTOR_STATIC);
    //setState(DEMO_AR); 
  
  }

  CalibrationCore::~CalibrationCore(){}
  
  void CalibrationCore::update(cv::Mat& cameraFrame){
    
    try{
      projectorFrame = cv::Scalar::all(0);
      //cameraFrame.copyTo(debugFrame);
      
      switch (currentState) {
      case CAMERA:
	if(!timeMinBetweenFrames()) return;
	calibrateCamera(cameraFrame);
	lastTime = cv::getTickCount();
	break;
      
      case PROJECTOR_STATIC:
	if(!timeMinBetweenFrames()) return;
	calibrateProjector(cameraFrame);
	lastTime = cv::getTickCount();
	break;
      
      case PROJECTOR_DYNAMIC:
	if(bProjectorRefreshLock){
	  if(camProjCalib.setDynamicProjectorImagePoints(cameraFrame)){
	    Log::info("Updating projection points ...");
	    if(!timeMinBetweenFrames()) return;
	    bProjectorRefreshLock = false;
	  }
	}
	else{
	  calibrateProjector(cameraFrame);
	  bProjectorRefreshLock = true;
	  lastTime = cv::getTickCount();
	}
	break;      
      case DEMO_AR:
	calculateDemo(cameraFrame);
	break;
      default: break;
      }
    }catch (cv::Exception e){
      cout << endl << "Exception caught: " << e.msg << endl;
    }  
  }

  /**
   * 
   * DRAW
   *
   */
  cv::Mat& CalibrationCore::draw(){
 
    switch (currentState) {
    case CAMERA:
      break;
    case PROJECTOR_STATIC:
    case PROJECTOR_DYNAMIC:
      drawProjectorPattern();
      break;
    case DEMO_AR:
      drawProjectorFinePattern();
    default:
      break;
    }
    
    return projectorFrame;
  }


  bool CalibrationCore::calibrateCamera(const cv::Mat& cameraFrame){
  
    CameraCalibration& calibrationCamera = camProjCalib.getCalibrationCamera();
  
    bool foundPattern = calibrationCamera.add(cameraFrame);
  
    if(foundPattern){
      drawCameraImagePoints();
      //cv::imshow(snapShotWindow, snapShot); 
    
      calibrationCamera.calibrate();
    
      if(calibrationCamera.size() >= numBoardsBeforeCleaning) {
	calibrationCamera.clean(maxReprojErrorCamera);
      
	if(calibrationCamera.getReprojectionError(calibrationCamera.size()-1) > maxReprojErrorCamera) {
	  Log::info("Board found, but reproj. error is too high, skipping");
	  return false;
	}
      }
    
      if (calibrationCamera.size() >= numBoardsFinalCamera){
	calibrationCamera.save("calibrationCamera.xml");
	Log::info("Camera calibration finished & saved to calibrationCamera.xml");
	setState(PROJECTOR_STATIC);
      }
    } else Log::error("Board not find");
  
    return foundPattern;
  }


  bool CalibrationCore::calibrateProjector(const cv::Mat& cameraFrame){
  
    //CameraCalibration& calibrationCamera = camProjCalib.getCalibrationCamera();
    ProjectorCalibration& calibrationProjector = camProjCalib.getCalibrationProjector();
  
    processImageForCircleDetection(cameraFrame);
  
    if(camProjCalib.addProjected(cameraFrame, processedImg)){
   
      //waitKey(1);
      Log::info("Calibrating projector");
      
      calibrationProjector.calibrate();
    
      if(calibrationProjector.size() >= numBoardsBeforeCleaning) {
      	Log::info("Cleaning");
	int numBoardRemoved;
	if(currState == PROJECTOR_STATIC)
	  numBoardRemoved = camProjCalib.cleanStereo(maxReprojErrorProjectorStatic);
	else	
	  numBoardRemoved = camProjCalib.cleanStereo(maxReprojErrorProjectorDynamic);
	
	Log::info(std::to_string(numBoardRemoved) + " boards removed");
	
	if(currState == PROJECTOR_DYNAMIC && calibrationProjector.size() < numBoardsBeforeDynamicProjection) {
	  Log::info("Too many boards removed, restarting to PROJECTOR_STATIC");
	  setState(PROJECTOR_STATIC);
	  return false;
	}
      }
      
      if (calibrationProjector.size() > 0){
	Log::info("Performing stereo-calibration");
	camProjCalib.stereoCalibrate();
	Log::success("Done");
      }
      
      if(currState == PROJECTOR_STATIC) {
	
	if( calibrationProjector.size() < numBoardsBeforeDynamicProjection) 
	  Log::info(std::to_string(numBoardsBeforeDynamicProjection - calibrationProjector.size()) + " boards to go before dynamic projection");
	else {
	  setState(PROJECTOR_DYNAMIC);
	  lastTime = cv::getTickCount();
	}
      }
      else{
	if( calibrationProjector.size() < numBoardsFinalProjector) 
	  Log::info(std::to_string(numBoardsFinalProjector - calibrationProjector.size()) + " boards to go to completion");
	else {
	  calibrationProjector.save("calibrationProjector.xml");
	  Log::success("Projector calibration finished & saved to calibrationProjector.xml");
        
	  camProjCalib.saveExtrinsics("CameraProjectorExtrinsics.xml");
	  Log::success("Stereo Calibration finished & saved to CameraProjectorExtrinsics.xml");
        
	  Log::success("Congrats, you made it ;)");
	  setState(DEMO_AR);
	}
      }
      return true;
    } 
    return false;
  }


  void CalibrationCore::processImageForCircleDetection(const cv::Mat& cameraFrame){    
    if(cameraFrame.type() != CV_8UC1) 
      cv::cvtColor(cameraFrame, processedImg, CV_RGB2GRAY);
    else 
      cameraFrame.copyTo(processedImg);   
  
    cv::threshold(processedImg, processedImg, circleDetectionThreshold, 255, cv::THRESH_BINARY_INV);
  }


  bool CalibrationCore::timeMinBetweenFrames(){
    double timeDiff = ((double)cv::getTickCount() - lastTime) /cv::getTickFrequency(); //time in second
    //cout << "seconds elapsed :" << timeDiff <<endl;
    return timeMinBetweenCaptures < timeDiff;
  }


  void CalibrationCore::drawCameraImagePoints(){
    vector<vector<cv::Point2f> > imagePoints = camProjCalib.getCalibrationCamera().getImagePoints();
    if(imagePoints.size() <= 0) return;
    for(size_t i = 0; i < imagePoints.back().size(); i++)
      cv::circle(snapShot, imagePoints.back()[i], 3,  CV_RGB(255,0,0), -1, 8);
  }


  void CalibrationCore::drawProjectorPattern(){
    vector<cv::Point2f> patternPoints = camProjCalib.getCalibrationProjector().getCandidateImagePoints();
    if(patternPoints.size() <= 0) return;
    for(size_t i = 0; i < patternPoints.size(); i++){
      // cout <<  patternPoints[i] << endl;
      cv::circle(projectorFrame, patternPoints[i], 18,  CV_RGB(255,255,255), -1, 8);
    }
  }

  void CalibrationCore::drawProjectorFinePattern(){
    vector<cv::Point2f> patternPoints = camProjCalib.getCalibrationProjector().getCandidateImagePoints();
    if(patternPoints.size() <= 0) return;
    for(size_t i = 0; i < patternPoints.size(); i++){
      // cout <<  patternPoints[i] << endl;
      cv::circle(projectorFrame, patternPoints[i], 6,  CV_RGB(255,255,255), -1, 8);
    }
  }

  // argosCalibrator States
  void CalibrationCore::setState(CalibState state){
  
    CameraCalibration& calibrationCamera = camProjCalib.getCalibrationCamera();
    ProjectorCalibration& calibrationProjector = camProjCalib.getCalibrationProjector();
  
    switch (state){
    case CAMERA:
      camProjCalib.resetBoards();
      break;
    case PROJECTOR_STATIC:
      calibrationCamera.load("calibrationCamera.xml",false);
      camProjCalib.resetBoards();
      calibrationCamera.setupCandidateObjectPoints();
      calibrationProjector.setStaticCandidateImagePoints();
      break;
    case PROJECTOR_DYNAMIC:
      break;
    case DEMO_AR:
      camProjCalib.load("calibrationCamera.xml", "calibrationProjector.xml", "CameraProjectorExtrinsics.xml");
      calibrationCamera.setupCandidateObjectPoints();
      break;
    default:
      break;
    }
  
    currentState = state;
    currStateString = getCurrentStateString();
    Log::info("Set state : " + getCurrentStateString());
  }
  
  string CalibrationCore::getCurrentStateString(){
    string name;
    switch (currState){
    case CAMERA:            name = "CAMERA"; break;
    case PROJECTOR_STATIC:  name = "PROJECTOR_STATIC"; break;
    case PROJECTOR_DYNAMIC: name = "PROJECTOR_DYNAMIC"; break;
    case DEMO_AR:           name = "DEMO_AR"; break;
    default: break;
    }
    return name;
  }
  
  void CalibrationCore::toGreyscale(const cv::Mat& inputFrame, cv::Mat& cameraFrame){
    //-Convert to greyScale (it must be a 3 channel image)
    if (inputFrame.type() == CV_8UC3) 
      cv::cvtColor(inputFrame, cameraFrame, CV_BGR2GRAY);
    else     
      cameraFrame = inputFrame;
  }

  bool CalibrationCore::calculateDemo(cv::Mat& cameraFrame){
    
    CameraCalibration& calibrationCamera = camProjCalib.getCalibrationCamera();
    ProjectorCalibration& calibrationProjector  = camProjCalib.getCalibrationProjector();
    
    vector<cv::Point2f> chessImgPts;
    bool bPrintedPatternFound = calibrationCamera.findBoard(cameraFrame, chessImgPts, true);
    
    if(bPrintedPatternFound) {
    
      cv::Mat boardRot;
      cv::Mat boardTrans;
      calibrationCamera.computeCandidateBoardPose(chessImgPts, boardRot, boardTrans);
      //const auto & camCandObjPts = calibrationCamera.getCandidateObjectPoints();        
      //vector<cv::Point3f> camCandObjPts = calibrationCamera.getCandidateObjectPoints();
      //cv::Point3f axisX = camCandObjPts[1] - camCandObjPts[0];
      //cv::Point3f axisY = camCandObjPts[calibrationCamera.getPatternSize().width] - camCandObjPts[0];
      //cv::Point3f pos   = camCandObjPts[0] - axisY * (calibrationCamera.getPatternSize().width-2);
            
      vector<cv::Point3f> auxObjectPoints;
      //for(int i = 0; i < calibrationCamera.getPatternSize().height; i++)
      //  for(int j = 0; j < calibrationCamera.getPatternSize().width; j++)
      //auxObjectPoints.push_back(cv::Point3f(float(j * calibrationCamera.getSquareSize()), float(i * calibrationCamera.getSquareSize()), 0));
      auxObjectPoints.push_back(calibrationCamera.getCandidateObjectPoints()[0]);
      auxObjectPoints.push_back(calibrationCamera.getCandidateObjectPoints()[calibrationCamera.getPatternSize().width-1]);
      auxObjectPoints.push_back(calibrationCamera.getCandidateObjectPoints()[calibrationCamera.getPatternSize().width*calibrationCamera.getPatternSize().height-1]);
      auxObjectPoints.push_back(calibrationCamera.getCandidateObjectPoints()[calibrationCamera.getPatternSize().width*(calibrationCamera.getPatternSize().height-1)]);
    
         
      cv::Mat rotObjToProj, transObjToProj;
    
      cv::composeRT(boardRot,  boardTrans,
		    camProjCalib.getCamToProjRotation(), camProjCalib.getCamToProjTranslation(),
		    rotObjToProj, transObjToProj);

      vector<cv::Point2f> out;
      projectPoints(cv::Mat(auxObjectPoints),
		    rotObjToProj, transObjToProj,
		    calibrationProjector.getDistortedIntrinsics().getCameraMatrix(),
		    calibrationProjector.getDistCoeffs(),
		    out);
    
      calibrationProjector.setCandidateImagePoints(out);
    }
  
    return bPrintedPatternFound;
  }
}
