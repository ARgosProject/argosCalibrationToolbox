#include "CameraProjectorCalibration.hpp"

void CameraProjectorCalibration::load(string cameraConfig, string projectorConfig, string extrinsicsConfig){
  calibrationCamera.load(cameraConfig);
  calibrationProjector.load(projectorConfig);
  loadExtrinsics(extrinsicsConfig);
}
    
CameraProjectorCalibration::CameraProjectorCalibration(int projectorWidth, int projectorHeight){
        
  calibrationCamera.setPatternSize(8, 5);
  calibrationCamera.setSquareSize(3.6);
  calibrationCamera.setPatternType(CHESSBOARD);
        
  calibrationProjector.setImagerSize(projectorWidth, projectorHeight);
  calibrationProjector.setPatternSize(4, 5);
  calibrationProjector.setPatternPosition(180,100);
  calibrationProjector.setSquareSize(36);
  calibrationProjector.setPatternType(ASYMMETRIC_CIRCLES_GRID);
}

CameraProjectorCalibration::CameraProjectorCalibration(){
  calibrationCamera.setPatternSize(8, 5);
  calibrationCamera.setSquareSize(3.6);
  calibrationCamera.setPatternType(CHESSBOARD);
  
  calibrationProjector.setImagerSize(800, 600);
  calibrationProjector.setPatternSize(4, 5);
  calibrationProjector.setPatternPosition(180,100);
  calibrationProjector.setSquareSize(36);
  calibrationProjector.setPatternType(ASYMMETRIC_CIRCLES_GRID);
}



void CameraProjectorCalibration::saveExtrinsics(string filename, bool absolute) const {
        
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  fs << "Rotation_Vector" << rotCamToProj;
  fs << "Translation_Vector" << transCamToProj;
}

void CameraProjectorCalibration::loadExtrinsics(string filename, bool absolute) {
        
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  fs["Rotation_Vector"] >> rotCamToProj;
  fs["Translation_Vector"] >> transCamToProj;
}
    
vector<cv::Point2f> CameraProjectorCalibration::getProjected(const vector<cv::Point3f>& pts,
							 const cv::Mat& rotObjToCam,
							 const cv::Mat& transObjToCam){
  cv::Mat rotObjToProj, transObjToProj;
        
  cv::composeRT(rotObjToCam,  transObjToCam,
		rotCamToProj, transCamToProj,
		rotObjToProj, transObjToProj);
        
  vector<cv::Point2f> out;
  projectPoints(cv::Mat(pts),
		rotObjToProj, transObjToProj,
		calibrationProjector.getDistortedIntrinsics().getCameraMatrix(),
		calibrationProjector.getDistCoeffs(),
		out);
  return out;
}
    
bool CameraProjectorCalibration::addProjected(const cv::Mat& img, cv::Mat& processedImg){
        
  vector<cv::Point2f> chessImgPts;
        
  bool bPrintedPatternFound = calibrationCamera.findBoard(img, chessImgPts, true);
  //cv::drawChessboardCorners(snapShot, calibrationCamera.getPatternSize(), cv::Mat(chessImgPts),bPrintedPatternFound);
  if(bPrintedPatternFound) {
    cout << "Detecting asimetric grid...";
    vector<cv::Point2f> circlesImgPts;    

    cv::SimpleBlobDetector::Params params;
    params.minArea = 10;
    params.minDistBetweenBlobs = 5;
    cv::Ptr<cv::FeatureDetector> blobDetector = new cv::SimpleBlobDetector(params);
    //cv::Ptr<cv::FeatureDetector> blobDetector = new cv::SimpleBlobDetector();

    //bool bProjectedPatternFound = cv::findCirclesGrid(processedImg, calibrationProjector.getPatternSize(), circlesImgPts, cv::CALIB_CB_ASYMMETRIC_GRID);   
    bool bProjectedPatternFound = cv::findCirclesGrid(processedImg, calibrationProjector.getPatternSize(), circlesImgPts, 
						      cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);   
    if(bProjectedPatternFound){
      //cv::drawChessboardCorners(snapShot, calibrationProjector.getPatternSize(), cv::Mat(circlesImgPts),bProjectedPatternFound);
      vector<cv::Point3f> circlesObjectPts;
      cv::Mat boardRot;
      cv::Mat boardTrans;
      calibrationCamera.computeCandidateBoardPose(chessImgPts, boardRot, boardTrans);
      calibrationCamera.backProject(boardRot, boardTrans, circlesImgPts, circlesObjectPts);
                
      calibrationCamera.imagePoints.push_back(chessImgPts);
      calibrationCamera.getObjectPoints().push_back(calibrationCamera.getCandidateObjectPoints());
      calibrationCamera.getBoardRotations().push_back(boardRot);
      calibrationCamera.getBoardTranslations().push_back(boardTrans);
                
      calibrationProjector.imagePoints.push_back(calibrationProjector.getCandidateImagePoints());
      calibrationProjector.getObjectPoints().push_back(circlesObjectPts);
      cout << "OK!" << endl;           
      return true;
    }
    else cout << "Failed!" << endl;
  }
  return false;
}
    
bool CameraProjectorCalibration::setDynamicProjectorImagePoints(const cv::Mat& img){
        
  vector<cv::Point2f> chessImgPts;
  bool bPrintedPatternFound = calibrationCamera.findBoard(img, chessImgPts, true);

  if(bPrintedPatternFound) {
            
    cv::Mat boardRot;
    cv::Mat boardTrans;
    calibrationCamera.computeCandidateBoardPose(chessImgPts, boardRot, boardTrans);
    const auto & camCandObjPts = calibrationCamera.getCandidateObjectPoints();        
    //vector<cv::Point3f> camCandObjPts = calibrationCamera.getCandidateObjectPoints();
    cv::Point3f axisX = camCandObjPts[1] - camCandObjPts[0];
    cv::Point3f axisY = camCandObjPts[calibrationCamera.getPatternSize().width] - camCandObjPts[0];
    cv::Point3f pos   = camCandObjPts[0] - axisY * (calibrationCamera.getPatternSize().width-2);
            
    vector<cv::Point3f> auxObjectPoints;
    for(int i = 0; i < calibrationProjector.getPatternSize().height; i++) {
      for(int j = 0; j < calibrationProjector.getPatternSize().width; j++) {
	auxObjectPoints.push_back(pos + axisX * float((2 * j) + (i % 2)) + axisY * i);
      }
    }
            
    cv::Mat Rc1, Tc1, Rc1inv, Tc1inv, Rc2, Tc2, Rp1, Tp1, Rp2, Tp2;
    Rp1 = calibrationProjector.getBoardRotations().back();
    Tp1 = calibrationProjector.getBoardTranslations().back();
    Rc1 = calibrationCamera.getBoardRotations().back();
    Tc1 = calibrationCamera.getBoardTranslations().back();
    Rc2 = boardRot;
    Tc2 = boardTrans;
            
    cv::Mat auxRinv = cv::Mat::eye(3,3,CV_32F);
    cv::Rodrigues(Rc1,auxRinv);
    auxRinv = auxRinv.inv();
    Rodrigues(auxRinv, Rc1inv);
    Tc1inv = -auxRinv*Tc1;
    cv::Mat Raux, Taux;
    composeRT(Rc2, Tc2, Rc1inv, Tc1inv, Raux, Taux);
    composeRT(Raux, Taux, Rp1, Tp1, Rp2, Tp2);
            
    vector<cv::Point2f> followingPatternImagePoints;
    projectPoints(cv::Mat(auxObjectPoints),
		  Rp2, Tp2,
		  calibrationProjector.getDistortedIntrinsics().getCameraMatrix(),
		  calibrationProjector.getDistCoeffs(),
		  followingPatternImagePoints);
            
    calibrationProjector.setCandidateImagePoints(followingPatternImagePoints);
  }
  return bPrintedPatternFound;
}
    
void CameraProjectorCalibration::stereoCalibrate(){
        
  const auto & objectPoints = calibrationProjector.getObjectPoints();
  // vector<vector<cv::Point3f> >  objectPoints = calibrationProjector.getObjectPoints();
  vector<vector<cv::Point2f> > auxImagePointsCamera;
  for (size_t i=0; i<objectPoints.size() ; i++ ) {
    vector<cv::Point2f> auxImagePoints;
    projectPoints(cv::Mat(objectPoints[i]),
		  calibrationCamera.getBoardRotations()[i],
		  calibrationCamera.getBoardTranslations()[i],
		  calibrationCamera.getDistortedIntrinsics().getCameraMatrix(),
		  calibrationCamera.getDistCoeffs(),
		  auxImagePoints);
            
    auxImagePointsCamera.push_back(auxImagePoints);
  }
        
  cv::Mat projectorMatrix     = calibrationProjector.getDistortedIntrinsics().getCameraMatrix();
  cv::Mat projectorDistCoeffs = calibrationProjector.getDistCoeffs();
  cv::Mat cameraMatrix        = calibrationCamera.getDistortedIntrinsics().getCameraMatrix();
  cv::Mat cameraDistCoeffs    = calibrationCamera.getDistCoeffs();
        
  cv::Mat fundamentalMatrix, essentialMatrix;
  cv::Mat rotation3x3;
        
  cv::stereoCalibrate(objectPoints,
		      auxImagePointsCamera,
		      calibrationProjector.imagePoints,
		      cameraMatrix, cameraDistCoeffs,
		      projectorMatrix, projectorDistCoeffs,
		      calibrationCamera.getDistortedIntrinsics().getImageSize(),
		      rotation3x3, transCamToProj,
		      essentialMatrix, fundamentalMatrix);
        
  cv::Rodrigues(rotation3x3, rotCamToProj);
}

void CameraProjectorCalibration::resetBoards(){
  calibrationCamera.resetBoards();
  calibrationProjector.resetBoards();
}
    
int CameraProjectorCalibration::cleanStereo(float maxReproj){
  int removed = 0;
  for(int i = calibrationProjector.size() - 1; i >= 0; i--) {
    if(calibrationProjector.getReprojectionError(i) > maxReproj) {
      calibrationProjector.remove(i);
      calibrationCamera.remove(i);
      removed++;
    }
  }
  return removed;
}
    
