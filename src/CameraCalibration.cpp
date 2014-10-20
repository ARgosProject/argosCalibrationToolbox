#include "CameraCalibration.hpp"

void CameraCalibration::setupCandidateObjectPoints(){
  candidateObjectPts.clear();
  for(int i = 0; i < patternSize.height; i++) {
    for(int j = 0; j < patternSize.width; j++) {
      candidateObjectPts.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
    }
  }
}
    
void CameraCalibration::computeCandidateBoardPose(const vector<cv::Point2f> & imgPts, cv::Mat& boardRot, cv::Mat& boardTrans){
  cv::solvePnP(candidateObjectPts, imgPts,
	       distortedIntrinsics.getCameraMatrix(),
	       distCoeffs,
	       boardRot, boardTrans);
}

bool CameraCalibration::backProject(const cv::Mat& boardRot64,
				    const cv::Mat& boardTrans64,
				    const vector<cv::Point2f>& imgPt,
				    vector<cv::Point3f>& worldPt) {
  if( imgPt.size() == 0 ) {
    return false;
  }
  else
    {
      cv::Mat imgPt_h = cv::Mat::zeros(3, imgPt.size(), CV_32F);
      for(size_t h=0; h<imgPt.size(); ++h ) {
	imgPt_h.at<float>(0,h) = imgPt[h].x;
	imgPt_h.at<float>(1,h) = imgPt[h].y;
	imgPt_h.at<float>(2,h) = 1.0f;
      }
      cv::Mat Kinv64 = getUndistortedIntrinsics().getCameraMatrix().inv();
      cv::Mat Kinv,boardRot,boardTrans;
      Kinv64.convertTo(Kinv, CV_32F);
      boardRot64.convertTo(boardRot, CV_32F);
      boardTrans64.convertTo(boardTrans, CV_32F);
            
      // Transform all image points to world points in camera reference frame
      // and then into the plane reference frame
      cv::Mat worldImgPt = cv::Mat::zeros( 3, imgPt.size(), CV_32F );
      cv::Mat rot3x3;
      cv::Rodrigues(boardRot, rot3x3);
            
      cv::Mat transPlaneToCam = rot3x3.inv()*boardTrans;
            
      for(size_t i=0; i<imgPt.size(); ++i ) {
	cv::Mat col = imgPt_h.col(i);
	cv::Mat worldPtcam = Kinv*col;
	cv::Mat worldPtPlane = rot3x3.inv()*(worldPtcam);
                
	float scale = transPlaneToCam.at<float>(2)/worldPtPlane.at<float>(2);
	cv::Mat worldPtPlaneReproject = scale*worldPtPlane-transPlaneToCam;
                
	cv::Point3f pt;
	pt.x = worldPtPlaneReproject.at<float>(0);
	pt.y = worldPtPlaneReproject.at<float>(1);
	pt.z = 0;
	worldPt.push_back(pt);
      }
    }
  return true;
}
    
