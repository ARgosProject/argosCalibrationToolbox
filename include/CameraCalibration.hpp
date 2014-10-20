#ifndef _ARGOS_CAMERACALIBRATION_H
#define _ARGOS_CAMERACALIBRATION_H

#include <opencv2/opencv.hpp>
#include "Calibration.hpp"

using namespace std; 

class CameraCalibration : public Calibration {
  
public:
  void computeCandidateBoardPose(const vector<cv::Point2f> & imgPts, cv::Mat& boardRot, cv::Mat& boardTrans);
  bool backProject(const cv::Mat& boardRot64, const cv::Mat& boardTrans64,
		   const vector<cv::Point2f>& imgPt,
		   vector<cv::Point3f>& worldPt);
  void setupCandidateObjectPoints();
  vector<cv::Point3f> getCandidateObjectPoints() { return candidateObjectPts; }
  
private:
  vector<cv::Point3f> candidateObjectPts;
};


#endif

