#ifndef _ARGOS_CAMERAPROJECTORCALIBRATION_H
#define _ARGOS_CAMERAPROJECTORCALIBRATION_H

#include <opencv2/opencv.hpp>
#include "CameraCalibration.hpp"
#include "ProjectorCalibration.hpp"

using namespace std;

class CameraProjectorCalibration {
  
public:
  CameraProjectorCalibration();
  CameraProjectorCalibration(int projectorWidth, int projectorHeight);

  void load(string cameraConfig = "calibrationCamera.xml",
	    string projectorConfig  = "calibrationProjector.xml",
	    string extrinsicsConfig = "CameraProjectorExtrinsics.xml");
  
  void update(cv::Mat& camMat);
  
  void saveExtrinsics(string filename, bool absolute = false) const;
  void loadExtrinsics(string filename, bool absolute = false);
  
  bool addProjected(const cv::Mat& img, cv::Mat& processedImg);
  
  bool setDynamicProjectorImagePoints(const cv::Mat& img);
  void stereoCalibrate();
  void resetBoards();
  int cleanStereo(float maxReproj);
  
  vector<cv::Point2f> getProjected(const vector<cv::Point3f> & ptsInWorld,
			       const cv::Mat & rotObjToCam = cv::Mat::zeros(3, 1, CV_64F),
			       const cv::Mat & transObjToCam = cv::Mat::zeros(3, 1, CV_64F));
  
  CameraCalibration & getCalibrationCamera() { return calibrationCamera; }
  ProjectorCalibration & getCalibrationProjector() { return calibrationProjector; }
  
  const cv::Mat & getCamToProjRotation() { return rotCamToProj; }
  const cv::Mat & getCamToProjTranslation() { return transCamToProj; }
  
protected:
  
  CameraCalibration calibrationCamera;
  ProjectorCalibration calibrationProjector;
  
  cv::Mat rotCamToProj;
  cv::Mat transCamToProj;
};

#endif
