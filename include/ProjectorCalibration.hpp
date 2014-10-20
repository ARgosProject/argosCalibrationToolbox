#ifndef _ARGOS_PROJECTORCALIBRATION_H
#define _ARGOS_PROJECTORCALIBRATION_H

#include <opencv2/opencv.hpp>
#include "Calibration.hpp"

using namespace std;

class ProjectorCalibration : public Calibration{
  
public:
  void setImagerSize(int width, int height);
  void setPatternPosition(float x, float y);
  void setStaticCandidateImagePoints();
  void setCandidateImagePoints(vector<cv::Point2f> pts);
  const vector<cv::Point2f> & getCandidateImagePoints() const { return candidateImagePoints; }
  
protected:
  cv::Size imagerSize;
  cv::Point2f patternPosition;
  
private:
  vector<cv::Point2f> candidateImagePoints;
};
#endif
