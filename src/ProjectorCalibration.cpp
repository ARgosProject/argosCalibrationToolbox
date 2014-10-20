#include "ProjectorCalibration.hpp"


void ProjectorCalibration::setStaticCandidateImagePoints(){
  candidateImagePoints.clear();
  cv::Point2f p;
  for(int i = 0; i < patternSize.height; i++) {
    for(int j = 0; j < patternSize.width; j++) {
     
      p.x = patternPosition.x + float(((2 * j) + (i % 2)) * squareSize);
      p.y = patternPosition.y + float(i * squareSize);
      candidateImagePoints.push_back(p);
    }
  }
}
    
void ProjectorCalibration::setCandidateImagePoints(vector<cv::Point2f> pts){
  candidateImagePoints = pts;
}
    
void ProjectorCalibration::setImagerSize(int width, int height) {
  imagerSize = cv::Size(width, height);
  distortedIntrinsics.setImageSize(imagerSize);
  undistortedIntrinsics.setImageSize(imagerSize);
  addedImageSize = imagerSize;
}
    
void ProjectorCalibration::setPatternPosition(float px, float py) {
  patternPosition = cv::Point2f(px, py);
}
    
