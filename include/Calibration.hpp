#ifndef _ARGOS_CALIBRATION_H
#define _ARGOS_CALIBRATION_H

#include <opencv2/opencv.hpp>

using namespace std;

/*
* Class Intrinsics
* 
*/ 
class Intrinsics{
public:
  Intrinsics();
  void setup(cv::Mat cameraMatrix, cv::Size imageSize, cv::Size sensorSize = cv::Size(0, 0));  
  void setImageSize(cv::Size imgSize);
  cv::Mat getCameraMatrix() const;
  cv::Size getImageSize() const;
  cv::Size getSensorSize() const;
  cv::Point2d getFov() const;
  double getFocalLength() const;
  double getAspectRatio() const;
  cv::Point2d getPrincipalPoint() const;
  void loadProjectionMatrix(float nearDist = 10., float farDist = 10000., cv::Point2d viewportOffset = cv::Point2d(0, 0)) const;
  
protected:
  cv::Mat cameraMatrix;
  cv::Size imageSize, sensorSize;
  cv::Point2d fov;
  double focalLength, aspectRatio;
  cv::Point2d principalPoint;
};

/*
* Class Calibration
* 
*/ 
enum CalibrationPattern {CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID};

class Calibration{
  
public:
  Calibration();
  
  void save(string filename, bool absolute = false) const;
  void load(string filename, bool absolute = false);
  
  void setPatternType(CalibrationPattern patternType);
  void setPatternSize(int xCount, int yCount);
  void setSquareSize(float squareSize);
  /// set this to the pixel size of your smallest square. default is 11
  void setSubpixelSize(int subpixelSize);
  
  bool add(const cv::Mat& currentFrame);
  //bool clean(float minReprojectionError = 2.f);
  void clean(float minReprojectionError = 2.f);
  bool calibrate();
  // bool calibrateFromDirectory(string directory);
  bool findBoard(const cv::Mat& currentFrame, vector<cv::Point2f> &pointBuf, bool refine = true);
  void setIntrinsics(Intrinsics& distortedIntrinsics, cv::Mat& distortionCoefficients);
  
  void undistort(cv::Mat& img, int interpolationMode = cv::INTER_NEAREST);
  void undistort(const cv::Mat& src, cv::Mat& dst, int interpolationMode = cv::INTER_NEAREST);
  
  //ofVec2f undistort(ofVec2f& src) const;
  //void undistort(vector<ofVec2f>& src, vector<ofVec2f>& dst) const;
  
  bool getTransformation(Calibration& dst, cv::Mat& rotation, cv::Mat& translation);
  
  float getReprojectionError() const;
  float getReprojectionError(int i) const;
  
  const Intrinsics& getDistortedIntrinsics() const;
  const Intrinsics& getUndistortedIntrinsics() const;
  cv::Mat getDistCoeffs() const;
  
  // if you want a wider fov, say setFillFrame(false) before load() or calibrate()
  void setFillFrame(bool fillFrame);
  
  int size() const;
  cv::Size getPatternSize() const;
  float getSquareSize() const;

  static vector<cv::Point3f> createObjectPoints(cv::Size patternSize, float squareSize, CalibrationPattern patternType);
  
  void customDraw();
  //void draw(int i) const;
  void draw3d() const;
  //void draw3d(int i) const;
  
  bool isReady();
  vector<vector<cv::Point2f> > imagePoints;

  vector<vector<cv::Point2f> > & getImagePoints() { return imagePoints; }


  //Calibration Patched--------------------------------------------------------------------
  void resetBoards() {
    objectPoints.clear();
    imagePoints.clear();
    boardRotations.clear();
    boardTranslations.clear();
  }
  void remove(int index){
    objectPoints.erase(objectPoints.begin() + index);
    imagePoints.erase(imagePoints.begin() + index);
    boardRotations.erase(boardRotations.begin() + index);
    boardTranslations.erase(boardTranslations.begin() + index);
  }
  cv::Size getPatternSize() { return patternSize; }
  vector<cv::Mat> & getBoardRotations() { return boardRotations; }
  vector<cv::Mat> & getBoardTranslations() { return boardTranslations; }
  vector<vector<cv::Point3f> > & getObjectPoints() { return objectPoints; }
  //---------------------------------------------------------------------------------------

  
protected:
  CalibrationPattern patternType;
  cv::Size patternSize, addedImageSize, subpixelSize;
  float squareSize;
  cv::Mat grayMat;

  cv::Mat distCoeffs;
  
  vector<cv::Mat> boardRotations, boardTranslations;
  vector<vector<cv::Point3f> > objectPoints;
  
  float reprojectionError;
  vector<float> perViewErrors;
  
  bool fillFrame;
  cv::Mat undistortBuffer;
  cv::Mat undistortMapX, undistortMapY;

  void updateObjectPoints();
  void updateReprojectionError();
  void updateUndistortion();
  
  Intrinsics distortedIntrinsics;
  Intrinsics undistortedIntrinsics;
  
  bool ready;

 };

#endif
