#include "Calibration.hpp"
#include <iostream>

// Intrinsics -----

Intrinsics::Intrinsics(){
}

void Intrinsics::setup(cv::Mat cameraMatrix, cv::Size imageSize, cv::Size sensorSize) {
  this->cameraMatrix = cameraMatrix;
  this->imageSize = imageSize;
  this->sensorSize = sensorSize;
  calibrationMatrixValues(cameraMatrix, imageSize, sensorSize.width, sensorSize.height,
			  fov.x, fov.y, focalLength, principalPoint, aspectRatio);
  
}

void Intrinsics::setImageSize(cv::Size imgSize) {
  imageSize = imgSize;
}

cv::Mat Intrinsics::getCameraMatrix() const {
  return cameraMatrix;
}

cv::Size Intrinsics::getImageSize() const {
  return imageSize;
}

cv::Size Intrinsics::getSensorSize() const {
  return sensorSize;
}

cv::Point2d Intrinsics::getFov() const {
  return fov;
}

double Intrinsics::getFocalLength() const {
  return focalLength;
}

double Intrinsics::getAspectRatio() const {
  return aspectRatio;
}

cv::Point2d Intrinsics::getPrincipalPoint() const {
  return principalPoint;
}
/*    
void Intrinsics::loadProjectionMatrix(float nearDist, float farDist, cv::Point2d viewportOffset) const {
  glViewport(viewportOffset.x, viewportOffset.y, imageSize.width, imageSize.height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  float w = imageSize.width;
  float h = imageSize.height;
  float fx = cameraMatrix.at<double>(0, 0);
  float fy = cameraMatrix.at<double>(1, 1);
  float cx = principalPoint.x;
  float cy = principalPoint.y;
  
  // Macro required for OpenGLES as it using different function name for frustrum
#ifdef TARGET_OPENGLES
  glFrustumf(
#else
	     glFrustum(
#endif
		       nearDist * (-cx) / fx, nearDist * (w - cx) / fx,
		       nearDist * (cy - h) / fy, nearDist * (cy) / fy,
		       nearDist, farDist);
	     glMatrixMode(GL_MODELVIEW);
	     glLoadIdentity();
	     gluLookAt(
		       0, 0, 0,
		       0, 0, 1,
		       0, -1, 0);
}
*/

Calibration::Calibration() :
  patternType(CHESSBOARD),
  patternSize(cv::Size(8, 5)), // based on Chessboard_A4.pdf, assuming world units are centimeters
  subpixelSize(cv::Size(11,11)),
  squareSize(3.6),
  reprojectionError(0),
  fillFrame(true),
  ready(false){
}

// Calibration --------------
void Calibration::save(string filename, bool absolute) const {
  if(!ready){
    cout << "Calibration::save() failed, because your calibration isn't ready yet!"<< endl;
  }
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  cv::Size imageSize = distortedIntrinsics.getImageSize();
  cv::Size sensorSize = distortedIntrinsics.getSensorSize();
  cv::Mat cameraMatrix = distortedIntrinsics.getCameraMatrix();
  fs << "cameraMatrix" << cameraMatrix;
  fs << "imageSize_width" << imageSize.width;
  fs << "imageSize_height" << imageSize.height;
  fs << "sensorSize_width" << sensorSize.width;
  fs << "sensorSize_height" << sensorSize.height;
  fs << "distCoeffs" << distCoeffs;
  fs << "reprojectionError" << reprojectionError;
  fs << "features" << "[";
  for(int i = 0; i < (int)imagePoints.size(); i++) {
    fs << "[" << imagePoints[i] << "]";
  }
  fs << "]";
}

void Calibration::load(string filename, bool absolute) {
  imagePoints.clear();
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  cv::Size imageSize, sensorSize;
  cv::Mat cameraMatrix;
  
  fs["cameraMatrix"] >> cameraMatrix;
  fs["imageSize_width"] >> imageSize.width;
  fs["imageSize_height"] >> imageSize.height;
  fs["sensorSize_width"] >> sensorSize.width;
  fs["sensorSize_height"] >> sensorSize.height;
  fs["distCoeffs"] >> distCoeffs;
  fs["reprojectionError"] >> reprojectionError;

  cv::FileNode features = fs["features"];
  for(cv::FileNodeIterator it = features.begin(); it != features.end(); it++) {
    vector<cv::Point2f> cur;
    (*it)[0] >> cur;
    imagePoints.push_back(cur);
   }
  
  addedImageSize = imageSize;
  distortedIntrinsics.setup(cameraMatrix, imageSize, sensorSize);
  updateUndistortion();
  ready = true;
  
}

void Calibration::setIntrinsics(Intrinsics& distortedIntrinsics, cv::Mat& distortionCoefficients){
  this->distortedIntrinsics = distortedIntrinsics;
  this->distCoeffs = distortionCoefficients;
  this->addedImageSize = distortedIntrinsics.getImageSize();
  updateUndistortion();
  this->ready = true;
}

void Calibration::setPatternType(CalibrationPattern patternType) {
  this->patternType = patternType;
}

void Calibration::setPatternSize(int xCount, int yCount) {
  patternSize = cv::Size(xCount, yCount);
}

void Calibration::setSquareSize(float squareSize) {
  this->squareSize = squareSize;
}

void Calibration::setFillFrame(bool fillFrame) {
  this->fillFrame = fillFrame;
}

void Calibration::setSubpixelSize(int subpixelSize) {
  subpixelSize = MAX(subpixelSize,2);
  this->subpixelSize = cv::Size(subpixelSize,subpixelSize);
}


bool Calibration::add(const cv::Mat& currentFrame) {
  addedImageSize = currentFrame.size();

  vector<cv::Point2f> pointBuf;

  // find corners
  bool found = findBoard(currentFrame, pointBuf);

  if (found)
    imagePoints.push_back(pointBuf);
  //else
  //  cout << "Calibration::add() failed" << endl;

  return found;
}


bool Calibration::findBoard(const cv::Mat& currentFrame, vector<cv::Point2f>& pointBuf, bool refine) {
  bool found = false;
  int chessFlags = CV_CALIB_CB_ADAPTIVE_THRESH;// | CV_CALIB_CB_NORMALIZE_IMAGE; no CV_CALIB_CB_FAST_CHECK, because it breaks on dark images (e.g., dark IR images from kinect)
  
  switch(patternType){
  case CHESSBOARD:
    cout << "Detecting chessboard ...";
    found = cv::findChessboardCorners(currentFrame, patternSize, pointBuf, chessFlags);
    
    // improve corner accuracy
    if(found) {
      if(currentFrame.type() != CV_8UC1) 
	cv::cvtColor(currentFrame, grayMat, CV_RGB2GRAY);
      else 
	grayMat = currentFrame;
      
      if(refine)
	// the 11x11 dictates the smallest image space square size allowed
	// in other words, if your smallest square is 11x11 pixels, then set this to 11x11
	cv::cornerSubPix(grayMat, pointBuf, subpixelSize,  cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));
    }
    break;
  case CIRCLES_GRID:
    cout << "Detecting circles grid ...";
    found = cv::findCirclesGrid(currentFrame, patternSize, pointBuf, cv::CALIB_CB_SYMMETRIC_GRID);
    break;
  case ASYMMETRIC_CIRCLES_GRID:
    cout << "Detecting asimetric circles grid ...";
    found = cv::findCirclesGrid(currentFrame, patternSize, pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);
    break;
  }
  if(found)cout << "OK!" << endl;
  else cout << "Failed!" << endl;
  
  return found;
}
/*
bool Calibration::clean(float minReprojectionError) {
  int removed = 0;
  for(int i = size() - 1; i >= 0; i--) {
    if(getReprojectionError(i) > minReprojectionError) {
      objectPoints.erase(objectPoints.begin() + i);
      imagePoints.erase(imagePoints.begin() + i);
      removed++;
    }
  }
  if(size() > 0) {
    if(removed > 0) 
      return calibrate();
    else return true;
  } 
  else {
    cout << "Calibration::clean() removed the last object/image point pair" << endl;
    return false;
  }
}
*/
void Calibration::clean(float minReprojectionError) {
  cout << "Cleaning ...";
  int removed = 0;
  for(int i = size() - 1; i >= 0; i--) {
    if(getReprojectionError(i) > minReprojectionError) {
      objectPoints.erase(objectPoints.begin() + i);
      imagePoints.erase(imagePoints.begin() + i);
      removed++;
    }
  }
  cout << "Done!" << endl;
  if(size() > 0 && removed > 0) 
    calibrate();
}

bool Calibration::calibrate() {
  if(size() < 1) {
    cout <<  "Calibration::calibrate() doesn't have any image data to calibrate from."<< endl;
    if(ready) 
      cout <<  "Calibration::calibrate() doesn't need to be called after Calibration::load()."<< endl;
    return ready;
  }
  
  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
  
  updateObjectPoints();

  int calibFlags = 0;
  float rms = cv::calibrateCamera(objectPoints, imagePoints, addedImageSize, cameraMatrix, distCoeffs, boardRotations, boardTranslations, calibFlags);
  cout << "RMS reprojection error: " << rms << endl;

  ready = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

  if(!ready) {
    cout << "Calibration::calibrate() failed to calibrate the camera"<< endl;
  }

  distortedIntrinsics.setup(cameraMatrix, addedImageSize);
  updateReprojectionError();
  updateUndistortion();

  return ready;
}

bool Calibration::isReady(){
  return ready;
}
/*
bool Calibration::calibrateFromDirectory(string directory) {
  ofDirectory dirList;
  ofImage cur;
  dirList.listDir(directory);
  for(int i = 0; i < (int)dirList.size(); i++) {
    cur.loadImage(dirList.getPath(i));
    if(!add(toCv(cur))) {
      cout << "Calibration::add() failed on " << dirList.getPath(i) << endl;
    }
  }
  return calibrate();
}
*/
void Calibration::undistort(cv::Mat& img, int interpolationMode) {
  img.copyTo(undistortBuffer);
  undistort(undistortBuffer, img, interpolationMode);
}
void Calibration::undistort(const cv::Mat& src, cv::Mat& dst, int interpolationMode) {
  cv::remap(src, dst, undistortMapX, undistortMapY, interpolationMode);
}
/*
ofVec2f Calibration::undistort(ofVec2f& src) const {
  ofVec2f dst;
  Mat matSrc = Mat(1, 1, CV_32FC2, &src.x);
  Mat matDst = Mat(1, 1, CV_32FC2, &dst.x);;
  undistortPoints(matSrc, matDst, distortedIntrinsics.getCameraMatrix(), distCoeffs);
  return dst;
}

void Calibration::undistort(vector<ofVec2f>& src, vector<ofVec2f>& dst) const {
  int n = src.size();
  dst.resize(n);
  Mat matSrc = Mat(n, 1, CV_32FC2, &src[0].x);
  Mat matDst = Mat(n, 1, CV_32FC2, &dst[0].x);
  undistortPoints(matSrc, matDst, distortedIntrinsics.getCameraMatrix(), distCoeffs);
}
*/

bool Calibration::getTransformation(Calibration& dst, cv::Mat& rotation, cv::Mat& translation) {
  //if(imagePoints.size() == 0 || dst.imagePoints.size() == 0) {
  if(!ready) {
    cout << "getTransformation() requires both Calibration objects to have just been calibrated"<< endl;
    return false;
  }
  if(imagePoints.size() != dst.imagePoints.size() || patternSize != dst.patternSize) {
    cout << "getTransformation() requires both Calibration objects to be trained simultaneously on the same board"<< endl;
    return false;
  }
  cv::Mat fundamentalMatrix, essentialMatrix;
  cv::Mat cameraMatrix = distortedIntrinsics.getCameraMatrix();
  cv::Mat dstCameraMatrix = dst.getDistortedIntrinsics().getCameraMatrix();
  // uses CALIB_FIX_INTRINSIC by default
  stereoCalibrate(objectPoints,
		  imagePoints, dst.imagePoints,
		  cameraMatrix, distCoeffs,
		  dstCameraMatrix, dst.distCoeffs,
		  distortedIntrinsics.getImageSize(), rotation, translation,
		  essentialMatrix, fundamentalMatrix);
  return true;
}
		
float Calibration::getReprojectionError() const {
  return reprojectionError;
}

float Calibration::getReprojectionError(int i) const {
  return perViewErrors[i];
}

const Intrinsics& Calibration::getDistortedIntrinsics() const {
  return distortedIntrinsics;
}

const Intrinsics& Calibration::getUndistortedIntrinsics() const {
  return undistortedIntrinsics;
}

cv::Mat Calibration::getDistCoeffs() const {
  return distCoeffs;
}

int Calibration::size() const {
  return imagePoints.size();
}

cv::Size Calibration::getPatternSize() const {
  return patternSize;
}

float Calibration::getSquareSize() const {
  return squareSize;
}


/*
void Calibration::customDraw() {
  for(int i = 0; i < size(); i++) {
    draw(i);
  }
}
*/
/*
void Calibration::draw(int i) const {
  ofPushStyle();
  ofNoFill();
  ofSetColor(ofColor::red);
  for(int j = 0; j < (int)imagePoints[i].size(); j++) {
    ofCircle(toOf(imagePoints[i][j]), 5);
  }
  ofPopStyle();
}
*/

// this won't work until undistort() is in pixel coordinates
/*
  void Calibration::drawUndistortion() const {
  vector<ofVec2f> src, dst;
  cv::Point2i divisions(32, 24);
  for(int y = 0; y < divisions.y; y++) {
  for(int x = 0; x < divisions.x; x++) {
  src.push_back(ofVec2f(
  ofMap(x, -1, divisions.x, 0, addedImageSize.width),
  ofMap(y, -1, divisions.y, 0, addedImageSize.height)));
  }
  }
  undistort(src, dst);
  ofMesh mesh;
  mesh.setMode(OF_PRIMITIVE_LINES);
  for(int i = 0; i < src.size(); i++) {
  mesh.addVertex(src[i]);
  mesh.addVertex(dst[i]);
  }
  mesh.draw();
  }
*/
/*
void Calibration::draw3d() const {
  for(int i = 0; i < size(); i++) {
    draw3d(i);
  }
}
*/
/*
void Calibration::draw3d(int i) const {
  ofPushStyle();
  ofPushMatrix();
  ofNoFill();

  applyMatrix(makeMatrix(boardRotations[i], boardTranslations[i]));

  ofSetColor(ofColor::fromHsb(255 * i / size(), 255, 255));

  ofDrawBitmapString(ofToString(i), 0, 0);

  for(int j = 0; j < (int)objectPoints[i].size(); j++) {
    ofPushMatrix();
    ofTranslate(toOf(objectPoints[i][j]));
    ofCircle(0, 0, .5);
    ofPopMatrix();
  }

  ofMesh mesh;
  mesh.setMode(OF_PRIMITIVE_LINE_STRIP);
  for(int j = 0; j < (int)objectPoints[i].size(); j++) {
    ofVec3f cur = toOf(objectPoints[i][j]);
    mesh.addVertex(cur);
  }
  mesh.draw();

  ofPopMatrix();
  ofPopStyle();
}
*/

void Calibration::updateObjectPoints() {
  vector<cv::Point3f> points = createObjectPoints(patternSize, squareSize, patternType);
  objectPoints.resize(imagePoints.size(), points);
}

void Calibration::updateReprojectionError() {
  vector<cv::Point2f> imagePoints2;
  int totalPoints = 0;
  double totalErr = 0;

  perViewErrors.clear();
  perViewErrors.resize(objectPoints.size());

  cout << "---------------" << endl;
  for(int i = 0; i < (int)objectPoints.size(); i++) {
    cv::projectPoints(cv::Mat(objectPoints[i]), boardRotations[i], boardTranslations[i], distortedIntrinsics.getCameraMatrix(), distCoeffs, imagePoints2);
    double err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);
    int n = objectPoints[i].size();
    perViewErrors[i] = sqrt(err * err / n);
    totalErr += err * err;
    totalPoints += n;
    cout << "Captured pattern " << i << " has error of " << perViewErrors[i] << endl;
  }

  reprojectionError = sqrt(totalErr / totalPoints);

  cout << "all views have error of " << reprojectionError << endl;;
  cout << "---------------" << endl;
}

void Calibration::updateUndistortion() {
  cv::Mat undistortedCameraMatrix = cv::getOptimalNewCameraMatrix(distortedIntrinsics.getCameraMatrix(), distCoeffs, distortedIntrinsics.getImageSize(), fillFrame ? 0 : 1);
  cv::initUndistortRectifyMap(distortedIntrinsics.getCameraMatrix(),distCoeffs, cv::Mat(), undistortedCameraMatrix, distortedIntrinsics.getImageSize(), 
			      CV_16SC2, undistortMapX, undistortMapY);

  undistortedIntrinsics.setup(undistortedCameraMatrix, distortedIntrinsics.getImageSize());
}

vector<cv::Point3f> Calibration::createObjectPoints(cv::Size patternSize, float squareSize, CalibrationPattern patternType) {
  vector<cv::Point3f> corners;
  switch(patternType) {
  case CHESSBOARD:
  case CIRCLES_GRID:
    for(int i = 0; i < patternSize.height; i++)
      for(int j = 0; j < patternSize.width; j++)
	corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
    break;
    
  case ASYMMETRIC_CIRCLES_GRID:
    for(int i = 0; i < patternSize.height; i++)
      for(int j = 0; j < patternSize.width; j++)
	corners.push_back(cv::Point3f(float(((2 * j) + (i % 2)) * squareSize), float(i * squareSize), 0));
    break;
  }
  return corners;
}
