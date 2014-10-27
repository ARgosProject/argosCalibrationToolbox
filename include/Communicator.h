#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>

/**
 * Protocolo de recepción de datos:
 *
 *   Tipo  Tamaño     Datos
 * +-------------------------------
 * | int  | int  | unsigned char* |
 * +-------------------------------
 */


using namespace std;
using namespace cv;

using boost::asio::ip::tcp;
namespace argosServer{
  class Communicator {
  public:
    enum Type {
      VECTOR_I   = 0,
      MATRIX_16F = 1,
      CV_MAT     = 2,
    };
  
  public:
    struct StreamType {
      Type type;
      int size;
      std::vector<unsigned char> data;
    };
  
  public:
    explicit Communicator(unsigned short port);
    ~Communicator();
  
    void waitForConnections();
  
    void receive();
    void proccessMatrix16f(StreamType& st);
    void proccessVectori(StreamType& st);
    void proccessCvMat(StreamType& st);
  
    int send() const;
    void addMatrix16f(const float* matrix);
    void addVectori(const std::vector<int>& vector);
    void addCvMat(const cv::Mat& mat);
    void addVectorCvMat(const std::vector<cv::Mat>& mats);

 
  private:
    boost::asio::io_service _ioService;
    tcp::socket* _tcpSocket;
    unsigned short _port;
    std::vector<unsigned char> _buff;
    cv::Mat cameraFrame;		// current frame from camera


  };
}
#endif
