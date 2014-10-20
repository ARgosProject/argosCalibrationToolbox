#include "Communicator.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "CalibrationCore.h"

#include "Log.h"

Communicator::Communicator(unsigned short port) : _port(port) {
  _tcpSocket = new tcp::socket(_ioService);
  
  CalibrationCore::getInstance();
}

Communicator::~Communicator() {
  if(_tcpSocket)
    delete _tcpSocket;
}

void Communicator::proccessCvMat(StreamType& st) {
  //Logger::Log::success("Nueva cv::Mat recibida. Size: " + std::to_string(st.size));
 

  // Decode cv::Mat
  cv::imdecode(st.data, cv::IMREAD_GRAYSCALE, &cameraFrame);            
  
  CalibrationCore::getInstance().update(cameraFrame);
    
  // Send Mat to Raspberry pi
  addCvMat(CalibrationCore::getInstance().draw());
  
}


void Communicator::waitForConnections() {
  tcp::acceptor a(_ioService, tcp::endpoint(tcp::v4(), _port));
  
  while(1) {
    Logger::Log::info("Servidor ARgos escuchando en puerto " + std::to_string(_port));
    a.accept(*_tcpSocket);
    Logger::Log::success("Nueva conexión de " + _tcpSocket->remote_endpoint().address().to_string());
    //std::thread(&Communicator::listenToCvMats, this).detach();
    receive();
  }
}

void Communicator::receive() {
  while(1) {
    StreamType st;
    unsigned char type_buf[sizeof(int)];
    unsigned char size_buf[sizeof(int)];
    unsigned char* data_buf;
    
    try {
      boost::asio::read(*_tcpSocket, boost::asio::buffer(&type_buf, sizeof(int)));  // Type
      memcpy(&st.type, &type_buf, sizeof(int));
      boost::asio::read(*_tcpSocket, boost::asio::buffer(&size_buf, sizeof(int))); // Size
      memcpy(&st.size, &size_buf, sizeof(int));

      data_buf = new unsigned char[st.size];
      boost::asio::read(*_tcpSocket, boost::asio::buffer(data_buf, st.size)); // Data
      st.data.insert(st.data.end(), &data_buf[0], &data_buf[st.size]);
      delete [] data_buf;

      switch(st.type) {
      case Type::VECTOR_I:
	proccessVectori(st);
	break;
      case Type::MATRIX_16F:
	proccessMatrix16f(st);
	break;
      case Type::CV_MAT:
	proccessCvMat(st);
	break;
      }

      if(!_buff.empty()) {
	send();
	_buff.clear();
      }
    }
    catch(boost::system::system_error const& e) {
      Logger::Log::error("Se perdió la conexión del cliente. " + std::string(e.what()));
      _tcpSocket->close();
      break;
    }
  }
}

void Communicator::proccessMatrix16f(StreamType& st) {
  Logger::Log::success("Nueva matriz de 16 float recibida. Size: " + std::to_string(st.size));

  int num_floats = st.size/sizeof(float);
  float tmp[num_floats];
  memcpy(&tmp[0], &st.data[0], sizeof(float)*(num_floats));

  // Procesar matriz (tmp) aquí

}

void Communicator::proccessVectori(StreamType& st) {
  Logger::Log::success("Nuevo std::vector<int> recibido. Size: " + std::to_string(st.size));

  int num_ints = st.size/sizeof(int);
  std::vector<int> tmp(num_ints);
  memcpy(&tmp[0], &st.data[0], sizeof(int)*(num_ints));

  // Procesar vector (tmp) aquí

}


int Communicator::send() const {
  int buff_size = _buff.size();
  //Logger::Log::info("Intentando enviar " + std::to_string(buff_size) + " bytes...");
  int bytes = boost::asio::write(*_tcpSocket, boost::asio::buffer(_buff, buff_size));
  //Logger::Log::success(std::to_string(bytes) + " bytes enviados.");

  return bytes;
}

void Communicator::addMatrix16f(const float* matrix) {
  int size = 16 * sizeof(float);
  unsigned char sMatrix[size];
  int type = Type::MATRIX_16F;
  memcpy(sMatrix, matrix, size);

  unsigned char packet_type[sizeof(int)];
  memcpy(packet_type, &type, sizeof(int));

  unsigned char packet_size[sizeof(int)];
  memcpy(packet_size, &size, sizeof(int));

  _buff.insert(_buff.end(), &packet_type[0], &packet_type[sizeof(int)]); // Tipo
  _buff.insert(_buff.end(), &packet_size[0], &packet_size[sizeof(int)]); // Tamaño
  _buff.insert(_buff.end(), &sMatrix[0], &sMatrix[size]);                // Datos
}

void Communicator::addVectori(const std::vector<int>& vector) {
  int size = vector.size() * sizeof(int);
  unsigned char sVector[size];
  int type = Type::VECTOR_I;
  memcpy(sVector, &vector[0], size);

  unsigned char packet_type[sizeof(int)];
  memcpy(packet_type, &type, sizeof(int));

  unsigned char packet_size[sizeof(int)];
  memcpy(packet_size, &size, sizeof(int));

  _buff.insert(_buff.end(), &packet_type[0], &packet_type[sizeof(int)]); // Tipo
  _buff.insert(_buff.end(), &packet_size[0], &packet_size[sizeof(int)]); // Tamaño
  _buff.insert(_buff.end(), &sVector[0], &sVector[size]);                // Datos
}

void Communicator::addCvMat(const cv::Mat& mat) {
  std::vector<unsigned char> mat_buff;
  std::vector<int> params;
  int type = Type::CV_MAT;

  params.push_back(cv::IMWRITE_JPEG_QUALITY);
  params.push_back(80);
  cv::imencode(".jpg", mat, mat_buff, params);

  unsigned char packet_type[sizeof(int)];
  memcpy(packet_type, &type, sizeof(int));

  int length = mat_buff.size();
  unsigned char packet_size[sizeof(int)];
  memcpy(packet_size, &length, sizeof(int));

  _buff.insert(_buff.end(), &packet_type[0], &packet_type[sizeof(int)]); // Tipo
  _buff.insert(_buff.end(), &packet_size[0], &packet_size[sizeof(int)]); // Tamaño
  _buff.insert(_buff.end(), mat_buff.begin(), mat_buff.end());           // Datos
}

void Communicator::addVectorCvMat(const std::vector<cv::Mat>& mats) {
  int size = mats.size();

  unsigned char num_mats[sizeof(int)];
  memcpy(num_mats, &size, sizeof(int));

  _buff.insert(_buff.end(), &num_mats[0], &num_mats[sizeof(int)]);
  for(int i = 0; i < size; ++i) {
    addCvMat(mats[i]);
  }
}




