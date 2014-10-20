#include "Log.h"

#include <ctime>

using namespace Logger;

bool Log::coloured_output = false;

void Log::plain(const std::string& msg) {
	std::cout << currentDateTime() << " " << msg << std::endl;
}

void Log::info(const std::string& msg) {
  if(coloured_output) {
    std::cout << "\033[" << Colour::FG_LIGHT_BLUE << "m" << currentDateTime() << " [INFO] " << msg << "\033[" << FG_DEFAULT << "m" << std::endl;
  }
  else {
    std::cout << currentDateTime() << " [INFO] " << msg << std::endl;
  }
}

void Log::error(const std::string& msg) {
  if(coloured_output) {
    std::cerr << "\033[" << Colour::FG_LIGHT_RED << "m" << currentDateTime() << " [ERROR] " << msg << "\033[" << FG_DEFAULT << "m" << std::endl;
  }
  else {
    std::cerr << currentDateTime() << " [ERROR] " << msg << std::endl;
  }
}

void Log::success(const std::string& msg) {
  if(coloured_output) {
    std::cout << "\033[" << Colour::FG_LIGHT_GREEN << "m" << currentDateTime() << " [SUCCESS] " << msg << "\033[" << FG_DEFAULT << "m" << std::endl;
  }
  else {
    std::cout << currentDateTime() << " [SUCCESS] " << msg << std::endl;
  }
}

const std::string Log::currentDateTime() {
	time_t now = time(0);
	tm tstruct;
	char buf[80];

	tstruct = *localtime(&now);
	strftime(buf, sizeof(buf), "[%d-%m-%Y %X]", &tstruct);

	return buf;
}

void Log::setColouredOutput(bool coloured) {
  coloured_output = coloured;
}
