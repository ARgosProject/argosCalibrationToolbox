#include "Log.h"

#include <ctime>

namespace argosServer {

  bool Log::coloured_output = false;

  void Log::plain(const std::string& msg, const std::string& filename) {
    std::cout << currentDateTime() << " " << msg << std::endl;

    if(!filename.empty()) {
      std::ofstream ofs(filename, std::ofstream::app);
      ofs << currentDateTime() << " " << msg << std::endl;
      ofs.close();
    }
  }

  void Log::info(const std::string& msg, const std::string& filename) {
    if(coloured_output)
      std::cout << "\033[" << Colour::FG_LIGHT_BLUE << "m";

    std::cout << currentDateTime() << " [INFO] " << msg << std::endl;

    if(coloured_output)
      std::cout << "\033[" << FG_DEFAULT << "m";

    if(!filename.empty()) {
      std::ofstream ofs(filename, std::ofstream::app);
      ofs << currentDateTime() << " [INFO] " << msg << std::endl;
      ofs.close();
    }
  }

  void Log::error(const std::string& msg, const std::string& filename) {
    if(coloured_output)
      std::cout << "\033[" << Colour::FG_LIGHT_RED << "m";

    std::cout << currentDateTime() << " [ERROR] " << msg << std::endl;

    if(coloured_output)
      std::cout << "\033[" << FG_DEFAULT << "m";

    if(!filename.empty()) {
      std::ofstream ofs(filename, std::ofstream::app);
      ofs << currentDateTime() << " [ERROR] " << msg << std::endl;
      ofs.close();
    }
  }

  void Log::success(const std::string& msg, const std::string& filename) {
    if(coloured_output)
      std::cout << "\033[" << Colour::FG_LIGHT_GREEN << "m";

    std::cout << currentDateTime() << " [SUCCESS] " << msg << std::endl;

    if(coloured_output)
      std::cout << "\033[" << FG_DEFAULT << "m";

    if(!filename.empty()) {
      std::ofstream ofs(filename, std::ofstream::app);
      ofs << currentDateTime() << " [SUCCESS] " << msg << std::endl;
      ofs.close();
    }
  }

  void Log::video(const std::string& msg, const std::string& filename) {
    if(coloured_output)
      std::cout << "\033[" << Colour::FG_LIGHT_YELLOW << "m";

    std::cout << currentDateTime() << " [VIDEO] " << msg << std::endl;

    if(coloured_output)
      std::cout << "\033[" << FG_DEFAULT << "m";

    if(!filename.empty()) {
      std::ofstream ofs(filename, std::ofstream::app);
      ofs << currentDateTime() << " [VIDEO] " << msg << std::endl;
      ofs.close();
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

}
