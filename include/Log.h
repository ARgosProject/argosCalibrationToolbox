#ifndef LOG_H
#define LOG_H

#include <string>
#include <iostream>
#include <vector>

namespace Logger {
  class Log {
  public:
    enum Colour {
      FG_DEFAULT          = 39,
      FG_BLACK            = 30,
      FG_RED              = 31,
      FG_GREEN            = 32,
      FG_YELLOW           = 33,
      FG_BLUE             = 34,
      FG_MAGENTA          = 35,
      FG_CYAN             = 36,
      FG_LIGHT_GRAY       = 37,
      FG_DARK_GRAY        = 90,
      FG_LIGHT_RED        = 91,
      FG_LIGHT_GREEN      = 92,
      FG_LIGHT_YELLOW     = 93,
      FG_LIGHT_BLUE       = 94,
      FG_LIGHT_MAGENTA    = 95,
      FG_LIGHT_CYAN       = 96,
      FG_WHITE            = 97,

      BG_DEFAULT          = 49,
      BG_RED              = 41,
      BG_GREEN            = 42,
      BG_BLUE             = 44,
    };

  public:
		// General
		static void plain(const std::string& msg);
    static void info(const std::string& msg);
    static void error(const std::string& msg);
    static void success(const std::string& msg);

		// Custom data
		template<typename T> static void vector(const std::vector<T>& vec);
		template<typename T> static void matrix(const T* matrix);

		// Helpers
		static const std::string currentDateTime();
		static void setColouredOutput(bool coloured);

  private:
    static bool coloured_output;
  };

	template<typename T>
	void Log::vector(const std::vector<T>& vec) {
		std::cout << currentDateTime() << " [ ";
		typename std::vector<T>::const_iterator i;
		for(i = vec.begin(); i != vec.end(); ++i)
			std::cout << *i << ' ';
		std::cout << "] " << std::endl;
	}

	template<typename T>
	void Log::matrix(const T* matrix) {
		for(int i = 0; i < 16; i += 4) {
			std::cout << currentDateTime() << " [ ";
			std::cout << matrix[i] << " " << matrix[i+1] << " " << matrix[i+2] << " " << matrix[i+3];
			std::cout << "] " << std::endl;
		}
	}
}

#endif
