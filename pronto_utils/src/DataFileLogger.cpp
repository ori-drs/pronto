#include "pronto_utils/DataFileLogger.hpp"

namespace pronto {

// creates a new_logging file
DataFileLogger::DataFileLogger() {
  suppress_logger = false;
}

DataFileLogger::DataFileLogger(std::string filename) {
        suppress_logger = false;
        fs.open(filename.c_str());
}

void DataFileLogger::Open(bool not_suppress, std::string filename) {
        suppress_logger = !not_suppress;

        if (!suppress_logger) {
                fs.open(filename.c_str());
        }

}

void DataFileLogger::Close() {
  if (!suppress_logger) {
    fs.close();
  }

}

void DataFileLogger::log(std::string data) {
  if (!suppress_logger) {
    fs << data;
  }
}

void DataFileLogger::operator<<(std::string to_log) {
  if (!suppress_logger) {
    log(to_log);
  }
}
}
