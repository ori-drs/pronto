#pragma once
#include <string>
#include <fstream>

namespace pronto {

class DataFileLogger {
private:
        std::ofstream fs;
        bool suppress_logger;

public:
        DataFileLogger();
        DataFileLogger(std::string filename);

        void Open(bool not_suppress, std::string filename);
        void Close();

        void log(std::string data);
        void operator<<(std::string to_log);
};
}
