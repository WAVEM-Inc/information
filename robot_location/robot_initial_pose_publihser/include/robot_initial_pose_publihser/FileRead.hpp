#ifndef ROBOT_LOCATION__ROBOT_INITIAL_POSE_PUBLISHER__FILEREAD__HPP_
#define ROBOT_LOCATION__ROBOT_INITIAL_POSE_PUBLISHER__FILEREAD__HPP_

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

class FileRead{
public:
    FileRead( std::string file_path) : file_path_(file_path) {}

    // 파일을 읽어서 문자열 벡터로 반환
    std::vector<std::string> read_file()  {
        std::vector<std::string> lines;
        std::ifstream fileStream(file_path_);
        if (fileStream.is_open()) {
            std::string line;
            while (std::getline(fileStream, line)) {
                lines.push_back(line);
            }
            fileStream.close();
        } else {
            std::cerr << "파일을 열 수 없습니다: " << file_path_ << std::endl;
        }

        return lines;
    }

private:
    std::string file_path_;
    
};

#endif