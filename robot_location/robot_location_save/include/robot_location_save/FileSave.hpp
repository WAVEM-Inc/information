#ifndef ROBOT_LOCATION_SAVE__FILESAVE__HPP_
#define ROBOT_LOCATION_SAVE__FILESAVE__HPP_
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include <sys/stat.h>
namespace fs = std::filesystem;
class FileSave {
private:
    std::string file_path_;
    std::string dir_path_;

public:
    // 생성자에서 저장 경로 초기화
    FileSave(std::string dir_path,std::string file_path) : dir_path_(dir_path),file_path_(file_path) {
   
    }
    // 문자열을 파일에 저장하는 함수
    void save_string(const std::string& data) {
     std::ofstream outputFile(file_path_);
        if (outputFile.is_open()) {
            outputFile << data;
            outputFile.close();
            std::cout << "문자열이 성공적으로 저장되었습니다.\n";
        } else {
            std::cerr << "파일을 열 수 없습니다.\n";
        }
    }
};


#endif