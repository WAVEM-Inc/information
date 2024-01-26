#include"RobotLocationSaveMGR.hpp"
#define FILE_PATH "/home/msi-341/RobotData/location.config"
#define DIR_PATH "/home/ubuntu/RobotData"
RobotLocationSaveMGR::RobotLocationSaveMGR():Node("robot_location_save_node"){
    file_save_ = std::make_unique<FileSave>(DIR_PATH,FILE_PATH);
    contant_ = std::make_unique<CONTANTS::Topic>();
    robot_pose_= this->create_subscription<PoseMSG>(contant_->robot_pose_,1,
                                                                    std::bind(&RobotLocationSaveMGR::robot_pose_callback,this,std::placeholders::_1));
    
}

void RobotLocationSaveMGR::robot_pose_callback(std::shared_ptr<PoseMSG> pose){
    std::cout<<"test"<<std::endl;
    file_save_->save_string(posestamped_to_string(*pose));
}
std::string RobotLocationSaveMGR::posestamped_to_string(PoseMSG pose){
    std::string result = std::to_string(pose.position.x)+'\n'+std::to_string(pose.position.y)+'\n'+std::to_string(pose.orientation.w)+'\n'+std::to_string(pose.orientation.z);
    return result;
}