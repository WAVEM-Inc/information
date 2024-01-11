#include "MGR/robot_type_manager.hpp"

RobotType::Manager::Manager() : Node("robot_type_publisher_node"){
    std::cout <<__LINE__ <<std::endl;
    this->declare_parameter<std::string>("robot_type","AMR");
    std::string robot_type;
    if (this->get_parameter("robot_type", robot_type)) {
        if(robot_type.compare("AGV")==0){
            robot_type_=Contants::Type::AGV;
        }
        else if(robot_type.compare("AMR")==0){
            robot_type_ = Contants::Type::AMR;
        }
        else if(robot_type.compare("NONE")==0){
            robot_type_ = Contants::Type::NONE;
        }
        else{
            rclcpp::shutdown();
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Parameter 'temp_robot_type' not found.");
    }
    std::cout <<__LINE__ <<std::endl;
    type_service_ = this->create_service<RobotTypeSRV>("/robot_type/service",std::bind(&RobotType::Manager::get_type,this,std::placeholders::_1,std::placeholders::_2));
    std::cout <<__LINE__ <<std::endl;
}

void RobotType::Manager::get_type(const std::shared_ptr<RobotTypeSRV::Request> req, const std::shared_ptr<RobotTypeSRV::Response> response){
    std::cout <<req->type<< " "<<req->check<<std::endl;

    if(req->check==1){ // 1 regist
        std::cout<<"registe"<<std::endl; 
        switch (static_cast<Contants::Type>(req->type))
        {
        case Contants::Type::AGV:
            robot_type_ = Contants::Type::AGV;
            response->description = "Success";
            break;
        case Contants::Type::AMR :
            robot_type_= Contants::Type::AMR;
            response->description = "Success";
            break;
        case Contants::Type::NONE: 
            response->description = "Success";
            robot_type_ = Contants::Type::NONE;
            break;
        }
    }

    else{ // 0 look_up
        std::cout<<"select "<<static_cast<int>(robot_type_)<<std::endl; 
        switch (robot_type_)
        {
            case Contants::Type::AGV:
                response->result = 2;
                response->description ="AGV";
                std::cout<<"result "<<static_cast<int>(response->result)<<std::endl;    
                break;
            case Contants::Type::AMR :
                response->result = 1;
                response->description ="AMR";
                std::cout<<"result "<<static_cast<int>(response->result)<<std::endl;     
                break;
            case Contants::Type::NONE: 
                response->result = 0;
                response->description ="NONE";
                std::cout<<"result "<<static_cast<int>(response->result)<<std::endl;    
                break;
        }
    }
}
    