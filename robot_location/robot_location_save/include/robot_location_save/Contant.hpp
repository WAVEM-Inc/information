#ifndef ROBOT_LOCATION_SAVE__CONTANT__HPP_
#define ROBOT_LOCATION_SAVE__CONTANT__HPP_
#include <iostream>
#define ROBOT_POSE "/robot_pose"
namespace CONTANTS{
    class Topic{
        public :
            const std::string robot_pose_;
            Topic():robot_pose_(ROBOT_POSE){
                
            }
    };
}
#endif 