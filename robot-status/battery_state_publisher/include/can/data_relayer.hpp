#ifndef DATA_RELAYER_H
#define DATA_RELAYER_H

#include <iostream>
#include <functional>
#include <map>
#include <memory.h>
#include <arpa/inet.h>

#include "can/w1candbc.h"


using namespace std;

class CanAdaptor;
/**
    @class   DataRelayer
    @date    2023-02-14
    @author  ys.kwon(ys.kwon@wavem.net)
    @brief   Wrapping classes for CAN data conversion and relaying
    @version 0.0.1
    @warning
*/
class DataRelayer {

  private:
    //void(*fpoint)(int,int,int);
    typedef std::function<void(int, int)> func_fault_callback; // Callback function pointer variable definition
    typedef std::function<void(int, int, int)> func_battery_callback;
    //typedef std::function<void(int,int,int)> func_other_callback; // Callback function pointer variable definition

    func_battery_callback battery_callback;
    func_fault_callback faultCallback;// Callback function pointer variable definition
 
    //func_other_callback otherCallback; // Callback function pointer variable definition

    bool system_endian_ = 0;
    CanAdaptor* canlib_ = NULL;

    map<int,int> magMap_; // code , MSG_ID
    map<int,string> channelMap_; // code , channel

  public:
    DataRelayer(); 
    virtual ~DataRelayer();

    void StopPostMessage(unsigned int id);
    void RegistBatteryCallback(void(*pfunc)(int,int,int));
    void ControlHardWare(bool horn, bool head_light, bool right_light, bool left_light);
    void SendMessageControlHardware(bool Horn,bool HeadLight,bool Right_Turn_Light, bool Left_Turn_Light);
  
   template<typename T>
   void RegistBatteryCallback(T * pClassType, void(T::*pfunc)(int,int,int)) {
          battery_callback = move(bind(pfunc, pClassType
        , placeholders::_1
        , placeholders::_2
        , placeholders::_3
        ));
    }


    void Run();


  private:
    void SetmsgMap(int svcid,int msgid,string device);  
  

    void Handler_BMS_System_Info(BMS_System_Info msg);
    unsigned short ConvertSpeedUnits(float vel);

    bool is_big_endian(){
      char buf[2] = {0,1};
      unsigned short *val = reinterpret_cast<unsigned short*>(buf);
      return *val == 1;
    }

 };

#endif //FUNCTIONCALLBACK_PARENT_H
