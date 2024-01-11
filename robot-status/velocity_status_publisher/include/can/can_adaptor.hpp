#ifndef CAN_ADAPTOR_H
#define CAN_ADAPTOR_H

#include <functional>
#include <iostream>
#include <sstream>
#include <memory>
#include <map>
#include <cstring>
#include <vector>
#include<unordered_map>

#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "w1candbc.h"
#include "cancallbackfunc.hpp"

#include <mutex>
//typedef unsigned char* byte;
#define byte unsigned char
//#define ONLY_SFF "C00007FF"
#define ONLY_SFF "7FF"

#define CAN_NO_FAULT 0x00 
#define CAN_DEVICE_FAULT 0x01 


class CanDump;
class CanSend;

/**
    @class   CanAdaptor
    @date    2023-02-14
    @author  ys.kwon(ys.kwon@wavem.net)
    @brief   CAN network relaying class
    @version 0.0.1
    @warning 
*/
class CanAdaptor {
 
  public:
    
    CanAdaptor(){
       instance = NULL;
    } 
    virtual ~CanAdaptor(){this->Release();};

  private:  

    map<int, std::shared_ptr<CanCallbackFunc>> funcsmap_;//?
    static CanAdaptor* instance;
  

    typedef std::function<void(VCU_Vehicle_Status_2)> func_vcu_meter_req;


    func_vcu_meter_req handler_vmr;
    bool isBigEndian_ = 0;

    std::shared_ptr<CanDump> ptr_can_dump_ = NULL;
    std::shared_ptr<CanSend> ptr_can_send_ = NULL;
    
    typedef std::map<unsigned int, pthread_t> ThreadMap;
    ThreadMap psotmsg_tm_;

  private:
    //int socketopen(char* device );
    //void socketclose();
    int  Send(vector<byte>  body, unsigned int msgid, string device ); //< can network 연동, cansend.c 참조
    void Receive(byte* data,int canid);
    int  CanOpen(int arc,vector<string> argv,CanAdaptor*,void(CanAdaptor::*func)(unsigned char* data,int canid));
       
    void PrintMapState(string name);



    void PostMessageByType(byte* body, unsigned int canid, string device );
    void PostMessageByType(byte* data, unsigned int canid, string device,int duration );
    
    int SOpen(vector<string> device); 
    int ROpen(vector<string> device); 
    
  public:   
    int  Initialize(bool endian); //< 초기화
    void Release(); //< 종료
    int  Open(vector<string> device); //< open can channel, warning : callback function을 전부 등록후 호출한다.

    bool IsConnected(string device);           
    void CheckSocketStatus(vector<string> device,std::function<void(int,int)> func);
    void StopPostMessage(unsigned int canid);
   /**
    * @brief Returns a singleton object.
    * @details 
    * @param 
    * @return CanAdaptor* singleton object.
    * @exception
    */
    static CanAdaptor* getInstance(){
      if ( instance == NULL ){
         std::shared_ptr<CanAdaptor> object = std::make_shared<CanAdaptor>();
         instance=object.get();        
      }    
      return instance;
    };

   
    template<typename T>
    void SetHandler(T *pClassType,void(T::*pfunc)(VCU_Vehicle_Status_2),int canid,string device){        
      handler_vmr = move(bind(pfunc, pClassType, placeholders::_1));
     // int canid = string2hex(msgid);
      
      std::shared_ptr<CanCallbackFunc> object = std::make_shared<CanCallbackFunc>(
                 canid
                ,device
                ,[&](byte* data) { 
                  // data를 MCU_Torque_Feedback 맞춰서 넣는다.
                  VCU_Vehicle_Status_2 r;
                  memcpy((void*)&r,data,CAN_MAX_DLEN);
                  //this->handler_h(r);
                  //cout<< "call MCU_Torque_Feedback" << endl;
                  handler_vmr((VCU_Vehicle_Status_2)r);
                  //cout<< "end handler_mtf" << endl;
                }
                );
      
       cout << "setHandler(VCU_Meter_Req) : " + device << ", canid : "<< canid << endl;          
       funcsmap_.insert(make_pair(canid,object));           
//      print_map_state("MCU_Torque_Feedback");
   }
   
    template<typename T>
    void PostCanMessage(T structTypeData,int msgid,string device){ 
      //cout <<  "post msg "<< endl;
      string msg("[send]<");
      msg.append(std::to_string(msgid)).append("> ").append(typeid(structTypeData).name()).append(" : ").append(device);
      cout << msg << endl;
      //byte* body = makeframebody(temp,data);
      //1) 타입별로 별고 처리가 필요하지 않은 경우 아래 사용
      byte body[CAN_MAX_DLEN];	         
	    memcpy(body,(void*)&structTypeData,CAN_MAX_DLEN);                
      PostMessageByType(body,msgid,device);      
      //2) 중간에 타입별로 처리가 필요한 경우 아래 사용
      //PostMessageByType(structType,msgid,device);      
   }

   template<typename T>
   void PostCanMessage(T structTypeData,int msgid,string device,int duration){ 
      //cout <<  "post msg "<< endl;
      string msg("[send]<");
      msg.append(std::to_string(msgid)).append("> ").append(typeid(structTypeData).name());
      cout << msg << endl;
      //byte* body = makeframebody(temp,data);
      //1) 타입별로 별고 처리가 필요하지 않은 경우 아래 사용
      byte body[CAN_MAX_DLEN];	         
      memcpy(body,(void*)&structTypeData,CAN_MAX_DLEN);                
      PostMessageByType(body,msgid,device,duration);      

      //2) 중간에 타입별로 처리가 필요한 경우 아래 사용
      //PostMessageByType(structType,msgid,device);      
   }

};

#endif //FUNCTIONCALLBACK_CHILD_H
