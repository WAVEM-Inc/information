#include <vector>
#include <unistd.h>

#include "can/can_adaptor.hpp"
#include "can/data_relayer.hpp"

#include "can/can_define.hpp"
DataRelayer::DataRelayer() {
  system_endian_ = is_big_endian();
}

DataRelayer::~DataRelayer() {
  if( canlib_ != NULL ){
    canlib_->Release();
  }
}


void DataRelayer::RegistBatteryCallback(void(*pfunc)(int,int,int)) {
  battery_callback = static_cast<void(*)(int,int,int)>(pfunc);
}


/**
* @brief service <-> message id / channel mapping
* @details Mapping so that can channel and can id can be changed according to environment variables later
* @param
* @return void
* @exception
*/
void DataRelayer::SetmsgMap(int svcid,int msgid,string device){
  magMap_.insert(make_pair(svcid,msgid));
  channelMap_.insert(make_pair(svcid,device));
}




void DataRelayer::Handler_BMS_System_Info(BMS_System_Info msg){
  cout << "[recv] BMS_System_Info : " << (int)msg.bms_sys_soc<<'\n';
  battery_callback((int)msg.bms_sys_soc, (int)msg.bms_sys_status, (int)msg.bms_charge_status);
}

void DataRelayer::ControlHardWare(bool horn, bool head_light, bool right_light, bool left_light){
   SendMessageControlHardware(horn, head_light, right_light, left_light);
}
  
void DataRelayer::SendMessageControlHardware(bool Horn,bool HeadLight,bool Right_Turn_Light, bool Left_Turn_Light){
  AD_Control_Body  dat_1;
  memset(&dat_1,0x00,CAN_MAX_DLEN);
  dat_1.ad_headlight = HeadLight?1:0;
  dat_1.ad_horn_control = Horn?1:0;
  dat_1.ad_left_turn_light = Left_Turn_Light?1:0;
  dat_1.ad_right_turn_light = Right_Turn_Light?1:0;
  dat_1.ad_body_msgcntr =  static_cast<int>(CAN1_HEART_BEAT::FIFTEEN); // new heartbeat
  dat_1.ad_brake_light = 0; // new
  canlib_->PostCanMessage<AD_Control_Body >(dat_1,AD_CONTROL_BODY ,device_type[CAN1]);
};


/**
* @brief run test
* @details
* @param
* @return void
* @exception
*/
void DataRelayer::Run(){

  canlib_ = CanAdaptor::getInstance();
  canlib_->Initialize(system_endian_);


  canlib_->SetHandler<DataRelayer>(this,&DataRelayer::Handler_BMS_System_Info,SYSTEM_INFO,device_type[CAN1]);
 
  // 수신 리스너 오픈
  vector<string> device;
  device.push_back(device_type[CAN1]);


  int ret = 0;
   
  //
  while(canlib_->Open(device) != 0 ){
    cout << "open fail" << endl;
    sleep(CAN_ALIVE_CHECKTIME);      
  }

  //포트 오픈 체크 스레드
  cout << "Start checking for can channel fault" << endl;
  canlib_->CheckSocketStatus(device,faultCallback);    
}

void DataRelayer::StopPostMessage(unsigned int id){
  canlib_->StopPostMessage(id);
}





