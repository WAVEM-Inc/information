#include <vector>
#include <unistd.h>

#include "can/can_adaptor.hpp"
#include "can/data_relayer.hpp"

#include "can/can_define.hpp"
#include "can/df_topic.hpp"
DataRelayer::DataRelayer() {
  system_endian_ = is_big_endian();
}

DataRelayer::~DataRelayer() {
  if( canlib_ != NULL ){
    canlib_->Release();
  }
}


void DataRelayer::RegistVelocityCallback(void(*pfunc)(double)) {
  velocity_callback = static_cast<void(*)(double)>(pfunc);
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

void DataRelayer::Handler_VCU_Meter_Req(VCU_Vehicle_Status_2 msg){
  cout << "[recv] VCU_Vehicle_Status_2 : " << (double)msg.vehicle_speed-VELOCITY_STANDARD<<'\n';
  velocity_callback((double)msg.vehicle_speed-VELOCITY_STANDARD);
}
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
  canlib_->SetHandler<DataRelayer>(this,&DataRelayer::Handler_VCU_Meter_Req,VCU_VEHICLE_STATUS_2,device_type[CAN1]);
 
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





