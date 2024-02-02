#ifndef CANDBC_H
#define CANDBC_H

#pragma pack(push, 1)

struct BMS_System_Info {
   unsigned short bms_battery_out_voltage;// Signal Name: BMS_Battery_Out_Voltage 2byte 16 bite start 0 
   unsigned short bms_battery_out_current;  // Signal Name: BMS_Battery_Out_Current 2byte 16 bite start 2
   unsigned char bms_sys_soc;// Signal Name: BMS_Sys_SOC 1byte 8bite start 4
   unsigned char bms_battery_max_temperature;// Signal Name: BMS_Battery_Max_Temperature start 5
   unsigned short bms_sys_status:3; // Signal Name: BMS_Sys_Status 2byte
   unsigned short bms_charge_status:2; // Signal Name: BMS_Charge_Status 2byte
   unsigned short bms_precharge_signal_status:2;// Signal Name: BMS_Precharge_Signal_Status 2byte
   unsigned short reserved:9;// Signal Name: BMS_Precharge_Signal_Status 2byte
};

#pragma pack(pop)

#endif
/*
struct VCU_DBS_Request{
unsigned long reserved1:17;
unsigned long vcu_dbs_request_flag:1;
unsigned long reserved2:22;
unsigned long vcu_dbs_pressure_request:16;
unsigned long reserved3:8;
};*/
