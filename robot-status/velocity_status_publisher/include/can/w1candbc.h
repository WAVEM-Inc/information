#ifndef CANDBC_H
#define CANDBC_H

#pragma pack(push, 1)

struct VCU_Vehicle_Status_2 {
	unsigned short vehicle_speed; // Start Bit 0 , Bit Length 16, totoal --> 16
	unsigned short vehicle_brake_pressure; //start Bit 16, Bit Length 16 , totoal --> 32
	unsigned short vehicle_steering_angle:10; // start bit 32 bit length 10, total --> 42
	unsigned short reserved1:6; // total --> 48
	unsigned short reserved2:12; //total --> 60
	unsigned short vehicle_status_2_msgcntr:4; // start bit 60 , bit length 4
};

struct VCU_Meter_Req{
	unsigned long long vcu_meter_req_voltage:10;
	unsigned long long vcu_meter_req_current:10;
	unsigned long long vcu_meter_req_soc:8;
	unsigned long long vcu_meter_req_mileage:20; //48
	unsigned long long reserved1:4;
	unsigned long long vcu_meter_req_errorcode:10;  
	unsigned long long vcu_meter_req_ready:1;
	unsigned long long vcu_meter_req_charge_state:1;
};

#pragma pack(pop)

#endif
