#ifndef ARD1939_H
#define ARD1939_H

// Arduino Definitions
#define MONITOR_BAUD_RATE                     115200
#define CAN_PORT                              0

#define CAN_OK                                0
#define CAN_ERROR                             1

// System Settings
#define SYSTEM_TIME                           10    // Milliseconds

#define TRANSPORT_PROTOCOL                    0
#define J1939_MSGLEN                          1785
#define MSGFILTERS                            100

#define SA_PREFERRED                      	  128
#define ADDRESSRANGEBOTTOM                	  129
#define ADDRESSRANGETOP                   	  247

#define GLOBALADDRESS                    	    255
#define NULLADDRESS                      	    254

// NAME Fields Default 
#define NAME_IDENTITY_NUMBER              	  0xFFFFFF
#define NAME_MANUFACTURER_CODE            	  0xFFF
#define NAME_FUNCTION_INSTANCE            	  0
#define NAME_ECU_INSTANCE                 	  0x01
#define NAME_FUNCTION                     	  0xFF
#define NAME_RESERVED                     	  0
#define NAME_VEHICLE_SYSTEM               	  0x7F
#define NAME_VEHICLE_SYSTEM_INSTANCE      	  0
#define NAME_INDUSTRY_GROUP               	  0x00
#define NAME_ARBITRARY_ADDRESS_CAPABLE    	  0x01

// Return Codes
#define ADDRESSCLAIM_INIT                     0
#define ADDRESSCLAIM_INPROGRESS           	  1
#define ADDRESSCLAIM_FINISHED             	  2
#define NORMALDATATRAFFIC                 	  2
#define ADDRESSCLAIM_FAILED               	  3

#define J1939_MSG_NONE                   	  0
#define J1939_MSG_PROTOCOL               	  1
#define J1939_MSG_NETWORKDATA                 2
#define J1939_MSG_APP                   	  3

// Compiler Settings
#define OK                                    0
#define ERR                                   1

// Debugger Settings
#define DEBUG                                 0


enum PGN {
    PGN_ET1 = 65262,  // Engine Temperature 1
    PGN_AMB = 65269,  // Ambient Conditions
    PGN_LFC = 65257,  // Fuel Consumption (Liquid)
    PGN_EEC1 = 61444, // Electronic Engine Controller 1
    PGN_EEC2 = 61443, // Electronic Engine Controller 2
    PGN_IC1 = 65270,  // Inlet/Exhaust Conditions 1
    PGN_TCO1 = 65132  // Tachograph
};

#if DEBUG == 1

  #define DEBUG_INIT() char sDebug[128];
  #define DEBUG_PRINTHEX(T, v) Serial.print(T); sprintf(sDebug, "%x\n\r", v); Serial.print(sDebug);
  #define DEBUG_PRINTDEC(T, v) Serial.print(T); sprintf(sDebug, "%d\n\r", v); Serial.print(sDebug);
  #define DEBUG_PRINTARRAYHEX(T, a, l) Serial.print(T); if(l == 0) Serial.print("Empty.\n\r"); else {for(int x=0; x<l; x++){sprintf(sDebug, "%x ", a[x]); Serial.print(sDebug);} Serial.print("\n\r");}
  #define DEBUG_PRINTARRAYDEC(T, a, l) Serial.print(T); if(l == 0) Serial.print("Empty.\n\r"); else {for(int x=0; x<l; x++){sprintf(sDebug, "%d ", a[x]); Serial.print(sDebug);} Serial.print("\n\r");}
  #define DEBUG_HALT() while(Serial.available() == 0); Serial.setTimeout(1); Serial.readBytes(sDebug, 1);
  #define DEBUG_PRINT(T)      Serial.print(T);
  #define DEBUG_PRINTLN(T)      Serial.println(T);

#else

  #define DEBUG_INIT()                                ;
  #define DEBUG_PRINTHEX(T, v)                        ;
  #define DEBUG_PRINTDEC(T, v)                        ;
  #define DEBUG_PRINTARRAYHEX(T, a, l)                ;
  #define DEBUG_PRINTARRAYDEC(T, a, l)                ;
  #define DEBUG_HALT()                                ;
  #define DEBUG_PRINT(T)                              ;
  #define DEBUG_PRINTLN(T)                            ;


#endif

struct v35
{
  int v36;
  bool v21;
  bool v37;
};

class ARD1939
{
  public: 
    // Initialization
    byte Init(int nSystemTime);
    void SetPreferredAddress(byte nAddr);
    void SetAddressRange(byte nAddrBottom, byte nAddrTop);
    void SetNAME(long lIdentityNumber, int nManufacturerCode, byte nFunctionInstance, byte nECUInstance, 
                           byte nFunction, byte nVehicleSystem, byte nVehicleSystemInstance, byte nIndustryGroup, byte nArbitraryAddressCapable);
    byte SetMessageFilter(long lPGN);
  
    // Read/Write - Check Status
    byte Operate(byte* nMsgId, long* lPGN, byte* pMsg, int* nMsgLen, byte* nDestAddr, byte* nSrcAddr, byte* nPriority);
    byte Transmit(byte nPriority, long lPGN, byte nSourceAddress, byte nDestAddress, byte* pData, int nDataLen);
  
    // Other Application Functions
    void Terminate(void);
    byte GetSourceAddress(void);
    void DeleteMessageFilter(long lPGN);
    
  private:
    byte f01(byte, byte*);
    bool f02(void);
    byte f03(byte*, byte*);
    byte f04(long*, byte*, int*, byte*, byte*, byte*);
    void f05(void);
    void f06(struct v35*);
    bool f07(long*, byte*);
    bool f08(long);
    bool f09(long);

#if TRANSPORT_PROTOCOL == 1
    byte f10(byte, long, byte, byte, byte*, int);
    void f11(byte);
    void f12(byte);
    byte f13(long, byte*, int, byte, byte, byte);
#endif
  
}; // end class ARD1939


struct PGN65262_ET1 {
    int8_t coolant_temp;
    int8_t fuel_temp;
    int16_t oil_temp;
    int16_t turbo_oil_temp;
    int8_t intercooler_temp;
    int8_t intercooler_thermostat_opening;
};

struct PGN65269_AMB {
    int8_t barometric_pressure;
    int16_t cab_interior_temp;
    int16_t ambient_air_temp;
    int8_t air_inlet_temp;
    int16_t road_surface_temp;
};

struct PGN65257_LFC {
    uint32_t engine_trip_fuel;
    uint32_t engine_total_fuel_used;
};

struct PGN61444_EEC1 {
    uint8_t engine_torque_mode;
    uint8_t driver_percent_torque;
    uint8_t actual_percent_torque;
    float engine_speed; // RPM
    uint8_t source_address;
    uint8_t starter_mode;
    uint8_t engine_demand_percent_torque;
};

struct AcceleratorPedalSwitch {
    uint8_t low_idle_switch : 2;      // SPN 558
    uint8_t kickdown_switch : 2;     // SPN 559
    uint8_t speed_limit_status : 2;  // SPN 1437
    uint8_t low_idle_switch_2 : 2;   // SPN 2970
};

struct VehicleAccelerationStatus {
    uint8_t acceleration_rate_limit : 2; // SPN 2979
};

struct EEC2 {
    AcceleratorPedalSwitch accel_pedal_switch; // SPN 558, 559, 1437, 2970
    uint8_t accel_pedal_pos1;                 // SPN 91
    uint8_t engine_percent_load;             // SPN 92
    uint8_t remote_accel_pedal_pos;          // SPN 974
    uint8_t accel_pedal_pos2;                // SPN 29
    VehicleAccelerationStatus accel_status;  // SPN 2979
    uint8_t max_available_torque;            // SPN 3357
};

struct IC1 {
    uint8_t particulate_trap_inlet_pressure;  // SPN 81
    uint8_t intake_manifold_pressure;        // SPN 102
    uint8_t intake_manifold_temperature;     // SPN 105
    uint8_t air_inlet_pressure;              // SPN 106
    uint8_t air_filter_differential_pressure; // SPN 107
    uint16_t exhaust_gas_temperature;        // SPN 173
    uint8_t coolant_filter_differential_pressure; // SPN 112
};


struct TCO1 {
    uint8_t driver1_working_state : 3;  // SPN 1612 (3 bits)
    uint8_t driver2_working_state : 3;  // SPN 1613 (3 bits)
    uint8_t vehicle_motion : 2;         // SPN 1611 (2 bits)

    uint8_t driver1_time_related_states : 4; // SPN 1617 (4 bits)
    uint8_t driver_card_driver1 : 2;         // SPN 1615 (2 bits)
    uint8_t vehicle_overspeed : 2;           // SPN 1614 (2 bits)

    uint8_t driver2_time_related_states : 4; // SPN 1618 (4 bits)
    uint8_t driver_card_driver2 : 2;         // SPN 1616 (2 bits)
    uint8_t system_event : 2;                // SPN 1622 (2 bits)

    uint8_t handling_information : 2;        // SPN 1621 (2 bits)
    uint8_t tachograph_performance : 2;      // SPN 1620 (2 bits)
    uint8_t direction_indicator : 2;         // SPN 1619 (2 bits)

    uint16_t output_shaft_speed;             // SPN 1623 (16 bits)
    uint16_t vehicle_speed;                  // SPN 1624 (16 bits, variable with pot)
};


struct J1939Data {
    PGN65262_ET1 et1;
    PGN65269_AMB amb;
    PGN65257_LFC lfc;
    PGN61444_EEC1 eec1;
    EEC2 eec2;
    IC1 ic1;
    TCO1 tco1;
};

void parseJ1939Message(PGN pgn, const uint8_t* pMsg, int nMsgLen, J1939Data& data);
void printDynamicData(const J1939Data& data);


#endif
