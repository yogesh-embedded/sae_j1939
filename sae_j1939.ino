// ------------------------------------------------------------------------
// ARD1939 - SAE J1939 Protocol Stack for Arduino Due
// ------------------------------------------------------------------------
//
//  This Arduino program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.

#include <stdlib.h>
#include <inttypes.h>

#include "ARD1939.h"

ARD1939 j1939;

J1939Data j1939Data;


int nCounter = 0;

void setup() 
{
  Serial.begin(MONITOR_BAUD_RATE);
  Serial.println("Starting ARD1939 setup...");

  if(j1939.Init(SYSTEM_TIME) == CAN_OK) {
    Serial.println("CAN Controller Init OK.");
  } else {
    Serial.println("CAN Controller Init Failed.");
  }
  
  j1939.SetPreferredAddress(SA_PREFERRED);
  Serial.println("Preferred address set.");

  j1939.SetAddressRange(ADDRESSRANGEBOTTOM, ADDRESSRANGETOP);
  Serial.println("Address range set.");

  j1939.SetMessageFilter(PGN_ET1);
  j1939.SetMessageFilter(PGN_AMB);
  j1939.SetMessageFilter(PGN_LFC);
  j1939.SetMessageFilter(PGN_EEC1);
  j1939.SetMessageFilter(PGN_EEC2);
  j1939.SetMessageFilter(PGN_IC1);
  j1939.SetMessageFilter(PGN_TCO1);

  j1939.SetNAME(NAME_IDENTITY_NUMBER,
               NAME_MANUFACTURER_CODE,
               NAME_FUNCTION_INSTANCE,
               NAME_ECU_INSTANCE,
               NAME_FUNCTION,
               NAME_VEHICLE_SYSTEM,
               NAME_VEHICLE_SYSTEM_INSTANCE,
               NAME_INDUSTRY_GROUP,
               NAME_ARBITRARY_ADDRESS_CAPABLE);
  Serial.println("NAME configured.");
  Serial.println("Setup complete.");
}

void loop()
{
  byte nMsgId, nDestAddr, nSrcAddr, nPriority;
  int nMsgLen;
  long lPGN;
  byte pMsg[J1939_MSGLEN];
  byte msgData[] = {0x38, 0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31};
  byte msgLong[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45};

  delay(SYSTEM_TIME);
  byte nJ1939Status = j1939.Operate(&nMsgId, &lPGN, &pMsg[0], &nMsgLen, &nDestAddr, &nSrcAddr, &nPriority);

  if (nJ1939Status == NORMALDATATRAFFIC) {
    nCounter++;
    if (nCounter == (int)(1000 / SYSTEM_TIME)) {
      nSrcAddr = j1939.GetSourceAddress();
      DEBUG_PRINTLN("Transmitting periodic message...");
      if (j1939.Transmit(6, 65280, nSrcAddr, 255, msgData, 8) == OK) 
      {
        DEBUG_PRINTLN("Message transmitted.");
      }
      nCounter = 0;
      nCounter = 0;
    }
  }

  if (nMsgId == J1939_MSG_APP) { 
      parseJ1939Message(static_cast<PGN>(lPGN), pMsg, nMsgLen, j1939Data);
      // printDynamicData(j1939Data);
  }

  {
    static unsigned long lastPrintTime = 0; // Tracks the last time the function was called
    unsigned long currentMillis = millis();
    // Check if 100 ms have passed since the last call
    if (currentMillis - lastPrintTime >= 100) {
        lastPrintTime = currentMillis; // Update the last call time

        // Call the function to print dynamic data
        printDynamicData(j1939Data);
    }
  }

  if (nMsgId == J1939_MSG_APP) {
    switch (nJ1939Status) {
      case ADDRESSCLAIM_INPROGRESS:
        DEBUG_PRINTLN("Address claim in progress...");
        break;
      case NORMALDATATRAFFIC:
        DEBUG_PRINTLN("Normal data traffic ongoing...");
        break;
      case ADDRESSCLAIM_FAILED:
        DEBUG_PRINTLN("Address claim failed!");
        break;
    }
  }
}
