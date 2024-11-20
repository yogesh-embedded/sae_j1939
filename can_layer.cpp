#include "ACAN_ESP32.h"

#define OK                                    0
#define ERR                                   1

// Module variables -------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------
uint32_t errorCode = 0;
uint32_t DESIRED_BIT_RATE = 250UL * 1000UL; // 250 kbps
long lCANBaudrate = 250UL * 1000UL;

// ------------------------------------------------------------------------------------------------------
// canInit
// ------------------------------------------------------------------------------------------------------
// lBaudrate = CAN_SPEED_250KBPS/CAN_SPEED_500KBPS
//
unsigned char canInit(long lBaudrate) {
    // Configure the CAN Module
    DESIRED_BIT_RATE = lBaudrate;
    ACAN_ESP32_Settings CAN_settings (DESIRED_BIT_RATE);
    CAN_settings.mRxPin = GPIO_NUM_5; // Replace with your desired Rx pin
    CAN_settings.mTxPin = GPIO_NUM_18; // Replace with your desired Tx pin
    CAN_settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode; // Set to normal mode

    errorCode = ACAN_ESP32::can.begin(CAN_settings);

    // Check for configuration errors
    if (errorCode != 0) {
        return ERR;
    }
    return OK;
}

// ------------------------------------------------------------------------------------------------------
// canCheckError
// ------------------------------------------------------------------------------------------------------
// 0 - No Error
// 1 - Errors exist
//
unsigned char canCheckError(void) {
    // Declarations
    unsigned char ErrorFlag = 0;

    // Retrieve error counts
    uint8_t txErrors = TWAI_TX_ERR_CNT_REG;
    uint8_t rxErrors = TWAI_RX_ERR_CNT_REG;

    // Check for transmission errors
    if (txErrors > 0) {
        ErrorFlag = 1;
    }

    return ErrorFlag;
}

// ------------------------------------------------------------------------------------------------------
// canTransmit
// ------------------------------------------------------------------------------------------------------
unsigned char canTransmit(long lID, unsigned char* pData, int nDataLen) {
    // Declarations
    CANMessage frame;

    // Configure the CAN frame
    frame.id = (uint32_t)lID;
    frame.len = nDataLen > 8 ? 8 : nDataLen; // Limit to 8 bytes
    for (int i = 0; i < frame.len; i++) {
        frame.data[i] = pData[i];
    }

    // Transmit the frame
    if (ACAN_ESP32::can.tryToSend(frame)) {
        return OK;
    } else {
        return ERR;
    }
}

// ------------------------------------------------------------------------------------------------------
// canReceive
// ------------------------------------------------------------------------------------------------------
unsigned char canReceive(long* lID, unsigned char* pData, int* nDataLen) {
    // Declarations
    CANMessage frame;

    // Check if a frame is available
    if (ACAN_ESP32::can.receive(frame)) {
        *lID = frame.id;
        *nDataLen = frame.len;

        for (int i = 0; i < *nDataLen; i++) {
            pData[i] = frame.data[i];
        }

        return OK;
    }

    return ERR;
}

// end can.cpp
