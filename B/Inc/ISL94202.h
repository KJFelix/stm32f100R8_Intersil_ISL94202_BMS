#ifndef __ISL94202_H__
#define  __ISL94202_H__
#include "stm32f1xx_hal.h"
#include  "stdbool.h"

#define slaveAdd   0x50
#define voltageAdd 0x8A
#define statusAdd  0x80

#define OvervoltageThreshold_L 0x00
#define OvervoltageThreshold_H 0x01
#define OvervoltageRecovery_L  0x02
#define OvervoltageRecovery_H  0x03






void ISL94202_updateReadings(void);
void ISL94202_updateStatus();

uint16_t ISL94202_getCellVoltageMV(uint8_t index);
//Returns the pack current in milliamps
uint16_t ISL94202_getPackCurrentMA();

//Returns a bitmask of the currently balancing cells
uint8_t ISL94202_getBalancingCells();

//Thresholds and releases
void ISL94202_setOVLockout(uint16_t mV);
void ISL94202_setOVThres(uint16_t mV);
void ISL94202_setOVRecovery(uint16_t mV);
void ISL94202_setUVThres(uint16_t mV);
void ISL94202_setUVRecovery(uint16_t mV);
void ISL94202_setUVLockout(uint16_t mV);
void ISL94202_setEOCThreshold(uint16_t mV);


//Current Settings
void ISL94202_setDischargeOC(uint8_t mV, uint16_t time, uint8_t timeBase);
void ISL94202_setChargeOC(uint8_t mV, uint16_t time, uint8_t timeBase);
void ISL94202_setShortCircuit(uint8_t mV, uint16_t time, uint8_t timeBase);


//Cell Balancing
void ISL94202_setCellBalanceTimes(uint8_t onTime, uint8_t offTime,
                                  uint8_t timeBase);
void ISL94202_setCellCountSleepTimes(uint8_t cellCount, uint8_t idleSleep,
                            uint8_t deepsleep);
void ISL94202_setCellBalanceDifference(uint16_t mV);
void ISL94202_setCellBalanceStartV(uint16_t mV);
void ISL94202_setCellBalanceStopV(uint16_t mV);
void ISL94202_setCellBalanceFaultLevel(uint16_t mV);



uint16_t ISL94202_milliVoltsToVScaleRaw(uint16_t mV);


//Chip Features Control
void ISL94202_setFeature1(bool CellFActivatesPSD, bool XT2Mode, bool TGain,
                 bool PreChargeFETEnabled, bool disableOpenWireScan,
                 bool OpenWireSetsPSD);

void ISL94202_setFeature2(bool CellBalanceDuringDischarge,
                          bool CellbalanceDuringCharge,
                 bool keepDFETonDuringCharge, bool keepCFETonDuringDischarge,
                 bool shutdownOnUVLO, bool enableBalanceAtEOC);
								 
//////////////////////// EEPROM \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
								 
void ISL94202_writeUserEEPROM(uint8_t address, uint8_t value);
uint8_t ISL94202_readUserEEPROM(uint8_t address);
void enableEEPROMAccess(void);
void disableEEPROMAccess(void);
//////////////////////// I2C Support  \\\\\\\\\\\\\\\\\\\\\\\\\\\\\								 
void ISL94202_writeEEPROMWord(uint8_t reg, uint16_t value);
void ISL94202_writeEEPROM(uint8_t reg, uint8_t value);
void ISL94202_writeEEPROMVoltage(uint8_t add, uint16_t mV, uint8_t headerFourBits);
void ISL94202_writeEEPROMTimeout(uint8_t add, uint16_t timeout, uint8_t timeScale, uint8_t headerFourBits);								 
								 
								 
#endif
