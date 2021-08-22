// Copyright (C) 2021 Patrick Frontzek
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
// der GNU General Public License, wie von der Free Software Foundation,
// Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
// veröffentlichten Version, weiter verteilen und/oder modifizieren.
//
// Dieses Programm wird in der Hoffnung bereitgestellt, dass es nützlich sein wird, jedoch
// OHNE JEDE GEWÄHR,; sogar ohne die implizite
// Gewähr der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
// Siehe die GNU General Public License für weitere Einzelheiten.
//
// Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
// Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.

#include <Arduino.h>
#include <LowPower.h>
#include <SoftwareSerial.h>
#include <DS3232RTC.cpp>
#include <EEPROM.h>

#define POW_10_6 10000000
#define POW_10_7 100000000
#define POW_10_8 1000000000
#define POW_10_9 10000000000
#define RANDOM_SEED 0x8706cbb6

#define HTTP_HOST "http://your.http.endpoint/update"

//SIM800L
#if VERSION == 1
#define PIN_GSM_RESET 9
#define PIN_GSM_POWER 2
#elif VERSION == 2
#define PIN_GSM_RESET 9
#define PIN_GSM_POWER 6
#elif VERSION == 6
#define PIN_GSM_POWER A0
#define PIN_GSM_RESET A1
#endif

#define PIN_RX 0
#define PIN_TX 1

//Spannungsmessung
#if VERSION == 1
#define PIN_U_POWER 5
#define PIN_U_IN A6
#elif VERSION == 2
#define PIN_U_POWER 5
#define PIN_U_IN A6
#elif VERSION == 6
#define PIN_U_POWER 5
#define PIN_U_IN A7
#endif

//Realtime Clock
#if VERSION == 1
#define PIN_RTC_POWER 12
#define PIN_RTC_RESET 11
#define PIN_RTC_INT 3
#define INT_RTC INT1
#elif VERSION == 2
#define PIN_RTC_POWER 12
#define PIN_RTC_RESET 11
#define PIN_RTC_INT 2
#define INT_RTC INT0
#elif VERSION == 6
#define PIN_RTC_INT 2
#define INT_RTC INT0
#define PIN_RTC_POWER 9
#define PIN_RTC_RESET 10
#endif

//Manipulation Interrupt
#define INT_MAN INT1
#define PIN_MAN 3

//Software Serial
#define PIN_SS_TX 8
#define PIN_SS_RX 7

//Buffer
#define CELLS_LENGTH_MAX 40

//Serial
#define debugSerial softSerial
#define SERIAL_DEBUG_BAUD 57600
#define SERIAL_GSM_BAUD 115200
#define SERIAL_GSM_BAUD_STRING "115200"
#define gsmSerial Serial

const uint64_t CMD_DEFAULT_TIMEOUT = 1000U;

enum RESPONSE_CHECK
{
  STARTS_WITH,
  CONTAINS,
  MATCHES,
  ENDS_WITH,
  NONE,
};

typedef struct Cell
{
  uint16_t mcc;
  uint16_t mnc;
  uint16_t lac;
  uint16_t cellid;
  uint8_t bsic;
  uint8_t rxl;
  uint16_t arfcn;
} Cell;

typedef struct UpdateHeader
{
  uint64_t id;
#if EXTRAS_VOLTAGE_RAW
  uint16_t voltage_ref;
  uint16_t voltage_raw;
#else
  float voltage;
#endif
  uint32_t uplink;
  uint32_t uplink_success;
  uint8_t version;
  uint8_t cells_length;
  uint8_t extras_flags;
} UpdateHeader;

enum EXTRAS_FLAGS
{
  VOLTAGE = 1 << 0,
  TEMPERATURE = 1 << 1,
  OPTION_NO_WAITTIME = 1 << 2,
  VOLTAGE_RAW = 1 << 3,
  ERROR_CODE = 1 << 4,
};

typedef struct Extras
{
#if EXTRAS_TEMPERATURE
  int16_t temperature;
#endif
#if DEBUG
#endif
} Extras;

void setup();
bool readResponseLine(uint32_t timeout);
bool testGSM();
void writeToGSM(char *cmd);
void writeToGSM(const char *cmd);
bool execAT(char *cmd, const char *expected, uint32_t timeout, RESPONSE_CHECK rc);
bool execAT(const char *cmd, const char *expected, uint32_t timeout, RESPONSE_CHECK rc);
bool execAT(const char *cmd, const char *expected, uint32_t timeout);
bool execAT(const char *cmd, const char *expected);
bool execAT(const char *cmd);
bool execAT(const char *cmd, uint32_t timeout);
void loop();
void initUnusedPins();
void initRTC();
void setRTC(time_t time);
void resetRTC();
void initVoltmeter();
void setVoltage();
void setTemperature();
void setGSMPower(bool power);
void clearInput();
void setId();
void startNetscan();
void startCENG();
bool sendHTTPData();
bool parseHTTPResponse();
void flashLED(uint8_t n);
void alarmIsr() {}
bool checkHttpStatus();