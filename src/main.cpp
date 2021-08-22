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

#define VERSION 6
#define DEBUG true
#define LED_ACTIVE true
#define EXTRAS_VOLTAGE true
#define EXTRAS_TEMPERATURE true
#define EXTRAS_OPTION_NO_WAITTIME true
#define EXTRAS_VOLTAGE_RAW true
#include "main.h"
#include "ArduinoUniqueID.h"

#define RESPONSE_BUFFER_SIZE 164U
char responseBuffer[RESPONSE_BUFFER_SIZE];
volatile uint8_t responseLength;
volatile uint8_t responseExpectedLength;
volatile uint64_t readResponseLineStart;
volatile long httpResponseCode;

#define CMD_BUFFER_SIZE 64U
char cmdBuffer[CMD_BUFFER_SIZE];
volatile uint8_t cmdLength;
volatile uint64_t cmdStart;

volatile UpdateHeader header = {
    0, //ID
#if EXTRAS_VOLTAGE_RAW
    0, //VOLTAGE_RAW
    0, //VOLTAGE_REF
#else
    0.0f, //VOLTAGE
#endif
    0, //UPLINK
    0, //UPLINK_SUCCESS
    4, //VERSION
    0, //CELLS
    0  //EXTRAS
#if EXTRAS_VOLTAGE
        | EXTRAS_FLAGS::VOLTAGE
#endif
#if EXTRAS_TEMPERATURE
        | EXTRAS_FLAGS::TEMPERATURE
#endif
#if EXTRAS_OPTION_NO_WAITTIME
        | EXTRAS_FLAGS::OPTION_NO_WAITTIME
#endif
#if EXTRAS_VOLTAGE_RAW
        | EXTRAS_FLAGS::VOLTAGE_RAW
#endif
#if DEBUG
#endif
};
volatile Cell cells[CELLS_LENGTH_MAX] = {};
volatile Extras extras = {};
volatile uint8_t uplinkFailedCount = 0;
volatile bool uplinkFailed = false;
volatile bool gsm_power = LOW;

const long BAUD_RATES[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};
const uint8_t BAUD_RATES_LENGTH = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

time_t timeNextWake, timeNow, timeDiff;

volatile const float VCC = 4.5;
volatile float internal1_1Ref;
volatile float value_digit;

void (*resetFunc)(void) = 0;

#define APN ""
#define APN_LOGIN true
#define APN_USER ""
#define APN_PWD ""

//{ AT-COMMANDS
const char SAPBR0_1[] PROGMEM = "AT+SAPBR=0,1";
const char SAPBR1_1[] PROGMEM = "AT+SAPBR=1,1";
const char SAPBR2_1[] PROGMEM = "AT+SAPBR=2,1";
const char SAPBR3_1_0[] PROGMEM = "AT+SAPBR=3,1,\"Contype\",\"GPRS\"";
const char SAPBR3_1_1[] PROGMEM = "AT+SAPBR=3,1,\"APN\",\"" APN "\"";
#if APN_LOGIN
const char SAPBR3_1_2[] PROGMEM = "AT+SAPBR=3,1,\"USER\",\"" APN_USER "\"";
const char SAPBR3_1_3[] PROGMEM = "AT+SAPBR=3,1,\"PWD\",\"" APN_PWD "\"";
#endif
const char *SAPBR3_1[] = {
    SAPBR3_1_0,
    SAPBR3_1_1,
#if APN_LOGIN
    SAPBR3_1_2,
    SAPBR3_1_3,
#endif
};
const uint8_t SAPBR3_1_LEN = sizeof(SAPBR3_1) / sizeof(SAPBR3_1[0]);
const char SAPBR4_1[] PROGMEM = "AT+SAPBR=4,1";
const char HTTPINIT[] PROGMEM = "AT+HTTPINIT";
const char HTTPPARA_0[] PROGMEM = "AT+HTTPPARA=\"CID\",1";
const char HTTPPARA_1[] PROGMEM = "AT+HTTPPARA=\"URL\",\"" HTTP_HOST "\"";
const char *HTTPPARA[] = {HTTPPARA_0, HTTPPARA_1};
const uint8_t HTTPPARA_LEN = sizeof(HTTPPARA) / sizeof(HTTPPARA[0]);
const char HTTPDATA__BASE[] PROGMEM = "AT+HTTPDATA=";
const uint8_t HTTPDATA__BASE_LEN = sizeof(HTTPDATA__BASE) - 1;
const char HTTPDATA__TIMEOUT[] PROGMEM = ",1000";
const char HTTPACTION_1[] PROGMEM = "AT+HTTPACTION=1";
const char HTTPREAD[] PROGMEM = "AT+HTTPREAD=0,12";
const uint8_t HTTPREAD_LEN = sizeof(HTTPREAD) - 1;
const char HTTPTERM[] PROGMEM = "AT+HTTPTERM";
const char CENG3_1[] PROGMEM = "AT+CENG=3,1";
const char CENG[] PROGMEM = "AT+CENG?";
const char CNETSCAN[] PROGMEM = "AT+CNETSCAN";
const char CNETSCAN1[] PROGMEM = "AT+CNETSCAN=1";
const char IPR[] PROGMEM = "AT+IPR=" SERIAL_GSM_BAUD_STRING;
const char ATW[] PROGMEM = "AT&W";
const char CPOWDN_1[] PROGMEM = "AT+CPOWD=1";
const char AT[] PROGMEM = "AT";
const char GSN[] PROGMEM = "AT+GSN";
//}

//{ AT-RESPONSES
const char ERROR[] PROGMEM = "ERROR";
const char OK[] PROGMEM = "OK";
const char NORMAL_POWER_DOWN[] PROGMEM = "NORMAL POWER DOWN";
const char CALL_READY[] PROGMEM = "Call Ready";
const char DOWNLOAD[] PROGMEM = "DOWNLOAD";
const char HTTPACTION_CHECK__BASE[] PROGMEM = "+HTTPACTION: 1,";
const char SAPBR2_1_CHECK__BASE[] PROGMEM = "+SAPBR: 1,1";
const char HTTPREAD_PREFIX[] PROGMEM = "+HTTPREAD: ";
const uint8_t HTTPREAD_PREFIX_LEN = sizeof(HTTPREAD_PREFIX) - 1;
const char CENG3_1_CHECK_PREFIX[] PROGMEM = "+CENG=3,1";
const uint8_t CENG3_1_CHECK_PREFIX_LEN = sizeof(CENG3_1_CHECK_PREFIX) - 1;
//}

const char EMPTY[] PROGMEM = "";
bool loggingActive = true;

void setup()
{
  responseBuffer[RESPONSE_BUFFER_SIZE - 1] = '\0';
  initUnusedPins();
  pinMode(LED_BUILTIN, OUTPUT);
  initRTC();

#if EXTRAS_VOLTAGE
  initVoltmeter();
#endif

  setGSMPower(HIGH);
  clearInput();
  setId();

  pinMode(PIN_MAN, INPUT_PULLUP);
  delay(50);
}

bool execAT(const char *cmd, const char *expected, uint32_t timeout)
{
  return execAT(cmd, expected, timeout, RESPONSE_CHECK::MATCHES);
}

bool execAT(const char *cmd, const char *expected)
{
  return execAT(cmd, expected, CMD_DEFAULT_TIMEOUT, RESPONSE_CHECK::MATCHES);
}

bool execAT(const char *cmd)
{
  return execAT(cmd, OK, CMD_DEFAULT_TIMEOUT, RESPONSE_CHECK::MATCHES);
}

bool execAT(const char *cmd, uint32_t timeout)
{
  return execAT(cmd, OK, timeout, RESPONSE_CHECK::MATCHES);
}

bool testGSM()
{
  return execAT(AT, OK, CMD_DEFAULT_TIMEOUT, RESPONSE_CHECK::MATCHES);
}

bool waitForResponse(const char *expected, uint32_t timeout, RESPONSE_CHECK rc)
{
  return execAT(EMPTY, expected, timeout, rc);
}

bool waitForResponse(const char *expected, uint32_t timeout)
{
  return execAT(EMPTY, expected, timeout, RESPONSE_CHECK::MATCHES);
}

bool readResponseLine(uint32_t timeout)
{
  readResponseLineStart = millis();
  responseLength = 0U;
  responseBuffer[0] = '\0';
  while (millis() - readResponseLineStart < timeout)
  {
    if (gsmSerial.available())
    {
      if (responseLength >= RESPONSE_BUFFER_SIZE - 1)
        break;
      responseBuffer[responseLength++] = (char)gsmSerial.read();
      responseBuffer[responseLength] = '\0';

      if (responseLength >= 2U && responseBuffer[responseLength - 2U] == '\r' && responseBuffer[responseLength - 1U] == '\n')
      {
        responseLength -= 2U;
        responseBuffer[responseLength] = '\0';
        return true;
      }
    }
  }
  responseBuffer[responseLength] = '\0';
  return false;
}

void writeToGSM(char *cmd)
{
  cmdLength = strlen(cmd);
  for (uint16_t _i = 0; _i < cmdLength; _i++)
  {
    gsmSerial.print(cmd[_i]);
  }
  gsmSerial.print('\r');
  gsmSerial.flush();
}

void writeToGSM(const char *cmd)
{
  cmdBuffer[0] = '\0';
  strncpy_P(cmdBuffer, cmd, CMD_BUFFER_SIZE < strlen_P(cmd) + 1 ? CMD_BUFFER_SIZE : strlen_P(cmd) + 1);
  writeToGSM(cmdBuffer);
}

bool execAT(char *cmd, const char *expected, uint32_t timeout, RESPONSE_CHECK rc)
{
  cmdLength = strlen(cmd);
  responseExpectedLength = strlen_P(expected);

  if (cmdLength > 0)
  {
    clearInput();
    writeToGSM(cmd);
  }
  cmdStart = millis();

  while (millis() - cmdStart < timeout && readResponseLine(timeout - (millis() - cmdStart)))
  {
    if (responseLength >= responseExpectedLength)
    {
      switch (rc)
      {
      case RESPONSE_CHECK::MATCHES:
        if (responseLength != responseExpectedLength)
          break;
        if (strcmp_P(responseBuffer, expected) == 0)
          return true;
        break;

      case RESPONSE_CHECK::STARTS_WITH:
        if (strncmp_P(responseBuffer, expected, responseExpectedLength) == 0)
          return true;
        break;

      case RESPONSE_CHECK::ENDS_WITH:
        if (strncmp_P(&responseBuffer[responseLength - responseExpectedLength], expected, responseExpectedLength) == 0)
          return true;
        break;

      case RESPONSE_CHECK::CONTAINS:
        if (strstr_P(responseBuffer, expected))
          return true;
        break;

      case RESPONSE_CHECK::NONE:
        break;

      default:
        break;
      }
    }

    if (responseLength == sizeof(ERROR) - 1 && strcmp_P(responseBuffer, ERROR) == 0)
      return false;
  }
  if (rc == RESPONSE_CHECK::NONE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool execAT(const char *cmd, const char *expected, uint32_t timeout, RESPONSE_CHECK rc)
{
  cmdBuffer[0] = '\0';
  strncpy_P(cmdBuffer, cmd, CMD_BUFFER_SIZE < strlen_P(cmd) + 1 ? CMD_BUFFER_SIZE : strlen_P(cmd) + 1);
  return execAT(cmdBuffer, expected, timeout, rc);
}

void loop()
{
  uplinkFailed = false;
  flashLED(1);
  clearInput();
  setGSMPower(HIGH);

  for (uint8_t _i = 0; _i < SAPBR3_1_LEN; _i++)
    execAT(SAPBR3_1[_i]);
  execAT(SAPBR4_1);

#if EXTRAS_OPTION_NO_WAITTIME
  do
  {
    header.uplink++;
    if (uplinkFailedCount == 0)
      header.uplink_success++;
#endif
#if EXTRAS_VOLTAGE
    setVoltage();
#endif
#if EXTRAS_TEMPERATURE
    setTemperature();
#endif

    loggingActive = false;
    for (uint8_t _i = 0; _i < 20; _i++)
    {
      if (execAT(SAPBR2_1, SAPBR2_1_CHECK__BASE, 1000U, RESPONSE_CHECK::STARTS_WITH) || execAT(SAPBR1_1, OK, 45000U, RESPONSE_CHECK::CONTAINS))
      {
        uplinkFailed = false;
        break;
      }
      else
      {
        uplinkFailed = true;
        delay(1000 * _i);
      }
    }
    loggingActive = true;

    if (uplinkFailed)
      break;

    if (uplinkFailed |= !execAT(HTTPINIT))
      break;
    for (uint8_t _i = 0; _i < HTTPPARA_LEN; _i++)
      uplinkFailed |= !execAT(HTTPPARA[_i]);
    if (uplinkFailed)
      break;
    startNetscan();
    startCENG();
    if (uplinkFailed |= header.cells_length <= 0)
      break;
    if (uplinkFailed |= !sendHTTPData())
      break;

    if (uplinkFailed |= !(execAT(HTTPACTION_1, 20000U) && checkHttpStatus()))
      break;
    ;
    if ((uplinkFailed = !parseHTTPResponse()))
      break;
#if DEBUG
#endif

    execAT(HTTPTERM, 10000U);
    uplinkFailedCount = 0;
#if EXTRAS_OPTION_NO_WAITTIME
  } while (timeNextWake == timeNow);
#endif

  execAT(HTTPTERM, 10000U);
  execAT(SAPBR0_1, 65000U);

  setGSMPower(LOW);
  if (uplinkFailed)
    uplinkFailedCount++;
  else
    uplinkFailedCount = 0;

#if EXTRAS_OPTION_NO_WAITTIME
  if (timeNow == timeNextWake)
    return;
#endif

  flashLED(2 + uplinkFailedCount);
  digitalWrite(PIN_RTC_POWER, HIGH);
  delay(50);
  setSyncProvider(RTC.get);
  if (uplinkFailedCount > 0)
  {
    header.uplink_success--;
    timeNextWake = RTC.get() + 60UL * (unsigned long)(pow(2, uplinkFailedCount - 1) < 120 ? pow(2, uplinkFailedCount - 1) : 120UL); // Warte nach jedem Fehlversuch 1 Minute länger höchstens jedoch 2 Stunden
  }
  if (timeNextWake + 5 > RTC.get())
  {
    RTC.setAlarm(ALM1_MATCH_DATE, (byte)second(timeNextWake), (byte)minute(timeNextWake), (byte)hour(timeNextWake), (byte)day(timeNextWake));
    RTC.alarmInterrupt(ALARM_1, true);
    delay(50);
    RTC.alarm(ALARM_1);
    delay(200);
    digitalWrite(PIN_RTC_POWER, LOW);

    pinMode(PIN_RTC_INT, INPUT_PULLUP);

    attachInterrupt(INT_RTC, alarmIsr, FALLING);
    delay(100);
    pinMode(PIN_TX, OUTPUT);
    digitalWrite(PIN_TX, LOW);
    pinMode(PIN_RX, OUTPUT);
    digitalWrite(PIN_RX, LOW);
    RTC.alarm(ALARM_1);

    // while (timeNextWake > RTC.get())
    // {
    if (digitalRead(PIN_RTC_INT) == LOW)
      LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
    else
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    // }

    detachInterrupt(INT_RTC);

    for (uint8_t i = 0; digitalRead(PIN_RTC_INT) != HIGH; i++)
    {
      pinMode(PIN_RTC_INT, INPUT_PULLUP);
      delay(50);
      digitalWrite(PIN_RTC_POWER, HIGH);
      delay(50);
      RTC.alarmInterrupt(ALARM_1, false);
      delay(50);
      RTC.alarmInterrupt(ALARM_1, true);
      delay(50);
      RTC.alarm(ALARM_1);
      delay(50);
      digitalWrite(PIN_RTC_POWER, LOW);
      if (i >= 20)
      {
        resetFunc();
        break;
      }
    }
  }
  else
  {
    digitalWrite(PIN_RTC_POWER, LOW);
  }
  pinMode(PIN_RTC_INT, INPUT_PULLUP);
}

bool checkHttpStatus()
{
  while (readResponseLine(10000UL))
  {
    if (strncmp_P(responseBuffer, HTTPACTION_CHECK__BASE, sizeof(HTTPACTION_CHECK__BASE) - 1) == 0)
    {
      httpResponseCode = strtol(&responseBuffer[sizeof(HTTPACTION_CHECK__BASE) - 1], NULL, 10);
      if (httpResponseCode == 200)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
  }

  return false;
}

void initRTC()
{
  pinMode(PIN_RTC_POWER, OUTPUT);
  digitalWrite(PIN_RTC_POWER, HIGH);
  pinMode(PIN_RTC_INT, INPUT_PULLUP);

  RTC.squareWave(SQWAVE_NONE);
  RTC.writeRTC(RTC_CONTROL, (RTC.readRTC(RTC_CONTROL) | _BV(BBSQW)));
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, false);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE);
  setSyncProvider(RTC.get);
  setTime(17, 7, 00, 17, 01, 2020);
  RTC.set(now());
  RTC.alarmInterrupt(ALARM_1, true);
  digitalWrite(PIN_RTC_POWER, LOW);
}

void setRTC(time_t time)
{
  digitalWrite(PIN_RTC_POWER, HIGH);
  delay(100);
  setTime(time);
  RTC.set(time);
  Wire.flush();
  digitalWrite(PIN_RTC_POWER, LOW);
}

void resetRTC()
{
  pinMode(PIN_RTC_RESET, OUTPUT);
  digitalWrite(PIN_RTC_POWER, HIGH);
  delay(50);
  timeNow = RTC.get();
  setTime(timeNow);
  digitalWrite(PIN_RTC_RESET, HIGH);
  delay(2000);
  digitalWrite(PIN_RTC_RESET, LOW);
  delay(1000);
  initRTC();
}

#if EXTRAS_TEMPERATURE
void setTemperature()
{
  digitalWrite(PIN_RTC_POWER, HIGH);
  delay(100);
  extras.temperature = RTC.temperature();
  digitalWrite(PIN_RTC_POWER, LOW);
}
#endif

#if EXTRAS_VOLTAGE
volatile uint8_t _adcl;
volatile uint8_t _adch;

// https://www.arduinoforum.de/arduino-Thread-Messen-der-eigenen-Betriebsspannung-mit-dem-Arduino
void initVoltmeter()
{
  pinMode(PIN_U_POWER, OUTPUT);
  digitalWrite(PIN_U_POWER, HIGH);
// Lesen 1,1V-Referenz als Input mit Referenz Betriebsspannung vcc
// Referenz Vcc und analog Input = interne Referenz 1,1V
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(10);           // Warten bis Referenz eingeschwungen
  ADCSRA |= _BV(ADSC); // Start Umwandlung
  while (bit_is_set(ADCSRA, ADSC))
    ; // Messen

#if EXTRAS_VOLTAGE_RAW
  header.voltage_ref = (uint16_t)ADCL;
  header.voltage_ref |= (uint16_t)ADCH << 8;
#else
  _adcl = ADCL; // must read ADCL first - it then locks ADCH
  _adch = ADCH; // unlocks both

  value_digit = 11L * (1.1 * VCC / (1.10 * 1023L / ((long)((long)((long)_adch << 8) | (long)_adcl)))) / 1023L; //Bei einem Spannungsteiler 10:1 -> Faktor 11
#endif
  analogReference(INTERNAL);
  analogRead(PIN_U_IN);
  delay(100);
  digitalWrite(PIN_U_POWER, LOW);
}

void setVoltage()
{
  digitalWrite(PIN_U_POWER, HIGH);
  delay(500);
#if EXTRAS_VOLTAGE_RAW
  header.voltage_raw = (uint16_t)analogRead(PIN_U_IN);
#else
  header.voltage = value_digit * analogRead(PIN_U_IN);
#endif
  digitalWrite(PIN_U_POWER, LOW);
}
#endif

void setGSMPower(bool power)
{
  if (power == gsm_power)
    return;
  pinMode(PIN_GSM_POWER, OUTPUT);
  if (power)
  {
    do
    {
      digitalWrite(PIN_GSM_POWER, LOW);
      delay(200);
      digitalWrite(PIN_GSM_POWER, HIGH);
      while (!gsmSerial)
        delay(10);
      delay(10);
      gsmSerial.begin(SERIAL_GSM_BAUD);
      if (waitForResponse(CALL_READY, 27000U))
        delay(1000);
      else
      {
        for (uint16_t _i = BAUD_RATES_LENGTH; _i > 0; _i--)
        {
          gsmSerial.begin(BAUD_RATES[_i - 1]);
          gsmSerial.write('\r');
          gsmSerial.write('\n');
          clearInput();

          if (testGSM())
          {
            execAT(IPR);
            execAT(ATW);
            gsmSerial.begin(SERIAL_GSM_BAUD);
            break;
          }
        }
      }
    } while (!testGSM());
    gsm_power = HIGH;
  }
  else
  {
    execAT(CPOWDN_1, NORMAL_POWER_DOWN, 5000U);
    delay(100);
    digitalWrite(PIN_GSM_POWER, LOW);
    delay(100);
    gsmSerial.end();
    gsm_power = LOW;
  }
}

void clearInput()
{
  gsmSerial.flush();
  delay(20);
  while (gsmSerial.available())
  {
    gsmSerial.read();
    if (!gsmSerial.available())
      delay(40);
  }
}

void setId()
{
  if (header.id > 0)
    return;

  writeToGSM(GSN);
  while (readResponseLine(1000U))
  {
    if (responseBuffer[0] >= '0' && responseBuffer[0] <= '9')
    {
      header.id = strtoul(strncpy(&responseBuffer[RESPONSE_BUFFER_SIZE - 8], responseBuffer, 7), NULL, 10);
      header.id = header.id * (uint64_t)pow(10, 8) + (uint64_t)strtoul(strncpy(&responseBuffer[RESPONSE_BUFFER_SIZE - 9], &responseBuffer[7], 8), NULL, 10);
      break;
    }
  }
  clearInput();
  for (uint8_t i = 0; i < UniqueIDsize; i++)
  {
    header.id ^= (UniqueID[i] << (24 - (i % 4) * 8));
  }

  header.id ^= RANDOM_SEED;

  randomSeed(header.id);
  double id = 0;
  for (int i = 2; i >= 0; i--)
    id += pow(POW_10_6, i) * random(POW_10_6, POW_10_7 - 1);
  header.id = id;
}

//{ NETSCAN
const char NETSCAN_OPERATOR_PREFIX[] PROGMEM = "Operator:";
const uint8_t NETSCAN_OPERATOR_PREFIX_LEN = sizeof(NETSCAN_OPERATOR_PREFIX) - 1;
const char NETSCAN_MCC_PREFIX[] PROGMEM = "MCC:";
const uint8_t NETSCAN_MCC_PREFIX_LEN = sizeof(NETSCAN_MCC_PREFIX) - 1;
const char NETSCAN_MNC_PREFIX[] PROGMEM = "MNC:";
const uint8_t NETSCAN_MNC_PREFIX_LEN = sizeof(NETSCAN_MNC_PREFIX) - 1;
const char NETSCAN_RXL_PREFIX[] PROGMEM = "Rxlev:";
const uint8_t NETSCAN_RXL_PREFIX_LEN = sizeof(NETSCAN_RXL_PREFIX) - 1;
const char NETSCAN_CELLID_PREFIX[] PROGMEM = "Cellid:";
const uint8_t NETSCAN_CELLID_PREFIX_LEN = sizeof(NETSCAN_CELLID_PREFIX) - 1;
const char NETSCAN_ARFCN_PREFIX[] PROGMEM = "Arfcn:";
const uint8_t NETSCAN_ARFCN_PREFIX_LEN = sizeof(NETSCAN_ARFCN_PREFIX) - 1;
const char NETSCAN_LAC_PREFIX[] PROGMEM = "Lac:";
const uint8_t NETSCAN_LAC_PREFIX_LEN = sizeof(NETSCAN_LAC_PREFIX) - 1;
const char NETSCAN_BSIC_PREFIX[] PROGMEM = "Bsic:";
const uint8_t NETSCAN_BSIC_PREFIX_LEN = sizeof(NETSCAN_BSIC_PREFIX) - 1;
const uint64_t NETSCAN_TIMEOUT = 45000U;

void startNetscan()
{
  header.cells_length = 0U;
  cmdStart = millis();

  clearInput();
  execAT(CNETSCAN1);
  delay(20);
  writeToGSM(CNETSCAN);

  while (header.cells_length < CELLS_LENGTH_MAX && (millis() - cmdStart) < NETSCAN_TIMEOUT - 1 && readResponseLine(NETSCAN_TIMEOUT - (millis() - cmdStart)))
  {
    if (strncmp_P(responseBuffer, NETSCAN_OPERATOR_PREFIX, NETSCAN_OPERATOR_PREFIX_LEN) == 0)
    {
      cells[header.cells_length].mcc = strtol(&strstr_P(responseBuffer, NETSCAN_MCC_PREFIX)[NETSCAN_MCC_PREFIX_LEN], NULL, 10);
      cells[header.cells_length].mnc = strtol(&strstr_P(responseBuffer, NETSCAN_MNC_PREFIX)[NETSCAN_MNC_PREFIX_LEN], NULL, 10);
      cells[header.cells_length].rxl = strtol(&strstr_P(responseBuffer, NETSCAN_RXL_PREFIX)[NETSCAN_RXL_PREFIX_LEN], NULL, 10);
      cells[header.cells_length].cellid = strtol(&strstr_P(responseBuffer, NETSCAN_CELLID_PREFIX)[NETSCAN_CELLID_PREFIX_LEN], NULL, 16);
      cells[header.cells_length].lac = strtol(&strstr_P(responseBuffer, NETSCAN_LAC_PREFIX)[NETSCAN_LAC_PREFIX_LEN], NULL, 16);
      cells[header.cells_length].bsic = strtol(&strstr_P(responseBuffer, NETSCAN_BSIC_PREFIX)[NETSCAN_BSIC_PREFIX_LEN], NULL, 16);
      cells[header.cells_length].arfcn = strtol(&strstr_P(responseBuffer, NETSCAN_ARFCN_PREFIX)[NETSCAN_ARFCN_PREFIX_LEN], NULL, 10);
      header.cells_length++;
    }
    else if (strcmp_P(responseBuffer, OK) == 0)
      break;
  }
  clearInput();
}
//}

//{ CENG
const char CENG__PREFIX[] PROGMEM = "+CENG: ";
const uint8_t CENG__PREFIX_LENGTH = sizeof(CENG__PREFIX) - 1;

void startCENG()
{
  clearInput();
  execAT(CENG3_1);
  writeToGSM(CENG);

  while (readResponseLine(1000U))
  {
    if (strncmp_P(responseBuffer, CENG3_1_CHECK_PREFIX, CENG3_1_CHECK_PREFIX_LEN) == 0)
      continue;
    if (strcmp_P(responseBuffer, OK) == 0)
      break;
    if (header.cells_length < CELLS_LENGTH_MAX && strncmp_P(responseBuffer, CENG__PREFIX, CENG__PREFIX_LENGTH) == 0 && responseLength > CENG__PREFIX_LENGTH + 23)
    {
      cells[header.cells_length].mcc = strtol(&responseBuffer[CENG__PREFIX_LENGTH + 3], NULL, 10);
      cells[header.cells_length].mnc = strtol(&responseBuffer[CENG__PREFIX_LENGTH + 7], NULL, 10);
      cells[header.cells_length].lac = strtol(&responseBuffer[CENG__PREFIX_LENGTH + 10], NULL, 16);
      cells[header.cells_length].cellid = strtol(&responseBuffer[CENG__PREFIX_LENGTH + 15], NULL, 16);
      cells[header.cells_length].bsic = strtol(&responseBuffer[CENG__PREFIX_LENGTH + 20], NULL, 10);
      cells[header.cells_length].rxl = strtol(&responseBuffer[CENG__PREFIX_LENGTH + 23], NULL, 10);
      cells[header.cells_length].arfcn = 0xffffU;
      if (cells[header.cells_length].cellid == 0xffff || cells[header.cells_length].cellid == 0U)
        continue;
      if (cells[header.cells_length].lac == 0xffff || cells[header.cells_length].lac == 0U)
        continue;
      header.cells_length++;
    }
  }
  clearInput();
}
//}

bool sendHTTPData()
{
  clearInput();
  cmdBuffer[0] = '\0';
  strcpy_P(cmdBuffer, HTTPDATA__BASE);
#if DEBUG
#else
  itoa(sizeof(UpdateHeader) + sizeof(Cell) * header.cells_length + sizeof(Extras), &cmdBuffer[HTTPDATA__BASE_LEN], 10);
#endif
  strcat_P(cmdBuffer, HTTPDATA__TIMEOUT);
  if (!execAT(cmdBuffer, DOWNLOAD, 1000UL, RESPONSE_CHECK::MATCHES))
    return false;
  delay(100);
  gsmSerial.write((char *)&header, sizeof(UpdateHeader));
  gsmSerial.write(((char *)&cells), sizeof(Cell) * header.cells_length);
  gsmSerial.write(((char *)&extras), sizeof(Extras));
#if DEBUG
#endif
  delay(100);
  gsmSerial.flush();
  return waitForResponse(OK, 5000U);
}

bool parseHTTPResponse()
{
  clearInput();
  writeToGSM(HTTPREAD);
  readResponseLine(1000U);
  if (strncmp_P(responseBuffer, HTTPREAD, HTTPREAD_LEN) != 0)
    return false;
  while (readResponseLine(1000U))
  {
    if (strncmp_P(responseBuffer, HTTPREAD_PREFIX, HTTPREAD_PREFIX_LEN) == 0)
      break;
  }
  if (strncmp_P(responseBuffer, HTTPREAD_PREFIX, HTTPREAD_PREFIX_LEN) != 0)
    return false;
  readResponseLine(1000U);

  memcpy(&timeNow, &responseBuffer[0], 4);
  memcpy(&timeNextWake, &responseBuffer[4], 4);
  memcpy(&timeDiff, &responseBuffer[8], 4);
  clearInput();

  setRTC(timeNow);

  return timeNextWake - timeNow == timeDiff;
}

void flashLED(uint8_t n)
{
  if (!LED_ACTIVE)
    return;
  for (uint8_t i = 0; i < n; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(400);
  }
}

void initUnusedPins()
{
  for (uint8_t i = 0; i <= 17U; i++)
  {
    switch (i)
    {
    case PIN_TX:
    case PIN_RX:
    case PIN_GSM_POWER:
    case PIN_RTC_INT:
    case PIN_RTC_POWER:
    case PIN_MAN:
#if EXTRAS_VOLTAGE
    case PIN_U_IN:
    case PIN_U_POWER:
#endif
#if DEBUG
    case PIN_SS_TX:
    case PIN_SS_RX:
#endif
      break;

    case PIN_GSM_RESET:
    case PIN_RTC_RESET:
    default:
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }
  }
  pinMode(A7, INPUT);
}

int main(void)
{
  init();
  setup();
  while (true)
    loop();
  return 0;
}
