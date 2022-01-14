#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerialWithHalfDuplex.h>
#include "MODBUS-CRC16.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerialWithHalfDuplex sOne(2, 2); // NOTE TX & RX are set to same pin for half duplex operation
int onoff_pin = 9;

String heater_display_states[] = {"off", "starting", "pre-heat", "retrying", "ignition", "running", "stop", "stopping", "cooldown"};
enum heater_state_enum {OFF, STARTING, PREHEAT, RETRYING_START, HEATING_UP, RUNNING, STOP_COMMAND_RECEIVED, STOPPING, COOLDOWN};
enum controller_command_enum {NONE = 0x00, TURN_ON = 0xa0, TURN_OFF = 0x05};

bool is_enabled = false;
unsigned long last_button_toggled_time = 0;
unsigned long last_sent_time = 0;
unsigned long last_updated_display_time = 0;
int send_request_interval_ms = 500;
int update_display_interval_ms = 500;
int package_transfer_delay_ms = 9; // 24 bytes = 192bits @ 25000bps is 7.6ms, we'll use 9ms for a bit of tolerance :)
int button_lock_ms = 2000;

struct Command {
  uint8_t Byte0;               //  [0] 0x76 - allows heater to save tuning params to EEPROM / 0x78 - prevents heater saving tuning params to EEPROM.  LCD controllers use 0x76 as first byte, rotary knobs use 0x78
  uint8_t Len;                 //  [1] always 0x16 == 22
  uint8_t Command;             //  [2] transient commands: 00: NOP, 0xa0 START, 0x05: STOP
  uint8_t ActualTemperature;   //  [3] 1deg6C / digit
  uint8_t DesiredDemand;       //  [4] typ. 1degC / digit, but also gets used for Fixed Hx demand too! [34]
  uint8_t MinPumpFreq;         //  [5] 0.1Hz/digit [12]
  uint8_t MaxPumpFreq;         //  [6] 0.1Hz/digit [55]
  uint8_t MinFanRPM_MSB;       //  [7] 16 bit - big endian MSB 1480RPM = 05C8HEX
  uint8_t MinFanRPM_LSB;       //  [8] 16 bit - big endian LSB : 1 RPM / digit
  uint8_t MaxFanRPM_MSB;       //  [9] 16 bit - big endian MSB 5000RPM = 1388HEX
  uint8_t MaxFanRPM_LSB;       // [10] 16 bit - big endian LSB : 1 RPM / digit
  uint8_t OperatingVoltage;    // [11] 120, 240 : 0.1V/digit [120]
  uint8_t FanSensor;           // [12] SN-1 or SN-2 [1]
  uint8_t OperatingMode;       // [13] 0x32:Thermostat, 0xCD:Fixed [0xCD]
  uint8_t MinTemperature;      // [14] Minimum settable temperature
  uint8_t MaxTemperature;      // [15] Maximum settable temperature
  uint8_t GlowDrive;           // [16] power to supply to glow plug
  uint8_t Prime;               // [17] 00: normal, 0x5A: fuel prime
  uint8_t Unknown1_MSB;        // [18] always 0x01
  uint8_t Unknown1_LSB;        // [19] always 0x2c  "300 secs = max run without burn detected"?
  uint8_t Altitude_MSB;        // [20] 0x01
  uint8_t Altitude_LSB;        // [21] 0x59 - 345m for Belarus
  uint8_t CRC_MSB;             // [22] CRC checksum
  uint8_t CRC_LSB;             // [23] CRC checksum
};

struct HeaterResponse {
  uint8_t Byte0;                // [0] 0x76 - allows heater to save tuning params to EEPROM / 0x78 - prevents heater saving tuning params to EEPROM.  LCD controllers use 0x76 as first byte, rotary knobs use 0x78
  uint8_t Len;                  // [1] always 0x16 == 22 bytes
  uint8_t RunState;             // [2] operating state
  uint8_t ErrState;             // [3] 0: OFF, 1: ON, 2+ (E-0n + 1)
  uint8_t SupplyV_MSB;          // [4] 16 bit - big endian MSB
  uint8_t SupplyV_LSB;          // [5] 16 bit - big endian MSB : 0.1V / digit
  uint8_t FanRPM_MSB;           // [6] 16 bit - big endian MSB
  uint8_t FanRPM_LSB;           // [7] 16 bit - big endian LSB : 1 RPM / digit
  uint8_t FanVoltage_MSB;       // [8] 16 bit - big endian MSB
  uint8_t FanVoltage_LSB;       // [9] 16 bit - big endian LSB : 0.1V / digit
  uint8_t HeatExchgTemp_MSB;    // [10] 16 bit - big endian MSB
  uint8_t HeatExchgTemp_LSB;    // [11] 16 bit - big endian LSB : 1 degC / digit
  uint8_t GlowPlugVoltage_MSB;  // [12] 16 bit - big endian MSB
  uint8_t GlowPlugVoltage_LSB;  // [13] 16 bit - big endian LSB : 0.1V / digit
  uint8_t GlowPlugCurrent_MSB;  // [14] 16 bit - big endian MSB
  uint8_t GlowPlugCurrent_LSB;  // [15] 16 bit - big endian LSB : 10mA / digit
  uint8_t ActualPumpFreq;       // [16] fuel pump freq.: 0.1Hz / digit
  uint8_t StoredErrorCode;      // [17]
  uint8_t Unknown1;             // [18] always 0x00
  uint8_t FixedPumpFreq;        // [19] fixed mode frequency set point: 0.1Hz / digit
  uint8_t Unknown2;             // [20] always 0x64  "100 ?"
  uint8_t Unknown3;             // [21] always 0x00
  uint8_t CRC_MSB;              // [22] checksum
  uint8_t CRC_LSB;              // [23] checksum
};

Command command;
HeaterResponse heater;

void setup() {
  pinMode(onoff_pin, INPUT_PULLUP);
  lcd.init();

  byte on_char[] = {
    0x00,
    0x04,
    0x04,
    0x04,
    0x04,
    0x04,
    0x00,
    0x00
  };
  byte off_char[] = {
    0x00,
    0x0E,
    0x11,
    0x11,
    0x11,
    0x0E,
    0x00,
    0x00
  };

  lcd.createChar(1, on_char);     //  Загружаем 1 символ "вкл" в ОЗУ дисплея
  lcd.createChar(2, off_char);  //  Загружаем 1 символ "выкл" в ОЗУ дисплея
  lcd.clear();
  lcd.print("Init...");

  //init default values for command
  command = {0x78, 0x16, 0x00, 0x00, 34, 12, 55, 0x05, 0xC8, 0x13, 0x88, 120, 0x01, 0xCD, 0x08, 0x23, 0x05, 0x00, 0x01, 0x2C, 0x0D, 0xAC, 0x00, 0x00};
  update_command_CRC();

  // initialize listening serial half-duplex port
  // 25000 baud, Tx and Rx channels of Chinese heater comms interface:
  // Tx/Rx data to/from heater, special baud rate for Chinese heater controllers
  sOne.begin(25000);
  lcd.clear();

  // display headers
  lcd.setCursor(0, 0);
  lcd.print("state:");
  lcd.setCursor(0, 1);
  lcd.print("cmd:");
  lcd.setCursor(11, 1);
  lcd.print("e-");
}

void loop() {
  prepare_command();

  send_command();

  // wait for data of TX gate are transfered
  delay(package_transfer_delay_ms);

  read_response();

  update_display();
}

void prepare_command() {

  if (digitalRead(onoff_pin) == LOW && (millis() - last_button_toggled_time) > button_lock_ms) {
    is_enabled = !is_enabled;
    last_button_toggled_time = millis();
  }

  int previous_command = command.Command;

  if (is_enabled == true && heater.RunState == OFF) {
    command.Command = TURN_ON;
  }
  else if (is_enabled == false && heater.RunState != OFF) {
    command.Command = TURN_OFF;
  }
  else {
    command.Command = NONE;
  }

  if (previous_command != command.Command) {
    update_command_CRC();
  }
}

void update_command_CRC() {
  uint8_t data[22] = {
    command.Byte0,
    command.Len,
    command.Command,
    command.ActualTemperature,
    command.DesiredDemand,
    command.MinPumpFreq,
    command.MaxPumpFreq,
    command.MinFanRPM_MSB,
    command.MinFanRPM_LSB,
    command.MaxFanRPM_MSB,
    command.MaxFanRPM_LSB,
    command.OperatingVoltage,
    command.FanSensor,
    command.OperatingMode,
    command.MinTemperature,
    command.MaxTemperature,
    command.GlowDrive,
    command.Prime,
    command.Unknown1_MSB,
    command.Unknown1_LSB,
    command.Altitude_MSB,
    command.Altitude_LSB
  };

  // calculate CRC
  CModBusCRC16 CRCengine;
  uint16_t crc = CRCengine.process(22, data);

  command.CRC_MSB = (crc >> 8) & 0xff;   // MSB of CRC in request[22]
  command.CRC_LSB = (crc >> 0) & 0xff;   // LSB of CRC in request[23]
}

void send_command() {
  if ((millis() - last_sent_time) > send_request_interval_ms)
  {
    uint8_t request[24] = {
      command.Byte0,
      command.Len,
      command.Command,
      command.ActualTemperature,
      command.DesiredDemand,
      command.MinPumpFreq,
      command.MaxPumpFreq,
      command.MinFanRPM_MSB,
      command.MinFanRPM_LSB,
      command.MaxFanRPM_MSB,
      command.MaxFanRPM_LSB,
      command.OperatingVoltage,
      command.FanSensor,
      command.OperatingMode,
      command.MinTemperature,
      command.MaxTemperature,
      command.GlowDrive,
      command.Prime,
      command.Unknown1_MSB,
      command.Unknown1_LSB,
      command.Altitude_MSB,
      command.Altitude_LSB,
      command.CRC_MSB,
      command.CRC_LSB
    };

    sOne.write(request, 24);
    last_sent_time = millis();
  }
}

void read_response() {
  static byte response_buffer[24];
  static int count = 0;

  // read from serial on D2
  while (sOne.available() > 0) {
    int inByte = sOne.read(); // read hex byte
    if (count < 24) // any bytes over 24 will simply be taken to clear the response buffer
    {
      response_buffer[count++] = inByte;
    }
  }

  if (count == 24) { // filled both frames – dump
    count = 0;

    // for each response will calculate and check CRC
    CModBusCRC16 CRCengine;
    uint16_t crc = CRCengine.process(22, response_buffer);

    // accept only valid CRC responses
    if (response_buffer[22] == ((crc >> 8) & 0xff) && response_buffer[23] == ((crc >> 0) & 0xff)) {
      heater.Byte0 = response_buffer[0];
      heater.Len = response_buffer[1];
      heater.RunState = response_buffer[2];
      heater.ErrState = response_buffer[3];
      heater.SupplyV_MSB = response_buffer[4];
      heater.SupplyV_LSB = response_buffer[5];
      heater.FanRPM_MSB = response_buffer[6];
      heater.FanRPM_LSB = response_buffer[7];
      heater.FanVoltage_MSB = response_buffer[8];
      heater.FanVoltage_LSB = response_buffer[9];
      heater.HeatExchgTemp_MSB = response_buffer[10];
      heater.HeatExchgTemp_LSB = response_buffer[11];
      heater.GlowPlugVoltage_MSB = response_buffer[12];
      heater.GlowPlugVoltage_LSB = response_buffer[13];
      heater.GlowPlugCurrent_MSB = response_buffer[14];
      heater.GlowPlugCurrent_LSB = response_buffer[15];
      heater.ActualPumpFreq = response_buffer[16];
      heater.StoredErrorCode = response_buffer[17];
      heater.Unknown1 = response_buffer[18];
      heater.FixedPumpFreq = response_buffer[19];
      heater.Unknown2 = response_buffer[20];
      heater.Unknown3 = response_buffer[21];
      heater.CRC_MSB = response_buffer[22];
      heater.CRC_LSB = response_buffer[23];
    }
  }
}

void update_display() {
  if ((millis() - last_updated_display_time) > update_display_interval_ms)
  {
    // state value
    lcd.setCursor(6, 0);
    lcd.print("        ");
    lcd.setCursor(6, 0);
    lcd.print(heater_display_states[int(heater.RunState)]);

    // on/off indicator
    lcd.setCursor(15, 0);
    if (is_enabled == true) {
      lcd.print("\1");
    }
    else {
      lcd.print("\2");
    }

    // current command
    lcd.setCursor(4, 1);
    lcd.print("   ");
    lcd.setCursor(4, 1);
    if (command.Command == TURN_ON) {
      lcd.print("start");
    }
    else if (command.Command == TURN_OFF) {
      lcd.print("stop ");
    }
    else {
      lcd.print("-    ");
    }

    // saved error code
    lcd.setCursor(13, 1);
    if (heater.StoredErrorCode < 100) {
      lcd.print("0");
    }
    if (heater.StoredErrorCode < 10) {
      lcd.print("0");
    }
    lcd.print(int(heater.StoredErrorCode));
  }
}

