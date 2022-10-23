#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <HardwareSerial.h>
//begin(baud-rate, protocol, RX pin, TX pin);.
HardwareSerial H1Serial(1);
HardwareSerial H2Serial(2);

uint8_t rx_H1_buffer[512];

uint16_t rx_H1_buffer_index = 0;

uint8_t rx_buffer_flag = 0;

uint8_t rx_H2_buffer[32];
uint16_t rx_H2_buffer_index = 0;
uint8_t rx_S1_buffer[32];
uint16_t rx_S1_buffer_index = 0;

struct DataStructure {
  uint16_t id;
  uint16_t v_bplus;
  int16_t ts[4];
  uint16_t v_raw;
  uint16_t v_bias;
  uint16_t v_pack;
  uint16_t v_chg;
  uint16_t v_12v;
  uint16_t v_cell[14];
  uint16_t charge_trips[8];
  uint16_t discharge_trips[8];
  uint8_t current_float_string[16];
  //char myLetter;
};
struct DataStructure dataStatus;

bool dataFind(const char* buffer, const char* keyworld, int data_string_size, int* result) {
  char *ret;
  ret = strstr(buffer, keyworld);
  if (ret) {
    //printf("found substring at address %p\n", ret);

    char str1[data_string_size + 1];
    memset(str1, 0, data_string_size + 1);
    //    int str1_size = 8;
    //    int keyworld_size = 10;


    //      if (rx_H1_buffer_index - (uint16_t)((uint8_t*)ret - (&rx_H1_buffer[0])) < (str1_size+keyworld_size)) {
    //        str1_size = rx_H1_buffer_index - (uint16_t)((uint8_t*)ret - (&rx_H1_buffer[0]));
    //      }

    memcpy(str1, ret + strlen(keyworld), data_string_size);
    bool detectDot = false;
    for (int i = 0; i < data_string_size; i++) {
      if (str1[i] == ',' || str1[i] == '[') {
        detectDot = true;
      }
      if (str1[i] < '0' || str1[i] > '9' || detectDot) {
        str1[i] = ' ';
      }

    }

    *result = atoi(str1);

    //dataStatus.v_bplus = x;
    //Serial.printf("found: %s%s, %d\n", keyworld, str1, *result);
    return true;
  }
  return false;
}

bool binaryStringFind(const char* buffer, const char* keyworld, int* result) {
  char *ret;
  ret = strstr(buffer, keyworld);
  if (ret) {
    //printf("found substring at address %p\n", ret);
    int data_string_size = 8;
    char str1[data_string_size + 1];
    memset(str1, 0, data_string_size + 1);
    //    int str1_size = 8;
    //    int keyworld_size = 10;


    //      if (rx_H1_buffer_index - (uint16_t)((uint8_t*)ret - (&rx_H1_buffer[0])) < (str1_size+keyworld_size)) {
    //        str1_size = rx_H1_buffer_index - (uint16_t)((uint8_t*)ret - (&rx_H1_buffer[0]));
    //      }

    memcpy(str1, ret + strlen(keyworld), data_string_size);
    bool detectBinary = true;
    for (int i = 0; i < data_string_size; i++) {
      if (str1[i] != '0' && str1[i] != '1') {
        detectBinary = false;
      }


    }
    if (detectBinary) {
      for (int i = 0; i < data_string_size; i++) {
        result[i] = str1[i] - '0';
      }
//      Serial.printf("found BIN: %s->%s.\n", keyworld, str1);
      return true;
    }

  }
  return false;
}

bool floatStringFind(const char* buffer, const char* keyworld, char* result) {
  char *ret;
  ret = strstr(buffer, keyworld);
  if (ret) {
    int data_string_size = 15;
    char str1[data_string_size + 1];
    memset(str1, 0, data_string_size + 1);

    memcpy(str1, ret + strlen(keyworld), data_string_size);

    for (int i = 0; i < data_string_size; i++) {
      if ((str1[i] < '0' || str1[i] > '9') && str1[i] != '+' && str1[i] != '-' && str1[i] != 'e' && str1[i] != '.') {
        str1[i] = ' ';
        //Serial.printf("found float: set %d -> ' '\n", i);
      }

    }
    int firstIndex = 0;
    //    int detectDot = -1;
    for (int i = 0; i < data_string_size; i++) {
      if (str1[i] != ' ') {
        result[firstIndex++] = str1[i];
      }
    }


//    Serial.printf("found float: [%.15s] -> [%.15s]\n", str1, result);

    return true;
  }
  return false;
}

static uint16_t MODBUS_CRC16_V1( const uint8_t *buf, uint16_t len )
{
  uint16_t crc1 = 0xFFFF;
  uint16_t i = 0;
  uint8_t b = 0;

  for ( i = 0; i < len; i++ )
  {
    crc1 ^= buf[i];

    for ( b = 0; b < 8; b++ )
    {
      if ( crc1 & 0x0001 )
      {
        crc1 >>= 1;
        crc1 ^= 0xA001;
      }
      else
      {
        crc1 >>= 1;
      }
    }
  }

  return crc1;
}

void setup() {
  ////Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(115200, SERIAL_8N1, 22, 21);
  //  Serial.begin(115200);
  Serial.printf("\n\nBMS monitor 20221023, %d\n", sizeof(dataStatus));
  H1Serial.begin(115200, SERIAL_8N1, 16, 17);
  //H2Serial.begin(115200, SERIAL_8N1, 22, 21);

  memset(&dataStatus, 0, sizeof(dataStatus));
  dataStatus.id = 0x1234;
}

void loop() {
  while (H1Serial.available() > 0) {
    rx_H1_buffer[rx_H1_buffer_index++] = H1Serial.read();


    if (rx_H1_buffer_index >= 512) {
      rx_H1_buffer_index = 0;
    }

    if (rx_H1_buffer_index == 0) {
      rx_buffer_flag = 2;
    }
    else if (rx_H1_buffer_index == 256) {
      rx_buffer_flag = 1;
    }
    //Serial.printf("H1Serial: %02X\n", rx_H1_buffer[rx_H1_buffer_index]);
  }
  if (rx_buffer_flag > 0) {

    int data_string_size;
    int result;
    bool ret = false;
    if (!ret) {
      char buffer[256 + 1];
      memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
      data_string_size = 8;
      result = 0;
      ret = dataFind((const char*) buffer, "BatteryV =", data_string_size, &result);
      if (ret) {
        dataStatus.v_bplus = result;
        ret = false;
      }
    }
    for (uint8_t i = 0; i < 4; i++) {
      if (!ret) {
        char keyword[5];
        keyword[0] = 'T';
        keyword[1] = 'S';
        keyword[2] = '0' + i + 1;
        keyword[3] = ':';
        keyword[4] = 0;
        char buffer[256 + 1];
        memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
        data_string_size = 4;
        result = 0;
        ret = dataFind((const char*) buffer, keyword, data_string_size, &result);
        if (ret) {
          dataStatus.ts[i] = result;
          ret = false;
        }
      }
    }

    if (!ret) {
      char buffer[256 + 1];
      memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
      data_string_size = 6;
      result = 0;
      ret = dataFind((const char*) buffer, "Vref(raw):", data_string_size, &result);
      if (ret) {
        dataStatus.v_raw = result;
        ret = false;
      }
    }

    if (!ret) {
      char buffer[256 + 1];
      memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
      data_string_size = 6;
      result = 0;
      ret = dataFind((const char*) buffer, "VbiasmV:", data_string_size, &result);
      if (ret) {
        dataStatus.v_bias = result;
        ret = false;
      }
    }

    if (!ret) {
      char buffer[256 + 1];
      memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
      data_string_size = 6;
      result = 0;
      ret = dataFind((const char*) buffer, "VpackmV:", data_string_size, &result);
      if (ret) {
        dataStatus.v_pack = result;
        ret = false;
      }
    }

    if (!ret) {
      char buffer[256 + 1];
      memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
      data_string_size = 6;
      result = 0;
      ret = dataFind((const char*) buffer, "VchgmV:", data_string_size, &result);
      if (ret) {
        dataStatus.v_chg = result;
        ret = false;
      }
    }

    if (!ret) {
      char buffer[256 + 1];
      memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
      data_string_size = 6;
      result = 0;
      ret = dataFind((const char*) buffer, "V12VmV:", data_string_size, &result);
      if (ret) {
        dataStatus.v_12v = result;
        ret = false;
      }
    }

    if (!ret) {
      char buffer[256 + 1];
      memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
      int results[8];
      ret = binaryStringFind((const char*)buffer, "ChgTrips: b", results);
      if (ret) {
        memcpy(dataStatus.charge_trips, results, sizeof(results));
        ret = false;
      }
    }

    if (!ret) {
      char buffer[256 + 1];
      memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
      int results[8];
      ret = binaryStringFind((const char*)buffer, "DischTrips: b", results);
      if (ret) {
        memcpy(dataStatus.discharge_trips, results, sizeof(results));
        ret = false;
      }
    }

    if (!ret) {
      char buffer[256 + 1];
      memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
      char results[15];
      ret = floatStringFind((const char*)buffer, "Current:", results);
      if (ret) {
        memcpy(dataStatus.current_float_string, results, sizeof(results));
        ret = false;
      }
    }

    for (uint8_t i = 0; i < 14; i++) {
      if (!ret) {
        if (i < 10) {
          char keyword[4];
          keyword[0] = '[';
          keyword[1] = '0' + i;
          keyword[2] = ']';
          keyword[3] = 0;
          char buffer[256 + 1];
          memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
          data_string_size = 6;
          result = 0;
          ret = dataFind((const char*) buffer, keyword, data_string_size, &result);
          if (ret) {
            dataStatus.v_cell[i] = result;
            ret = false;
          }
        }
        else {
          char keyword[5];
          keyword[0] = '[';
          keyword[1] = '0' + (i / 10);
          keyword[2] = '0' + (i % 10);
          keyword[3] = ']';
          keyword[4] = 0;
          char buffer[256 + 1];
          memcpy(buffer, &rx_H1_buffer[(rx_buffer_flag - 1) * 256], 256);
          data_string_size = 6;
          result = 0;
          ret = dataFind((const char*) buffer, keyword, data_string_size, &result);
          if (ret) {
            dataStatus.v_cell[i] = result;
            ret = false;
          }
        }

      }
    }




    if (rx_buffer_flag == 1) {
      memset(&rx_H1_buffer[0], 0, 256);
    }
    else if (rx_buffer_flag == 2) {
      memset(&rx_H1_buffer[256], 0, 256);
    }

    rx_buffer_flag = 0;
  }



  //  while (H2Serial.available() > 0) {
  //    rx_H2_buffer[rx_H2_buffer_index++] = H2Serial.read();
  //
  //
  //    if (rx_H2_buffer_index >= 32) {
  //      rx_H2_buffer_index = 0;
  //    }
  //    //Serial.printf("H2Serial: %02X\n", byteFromSerial);
  //  }
  //  if (rx_H2_buffer_index > 0) {
  //    H2Serial.printf("H2Serial len=%d\n", rx_H2_buffer_index);
  //    //    modbus_active_time = millis();
  //    if (rx_H2_buffer_index == 1 && rx_H2_buffer[0] == 0x11) {
  //      H2Serial.printf("H2Serial\n");
  //    }
  //
  //
  //    rx_H2_buffer_index = 0;
  //  }

  while (Serial.available() > 0) {
    rx_S1_buffer[rx_S1_buffer_index++] = Serial.read();


    if (rx_S1_buffer_index >= 32) {
      rx_S1_buffer_index = 0;
    }
    //Serial.printf("S1Serial: %02X\n", byteFromSerial);
  }
  if (rx_S1_buffer_index > 0) {
    //Serial.printf("S1Serial len=%d\n", rx_S1_buffer_index);
    //    modbus_active_time = millis();
    if (rx_S1_buffer_index == 1 && rx_S1_buffer[0] == 'R') {
      //Serial.printf("S1Serial\n");
      int bytes_len = sizeof(dataStatus) + 2;
      uint8_t msg[bytes_len];
      memcpy(msg, &dataStatus, sizeof(dataStatus));
      uint16_t crc16 = MODBUS_CRC16_V1(msg, bytes_len - 2);
      msg[bytes_len - 2] = (uint8_t)(crc16 & 0xFF);
      msg[bytes_len - 1] = (uint8_t)(crc16 / 256);

      Serial.write(msg, bytes_len);
    }


    rx_S1_buffer_index = 0;
  }

}
