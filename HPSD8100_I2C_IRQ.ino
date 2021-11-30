// Include Arduino Wire library for I2C, URL: https://github.com/PaulStoffregen/Wire
#include <Wire.h>

// Include CRC calculation library, URL: https://github.com/RobTillaart/CRC
#include "CRC.h"

// Include softwareSerial library, URL: https://github.com/PaulStoffregen/SoftwareSerial
#include <SoftwareSerial.h>
#define RxPin 0
#define TxPin 1
SoftwareSerial portOne(RxPin,TxPin);

// Define Slave I2C Address
#define SLAVE_ADDR 117         // Adress: 0x75

float Coeff1 [13];             // Array of pressure calibration coefficients
int16_t Coeff2 [4];            // Array of temperature calibration coefficients
void(* resetFunc) (void) = 0;  // Reset fuction at address 0

#define interruptPin 19        // Set interrupt pin

struct MeasuredData{ float temp; float pressure; };
boolean EOC = false;

int inter = 0;

void setup() {
  Wire.begin();                     // Initialize I2C communications as Master
  portOne.begin(9600);              // Setup serial monitor
  HPSD8100_init();                  // Device initialization
  pinMode(interruptPin, INPUT);     // Set interrupt pin as input
  attachInterrupt(digitalPinToInterrupt(interruptPin), EndOfConversion, HIGH);
}

void loop() {
  if(EOC){
  struct MeasuredData measurements = HPSD_measure();                      // Get value for temperature and pressure
  float T = measurements.temp;                                            // Calculated value of temperature
  float P = measurements.pressure;                                        // Calculated value of pressure
  String string_T = String (T, 1);                                        // Convert temperature float value to string with 1 decimal place
  String string_P = String (P, 2);                                        // Convert pressure float value to string with 2 decimal places
  String string_serial = "T= " + string_T + " P= " + string_P + (char)0;  // String for serial output, Valid value
  portOne.println (string_serial);                                        // Print results
  EOC = false;                                                            // End of conversion flag is reset
  }
}

void EndOfConversion(){
  EOC= true;                                                              // End of conversion flag is set
}

void HPSD8100_init(){
  // Write reset sequence
  Wire.beginTransmission(SLAVE_ADDR);    // Begin transmitting to sensor
  Wire.write((byte)0xEC);                // Write 1 byte to register address 0xEC
  Wire.write((byte)0x00);                // Write any data to register 0xEC to Reset device
  Wire.endTransmission();                // End transmitting to sensor
  delay(200);

  Wire.beginTransmission(SLAVE_ADDR);    // Begin transmitting to sensor
  Wire.write((byte)0xFA);                // Write 1 byte to register adress 0xFA
  Wire.write((byte)0b00101000);          // Write 1 byte in Trim and test register, set SDO as End Of Conversion (EOC) signal
  Wire.endTransmission();                // End transmitting to sensor
  delay(200);

  Wire.beginTransmission(SLAVE_ADDR);    // Begin transmitting to sensor
  Wire.write((byte)0xFF);                // Write 1 byte to register adress 0xFF
  Wire.write((byte)0b10000000);          // Write 1 byte in Clock oscillator frequency trim register, enable SDO output
  Wire.endTransmission();                // End transmitting to sensor
  delay(200);
  
  // Write settings to device 
  Wire.beginTransmission(SLAVE_ADDR);    // Begin transmitting to sensor
  Wire.write((byte)0xED);                // Write 3 bytes, start address 0xED, followed by 0xEE, 0xEF
  Wire.write((byte)0x05);                // Write 1 byte in register 0xED to activate EEPROM for read and write
  Wire.write((byte)0x00);                // Write 1 byte in Configuration register 0xEE (default settings), set values according to datasheet
  //Wire.write((byte)0xFF);              // Write 1 byte in Control register 0xEF (default settings), set values according to datasheet, ultra high resolution
  Wire.write((byte)0xB7);                // Write 1 byte in Control register 0xEF (default settings), set values according to datasheet, standard resolution in normal mode
  Wire.endTransmission();                // End transmitting to sensor
  delay(200);
  
  // Read data from device EEPROM
  byte epprom_data [64];
  Wire.beginTransmission(SLAVE_ADDR);    // Begin transmitting to sensor
  Wire.write((byte)0x80);                // EEPROM start address
  Wire.endTransmission();                // End transmitting to sensor
  for (int8_t i=0; i < 63; i++){         // Read 63 registers (63 x 8 bytes)
    Wire.requestFrom(SLAVE_ADDR,1);
    while (Wire.available()) {
      epprom_data[i] = Wire.read();      // Store data from sensor to array
    }
  }
   delay(200);
   
  // Deactivate EEPROM
  Wire.beginTransmission(SLAVE_ADDR);    // Begin transmitting to sensor
  Wire.write((byte)0xED);                // Write 3 bytes, start address in 0xED (EEPROM control register)
  Wire.write((byte)0x00);                // Write 1 byte in register 0xED to deactivate EEPROM for read and write
  Wire.endTransmission();                // End transmitting to sensor
  delay(200);

  // Convert Calibration Coefficients from byte to float
  // Coeff. array {0:a0, 1:a1, 2:a2, 3:c00, 4:c01, 5:c10, 6:c11, 7:c02, 8:c20, 9:c12, 10:pmin, 11:pmax}
  for (int8_t i= 0; i < 12; i++){
    uint32_t x = (uint32_t)epprom_data[i*4]<<24|
                 (uint32_t)epprom_data[i*4+1]<<16|
                 (uint32_t)epprom_data[i*4+2]<<8|
                 (uint32_t)epprom_data[i*4+3];
    Coeff1 [i]= *(float*)&x;
  }

  // Convert Calibration coefficients from byte to integer 16
  // Coeff. array {0:tmin, 1:tmax, 2:tref}
  for (int8_t i= 0; i < 3; i++){
    int16_t x = (int16_t)epprom_data[48+i*2]<<8|
                (int16_t)epprom_data[49+i*2];
    Coeff2 [i]= *(int16_t*)&x;
  }

  byte crc= crc8((uint8_t *)epprom_data, 57, 0x07); // Calculate CRC
  if (crc != epprom_data[57]){                      // Compare calculated CRC with CRC value from EEPROM
    Serial.println("EEPROM read error");            // If communication error occured Error message is printed and device resets after 2s
    delay(2000);
    resetFunc();                                    // Software reset
  }
}

struct MeasuredData HPSD_measure (){               // Calculate temperature and pressure values
  
  struct MeasuredData measured;
  byte ADC_data [7];

  // Read ADC result: pressure MSB, LSB, XLSB, temperaure MSB, LSB, XLSB
  Wire.beginTransmission(SLAVE_ADDR);               // Begin transmitting to sensor
  Wire.write((byte)0xF1);                           // EEPROM start address
  Wire.endTransmission();                           // End transmitting to sensor
  
  for (int8_t i=0; i < 6; i++){           
    Wire.requestFrom(SLAVE_ADDR,1);
    while (Wire.available()) {
      ADC_data[i] = Wire.read();                    // Read ADC values
    }
  }

  // Convert 2x 3 bytes ADC values to type int32_t
  // ADC array {0:DP, 1:DT}
  int32_t ADC_value [] ={0, 0};

  for (int8_t i= 0; i < 2; i++){
    int32_t x  = (int32_t)ADC_data[i*3]<<16|
                 (int32_t)ADC_data[i*3+1]<<8|
                 (int32_t)ADC_data[i*3+2];
    ADC_value [i]= *(int32_t*)&x;
  }

  // Calculate temperature and pressure values
  float dt = Coeff1[0] + (Coeff1[1] + Coeff1[2] * ADC_value[1])* ADC_value[1];
  measured.temp = dt + Coeff2[2];                                                                        // Calculated temperature value
  measured.pressure = Coeff1[3] + (Coeff1[5] + Coeff1[6] * dt + Coeff1[8] * ADC_value[0])* ADC_value[0]   
                      + (Coeff1[4] + Coeff1[7] * dt + (Coeff1[9] * ADC_value[0])*dt)*dt;                 // Calculated pressure value
  delay(5);
  return measured;
}
