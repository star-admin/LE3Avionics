#include <MCP23S17.h>
#include <ADS126X.h>
#include <SPI.h>

#define SENSE_CS_1 40
#define SENSE_DRDY_1 4 
#define SENSE_CS_2 42
#define SENSE_DRDY_2 35
#define PYRO_CS_1 16
#define PYRO_CS_2 4
#define MOSI 5
#define MISO 41
#define CLK 13

#define VALVE_FUP 12   // Fuel upstream solenoid open
#define VALVE_FDP 10  // Fuel downstream solenoid open
#define VALVE_OUP 13    // LOX upstream solenoid open
//#define VALVE_ODP 33  // LOX downstream solenoid open
#define FUELVENT 14 
#define LOXVENT 15
#define PRESSURELINE 11

#define FUELMAIN 9 //actuators
#define LOXMAIN 8 //actuators

//#define UP_PRESSURE 10

#define PT_I 10  // INJECTOR
#define PT_P 5 //UPSTREAM PRESSURE FOR BOTH FUEL AND OXIDIZER
#define PT_O1 4   // OX TANK PRESSURE 
#define PT_F1 2 // FUEL TANK PRESSURE
#define PT_O2 1   // LOX DOWNSTREAM pressure
#define PT_F2 9 // FUEL DOWNSTREAM pressure

ADS126X SENSE_1;
MCP23S17* PYRO_1_MCP;
//MCP23S17* PYRO_2_MCP;


// Pressure transducers
// Fuel path



void setup() {
  Serial.begin(115200);
  delay(500);

  SPI.begin(CLK, MISO, MOSI, -1);
  delay(500);

  PYRO_1_MCP = new MCP23S17(PYRO_CS_1, 0x00, &SPI);
  //PYRO_2_MCP = new MCP23S17(PYRO_CS_2, 0x00, &SPI);

  SENSE_1.begin(SENSE_CS_1);
  SENSE_1.startADC1();
  SENSE_1.setRate(ADS126X_RATE_1200);
  SENSE_1.setReference(ADS126X_REF_NEG_VSS, ADS126X_REF_POS_VDD);
  uint8_t power = SENSE_1.readRegister(ADS126X_POWER);
  power &= ~(1 << 2);
  SENSE_1.writeRegister(ADS126X_POWER);

  bool status = PYRO_1_MCP->begin();
  Serial.println(status ? "Started Pyro 1 GPIO EX" : "Failed to start Pyro 1 GPIO EX");
  delay(100);

  for (int pin = 0; pin < 16; pin++) {
    PYRO_1_MCP->pinMode8(pin, 0x00);  // Set pin as output (0 = output)
  }

  // Set all pins HIGH
  for (int pin = 0; pin < 16; pin++) {
    PYRO_1_MCP->write1(pin, 1);  // Set pin HIGH
  }


  PYRO_1_MCP->write1(PRESSURELINE, 1);
  //delay(50);
  PYRO_1_MCP->write1(VALVE_FUP, 1);
  //delay(50);
  PYRO_1_MCP->write1(VALVE_FDP, 1);
  //delay(50);
  PYRO_1_MCP->write1(VALVE_OUP, 1);
  //delay(50);
  PYRO_1_MCP->write1(FUELVENT, 0);
  //delay(50);
  PYRO_1_MCP->write1(LOXVENT, 0);
  //delay(50);
  PYRO_1_MCP->write1(FUELMAIN, 0);
  //delay(50);
  PYRO_1_MCP->write1(LOXMAIN, 0);

}

float readPT(int channel) {
  delay(10);
  SENSE_1.readADC1(channel, ADS126X_AINCOM);
  delay(10);
  long raw = SENSE_1.readADC1(channel, ADS126X_AINCOM);
  float voltage = (float)raw * 5.0 / 2147483648.0;
  return voltage;
}

void loop() {

  //delay(50);


  //Serial.print(readPT(PT_P)); Serial.print(", ");
  Serial.print(readPT(PT_F1)); Serial.print(" ");
  Serial.print(readPT(PT_O1)); Serial.print(" ");
  Serial.print(readPT(PT_O2)); Serial.print(" ");
  Serial.print(readPT(PT_F2)); Serial.print(" ");
  Serial.print(readPT(PT_I)); Serial.println(" ");
  delay(50);

}

