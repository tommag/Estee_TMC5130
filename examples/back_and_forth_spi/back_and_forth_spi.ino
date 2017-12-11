#include <Arduino.h>
#include <SPI.h>
#include <Estee_TMC5130.h>

uint8_t TMC_CS = PIN_A5;  // Chip select, Active LOW. next to SCK/MOSI/MISO on the Adafruit Feather M0
uint8_t TMC_EN = 10;      // Drive enable pin. Active LOW. Optional, you can also just tie to ground
uint8_t BLE_CS = 8;       // Hidden pin #8 which controls the SS line for the Bluetooth module SPI comms

Estee_TMC5130_SPI tmc = Estee_TMC5130_SPI(TMC_CS);

void setup()
{
  // init serial coms
  Serial.begin(9600);
  SPI.begin();

  // chip select & drive enable
  pinMode(TMC_EN, OUTPUT);
  digitalWrite(TMC_EN, HIGH); // disabled for start
  pinMode(TMC_CS, OUTPUT);
  digitalWrite(TMC_EN, HIGH);

  // disable BLE talking on SPI bus
#if defined(ARDUINO_SAMD_ZERO)
  pinMode(BLE_CS, OUTPUT);
  digitalWrite(BLE_CS, HIGH);
#endif

  // This sets the motor currents for HOLD & RUN, as well as the natural motor direction for positive moves
  tmc.begin(4, 4, Estee_TMC5130::NORMAL_MOTOR_DIRECTION);

  // drive *MUST* be disabled when testing frequency scaling
//  digitalWrite(TMC_EN, HIGH);
//  float freq = tmc.updateFrequencyScaling();
//  Serial.print("frequency scaling: ");
//  Serial.println(freq);
//  delay(5000);

  // ramp definition
  uint32_t onerev = 200*256;
  tmc.writeRegister(TMC5130_Reg::VSTART, 0x0);
  tmc.writeRegister(TMC5130_Reg::A_1, 1000);
  tmc.writeRegister(TMC5130_Reg::V_1, 50000);
  tmc.writeRegister(TMC5130_Reg::AMAX, 500);
  tmc.writeRegister(TMC5130_Reg::VMAX, 200000);
  tmc.writeRegister(TMC5130_Reg::DMAX, 700);
  tmc.writeRegister(TMC5130_Reg::D_1, 1400);
  tmc.writeRegister(TMC5130_Reg::VSTOP, 10);
  tmc.writeRegister(TMC5130_Reg::TZEROWAIT, 0);
  tmc.writeRegister(TMC5130_Reg::RAMPMODE, TMC5130_Reg::POSITIONING_MODE);

  Serial.println("starting up");

  // enable the drive
  digitalWrite(TMC_EN, LOW);
}

// periodic delay
uint32_t t_echo = millis();
uint32_t t_dirchange = millis();
bool dir = false;

void loop()
{
  uint32_t now = millis();

  // every n seconds or so...
  if ( now - t_dirchange > 3000 )
  {
    t_dirchange = now;

    // reverse direction
    dir = !dir;
    tmc.writeRegister(TMC5130_Reg::XTARGET, dir?(200*256):0);  // 1 full rotatation = 200s/rev * 256microsteps
  }

  // print out current position
  if( now - t_echo > 100 )
  {
    t_echo = now;

    // get the current target position
    int32_t xactual = 0, vactual = 0;
    xactual = tmc.readRegister(TMC5130_Reg::XACTUAL);
    vactual = tmc.readRegister(TMC5130_Reg::VACTUAL);

    Serial.print("xpos,v:");
    Serial.print(xactual);
    Serial.print(", ");
    Serial.println(vactual);
  }

}
