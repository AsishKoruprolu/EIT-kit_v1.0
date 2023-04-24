#include "EITKitArduino.h"

EITKitArduino *eit = NULL;

void setup() {
  // put your setup code here, to run once:
  eit = new EITKitArduino(16,1,2,AD,AD,0); //intialize an object of the class EITKitArduino
  // pinMode(A5,OUTPUT);
  // pinMode(26,OUTPUT);
  // SPIClass * vspi = new SPIClass(VSPI);
  // SPIClass * hspi = new SPIClass(HSPI);

  // hspi->setFrequency(SPI_FREQ_SLOW);
  // hspi->begin(HSPI_SCK_PIN, MISO, HSPI_MOSI_PIN); //Explicitly mentioning the SCLK and SDI(MOSI) pins so that the bus is not initialized with default pin settings
  // hspi->setHwCs(false);
  // vspi->setFrequency(SPI_FREQ_FAST);
  // vspi->begin(VSPI_SCK_PIN, MISO, VSPI_MOSI_PIN);
  // vspi->setHwCs(false);
  
  // pinMode(CHIP_SEL_MEAS, OUTPUT);

  // AD5270_LockUnlock(CHIP_SEL_MEAS, 0);

  // digitalWrite(CHIP_SEL_MEAS, LOW);
  // delay(100);
  // AD5270_Set(CHIP_SEL_MEAS, 128);
  // delay(100);
  // AD5270_Shutdown(CHIP_SEL_MEAS, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(A5,HIGH);
  // delay(10);
  // digitalWrite(A5,LOW);
  // delay(10);
  // digitalWrite(26,HIGH);
  // delay(10);
  // digitalWrite(26,LOW);
  // delay(10);
}

// void AD5270_LockUnlock(const int chip_select, uint8_t lock){
//   AD5270_Write(chip_select, CMD_WR_CTRL, lock ? 0 : 0x002);
// }

// /* Enable/disable hardware shutdown */
// void AD5270_Shutdown(const int chip_select, uint8_t shutdown){
//   AD5270_Write(chip_select, CMD_SHTDN, shutdown ? 1 : 0);
// }

// /* Set the value of the digital rheostat - range is 0-0x3FF (0-100kOhm) */
// void AD5270_Set(const int chip_select, uint16_t val)
// {
//   AD5270_Write(chip_select, CMD_WR_RDAC, val);
// }

// /* Write a 12-bit data word into a register. Register addresses are 4 bits */
// void AD5930_Write(uint8_t reg, uint16_t data){
//   uint16_t data_word = ((reg & 0x0F) << 12) | (data & 0x0FFF);

//   #if defined(ARDUINO_ARCH_ESP32)
//   vspi_write_word(CHIP_SEL_AD5930, data_word, MSBFIRST, SPI_MODE1);
//   #elif defined(__IMXRT1062__) // for Teensy 4.0
//   digitalWrite(CHIP_SEL_AD5930, LOW);
//   spi_write(MOSI_PIN, SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
//   digitalWrite(CHIP_SEL_AD5930, HIGH);
//   #endif
// }

// void vspi_write_word(const int chip_select, uint16_t data_to_send, const uint8_t bit_order, const uint8_t mode) {
//   vspi->beginTransaction(SPISettings(SPI_FREQ_FAST, bit_order, mode));
//   digitalWrite(chip_select, LOW);   //pull SS slow to prep other end for transfer
//   vspi->transfer16(data_to_send);
//   digitalWrite(chip_select, HIGH);  //pull ss high to signify end of data transfer
//   vspi->endTransaction();
// }
