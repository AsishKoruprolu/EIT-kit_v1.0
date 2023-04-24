

/* The ESP32 has four SPi buses, however as of right now only two of
 * them are available to use, HSPI and VSPI. Simply using the SPI API 
 * as illustrated in Arduino examples will use VSPI, leaving HSPI unused.
 * 
 * However if we simply intialise two instance of the SPI class for both
 * of these buses both can be used. However when just using these the Arduino
 * way only will actually be outputting at a time.
 * 
 * Logic analyser capture is in the same folder as this example as
 * "multiple_bus_output.png"
 * 
 * created 30/04/2018 by Alistair Symonds
 */
#include <SPI.h>
#include "MCP23S17.h"

// Define ALTERNATE_PINS to use non-standard GPIO pins for SPI bus

  #define VSPI_MISO   MISO
  #define VSPI_MOSI   23
  #define VSPI_SCLK   18
  // #define VSPI_SS     32

#define SPI_FREQ_FAST      2000000
#define SPI_FREQ_SLOW      500000
#define MAX_ELECTRODES     64 // maximum electrodes that can be used

#define AD5930_CLK_FREQ    1000000
#define TEST_FREQ          150000

#define CHIP_SEL_AD5930    25   // Chip select pin for AD5930: Signal generator

#define AD5930_MSBOUT_PIN  35
// AD5930 register addresses - signal generator
#define CTRL_REG           0x00
#define NUM_INCR_REG       0x01
#define DFREQ_LOW_REG      0x02
#define DFREQ_HIGH_REG     0x03
#define TIME_INCR_REG      0x04
#define TIME_BURST_REG     0x08
#define SFREQ_LOW_REG      0x0C
#define SFREQ_HIGH_REG     0x0D


#define AD5930_INT_PIN     4  // Pulse high to reset internal state machine
#define AD5930_CTRL_PIN    27  // Pull high to start frequency sweep. Pull low to end the burst. Pull high again to increment frequency
#define AD5930_STANDBY_PIN 21  // Pull high to power down



#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif

static const int spiClk = 1000000; // 1 MHz
// static const uint8_t elec_to_mux[MAX_ELECTRODES] = { 9, 10, 11, 8, 7, 6, 5, 4, 3, 2, 1, 0, 12, 13, 14, 15, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16};

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;


void setup() {
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  
  //clock miso mosi ss
  vspi->setFrequency(SPI_FREQ_FAST);
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI); //SCLK, MISO, MOSI, SS
  vspi->setHwCs(false);

  pinMode(CHIP_SEL_AD5930, OUTPUT);
  digitalWrite(CHIP_SEL_AD5930, HIGH);


  // For the AD5930 signal generator
  pinMode(AD5930_INT_PIN, OUTPUT);
  pinMode(AD5930_CTRL_PIN, OUTPUT);
  pinMode(AD5930_STANDBY_PIN, OUTPUT);
  // pinMode(AD5930_MSBOUT_PIN, INPUT);
  digitalWrite(AD5930_INT_PIN, LOW); 
  digitalWrite(AD5930_CTRL_PIN, LOW);
  digitalWrite(AD5930_STANDBY_PIN, LOW);
  delay(20);

  AD5930_Write(vspi, CTRL_REG, 0b011101110011); //Control Register Programming
  delay(10);
  AD5930_Set_Start_Freq(vspi, TEST_FREQ);
  delay(10);
  AD5930_Write(vspi, NUM_INCR_REG, 0x005);
  delay(10);
  AD5930_Write(vspi, DFREQ_LOW_REG, 0x001);
  delay(10);
  AD5930_Write(vspi, DFREQ_HIGH_REG, 0x000);
  delay(10);
  AD5930_Write(vspi, TIME_INCR_REG, 0x0A0);
  delay(10);
  AD5930_Write(vspi, TIME_BURST_REG, 0x050); 
  
  /* Start the frequency sweep */
  digitalWrite(AD5930_CTRL_PIN, HIGH);
  delay(3);
  digitalWrite(AD5930_CTRL_PIN, LOW);
  delay(10);
  digitalWrite(AD5930_CTRL_PIN, HIGH);
  delay(3);
  digitalWrite(AD5930_CTRL_PIN, LOW);
  delay(10);

}

// the loop function runs over and over again until power down or reset
void loop() {
  //use the SPI buses
  // spiCommand(vspi, 0x1C02); // junk data to illustrate usage
  // spiCommand(hspi, 0b11001100);
  // delay(100);
  
}

void spiCommand(SPIClass *spi, uint16_t data, const uint8_t bit_order, const uint8_t mode,const int chip_select) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(SPI_FREQ_FAST, MSBFIRST, SPI_MODE0));
  digitalWrite(chip_select, LOW); //pull SS slow to prep other end for transfer
  spi->transfer16(data);
  digitalWrite(chip_select, HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
}

// HSPI addresses devices through IO expander
void hspi_write_byte(const int chip_select, uint8_t data_to_send, const uint8_t bit_order, const uint8_t mode, MCP23S17 &IOExpander){
  hspi->beginTransaction(SPISettings(SPI_FREQ_SLOW, bit_order, mode));
  IOExpander.digitalWrite(chip_select, LOW);
  hspi->transfer(data_to_send);
  IOExpander.digitalWrite(chip_select, HIGH);
  hspi->endTransaction();
}

void mux_write(const int chip_select, uint8_t pin_sel, uint8_t enable, MCP23S17 &IOExpander){
  uint8_t value;
  if (enable) {
      value = pin_sel & 0x1F;
   } else {
      value = 0xC0 | (pin_sel & 0x1F);
   }
   hspi_write_byte(chip_select, value, MSBFIRST, SPI_MODE1, IOExpander);
}

/* Write a 12-bit data word into a register. Register addresses are 4 bits */
void AD5930_Write(SPIClass *spi, uint8_t reg, uint16_t data){
  uint16_t data_word = ((reg & 0x0F) << 12) | (data & 0x0FFF);
  // vspi_write_word(CHIP_SEL_AD5930, data_word, MSBFIRST, SPI_MODE1);
  spiCommand(spi, data_word, MSBFIRST, SPI_MODE1, CHIP_SEL_AD5930);
}

/* Program the given frequency (in Hz) */
void AD5930_Set_Start_Freq(SPIClass *spi, uint32_t freq){
  uint32_t scaled_freq = (freq * 1.0 / AD5930_CLK_FREQ) * 0x00FFFFFF;
  uint16_t freq_low = scaled_freq & 0x0FFF;
  uint16_t freq_high = (scaled_freq >> 12) & 0x0FFF;

  AD5930_Write(spi, SFREQ_LOW_REG, freq_low);
  AD5930_Write(spi, SFREQ_HIGH_REG, freq_high);
}

void vspi_write_word(const int chip_select, uint16_t data_to_send, const uint8_t bit_order, const uint8_t mode) {
  vspi->beginTransaction(SPISettings(SPI_FREQ_FAST, bit_order, mode));
  digitalWrite(chip_select, LOW);   //pull SS slow to prep other end for transfer
  vspi->transfer16(data_to_send);
  digitalWrite(chip_select, HIGH);  //pull ss high to signify end of data transfer
  vspi->endTransaction();
}

