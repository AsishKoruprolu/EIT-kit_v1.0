

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
  #define VSPI_SS     32

  #define HSPI_MISO   26
  #define HSPI_MOSI   13
  #define HSPI_SCLK   14
  #define HSPI_SS     33

#define SPI_FREQ_FAST      2000000
#define SPI_FREQ_SLOW      500000
#define MAX_ELECTRODES     64 // maximum electrodes that can be used

// Chip sel for MCP23S18
  #define CHIP_SEL_MCP23S17  12

/* IO expander pin mappings */

/* Chip Select for MUX for electrodes 0-31 */
#define CHIP_SEL_MUX_VN   7    // Chip select pin for VMEAS_2 --> CS_MUX4 on EIT-board
#define CHIP_SEL_MUX_VP  6    // Chip select pin for VMEAS_1 --> CS_MUX2 on EIT-Board
#define CHIP_SEL_MUX_SINK    5    // Chip select for I_IN2 --> CS_MUX3 on EIT-board
#define CHIP_SEL_MUX_SRC    9    // Chip select for I_IN1 --> CS_MUX1 on EIT_board

/* Chip Select for MUX for electrodes 32-63 */
#define CHIP_SEL_MUX_VN_2  3    // Chip select pin for VMEAS_2 --> CS_MUX8 on EIT-board
#define CHIP_SEL_MUX_VP_2  8    // Chip select pin for VMEAS_1 --> CS_MUX6 on EIT-Board
#define CHIP_SEL_MUX_SINK_2    2    // Chip select for I_IN2 --> CS_MUX7 on EIT-board
#define CHIP_SEL_MUX_SRC_2    4     // Chip select for I_IN1 --> CS_MUX5 on EIT_board


#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif

static const int spiClk = 1000000; // 1 MHz
static const uint8_t elec_to_mux[MAX_ELECTRODES] = { 9, 10, 11, 8, 7, 6, 5, 4, 3, 2, 1, 0, 12, 13, 14, 15, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16};

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;


void setup() {
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  MCP23S17 IOExpander(vspi, CHIP_SEL_MCP23S17, 0);
  
  //clock miso mosi ss
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI); //SCLK, MISO, MOSI, SS
  IOExpander.begin();
  vspi->setHwCs(false);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI); //SCLK, MISO, MOSI, SS

  pinMode(CHIP_SEL_MCP23S17, OUTPUT);
  digitalWrite(CHIP_SEL_MCP23S17, HIGH);

  // For the MUX connected via MCP23S17 on port A
  IOExpander.pinMode(CHIP_SEL_MUX_SRC, OUTPUT);
  IOExpander.pinMode(CHIP_SEL_MUX_SINK, OUTPUT);
  IOExpander.pinMode(CHIP_SEL_MUX_VP, OUTPUT);
  IOExpander.pinMode(CHIP_SEL_MUX_VN, OUTPUT);
  IOExpander.digitalWrite(CHIP_SEL_MUX_SRC, HIGH);
  IOExpander.digitalWrite(CHIP_SEL_MUX_SINK, HIGH);
  IOExpander.digitalWrite(CHIP_SEL_MUX_VP, HIGH);
  IOExpander.digitalWrite(CHIP_SEL_MUX_VN, HIGH);

  IOExpander.pinMode(CHIP_SEL_MUX_SRC_2, OUTPUT);
  IOExpander.pinMode(CHIP_SEL_MUX_SINK_2, OUTPUT);
  IOExpander.pinMode(CHIP_SEL_MUX_VP_2, OUTPUT);
  IOExpander.pinMode(CHIP_SEL_MUX_VN_2, OUTPUT);
  IOExpander.digitalWrite(CHIP_SEL_MUX_SRC_2, HIGH);
  IOExpander.digitalWrite(CHIP_SEL_MUX_SINK_2, HIGH);
  IOExpander.digitalWrite(CHIP_SEL_MUX_VP_2, HIGH);
  IOExpander.digitalWrite(CHIP_SEL_MUX_VN_2, HIGH);
  
  mux_write(CHIP_SEL_MUX_VN,0,0,IOExpander);//VMEAS_2
  mux_write(CHIP_SEL_MUX_SINK,7,1,IOExpander);//IOUT_2
  mux_write(CHIP_SEL_MUX_SRC,3,1,IOExpander);//IOUT_1
  mux_write(CHIP_SEL_MUX_VP,30,0,IOExpander);//VMEAS_1
  // for(int i = 1; i <= 18; i=i+1) {
  //   mux_write(CHIP_SEL_MUX_VN,i,1,IOExpander);
  //   delay(5000);
  // }

}

// the loop function runs over and over again until power down or reset
void loop() {
  //use the SPI buses
  // spiCommand(vspi, 0x1C02); // junk data to illustrate usage
  // spiCommand(hspi, 0b11001100);
  // delay(100);
  
}

void spiCommand(SPIClass *spi, uint16_t data) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  spi->transfer16(data);
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
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

