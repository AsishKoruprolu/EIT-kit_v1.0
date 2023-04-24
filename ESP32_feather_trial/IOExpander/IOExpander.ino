

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
  #define HSPI_MOSI   27
  #define HSPI_SCLK   25
  #define HSPI_SS     33

#define SPI_FREQ_FAST      2000000

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


#define ADC_PWRDN 0 //Power down pin of ADC
#define ADC_OE  1 //Output Enable pin of ADC

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif

static const int spiClk = 1000000; // 1 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

void setup() {
  Serial.begin(115200);
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  MCP23S17 IOExpander(vspi, CHIP_SEL_MCP23S17, 0);
  
  //clock miso mosi ss
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI);; //SCLK, MISO, MOSI, SS
  IOExpander.begin();
  vspi->setHwCs(false);

  // Set CS back to high in case HSPI initialization overwrote it
  pinMode(CHIP_SEL_MCP23S17, OUTPUT);
  digitalWrite(CHIP_SEL_MCP23S17, HIGH);

   // For the MUX connected via MC23S17 on port A
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

  IOExpander.pinMode(ADC_PWRDN, OUTPUT);
  IOExpander.digitalWrite(ADC_PWRDN, LOW);
  IOExpander.pinMode(ADC_OE, OUTPUT);
  IOExpander.digitalWrite(ADC_OE, LOW);

  Serial.println("Halwa");



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
