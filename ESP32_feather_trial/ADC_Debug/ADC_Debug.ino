

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

// Map ADC pins to GPIO pins
#define ADC_BIT0   27 // LSb
#define ADC_BIT1   26
#define ADC_BIT2   15
#define ADC_BIT3   4
#define ADC_BIT4   16
#define ADC_BIT5   17
#define ADC_BIT6   5
#define ADC_BIT7   19
#define ADC_BIT8   21
#define ADC_BIT9   22


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

  Serial.println("GPIO converted val: "+String(gpio_convert(gpio_read())));
  // IOExpander.digitalWrite(ADC_PWRDN, HIGH);
  // IOExpander.digitalWrite(ADC_OE, HIGH);
}

// the loop function runs over and over again until power down or reset
void loop() {

}

void spiCommand(SPIClass *spi, uint16_t data) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  spi->transfer16(data);
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
}


/* Read GPIO 0-31 (takes ~50.1ns) */
uint32_t gpio_read(){
  return REG_READ(GPIO_IN_REG);
}

/* Convert GPIO reading to 10-bit unsigned integer */
uint16_t gpio_convert(uint32_t gpio_reg){

  const uint8_t bit_to_gpio[10] = {ADC_BIT0, ADC_BIT1, ADC_BIT2, ADC_BIT3, ADC_BIT4, ADC_BIT5, ADC_BIT6, ADC_BIT7, ADC_BIT8, ADC_BIT9};
  uint16_t val = 0;
  
  for (int i = 0; i < 10; i++) {
    uint32_t gpio_val = gpio_reg & (1 << bit_to_gpio[i]);
    //Debug purpose
    // String adc_bit = "i: "+String(i)+"\t";
    // Serial.println(adc_bit);
    // Serial.println("Digital Read O/P: "+String(digitalRead(bit_to_gpio[i])));
    // Serial.println("gpio_reg: "+String(gpio_reg,HEX));
    // Serial.println("GPIO value: "+String(gpio_val,HEX));

    if (gpio_val) {
      uint32_t bit_val;
      if (bit_to_gpio[i] > i)
        bit_val = gpio_val >> (bit_to_gpio[i] - i);
      else
        bit_val = gpio_val << (i - bit_to_gpio[i]);
      val |= bit_val;
    }
  }
  return val;
}
