

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

// Define ALTERNATE_PINS to use non-standard GPIO pins for SPI bus

  #define VSPI_MISO   MISO
  #define VSPI_MOSI   23
  #define VSPI_SCLK   18
  // #define VSPI_SS     32

  #define HSPI_MISO   26
  #define HSPI_MOSI   27
  #define HSPI_SCLK   25
  // #define HSPI_SS     33

  #define SPI_FREQ_FAST      2000000

#define CHIP_SEL_DRIVE     32   // Chip select pin for driving digital rheostat potentiometer
#define CHIP_SEL_MEAS      33   // Chip select pin for measuring digital rheostat potentiometer

// AD5270 commands - new digital potentiometer
#define CMD_WR_RDAC        0x01
#define CMD_RD_RDAC        0x02
#define CMD_ST_RDAC        0x03
#define CMD_RST            0x04
#define CMD_RD_MEM         0x05
#define CMD_RD_ADDR        0x06
#define CMD_WR_CTRL        0x07
#define CMD_RD_CTRL        0x08
#define CMD_SHTDN          0x09

#define R_FULLSCALE        100000 //100kohm fullscale resistance


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
  
  //clock miso mosi ss
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI); //SCLK, MISO, MOSI, SS

  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI); //SCLK, MISO, MOSI, SS


  pinMode(CHIP_SEL_DRIVE, OUTPUT);
  digitalWrite(CHIP_SEL_DRIVE, HIGH);
  pinMode(CHIP_SEL_MEAS, OUTPUT);
  digitalWrite(CHIP_SEL_MEAS, HIGH);
  // pinMode(vspi->pinSS(), OUTPUT); //VSPI SS
  // pinMode(hspi->pinSS(), OUTPUT); //HSPI SS
  // spiCommand(vspi, 0x1C02);
  // delay(10);
  // spiCommand(vspi, 0x07FF);
  // delay(10);
  // spiCommand(vspi, 0x0800);

  AD5270_LockUnlock(vspi,MSBFIRST, SPI_MODE1,CHIP_SEL_DRIVE,0);
  delay(1);
  // AD5270_Set(vspi,MSBFIRST, SPI_MODE1,CHIP_SEL_DRIVE,0x03FF);
  AD5270_SetResistance(vspi,MSBFIRST,SPI_MODE1,CHIP_SEL_DRIVE,25000); //Max res. = 100kOhms
  delay(1);
  // AD5270_Shutdown(vspi,MSBFIRST,SPI_MODE1,CHIP_SEL_DRIVE,0);
  
  AD5270_LockUnlock(vspi,MSBFIRST, SPI_MODE1,CHIP_SEL_MEAS,0);
  delay(1);
  // AD5270_Set(vspi,MSBFIRST, SPI_MODE1,CHIP_SEL_DRIVE,0x03FF);
  AD5270_SetResistance(vspi,MSBFIRST,SPI_MODE1,CHIP_SEL_MEAS,25000); //Max res. = 100kOhms
  delay(1);


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
  spi->beginTransaction(SPISettings(SPI_FREQ_FAST, bit_order, mode));
  digitalWrite(chip_select, LOW); //pull SS slow to prep other end for transfer
  spi->transfer16(data);
  digitalWrite(chip_select, HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
}

void AD5270_Write(SPIClass *spi, uint8_t cmd, const uint8_t bit_order, const uint8_t mode, const int chip_select, uint16_t data){
  uint16_t data_word = ((cmd & 0x0F) << 10) | (data & 0x03FF);
  // Serial.print("AD5270 Write command: "+String(data_word,HEX));
  spiCommand(spi, data_word, bit_order, mode, chip_select);
}

/* Enable/disable rheostat value changes */
void AD5270_LockUnlock(SPIClass *spi, const uint8_t bit_order, const uint8_t mode, const int chip_select, uint8_t lock){
  AD5270_Write(spi, CMD_WR_CTRL, bit_order, mode, chip_select, lock ? 0 : 0x002);
}

/* Enable/disable hardware shutdown */
void AD5270_Shutdown(SPIClass *spi, const uint8_t bit_order, const uint8_t mode, const int chip_select, uint8_t shutdown){
  AD5270_Write(spi, CMD_SHTDN, bit_order, mode, chip_select, shutdown ? 1 : 0);
}

/* Set the value of the digital rheostat - range is 0-0x3FF (0-100kOhm) */
void AD5270_Set(SPIClass *spi, const uint8_t bit_order, const uint8_t mode, const int chip_select, uint16_t val)
{
  AD5270_Write(spi, CMD_WR_RDAC, bit_order, mode, chip_select, val);
}

/* Set the desired value of resistance in kOhm (<100kOhm) */
void AD5270_SetResistance(SPIClass *spi, const uint8_t bit_order, const uint8_t mode, const int chip_select, uint32_t val)
{
  uint32_t rdac_data = ((val * 1.0/R_FULLSCALE) * 0xFF); //AD5271 has 8-bit resolution
  uint16_t code = rdac_data & 0x03FFF;
  uint16_t rdac_code = code << 2;

  AD5270_Set(spi, bit_order, mode, chip_select, rdac_code);
}

