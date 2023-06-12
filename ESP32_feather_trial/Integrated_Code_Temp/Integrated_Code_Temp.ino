

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
#include "SPIFFS.h"

// Define ALTERNATE_PINS to use non-standard GPIO pins for SPI bus

  #define VSPI_MISO   MISO
  #define VSPI_MOSI   23
  #define VSPI_SCLK   18
  // #define VSPI_SS     32

  #define HSPI_MISO   36
  #define HSPI_MOSI   13
  #define HSPI_SCLK   14
  // #define HSPI_SS     33

#define SPI_FREQ_FAST      2000000
#define SPI_FREQ_SLOW      500000
#define MAX_ELECTRODES     64 // maximum electrodes that can be used  

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

//Trigger
#define TRIGGER A3

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
#define ADC_AVG 1
#define MAX_SAMPLES 200

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif

static const int spiClk = 1000000; // 1 MHz
static const uint8_t elec_to_mux[MAX_ELECTRODES] = { 9, 10, 11, 8, 7, 6, 5, 4, 3, 2, 1, 0, 12, 13, 14, 15, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16};

//uninitalised pointers to SPI objects
// SPIClass * vspi = NULL;
// SPIClass * hspi = NULL;

//Variables for edge detection
int TrigState = 0;        // current state of the button
int lastTrigState = 1;    // previous state of the button
int t1,t2,t3;
int cycles = 0;
uint16_t dc_values[2000];
uint32_t read_values[2000];
uint16_t time_period[2000];
double pk_pk_values[400];
int print_flag=0;

//Variables for automating data collection
char* start = "Start";
char* stop = "Stop";
uint8_t received[8] = {0};
int NewSerialData = 0;

SPIClass * vspi = new SPIClass(VSPI);
SPIClass * hspi = new SPIClass(HSPI);
MCP23S17 IOExpander(vspi, CHIP_SEL_MCP23S17, 0);


void setup() {
  Serial.begin(115200);
  // initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  // vspi = new SPIClass(VSPI);
  // hspi = new SPIClass(HSPI);
  // MCP23S17 IOExpander(vspi, CHIP_SEL_MCP23S17, 0);
  
  //clock miso mosi ss
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI); //SCLK, MISO, MOSI, SS
  hspi->setHwCs(false);

  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI); //SCLK, MISO, MOSI, SS
  IOExpander.begin();
  vspi->setHwCs(false);
  

  //MUX Sketch Code
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
  
  // mux_write(CHIP_SEL_MUX_VN,5,1,IOExpander);//VMEAS_2
  // mux_write(CHIP_SEL_MUX_SINK,7,0,IOExpander);//IOUT_2
  // mux_write(CHIP_SEL_MUX_SRC,1,1,IOExpander);//IOUT_1
  // mux_write(CHIP_SEL_MUX_VP,3,1,IOExpander);//VMEAS_1

  //Setting the trigger pin that will take ADC clock as a reference to initiate ADC sampling
  pinMode(TRIGGER,INPUT);

  // ADC input
  pinMode(ADC_BIT0, INPUT);
  pinMode(ADC_BIT1, INPUT);
  pinMode(ADC_BIT2, INPUT);
  pinMode(ADC_BIT3, INPUT);
  pinMode(ADC_BIT4, INPUT);
  pinMode(ADC_BIT5, INPUT);
  pinMode(ADC_BIT6, INPUT);
  pinMode(ADC_BIT7, INPUT);
  pinMode(ADC_BIT8, INPUT);
  pinMode(ADC_BIT9, INPUT);

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

  // t1 = micros();
  // mux_write(CHIP_SEL_MUX_SRC,9,1,IOExpander);//IOUT_1
  // mux_write(CHIP_SEL_MUX_SINK,8,1,IOExpander);//IOUT_2
  // delay(100);
  // pk_pk_values[0] = calculate_pk_pk(2000,read_values);
  // Serial.println("Peak to Peak(V): "+ String(pk_pk_values[0],5));
  mux_write(CHIP_SEL_MUX_VP,0,0,IOExpander);//IOUT_1
  mux_write(CHIP_SEL_MUX_VP,0,0,IOExpander);//IOUT_2

  static const uint8_t src_electrode_order[8] = {1,2,3,4,5,6,7,0}; //c
  static const uint8_t sink_electrode_order[8] = {0,1,2,3,4,5,6,7};
  static const uint8_t vp_electrode_order[8] = {0,1,2,3,4,5,6,7}; //c
  static const uint8_t vn_electrode_order[8] = {7,0,1,2,3,4,5,6};

  for(int i=0;i<8;i=i+1){
    Serial.println("Source Channels: "+String(src_electrode_order[i]+1)+", "+ String(sink_electrode_order[i]+1));
    mux_write(CHIP_SEL_MUX_SRC,src_electrode_order[i],1,IOExpander);//IOUT_1
    mux_write(CHIP_SEL_MUX_SINK,sink_electrode_order[i],1,IOExpander);//IOUT_2
    delay(2000);
    for(int j=0;j<8;j=j+1){
      mux_write(CHIP_SEL_MUX_VP,vp_electrode_order[j],1,IOExpander);//VMEAS_1
      mux_write(CHIP_SEL_MUX_VN,vn_electrode_order[j],1,IOExpander);//VMEAS_2
      delay(1000);
      
      pk_pk_values[8*i+j] = calculate_pk_pk_ensemble(2000,read_values);
      Serial.println("Peak to Peak(V) for electrodes: "+ String(vp_electrode_order[j]+1)+", "+ String(vn_electrode_order[j]+1)+" "+String(pk_pk_values[8*i+j],5));
    }
    // Serial.println("Peak to Peak(V): "+ String(calculate_pk_pk(2000,read_values),5));
    

    // mux_write(CHIP_SEL_MUX_SRC,4,1,IOExpander);//IOUT_1
    // mux_write(CHIP_SEL_MUX_SINK,0,1,IOExpander);//IOUT_2
  }

  for(int i=0;i<64;i=i+1){
    Serial.println(pk_pk_values[i],5);
  }
  // mux_write(CHIP_SEL_MUX_SRC,1,1,IOExpander);//IOUT_1
  // mux_write(CHIP_SEL_MUX_SINK,0,1,IOExpander);//IOUT_2
  // mux_write(CHIP_SEL_MUX_VP,1,1,IOExpander);//VMEAS_1
  // mux_write(CHIP_SEL_MUX_VN,0,1,IOExpander);//VMEAS_2
  // Serial.println("Peak to Peak(V): "+ String(calculate_pk_pk_ensemble(2000,read_values,10),5));

}

// the loop function runs over and over again until power down or reset
void loop() {

}

void spiCommand(SPIClass *spi, uint16_t data, const uint8_t bit_order, const uint8_t mode,const int chip_select) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(SPI_FREQ_FAST, bit_order, mode));
  digitalWrite(chip_select, LOW); //pull SS slow to prep other end for transfer
  spi->transfer16(data);
  digitalWrite(chip_select, HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
}
void vspiCommand(uint16_t data, const uint8_t bit_order, const uint8_t mode,const int chip_select) {
  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(SPI_FREQ_FAST, bit_order, mode));
  digitalWrite(chip_select, LOW); //pull SS slow to prep other end for transfer
  vspi->transfer16(data);
  digitalWrite(chip_select, HIGH); //pull ss high to signify end of data transfer
  vspi->endTransaction();
}

void AD5270_Write(SPIClass *spi, uint8_t cmd, const uint8_t bit_order, const uint8_t mode, const int chip_select, uint16_t data){
  uint16_t data_word = ((cmd & 0x0F) << 10) | (data & 0x03FF);
  // Serial.print("AD5270 Write command: "+String(data_word,HEX));
  spiCommand(spi, data_word, bit_order, mode, chip_select);
  // vspiCommand(data_word, bit_order, mode, chip_select);
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


/* Read GPIO 0-31 (takes ~50.1ns) */
uint32_t gpio_read(){
  return REG_READ(GPIO_IN_REG);
}

/* Convert GPIO reading to 10-bit unsigned integer */
uint16_t gpio_convert(uint32_t gpio_reg){

  const uint8_t bit_to_gpio[10] = {ADC_BIT9, ADC_BIT8, ADC_BIT7, ADC_BIT6, ADC_BIT5, ADC_BIT4, ADC_BIT3, ADC_BIT2, ADC_BIT1, ADC_BIT0};
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

double calculate_pk_pk(int cycles, uint32_t * read_values){
  int k=0;
  uint32_t sample_sum = 0;
  uint32_t square_sum = 0;
  uint32_t avg;
  uint16_t adc_buf[cycles];
  int TrigState = 1;
  int lastTrigState = 0;

  while(k<cycles){
    TrigState = digitalRead(TRIGGER);

    if(lastTrigState==1 && TrigState==0){
      read_values[k]=gpio_read();
      // Serial.println(read_values[k]);
      k += 1;
    }
    lastTrigState = TrigState; //changing past trigger value to current value
  }

  for(int i=0; i<k; i=i+1){
    adc_buf[i] = gpio_convert(read_values[i]);
    // Serial.println(adc_buf[i]);
    sample_sum += adc_buf[i];
  }

  avg = sample_sum/k;
  for(int i=0; i<k; i=i+1){
    square_sum += (adc_buf[i]-avg)*(adc_buf[i]-avg);
  }

  uint16_t rms_10bit = sqrt(square_sum/k);
  uint16_t pk_pk_10bit = rms_10bit * sqrt(2) * 2;
  double pk_pk = (double)pk_pk_10bit * 1.031 / 1024;

  return(pk_pk);
}

double calculate_pk_pk_ensemble(int cycles, uint32_t * read_values,int averages){
  double pk_pk_avg = 0;
  for(int i=0;i<averages;i=i+1){
    pk_pk_avg += calculate_pk_pk(cycles,read_values);
  }
  return(pk_pk_avg/averages);
}

uint32_t read_signal(double * rms, uint16_t * adc_buf, uint32_t * gpio_buf, int num_samples){
  uint16_t i, j;

  uint8_t adc_half_period_count = 0;
  uint16_t zero_cross_index = 0;

  uint32_t time1, time2;
  // uint32_t count, num_cycles;
  uint32_t sample_sum, total_sum = 0;
  // uint32_t phase_val;

  // uint16_t adc_buf[100];              // Store converted ADC samples of the input waveform
  // uint32_t gpio_buf[500];     // Store raw GPIO readings

  time1 = micros();

  /* Collect samples */
  for (i = 0; i < num_samples*ADC_AVG; i++)
  {
    gpio_buf[i] = gpio_read();
    delayMicroseconds(100);
  }

  time2 = micros();

  /* Process samples */
  for (i = 0; i < num_samples; i++)
  {
    /* Extract integer from GPIO reading */
    for (j = 0, sample_sum = 0; j < ADC_AVG; j++)
      sample_sum += gpio_convert(gpio_buf[i*ADC_AVG+j]);    // Get 10-bit ADC value from raw GPIO value
    adc_buf[i] = sample_sum / ADC_AVG;

    /* Store product for RMS calculation */
    int16_t adc_val = (int16_t)adc_buf[i] - 512;
    total_sum += adc_val * adc_val;

    if (i > 0)
    {
      /* Signal at midpoint, entering peak or trough */
      if ((adc_buf[i] > 512 && adc_buf[i - 1] <= 512) || (adc_buf[i] < 512 && adc_buf[i - 1] >= 512))
      {
        adc_half_period_count++;

        /* Record index of first rising zero point */
        if (adc_buf[i] > 512 && zero_cross_index == 0)
          zero_cross_index = i;
      }
    }
  }

  /* Calculate peak-to-peak magnitude and RMS */
  uint16_t rms_10bit = sqrt(total_sum / num_samples);
  // uint16_t pk_pk_10bit = rms_10bit * sqrt(2) * 2;
  double rms_result = (double)rms_10bit * 1.031 / 1024;

  Serial.println("RMS Value: "+String(rms_result));

  if (rms)
    *rms = rms_result;

  // if (error_rate)
  // {
  //   // Compare measured signal to sine wav 
  //   uint8_t compare_periods = 2;
  //   if ((num_samples - zero_cross_index) >= (samples_per_period * compare_periods))
  //     *error_rate = sine_compare(adc_buf + zero_cross_index, pk_pk_10bit, samples_per_period, compare_periods);
  // }

  // if (debug)
  // {  
  //   for (int i = 0; i < num_samples; i++){
  //     Serial.println(adc_buf[i]);
  //   }
  // }
  return (time2 - time1);
}