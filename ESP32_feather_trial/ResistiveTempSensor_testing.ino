//Updated for Biocoin pins

#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

#define PIN_TEMP A1
uint32_t temp_pin = PIN_TEMP;             

//#define TEMP_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define TEMP_MV_PER_LSB   (0.18310547F)   // 3.0V ADC range and 14-bit ADC resolution = 3000mV/16384
//#define TEMP_MV_PER_LSB   (0.10986328F)     // 1.8V ADC range and 14-bit ADC resolution = 1800mV/16384

#define TEMP_DIVIDER      (0.50000000F)          // division factor from voltage divider on temp sensor 100k/(100k + 100k)
#define TEMP_DIVIDER_COMP (2.00000000F)          // Compensation factor for the VBAT divider, inverse of VBAT_DIVIDER

#define REAL_TEMP_MV_PER_LSB (TEMP_DIVIDER_COMP * TEMP_MV_PER_LSB)

char* start = "Start";
char* stop = "Stop";
uint8_t received[8] = {0};
int NewSerialData = 0;


void setup() {
  
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  pinMode(PIN_TEMP, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Get a single ADC sample and throw it away
  readTEMP();
}

void loop() {

  int i = 0;

  while(Serial.available() > 0) 
  {  
    received[i] = Serial.read();
    i++;
    NewSerialData = 1;
  }

/* 
  //serial echo for debug only
  if (NewSerialData)
  {
    //echo back the serial data
    for(int i = 0; i < 8; i++)
    {
      Serial.print((char)received[i]);
    }
    Serial.println();
    NewSerialData = 0;
  }
*/


  //if we receive a start command, keep taking data readings
  if (!strcmp(start, (char*) received))
  {
    digitalWrite(LED_BUILTIN, HIGH);
    // Get a raw ADC reading
    float temp_mv = readTEMP();

    // Display the results in units of [mV]
    Serial.println(temp_mv);
    delay(1000);
  }
  else
  {
    //Don't take readings
    digitalWrite(LED_BUILTIN, LOW);

  }

}


float readTEMP(void) 
{
  float raw;

  // Set the analog reference (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

    //Set the ADC Sample and Hold Acquisition time
  analogSampleTime(SAADC_CH_CONFIG_TACQ_40us);

  //Set the ADC to Oversample and perform sample averaging: averages 2^OVERSAMPLE samples per reading
  analogOversampling(SAADC_OVERSAMPLE_OVERSAMPLE_Over16x);

  // Set the resolution
  analogReadResolution(14); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(10);

  // Get the raw ADC value and average some readings
  int i = 0;
  float count = 1000;
  float sum = 0;
  for(i = 0; i < count; i++){
        raw = analogRead(temp_pin);
        sum = raw + sum;
  }
  sum = sum / count;

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account
  return sum * REAL_TEMP_MV_PER_LSB;
}
