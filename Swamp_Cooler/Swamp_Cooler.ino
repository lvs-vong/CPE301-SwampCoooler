

//CPE301 Team Project for Swamp Cooler
//Authors: Elvis Vong and Nasrin Juana
#include <Arduino.h>
#include <LiquidCrystal.h> //for the LCD1602 Display
#include <dht_nonblocking.h> //for the temperature/humidity sensor

//getting the addresses for the ADC registers
volatile unsigned char* myADMUX = (unsigned char*)0x7C;
volatile unsigned char* myADCSRB = (unsigned char*)0x7B;
volatile unsigned char* myADCSRA = (unsigned char*)0x7A;
volatile unsigned int* myDATA = (unsigned int*)0x78;


//initializing some stuff for the temperature/humidity sensor
#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );


void setup() 
{
  adc_init();//initializes the ADC
  Serial.begin(9600);
}

void loop() 
{
  unsigned int adc_waterLevel = adc_read(15); //reading from pin A15
  Serial.println(adc_waterLevel);

  float temperature;
  float humidity;

  /* Measure temperature and humidity.  If the functions returns
     true, then a measurement is available. */
  if( measure_environment( &temperature, &humidity ) == true )
  {
    Serial.print( "T = " );
    Serial.print( temperature, 1 );
    Serial.print( " deg. C, H = " );
    Serial.print( humidity, 1 );
    Serial.println( "%" );
  }


  
}

//================================ADC Function Implementations======================================//
void adc_init()
{
  //setting up the A register
  *myADCSRA |= 0x80; // enable the ADC
  *myADCSRA &= 0xDF; // disables the auto trigger mode
  *myADCSRA &= 0xF7; // disable ADC interrupt
  *myADCSRA &= 0xF8; // set prescaler selection to slow reading

  //setting up the B register
  *myADCSRB &= 0xF7; // reset channel and gain bits
  *myADCSRB &= 0xF8; // set free running mode

  //setting up the MUX Register
  *myADMUX &= 0x7F; // AVCC analog reference
  *myADMUX |= 0x40; // AVCC analog reference
  *myADMUX &= 0xDF; // right adjust result
  *myADMUX &= 0xE0; // reset the channel and gain bits
   
    
}

unsigned int adc_read(unsigned char adc_channel)
{
  *myADMUX &= 0xE0; // clears the channel selection bits
  *myADCSRB &= 0xF7; // clears the channel selection bits

  if (adc_channel > 7)
  {
    adc_channel -= 8; //setting the channel selection bits, but remove the most significant bit

    *myADCSRB |= 0x08; //turns on mux5
  }

  *myADMUX += adc_channel; //set the channel selection bits

  *myADCSRA |= 0x40; //start conversion
  while ( (*myADCSRA & 0x40) != 0 ); //waiting for the conversion to finish

  return *myDATA; //returning the result
}
//=====================================================================================================//
//================================Temperature and Humidity=============================================//
/*
 * Poll for a measurement, keeping the state machine alive.  Returns
 * true if a measurement is available. 
 * Taken from the example code dht_unblocking.ino
 */
static bool measure_environment( float *temperature, float *humidity )
{
  static unsigned long measurement_timestamp = millis( );

  /* Measure once every four seconds. */
  if( millis( ) - measurement_timestamp > 3000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

  return( false );
}
//======================================================================================================//
