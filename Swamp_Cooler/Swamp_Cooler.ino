//CPE301 Team Project for Swamp Cooler
//Authors: Elvis Vong and Nasrin Juana
#include <Arduino.h>
#include <LiquidCrystal.h> //for the LCD1602 Display
#include <dht.h>
#include <Servo.h>

//getting the addresses for the ADC registers
volatile unsigned char* myADMUX = (unsigned char*)0x7C;
volatile unsigned char* myADCSRB = (unsigned char*)0x7B;
volatile unsigned char* myADCSRA = (unsigned char*)0x7A;
volatile unsigned int* myDATA = (unsigned int*)0x78;

//PORTB registers
volatile unsigned char *portB = (unsigned char*) 0x25;
volatile unsigned char *ddrB = (unsigned char*) 0x24;
volatile unsigned char *pinB = (unsigned char*) 0x23;
//inside port b
#define greenLED 7 //idle state
#define redLED 6 //error state
#define yellowLED 5 //diabled state
#define blueLED 4 //running state and will also turn on the fan

//PORTH registers
volatile unsigned char *portH = (unsigned char*) 0x102;
volatile unsigned char *ddrH = (unsigned char*) 0x101;
volatile unsigned char *pinH = (unsigned char*) 0x100;
//inside port h



volatile int servoPos = 0;

LiquidCrystal lcd(9, 8, 7, 6, 5, 4);

//PORTE registers
volatile unsigned char *portE = (unsigned char*) 0x2E;
volatile unsigned char *ddrE = (unsigned char*) 0x2D;
volatile unsigned char *pinE = (unsigned char*) 0x2C;
#define servoSwitch 2 //pin 2, for interrupts
#define powerSwitch 3 //pin 3, for interrupts
Servo myservo;  // create servo object to control a servo
bool servoState = 0;

bool powerState = 1;

#define dht_apin A15 // Analog Pin sensor is connected to
dht DHT;

String date = "01/01/2000"; //         mm/dd/yyyy
unsigned int currentHours = 0;
unsigned int currentMinutes = 0;
unsigned int currentSeconds = 0;

void setup() 
{
  adc_init();//initializes the ADC
  *ddrB |= 0xF0; // 1111 0000, turns pin 13, 12, 11, and 10 to output
  *ddrE &= 0xCF; // 1100 1111, turns pin 2 and 3 to input
  *portE |= 0x30; // 0011 0000, turns on pullup resistors for pin 5 and 4
  Serial.begin(9600);
  delay(1000);//Wait before accessing Sensor
  setTime();
  lcd.begin(16, 2); //initializes lcd
  myservo.attach(A7);  // attaches the servo on pin A14 to the servo object
  attachInterrupt(0, updateServo_ISR, RISING); //interupt for pin 2
  attachInterrupt(1, togglePower_ISR, RISING); //interupt for pin 3
}

void loop() 
{
  unsigned int adc_waterLevel = adc_read(0); //reading from pin A0
  DHT.read11(dht_apin);
  unsigned int temperature = DHT.temperature;
  float humidity = DHT.humidity;
  
  if(powerState == 0) //check power state, if off turn everything off except yellow LED
  {
    if(*portB & 0x01 << blueLED == 1)///checks if it is on
    {
      write_pb(blueLED, 0); //turn fan and blue led off
      readTime(); //record time because the fan flipped to off
      Serial.println("Power Off");
      Serial.println();
    }
    write_pb(greenLED, 0);
    write_pb(redLED, 0);
    write_pb(yellowLED, 1); //disabled state
    write_pb(blueLED, 0);

    lcd.noDisplay(); //turn off display
  }
  else if(adc_waterLevel < 100) //check water if below threshold, turn off fan if so
  {
    if(*portB & 0x01 << blueLED == 1)//checks if it is on
    {
      write_pb(blueLED, 0); //turn fan and blue led off
      readTime(); //record time because the fan flipped to off
      Serial.println("Power Off");
      Serial.println();
    }
    write_pb(greenLED, 0);
    write_pb(redLED, 1); //error state
    write_pb(yellowLED, 0);

    lcd.display(); //turns on display if off
    updateLCDError();
  }
  else if(DHT.temperature > 19) //check temperature if above threshold, turn on fan
  {
      write_pb(blueLED, 1); //running state
      readTime(); //record time because the fan flipped
    
    write_pb(greenLED, 0);
    write_pb(redLED, 0);
    write_pb(yellowLED, 0);
    
    
    lcd.display(); //turns on display if off
    updateLCD();
    
    
  }
  else //idle
  {
    if(*portB & 0x01 << blueLED == 1)//checks if it is on
    {
      write_pb(blueLED, 0); //turn fan and blue led off
      readTime(); //record time because the fan flipped to off
      Serial.println("Power Off");
      Serial.println();
    }
    
    write_pb(greenLED, 1);
    write_pb(redLED, 0);
    write_pb(yellowLED, 0);


    lcd.display(); //turns on display if off
    updateLCD();
  }

  delay(5000); //wait for 5 seconds
  updateTime(5);//update time by 5 seconds
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
//=====================================time/date=======================================================//
void setTime()//setting up the date and time
{
  Serial.print("Current Date (mm/dd/yyyy): ");
  while(Serial.available() == 0 ) {}
  date = Serial.readString();
  Serial.println(date);

  String temp; //used to help keep the integers

  Serial.print("Current Hour (military): ");
  while(Serial.available() == 0 ) {}
  temp = Serial.readString();
  currentHours = temp.toInt();
  Serial.println(currentHours);
  
  Serial.print("Current Minutes: ");
  while(Serial.available() == 0 ) {}
  temp = Serial.readString();
  currentMinutes = temp.toInt();
  Serial.println(currentMinutes);
}
void updateTime(unsigned int deltaSeconds) //will update the time accordingly based on the change of time in seconds
{
  currentSeconds += deltaSeconds;
  if(currentSeconds >= 60) //will convert to a minute if overflow
  {
    currentSeconds -= 60;
    currentMinutes += 1;
  }
  if(currentMinutes >= 60) //will convert to an hour if overflow
  {
    currentMinutes -= 60;
    currentHours += 1;
  }
  if(currentHours >= 24) //will just reset back to 0
  {
    currentHours -= 24;
  }
}
void readTime() //will display the current time and date
{
  Serial.print(date);
  Serial.print(" ");
  Serial.print(currentHours);
  Serial.print(":");
  Serial.print(currentMinutes);
  Serial.print(":");
  Serial.print(currentSeconds);
  Serial.println(" (h:m:s)");
}

//======================================================================================================//
void write_pb(unsigned char pin_num, unsigned char state)
{
  if(state == 0) //off state
  {
    *portB &= ~(0x01 << pin_num);
  }
  else //on state
  {
    *portB |= 0x01 << pin_num;
  }
}
void write_ph(unsigned char pin_num, unsigned char state)
{
  if(state == 0) //off state
  {
    *portH &= ~(0x01 << pin_num);
  }
  else //on state
  {
    *portH |= 0x01 << pin_num;
  }
}

void updateLCD() //display temp and humidity on LCD
{
  lcd.clear();//clears the screen
  lcd.print("humidity: ");
  lcd.print(DHT.humidity);
  lcd.print("%");
  lcd.setCursor(0,1); //move to the second line
  lcd.print("temp: ");
  lcd.print(DHT.temperature); 
  lcd.println("C  ");
}
void updateLCDError() //display error on LCD
{
  lcd.clear();//clears the screen
  lcd.print("ERROR!");
  lcd.setCursor(0,1); //move to the second line
  lcd.print("WATER LEVEL LOW");
}
void updateServo_ISR()
{
  noInterrupts();//disables interrupts
  Serial.print("sweep");
  //flip the state to keep the servo within 0-180
  if(servoPos == 0)
    servoState = 0; 
  if(servoPos == 180)
    servoState = 1;
  
  if(servoState == 0)// clockwise
    servoPos += 45;
  else //counter clockwise
    servoPos -= 45;
    
  Serial.println(servoPos);
  myservo.write(servoPos);//move the servo
  delay(500); //so it won't do a double input
  interrupts();//renables interrupts
}
void togglePower_ISR()
{
  noInterrupts();//disables interrupts
  Serial.println("toggled power");
  powerState = !powerState; //flip the state
  delay(500);
  interrupts();//renables interrupts
  
}
