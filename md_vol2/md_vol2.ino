#include "TimerOne.h"

/**
// Dasygenis Minas
// Arduino + ESP8266 + 2 relay + DHT
// January, 2016
// Uses code from: http://playground.arduino.cc/Code/EepromUtil

 */
 
// We use the ITE EP8266 Library
// https://github.com/itead/ITEADLIB_Arduino_WeeESP8266/archive/master.zip
// Remember to edit the ESP8266.h and uncomment the softwareserial
#include "ESP8266.h"


// we require the library TimerOne from the Arduino Manage Libraries
/*
 * Put the defines here
 */
char SSID[20];
char PASSWORD[20];
char lanID[] = "arduino_esp8266-01";
char wwwserverName[] = "zafora.icte.uowm.gr";  //  URL
unsigned long lastConnectionTime = 0;
const unsigned long postingInterval = 10 * 1000; //delay between updates to main server (10 secs)
#define serverName  "zafora.icte.uowm.gr"
#define HOST_PORT   (80)
/*
 * END Defines here
 */

#define maxretries 4

// We need this for watchdog
#include <avr/wdt.h>

int relaypins[4] = {6, 7};   //where are our two relays
int relaystate[4] = {1, 1 }; //initially close
int nrrelays = 2; //Number of relays

unsigned long startupmillis;      //to check when a day has passed for reboot

#include <EEPROM.h>
#define MAXBYTES 13
#define uint8_t byte
byte eepromatmega[MAXBYTES];
#define SERIAL_TIMEOUT 15000  //15 sec
#define BASE_EEPROM 0 //at possition 0 length of ssid, at possition 1 length of psk
#define SSIDADDRESS 10
#define PSKADDRESS 50

//last time the WDT was ACKd by the application
unsigned long lastUpdate=0;

//time, in ms, after which a reset should be triggered
unsigned long WDTtimeout=15000; //15 seconds



//
// Absolute min and max eeprom addresses.
// Actual values are hardware-dependent.
//
// These values can be changed e.g. to protect
// eeprom cells outside this range.
//
const int EEPROM_MIN_ADDR = 0;
const int EEPROM_MAX_ADDR = 511;




#define ONE_WIRE_BUS 4  //pin of the DHT data pin
#include <OneWire.h>
OneWire oneWire(ONE_WIRE_BUS);

// Counter of no connections
int noconnectioncounter = 0;


//variable to indicate wether this is the first connection
unsigned int counterrun = 0;
unsigned int counterrunprevious = 0;

#include <SoftwareSerial.h>
SoftwareSerial swSerial(9,8);//RX,TX
ESP8266 wifi(swSerial);
int  status=0; //status flag and error code

byte readchars;


//
//
//
//
//
// FUNCTIONS HERE
//
//
//
//
//


void longWDT(void)
{
  //dont forget to occasionally update with :  lastUpdate=millis();
  if((millis()-lastUpdate) >WDTtimeout)
  {
    //enable interrupts so serial can work
    sei();

    //detach Timer1 interrupt so that if processing goes long, WDT isn't re-triggered
    Timer1.detachInterrupt();

    //flush, as Serial is buffered; and on hitting reset that buffer is cleared
    Serial.println("WDT triggered");
    Serial.flush();

    //call our reset function
    software_Reset();
  }
}





// Print the startup message
void motd()
{
  Serial.println();
  Serial.println(F("----------------------------------------------------------------------"));
  Serial.println(F("DASYGENIS v107 Wireless-WebControl/Temperature, 2016, mdasyg@ieee.org"));
  Serial.println(F("Use this at your own risk."));
  Serial.println(F("---------------------------------------------------------------------"));
  return;
}


//Reads a byte from the EEPROM.
//Locations that have never been written to have the value of 255.
void LoadEeprom(int length)
{
  int ind;
  for (ind = 0; ind <= length ; ind++)
  {
    eepromatmega[ind] = EEPROM.read(BASE_EEPROM + ind);
  }
}




//Write a byte to the EEPROM.
//An EEPROM write takes 3.3 ms to complete.
//The EEPROM memory has a specified life of 100,000 write/erase cycles
void WriteEeprom(int length)
{
  Serial.println("Saving to EEPROM");
  int ind;
  for (ind = 0; ind <= length ; ind++)
  {
    EEPROM.write(BASE_EEPROM + ind, eepromatmega[ind]);
  }
}







//if the serial '9' is pressed, then do a reboot
void examine_serial_and_reboot()
{
  char rebootbuffer[2];
  //Sleep for 1 sec
  //either with delay
  //delay(1000);
  //or better to wait for input
  Serial.setTimeout(1000);

  wdt_reset(); //lets tap the doggie..
  int readchars2 = Serial.readBytes(rebootbuffer, 1);
  if (readchars2 > 0 )
  {
    Serial.println(atoi(rebootbuffer));
    if(atoi(rebootbuffer) == 9)
    {
      Serial.println(F("Reboot after user request."));
      delay(1000);
      software_Reset();
    }
  }
}






// Perform a software reset, by just starting from instruction memory @ 0
// the improved version uses the watchdog to fire up
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
  wdt_enable(31); //just to be sure that watchdog is on
  delay(500000);//by delaying more than 8 seconds we force the doggie to take action
  asm volatile ("  jmp 0");
}


void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}




//Set the relay pins accordingly to the state table
void act_on_relays()
{
/*  wdt_reset(); //lets tap the doggie..
  int k;
  for (k = 0; k < nrrelays; k++)
  {

    Serial.print(F("State of [ "));
    Serial.print(k);
    Serial.print(F(" ] is [ "));
    Serial.print(relaystate[k]);
    Serial.println(F(" ]"));

    if (relaystate[k] == 0)
    {
      digitalWrite(relaypins[k], LOW);
    }
    else if (relaystate[k] == 1)
    {
      digitalWrite(relaypins[k], HIGH);
    }


  }
*/  return;
}






//
// Dump eeprom memory contents over serial port.
// For each byte, address and value are written.
//
void eeprom_serial_dump_column() {
  // counter
  int i;

  // byte read from eeprom
  byte b;

  // buffer used by sprintf
  char buf[10];

  for (i = EEPROM_MIN_ADDR; i <= EEPROM_MAX_ADDR; i++) {
    b = EEPROM.read(i);
    sprintf(buf, "%03X: %02X", i, b);
    Serial.println(buf);
  }
}



//
// Reads the specified number of bytes from the specified address into the provided buffer.
// Returns true if all the bytes are successfully read.
// Returns false if the star or end addresses aren't between
// the minimum and maximum allowed values.
// When returning false, the provided array is untouched.
//
// Note: the caller must ensure that array[] has enough space
// to store at most numBytes bytes.
//
boolean eeprom_read_bytes(int startAddr, byte array[], int numBytes) {
  int i;

  // both first byte and last byte addresses must fall within
  // the allowed range
  if (!eeprom_is_addr_ok(startAddr) || !eeprom_is_addr_ok(startAddr + numBytes)) {
    return false;
  }

  for (i = 0; i < numBytes; i++) {
    array[i] = EEPROM.read(startAddr + i);
  }

  return true;
}




//
// Writes an int variable at the specified address.
// Returns true if the variable value is successfully written.
// Returns false if the specified address is outside the
// allowed range or too close to the maximum value
// to store all of the bytes (an int variable requires
// more than one byte).
//
boolean eeprom_write_int(int addr, int value) {
  byte *ptr;

  ptr = (byte*)&value;
  return eeprom_write_bytes(addr, ptr, sizeof(value));
}



//
// Reads an integer value at the specified address.
// Returns true if the variable is successfully read.
// Returns false if the specified address is outside the
// allowed range or too close to the maximum vlaue
// to hold all of the bytes (an int variable requires
// more than one byte).
//
boolean eeprom_read_int(int addr, int* value) {
  return eeprom_read_bytes(addr, (byte*)value, sizeof(int));
}


//
// Writes a string starting at the specified address.
// Returns true if the whole string is successfully written.
// Returns false if the address of one or more bytes
// fall outside the allowed range.
// If false is returned, nothing gets written to the eeprom.
//
boolean eeprom_write_string(int addr, const char* string) {
  // actual number of bytes to be written
  int numBytes;

  // we'll need to write the string contents
  // plus the string terminator byte (0x00)
  numBytes = strlen(string) + 1;

  return eeprom_write_bytes(addr, (const byte*)string, numBytes);
}





//
// Reads a string starting from the specified address.
// Returns true if at least one byte (even only the
// string terminator one) is read.
// Returns false if the start address falls outside
// or declare buffer size os zero.
// the allowed range.
// The reading might stop for several reasons:
// - no more space in the provided buffer
// - last eeprom address reached
// - string terminator byte (0x00) encountered.
// The last condition is what should normally occur.
//
boolean eeprom_read_string(int addr, char* buffer, int bufSize) {
  // byte read from eeprom
  byte ch;

  // number of bytes read so far
  int bytesRead;

  // check start address
  if (!eeprom_is_addr_ok(addr)) {
    return false;
  }

  // how can we store bytes in an empty buffer ?
  if (bufSize == 0) {
    return false;
  }

  // is there is room for the string terminator only,
  // no reason to go further
  if (bufSize == 1) {
    buffer[0] = 0;
    return true;
  }

  // initialize byte counter
  bytesRead = 0;

  // read next byte from eeprom
  ch = EEPROM.read(addr + bytesRead);

  // store it into the user buffer
  buffer[bytesRead] = ch;

  // increment byte counter
  bytesRead++;

  // stop conditions:
  // - the character just read is the string terminator one (0x00)
  // - we have filled the user buffer
  // - we have reached the last eeprom address
  while ( (ch != 0x00) && (bytesRead < bufSize) && ((addr + bytesRead) <= EEPROM_MAX_ADDR) ) {
    // if no stop condition is met, read the next byte from eeprom
    ch = EEPROM.read(addr + bytesRead);

    // store it into the user buffer
    buffer[bytesRead] = ch;

    // increment byte counter
    bytesRead++;
  }

  // make sure the user buffer has a string terminator
  // (0x00) as its last byte
  if ((ch != 0x00) && (bytesRead >= 1)) {
    buffer[bytesRead - 1] = 0;
  }

  return true;
}




//zero out a string, so not to carry leftovers
void zero_string(char * array, int size)
{
  int i;
  for(i = 0; i < size; i++)
  {
    array[i] = '\0';
  }

}



//
int serialprompt(const char *prompt)
{
  char buffer[10];
  delay(10);
  serialFlush();
  delay(10);
  //we do not want timeout to be defined at prompt
  //Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println(prompt);
  wdt_reset(); //lets tap the doggie..
  int readchars = Serial.readBytes(buffer, 1);
  wdt_reset(); //lets tap the doggie..
  delay(200);  // Catch Due reset problem
  return readchars;
}


int serialword(const char *msg, int count, char *array)
{
  char buffer[count];
  int n = 0;
  delay(10);
  serialFlush();
  delay(10);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.print(msg);
  Serial.print(F(" : [current: "));
  Serial.print(array);
  Serial.print(F("]("));
  Serial.print(count);
  Serial.print(F(")-->"));
  readchars = Serial.readBytes(buffer, count);

  //The user enter smt then do copy it
  if(readchars > 0)
  {
  strcpy (array, buffer);
  }
  //otherwise do not do smt.
  return readchars;
}



int serialnumber(const char *msg, int count, int defaultvalue)
{

  char buffer[count];
  int n = 0;
  delay(10);
  serialFlush();
  delay(10);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println();
  Serial.print(msg);
  Serial.print(F(" : (current: "));
  if (defaultvalue < 9)
  {
    Serial.print("0");
  }
  Serial.print(defaultvalue);
  Serial.print(F(" )=> "));
  readchars = Serial.readBytes(buffer, count);

#ifdef DEBUG
  Serial.print(F("===>["));
  Serial.print(buffer);
  Serial.println(F("]"));
#endif

  delay(200);  // Catch Due reset problem
  if(readchars > 0)
  {
    n = atoi(buffer);
  }
  else
  {
    n = defaultvalue;
  }
  Serial.println(n);
  return n;
}




void fmtDouble(double val, byte precision, char *buf, unsigned bufLen =0xffff);
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen =0xffff, byte width = 0);

//
// Produce a formatted string in a buffer corresponding to the valueprovided.
// If the 'width' parameter is non-zero, the value will be padded withleading
// zeroes to achieve the specified width.  The number of characters added to
// the buffer (not including the null termination) is returned.
//
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen, byte width)
{
  if (!buf || !bufLen)
    return(0);

// produce the digit string (backwards in the digit buffer)
  char dbuf[10];
  unsigned idx = 0;
  while (idx < sizeof(dbuf))
  {
    dbuf[idx++] = (val % 10) + '0';
    if ((val /= 10) == 0)
      break;
  }

// copy the optional leading zeroes and digits to the target buffer
  unsigned len = 0;
  byte padding = (width > idx) ? width - idx : 0;
  char c = '0';
  while ((--bufLen > 0) && (idx || padding))
  {
    if (padding)
      padding--;
    else
      c = dbuf[--idx];
    *buf++ = c;
    len++;
  }

// add the null termination
  *buf = '\0';
  return(len);
}


//
// Returns true if the address is between the
// minimum and maximum allowed values,
// false otherwise.
//
// This function is used by the other, higher-level functions
// to prevent bugs and runtime errors due to invalid addresses.
//
boolean eeprom_is_addr_ok(int addr) {
  return ((addr >= EEPROM_MIN_ADDR) && (addr <= EEPROM_MAX_ADDR));
}




//
// Writes a sequence of bytes to eeprom starting at the specified address.
// Returns true if the whole array is successfully written.
// Returns false if the start or end addresses aren't between
// the minimum and maximum allowed values.
// When returning false, nothing gets written to eeprom.
//
boolean eeprom_write_bytes(int startAddr, const byte* array, int numBytes) {
  // counter
  int i;

  // both first byte and last byte addresses must fall within
  // the allowed range
  if (!eeprom_is_addr_ok(startAddr) || !eeprom_is_addr_ok(startAddr + numBytes)) {
    return false;
  }

  for (i = 0; i < numBytes; i++) {
    EEPROM.write(startAddr + i, array[i]);
  }

  return true;
}






//
// Format a floating point value with number of decimal places.
// The 'precision' parameter is a number from 0 to 6 indicating thedesired decimal places.
// The 'buf' parameter points to a buffer to receive the formatted string.This must be
// sufficiently large to contain the resulting string.  The buffer's length may be
// optionally specified.  If it is given, the maximum length of the generated string
// will be one less than the specified value.
//
// example: fmtDouble(3.1415, 2, buf); // produces 3.14 (two decimal places)
//
void fmtDouble(double val, byte precision, char *buf, unsigned bufLen)
{
  if (!buf || !bufLen)
    return;

// limit the precision to the maximum allowed value
  const byte maxPrecision = 6;
  if (precision > maxPrecision)
    precision = maxPrecision;

  if (--bufLen > 0)
  {
    // check for a negative value
    if (val < 0.0)
    {
      val = -val;
      *buf = '-';
      bufLen--;
    }

    // compute the rounding factor and fractional multiplier
    double roundingFactor = 0.5;
    unsigned long mult = 1;
    for (byte i = 0; i < precision; i++)
    {
      roundingFactor /= 10.0;
      mult *= 10;
    }

    if (bufLen > 0)
    {
      // apply the rounding factor
      val += roundingFactor;

      // add the integral portion to the buffer
      unsigned len = fmtUnsigned((unsigned long)val, buf, bufLen);
      buf += len;
      bufLen -= len;
    }

    // handle the fractional portion
    if ((precision > 0) && (bufLen > 0))
    {
      *buf++ = '.';
      if (--bufLen > 0)
        buf += fmtUnsigned((unsigned long)((val - (unsigned long)val) * mult), buf, bufLen, precision);
    }
  }

// null-terminate the string
  *buf = '\0';
}









//We connect to the server and check the response
void wireless_process(void)
{

int i,round;


lastUpdate=millis(); //update our custom watchdog

char myrequest[500]; //construct the request AND receive here
char postdata[]="category=mahts";
zero_string(myrequest, 500);

wdt_reset(); //lets tap the doggie..

Serial.print(F("connecting to status server..."));


  //Connect to server
status=0;
i=0;
wdt_disable();
while( i<maxretries && status == 0 ) {
      lastUpdate=millis(); //update our custom watchdog
      wdt_reset(); //lets tap the doggie..
      int W=wifi.createTCP(serverName, HOST_PORT);
      wdt_reset(); //lets tap the doggie..
    if (W) {
        Serial.print(F("create tcp ok\r\n"));\
        status=1;
        //First thing is to zero the no conection counter
        noconnectioncounter = 0;
    } else {
        i++;
        //Serial.print("create tcp err\r\n");
        Serial.print(F("-ERROR-IN-CREATETCP-CODE: "));
        Serial.println(W);
        wifi.releaseTCP();
        delay(2000);
        status=0;
        noconnectioncounter++;
    }
}
wdt_enable(31);
wdt_reset(); //lets tap the doggie..
//Did we managed to connect?
if ( i==maxretries && status==0 ) { software_Reset(); }

if ( noconnectioncounter > maxretries ) { software_Reset(); }
/*
//construct response
strcat(myrequest,"CONNECT zafora.icte.uowm.gr");
strcat(myrequest,"?HTTP/1.0");
sprintf(myrequest +strlen(myrequest), "\r\n");
sprintf(myrequest +strlen(myrequest), "\r\n");*/
/*
wdt_reset(); //lets tap the doggie..
wifi.send((const uint8_t*)myrequest, strlen(myrequest));
wdt_reset(); //lets tap the doggie..
zero_string(myrequest,512);
wdt_reset(); //lets tap the doggie..



//construct response
strcat(myrequest,"POST /CE_lab05/connect.php?ip=");
strcat(myrequest,wifi.getLocalIP().c_str());
strcat(myrequest,"&reversal=1");
strcat(myrequest,"&name=");
strcat(myrequest,lanID);
//strcat(myrequest,"&extra=");
//strcat(myrequest,"&counterrun=");
strcat(myrequest," HTTP/1.0");
sprintf(myrequest +strlen(myrequest), "\r\n");
strcat(myrequest,"User-Agent: ESP82MITSOS");
sprintf(myrequest +strlen(myrequest), "\r\n");
strcat(myrequest,"Host: ");
strcat(myrequest,wwwserverName);
sprintf(myrequest +strlen(myrequest), "\n");
strcat(myrequest,"Connection: closed");
sprintf(myrequest +strlen(myrequest), "\r\n");
Serial.println(myrequest);

wdt_reset(); //lets tap the doggie..
wifi.send((const uint8_t*)myrequest, strlen(myrequest));
wdt_reset(); //lets tap the doggie..
zero_string(myrequest,512);
wdt_reset(); //lets tap the doggie..
*/

//construct response
strcat(myrequest,"GET /~ictest00760/inserted_category.php?category=maths\r\n");
/*
strcat(myrequest,wifi.getLocalIP().c_str());
strcat(myrequest,"&reversal=1");
strcat(myrequest,"&name=");
strcat(myrequest,lanID);*/
strcat(myrequest," HTTP/1.0\r\n");
//strcat(myrequest,"User-Agent: ESP82MITSOS\r\n");
//strcat(myrequest,"Host: ");
//strcat(myrequest,wwwserverName);
//strcat(myrequest,"\r\n");
//strcat(myrequest,"Connection: close\r\n");
//strcat(myrequest,"Content-Type: application/x-www-form-urlencoded\r\n\r\n");
//strcat(myrequest,"Content-Lenght: ");
//char st[]={strlen(postdata)};
//strcat(myrequest,st);
//strcat(myrequest,"14\r\n\r\n");
//strcat(myrequest,postdata);

Serial.println(myrequest);

/*
strcat(myrequest,"GET /~ictest00760/inserted_category.php?");
//strcat(myrequest,wifi.getLocalIP().c_str());
//strcat(myrequest,"&reversal=1");
//strcat(myrequest,"&name=");
//strcat(myrequest,lanID);
strcat(myrequest,"category=maths");
Serial.println(myrequest);
*/
wdt_reset(); //lets tap the doggie..
wifi.send((const uint8_t*)myrequest, strlen(myrequest));
wdt_reset(); //lets tap the doggie..

lastUpdate=millis(); //update our custom watchdog

zero_string(myrequest,512);
wdt_reset(); //lets tap the doggie..

uint32_t len=10;
round=0;
   
wdt_disable(); //Disable watchdog during our wait
Serial.println();
Serial.println(F("Received:["));
while ( len > 0 )
{

  lastUpdate=millis(); //update our custom watchdog
  wdt_reset(); //lets tap the doggie..
  len = wifi.recv((uint8_t*)myrequest, sizeof(myrequest), 10000);
  
  wdt_reset(); //lets tap the doggie..
     if (len > 0) {
       wdt_reset(); //lets tap the doggie..
        for(uint32_t i = 0; i < len; i++) {
             wdt_reset(); //lets tap the doggie..
            //Serial.print((char)buffer[i]);
            Serial.print(myrequest[i]);
            char c;
            //if ( myrequest[i] == 1  && myrequest[i+1] == 13 && myrequest[i+2] == 10 && myrequest[i+3] == 13 )
            if (round==1) //we noticed that in 2nd round we got the data
              {
               //ok now we have the data
                //Serial.print(F(" DATA:["));
                for (byte j = 0; j < nrrelays; j++)
                {
                  lastUpdate=millis(); //update our custom watchdog
                  wdt_reset(); //lets tap the doggie..
                  c=myrequest[i+j];
                  //Serial.print("@");Serial.print(c);Serial.println("@");
                            if ( c - 48 == 1 )
                            {
                            relaystate[j] = 1 ;
                            Serial.println(F("Scheduling state to 1"));
                            }
                          else
                            {
                            relaystate[j] = 0;
                            Serial.println(F("Scheduling state to 0"));
                            }
                //act_on_relays();
                }//end for every byte
                break;
               //round++; //go to next round to get out
              } //end for checking 10/13

        }//end for i up to length

    }//end if len>0
    //zero_string(myrequest,1024);

    wdt_reset(); //lets tap the doggie..
    delay(1000); //wait a bit for data to arrive
    Serial.println(F("+++"));
    round++;
    wdt_reset(); //lets tap the doggie..
    lastUpdate=millis(); //update our custom watchdog
}//end while len>0
wdt_enable(31);

Serial.print(F("]\r\n"));

lastUpdate=millis(); //update our custom watchdog
status=0;
i=0;
//while( i<maxretries && status == 0 )
{
wdt_reset(); //lets tap the doggie..
lastUpdate=millis(); //update our custom watchdog
    if (wifi.releaseTCP()) {
        Serial.print(F("release tcp ok\r\n"));
        status=1;
    } else {
        i++;
        Serial.print(F("release tcp err\r\n"));
        status=0;
    }
wdt_reset(); //lets tap the doggie..
}//end while
wdt_reset(); //lets tap the doggie..
lastUpdate=millis(); //update our custom watchdog
}//end loop







//
//
//
//
//
// SETUP STARTS HERE
//
//
//
//
//



void setup(void)
{
int i=0;
int k;
Serial.begin(9600);

motd();


 startupmillis=millis();

wdt_disable(); //no watchdog pls

Serial.println(F("Checking EEPROM"));
LoadEeprom(2);
//check if we have saved smt
//factory default value of eeprom is 255
if ( eepromatmega[0]  > 0 && eepromatmega[0]!=255 ) {
  Serial.print(F("Reading from EEPROM"));
  //Serial.println(eepromatmega[0]);
  //Serial.println(eepromatmega[1]);
  //put a max value of 10, otherwise error occurs
  eeprom_read_string(SSIDADDRESS, SSID, eepromatmega[0]);
  eeprom_read_string(PSKADDRESS, PASSWORD, eepromatmega[1]);
  Serial.println();
  Serial.print(F("Current SSID: "));
  Serial.println(SSID);
  Serial.print(F("Current PASSWORD: "));
  Serial.println(PASSWORD);
}





    Serial.print(F("Setup begin\r\n"));
    // We first have to reset the ESP
    wdt_reset(); //in the setup lets tap the doggie..


  Serial.print(F("Type akey/enter  to set SSID/PASS within 3 secs"));
  Serial.setTimeout(3000);
  readchars = serialprompt(">");
  wdt_reset(); //in the setup lets tap the doggie..

//Did the user requested to set the time?
//Lets do it..
//OR if nobody has every written here
  if( readchars > 0 || eepromatmega[0]==0 || eepromatmega[0]==255 )
  {
     Serial.println(F("Configuring SSID/PASS"));
wdt_disable(); //disable the watchdog in the setup loop

int lenSSID=serialnumber("SSID length", 1, eepromatmega[0]);
lenSSID++;//to include the \0 also
serialword("SSID", lenSSID , SSID);
int lenPASS=serialnumber("PASS length", 1, eepromatmega[1]);lenPASS++;//to include the \0 also 
serialword("PASS",lenPASS , PASSWORD);
//we want to remove the extra chars on the end
zero_string(&SSID[lenSSID+1],5);
zero_string(&PASSWORD[lenPASS+1],5);
Serial.println();
Serial.print(F("Given SSID:"));
Serial.println(SSID);
Serial.print(F("Given PASSWORD:"));
Serial.println(PASSWORD);



    int save_or_not = serialnumber("Do you want to save it? 0=NO, 1=YES ",1, 0);
    if(save_or_not == 1)
    {

      eepromatmega[0]=lenSSID;
      eepromatmega[1]=lenPASS;
      //we save the info length
      WriteEeprom(3);
      //and also the strings

      eeprom_write_string(SSIDADDRESS, SSID);
      eeprom_write_string(PSKADDRESS, PASSWORD);
     }//end save_block


  }//end the setup procedure

//Let put our 2nd watchdog here
lastUpdate=millis();
Timer1.initialize(1000000); //1 second pulses
Timer1.attachInterrupt(longWDT); //code to execute



// We enterring functions that may carry big delay
// We have to enable the watchdog
Serial.println(F("Init watchdog"));
   wdt_enable(31); //After setup we enable the doogie.
Serial.println(F("Init ESP"));
    wifi.restart();
    wdt_reset(); //in the setup lets tap the doggie..
    delay(5000);
    wdt_reset(); //in the setup lets tap the doggie..

    Serial.print(F("ESP8266 FW Version: "));
    Serial.println(wifi.getVersion().c_str());
    //initially we do not have any connection problems
    noconnectioncounter = 0;

lastUpdate=millis();
    for(k = 0; k < nrrelays; k++)
    {
    //Set as output, with HIGH signal (first write and then set pinMode)
    digitalWrite(relaypins[k], HIGH); //we need this, otherwise the relay will toggle state
    pinMode(relaypins[k], OUTPUT);
    //Default to be off
    //digitalWrite(relaypins[k], LOW);
    //delay(10);
    }

  //States carry init values, so no need to change
  Serial.print(F("Configured relays:"));
  Serial.println(nrrelays);


lastUpdate=millis();
wdt_reset(); //in the setup lets tap the doggie..
Serial.println(F("Setting ESP as client:"));
    if (wifi.setOprToStation()) {
        Serial.print(F("ESP setup as client OK\r\n"));
    } else {
        Serial.print(F("ESP setup as client ERROR\r\n"));
    }
wdt_reset(); //in the setup lets tap the doggie..


i=0;
wdt_reset(); //in the setup lets tap the doggie..
delay(5000); //ESP may like some time to bootup
wdt_reset(); //in the setup lets tap the doggie..
Serial.println(F("Trying to connect to AP"));
wdt_disable();
while(i<maxretries && status == 0 )
{
  lastUpdate=millis();
  wdt_reset(); //in the setup lets tap the doggie.
    if (wifi.joinAP(SSID, PASSWORD)) {
        wdt_reset(); //in the setup lets tap the doggie.
        Serial.print(F("Join AP success\r\n"));
        Serial.print(F("IP: "));
        Serial.println(wifi.getLocalIP().c_str());
        status=1;
    } else {
        wdt_reset(); //in the setup lets tap the doggie.
        i++;
        Serial.print(F("Join AP failure\r\n"));
        status=0;
    }
}//end while
wdt_enable(31);
wdt_reset(); //in the setup lets tap the doggie..


// we did not managed to connect, do a quick reboot
if ( i == maxretries ) { software_Reset(); }

// we did not managed to connect, do a quick reboot
if ( status == 0 ) { software_Reset(); }


 if (wifi.disableMUX()) {
        Serial.print(F("single connection only ok\r\n"));
    } else {
        Serial.print(F("single connection err\r\n"));
    }
wdt_reset(); //in the setup lets tap the doggie..

  //set maximum resolution
  //9->delay(94)
  //10->delay(188)
  //11->..375
  //12->..750







    Serial.print(F("setup end\r\n"));
wdt_reset(); //in the setup lets tap the doggie..
lastUpdate=millis();
}










void loop(void)
{

    wdt_reset(); //in the setup lets tap the doggie..


    int i=0;

    Serial.println();
    //act_on_relays();
    examine_serial_and_reboot();

   if ( millis() - startupmillis > 86400000 )
    {
    //a full day has passed...time for a reboot
    software_Reset();
    }

   if ( millis() - lastConnectionTime > postingInterval)
    {
    lastConnectionTime = millis(); //update the connection counter
    counterrunprevious=counterrun;
    wireless_process(); //check the server
    counterrun=counterrun+1;
        if ( counterrunprevious > counterrun)
       {//wrap arround has occured, init it to a bigger value not to indicate reboot
        Serial.println(F("Wrap occured"));
       counterrun=10;
       }
   Serial.print(F("Iteration Counter: "));
   Serial.println(counterrun);
  }


 delay(4000);
}


