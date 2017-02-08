////////////////////////////////////////////////
///////////// Who am I?
// Universitat Politècnica de Catalunya (UPC - BarcelonaTECH) - ESEIAAT
// EUROAVA TERRASSA// UPSC SPACE PRGOGRAM
// CODE DEVELOPED FOR "EVERY CHILD WANTS TO BE AN ASTRONATUT" UPC SPACE PROGRAM
// Iridium + GPS UBLOX8M + BMP180 + 2 Relays(optional) + LM36
// This code is meant to be used in HABs type missions.
// It consists a IRIDIUM modem, to send messages over the world, a GPS with flight mode,
// a BMP180 barometric-temperature sensor and 2 realys (to ignitate/activate peripherals, if necessary)
///////////// What we are sending
// We send time, longitude (N/S), latitutde (E/W), 
// number of GPS satel used, altitude GPS, state (0 or 1, depending on if we have GPS signal)
// Temperature BMP180, altitude BMP180 (just in case GPS altitude is not so accurated) and temp from LM36 (outside). Also, redundant systems are always good.
///////////// Instructions
// First, configure IRIDIUM. Make sure you have enough credits to use this modem, going to https://rockblock.rock7.com/Operations 
// and login. Please make sure that if we are sending very large messages, we could spend 2 credits instead of 1. 
// Calculte total time of the launch, add a safety factor of 1.5, and buy messages enough to fullfill the mission.
// IT IS NOT RECCOMENDED BUY CREDITS WHEN IRIDIUM IS WORKING. IRIDIUM COULD STOP WORKING. BUY CREDITS BEFORE THE MISSION!  
///////////// 
// Questions, please refer to marc.cortes.fargas@upcprogrm.space (prefered) or marc.c.fargas@gmail.com
// Made by Marc Cortés Fargas.
// This code is under GNU GENERAL PUBLIC LICENSE. This program comes with absolutely no warranty, provided as it is. 
// Sparkfun's BMP180 library is used, as well as PString and IRIDIUMSBD developed for others. All credit for 
// this libraries to their owners!
//
//
// REV-25.01.201
////////////////////////////////////////////////





//--- Include libraries and initialize variables ---//

//Include libraries

#include <Wire.h>
#include <PString.h>
#include <SoftwareSerial.h>
#include <IridiumSBD.h>
#include <SFE_BMP180.h>
#include <OneWire.h>
#include <DallasTemperature.h>



////////////////////////////////////////////////////////////////////////////////////////BMP180
// Standard I2C connection of BMP180 sensor

SFE_BMP180 pressure;
double p0 = 1015; // pressure from we are calculating, in mbar. 1015mbar is pressure at sea level.


/////////////////////////////////////////////////////////////////////////////////////////GPS constants

char altitut [20];
  byte bytegps = 0;      // Will store each byte that comes through the serial.
  char datagps[100]="";  // Array of characters to store each line of data.
  int i = 0;             // datagps index
  byte gps_set_sucess = 0 ;
  bool boyan;
  int coma1;
  int coma2;
  char latitut2 [15];
  char longitut2 [15];
  int comas [15];
  char NMEA [10];
  char hora [15];
  char NS[5];
  char EW[5];
  char fix[5];
  char satel[5];
  int n=0;
  bool gps_trobat = false;
  int state = 0;
  #define txPin 10      //tx pin in gps connection
  #define rxPin 9     //rx pin in gps connection
///////////////////////////////A led turns off when GPS signal is acquired and we start sending messages via Iridium
  int led_state = 6; //DIGITAL PIN 6 to STATE LED
  
////////////////////////////////////

///////////////////////////////////////////////DALLASWS2801
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/////////////////////////////////////////////////////

/////////////////////////////////////////////////Rely on the relay. Just two digital OUT pins to use "recieve" option of IRIDIUM.
  int rele2 = 11; //pin to rele1
  int rele1 = 12; //pin to rel2
////////////////////////////////////////////////


// Software Serials
SoftwareSerial gps = SoftwareSerial(rxPin, txPin); //Serial to connect to the GPS (Rx,Tx) digital 10 to GPS RX and digital 9 to GPS TX


/////////////////////////////////////////////////IRIDIUM
SoftwareSerial iridiumSerial(7, 8); //Serial to connect to the Iridium model (Rx,Tx) digital 12 to IRIDIUM RX and digital 11 to IRIDIUM TX
IridiumSBD isbd(iridiumSerial, 5);




void setup() {
//Serial.begin(9600); //start (or not) Serial communication with computer. 
iridiumSerial.begin(19200);
pinMode(led_state,OUTPUT);

///////////////////BMP180

pressure.begin();
//////////////////END BMP180

/////////////////DALLAS
sensors.begin();
/////////////////END DALLAS

//////////////////Setup PINS GPS
pinMode(rxPin, INPUT);
pinMode(txPin, OUTPUT);
Serial.begin(9600);
Serial.println("Start looking GPS");
  check_state(); //turning on/off a LED depending on GPS signal. if GPS, off. 
  gps_flightmode();//start GPS in flight mode
  while (state == 0){gpsloop();} //make sure we have latitude and longitude before continue.
  check_state(); //GPS off
  Serial.println("GPS Found");
  
//////////////////IRIDIUM
isbd.attachConsole(Serial);
isbd.attachDiags(Serial);
isbd.setPowerProfile(0);




}










void loop() {

isbd.adjustSendReceiveTimeout(120); // 
Serial.println("Enter loop");
/////////////////////////////////////////BMP180
double T, P, a;
char status;
status = pressure.startTemperature();
if (status != 0)
{
    // Wait for the measurement to complete:
    delay(status);

    status = pressure.getTemperature(T);}
    Serial.print("temperature: ");
    Serial.print(T,2);
    Serial.print(" deg C, ");
    status = pressure.startPressure(3);
    if (status != 0)
    {
        // Wait for the measurement to complete:
        delay(status);


        status = pressure.getPressure(P,T);}
        Serial.print("absolute pressure: ");
        Serial.print(P,2);
        Serial.print(" mb, ");
        a = pressure.altitude(P,p0);
        Serial.print("computed altitude: ");
        Serial.print(a,0);




/////////////////////////////////////////END BMP180

////////////////////////////////////////DALLAS

sensors.requestTemperatures();
double dallas_temp = sensors.getTempCByIndex(0);
Serial.println("Dallas temp is");
Serial.println(dallas_temp);


///////////////////////////////////////END DALLAS



///////////////////////////////////////GPS
boyan = false;
while (boyan == false){ gpsloop();}//Only send a message if we have GPS properly connected. (redundant, just in case)
check_state(); //turning on/off a LED depending on GPS signal.

Serial.println(hora);
//Serial.print(latitut2);RAW latitut
Serial.print("  ");
Serial.println(NS);
//Serial.print(longitut2); RAW longitut
Serial.print("  ");
Serial.println(EW);
Serial.println(fix);
Serial.println(satel);
Serial.println(altitut);
Serial.println(state);

///////////////////////////////////////GPS end


 ///////////////////////////////////////IRIDIUM
 bool ISBDCallback();
 iridiumSerial.listen();

  //  Start talking to the RockBLOCK and power it up
  Serial.println("Beginning to talk to the RockBLOCK...");
  

  if (isbd.begin() == ISBD_SUCCESS)
  {
    char outBuffer[100]; // Always try to keep message short
    PString mssg(outBuffer, sizeof(outBuffer)); 
    // mssg.print(hora);
    // mssg.print(",");
    // mssg.print(latitut2); // sendig RAW latitut
    // converting latitut2 and longitut2 to google-friendly data
    float lat_degInt = float(int(atof(latitut2)/100));
    float lat_degDec = float((atof(latitut2)-lat_degInt*100)/60);
    float lat_easy = lat_degInt + lat_degDec;
    float long_degInt = float(int(atof(longitut2)/100));
    float long_degDec = float((atof(longitut2)-long_degInt*100)/60);
    float long_easy = long_degInt + long_degDec;
    
    
    // Adjusting all message to one array
    mssg.print(lat_easy,4); // 4 means significant decimals. 
   // mssg.print(char(176)); //degree symbol;
   mssg.print(NS);
   mssg.print(",");
    // mssg.print(longitut2); // sending RAW longitut
    mssg.print(long_easy,4); // 4 means significant decimals. 
  //  mssg.print(char(176));;
  mssg.print(EW);
    // mssg.print(",");
    // mssg.print(fix);
    mssg.print(",");
    mssg.print(atof(altitut),0);
    mssg.print(",");
    mssg.print(T,0);
    mssg.print(",");
    mssg.print(dallas_temp,1);
    mssg.print(",");
    mssg.print(a,0);
    mssg.print(",");
    mssg.print(satel);
    mssg.print(",");
    mssg.print(state);
    
    
    Serial.println("lat is");
    Serial.println(latitut2);
    Serial.println("latieasey is");
    Serial.println(lat_easy);

    Serial.print("Transmitting message: ");
    Serial.println(outBuffer);
//    uint8_t buffer[1] = 
//    { 1 };
//    size_t  bufferSize = sizeof(buffer);
//    isbd.sendReceiveSBDText(outBuffer, buffer, bufferSize);
//    Serial.println(buffer[0]);
//    if (buffer[0]!=1){pinMode(rele1,HIGH); delay(5000);pinMode(rele1,LOW); buffer[0] = 1; Serial.println("FIRE!");} //if we send anything through iridium, rele1'll be high for 5 seconds. 

isbd.sendSBDText(outBuffer);



}


//////////////////////////////////////END IRIDIUM

Serial.println("Ending loop");
delay(5000); //manual delay (iridium's delay does now always work properly

}





void gps_flightmode(){
  gps_set_sucess = 0 ;     
  gps.begin(9600); 
  // START OUR SERIAL DEBUG PORT
  Serial.begin(9600);
  Serial.println("GPS Level Convertor Board Test Script");
  Serial.println("03/06/2012 2E0UPU");
  Serial.println("Initialising....");
  //
  // THE FOLLOWING COMMAND SWITCHES MODULE TO 4800 BAUD
  // THEN SWITCHES THE SOFTWARE SERIAL TO 4,800 BAUD
  //
  gps.print("$PUBX,41,1,0007,0003,4800,0*13\r\n"); 
  gps.begin(4800);
  gps.flush();

  //  THIS COMMAND SETS FLIGHT MODE AND CONFIRMS IT 
  Serial.println("Setting uBlox nav mode: ");
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                        };
    while(!gps_set_sucess)
    {
      sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
      gps_set_sucess=getUBX_ACK(setNav);
    }
    gps_set_sucess=0;

  // THE FOLLOWING COMMANDS DO WHAT THE $PUBX ONES DO BUT WITH CONFIRMATION
  // UNCOMMENT AS NEEDED
  
  Serial.println("Switching off NMEA GLL: ");
  uint8_t setGLL[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B                   };
   while(!gps_set_sucess)
   {    
     sendUBX(setGLL, sizeof(setGLL)/sizeof(uint8_t));
     gps_set_sucess=getUBX_ACK(setGLL);
   }
   gps_set_sucess=0;
   Serial.println("Switching off NMEA GSA: ");
   uint8_t setGSA[] = { 
     0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32                   };
     while(!gps_set_sucess)
     {  
       sendUBX(setGSA, sizeof(setGSA)/sizeof(uint8_t));
       gps_set_sucess=getUBX_ACK(setGSA);
     }
     gps_set_sucess=0;
     Serial.println("Switching off NMEA GSV: ");
     uint8_t setGSV[] = { 
       0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39                   };
       while(!gps_set_sucess)
       {
         sendUBX(setGSV, sizeof(setGSV)/sizeof(uint8_t));
         gps_set_sucess=getUBX_ACK(setGSV);
       }
       gps_set_sucess=0;
//   Serial.print("Switching off NMEA RMC: ");
//   uint8_t setRMC[] = { 
//   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40                   };
//   while(!gps_set_sucess)
//   {
//   sendUBX(setRMC, sizeof(setRMC)/sizeof(uint8_t));
//   gps_set_sucess=getUBX_ACK(setRMC);
//   }

}




// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    gps.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  gps.println();
}


// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B

  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }

    // Make sure data is available to read
    if (gps.available()) {
      b = gps.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }

    }
  }
}





/////////////////////////////////////////////////////////////// AUXILIAR FUNCTIONS GPS
void gpsloop(){
 gps.flush();
 gps.begin(4800);

  // Prepare all for reading gps Serial Port
  memset(datagps, 0, sizeof(datagps));    // Remove previous readings
  bytegps = 0;                            // Remove data
  bytegps = gps.read();
 // Serial.println(bytegps);// Read the byte that is in the gps serial port
 

  // Find the desired string
  while(bytegps != '$'){
 //   Serial.println(gps.read());
 bytegps = gps.read();
}


  //Serial.println("New gps data found!");
  
  // Save the string in an array
    // First character of the array: '$'
    datagps[0] = '$';
    // gps data (GPGGA,...)
    i=1;
    while(bytegps != '*' ){
      bytegps = gps.read();
      // Some 255 characters appear among the characters of our interest.
        // We will make sure none of this characters is stored.
        if(bytegps != 255 ){
        //Serial.println(bytegps);
        datagps[i]=bytegps;
        i++;
      }
    }
  //  Serial.println("boyan0");
    // Checksum (last two numbers in HX code) -- This could be commented if not used
    while(bytegps != '\r' ){ // Also 13 is '\r' character (Carriage Return) in ASCII code
    bytegps = gps.read();
    if(bytegps != '\r' && bytegps != 255 ){
      datagps[i]=bytegps;
      i++;
    }
  }
        //Serial.println("boyan1");

  // Print all the datagps received
  //print_datagps();
  
  // Is GPGGA?
//Serial.print(datagps);
if (datagps[4]=='G'){print_datagps(); 
  Serial.print(datagps); 
  boyan = true; }
 // if (datagps[4]=='M'){print_datagps(); Serial.print(datagps); Serial.println("Hem trobam GNGGA"); Serial.println("Latitut   "); Serial.println(latitut); Serial.println("Longitut   "); Serial.println(longitut); boyan = true;}
 
}

void print_datagps(){
 state = 0;

 Serial.println("");
 gps_trobat = false;


 n = 0;
 for(int m = 0; m<100; m++){

  if(datagps[m]==','){comas[n]=m; n = n+1;

    }}



////////////////////////////////////////////CHECK IF WE HAVE LATITUDE, LONGITUDE AND TIME. 
if(comas[2]-comas[1] == 11 && comas[1]-comas[0] == 10  && comas[4]-comas[3]== 12){
  clean_all();
  gps_trobat = true;
  state = 1;
//////////////////////////////////////////FIND AND SAVE VARIABLES SUCH AS TIME, LATITUDE, ALTITUDE...  
///////////////////////////FIND TIME

for(int o = comas[0]-1; o <comas[1];o++){

  hora[o-comas[0]-1]=datagps[o];
}

///////////////////////////FIND LATITUDE2
for(int o = comas[1]-1; o <comas[2];o++){
  latitut2[o-comas[1]-1]=datagps[o];
}
///////////////////////////FIND N/S indicator
for(int o = comas[2]-1; o <comas[3];o++){
  NS[o-comas[2]-1]=datagps[o];
}

///////////////////////////FIND longitut
for(int o = comas[3]-1; o <comas[4];o++){
  longitut2[o-comas[3]-1]=datagps[o];
}

///////////////////////////FIND EW
for(int o = comas[4]-1; o <comas[5];o++){
  EW[o-comas[4]-1]=datagps[o];
}
///////////////////////////FIND fix
for(int o = comas[5]-1; o <comas[6];o++){
  fix[o-comas[5]-1]=datagps[o];
}

///////////////////////////FIND satel
for(int o = comas[6]-1; o <comas[7];o++){
  satel[o-comas[6]-1]=datagps[o];
}

///////////////////////////FIND altitut
for(int o = comas[8]-1; o <comas[9];o++){
  altitut[o-comas[8]-1]=datagps[o];
}
}


if(gps_trobat == false){Serial.println("No tenim GPS. Enviem dades antigues."); state = 0;}
else{Serial.println("Tenim GPS");}
//Serial.println(hora);
//Serial.print(latitut2);
//Serial.print("  ");
//Serial.println(NS);
//Serial.print(longitut2);
//Serial.print("  ");
//Serial.println(EW);
//Serial.println(fix);
//Serial.println(satel);
//Serial.println(altitut);
//Serial.println(state);



}

void clean_all(){

  memset(hora, 0, sizeof(hora));
  memset(latitut2, 0, sizeof(latitut2));
  memset(longitut2, 0, sizeof(longitut2));
  memset(altitut, 0, sizeof(altitut));
  memset(EW, 0, sizeof(EW));
  memset(NS, 0, sizeof(NS));
  memset(fix, 0, sizeof(fix));
  memset(satel, 0, sizeof(satel));
  
  
}

void check_state(){
  pinMode(led_state, OUTPUT);
  if (state == 1){digitalWrite(led_state, LOW);}
  if (state == 0 ){digitalWrite(led_state,HIGH);}
  
}

