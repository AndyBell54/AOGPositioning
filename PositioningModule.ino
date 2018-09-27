#include <Wire.h>
//#include <EtherCard.h>
#include <IPAddress.h>
#include <AltSoftSerial.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define UDP_TX_PACKET_MAX_SIZE 200
#define IP_LEN 4

#define CMPS12_ADDRESS 0x60 //I2C address of CMPS12 shifted right one bit for arduino wire library
#define HEADING_16 2         //Register to read 8bit heading angle from

#define GPS_PGN_HI 0x80     //PGN for GPS message - High byte
#define GPS_PGN_LO 0x00     //PGN for GPS message - Low byte

#define GPS_BUFFER_SIZE 82  //Maximum size allocated for NMEA sentence

AltSoftSerial mySerial;		//Instance of AltSoftSerial to receive NMEA messages from receiver

char gpsBuffer[GPS_BUFFER_SIZE];	//Buffer for receiving gps serial communications
bool newSentence = false;			//Flag to indicate beginning of new sentence

long lat, lon;
unsigned long chars;

int dog2Pin = A1;
unsigned char heading16Hi, heading16Lo;	//High and low bytes to store 16bit heading data
unsigned int heading16;					//16bit heading value

const unsigned int LOOP_TIME = 100; //10hz
unsigned int lastTime = LOOP_TIME;
unsigned int currentTime = LOOP_TIME;
unsigned int dT = 50000;
unsigned int sensorReadings = 4;

//Kalman variables
float rollK = 0, Pc = 0.0, G = 0.0, P = 1.0, Xp = 0.0, Zp = 0.0;
int XeRoll = 0;
const float varRoll = 0.1; // variance,
const float varProcess = 0.005; //smaller is more filtering

//inclinometer variable
int roll = 0;

//Array to send data back to AgOpenGPS
byte IMUtoSend[] = {0,0,0,0,0,0,0,0,0,0};

// ethernet interface ip address
static byte myip[] = { 192,168,100,90 };
// gateway ip address
static byte gwip[] = { 192,168,100,254 };
//DNS- you just need one anyway
static byte myDNS[] = { 8,8,8,8 };
//mask
static byte mask[] = { 255,255,255,0 };
//this is port of this module
unsigned int portIMU = 5566;
unsigned int portGPS = 5555;

//sending back to where and which port
static byte ipDestination[] = { 192, 168, 100, 255 };
unsigned int portDestination = 9999;

// ethernet mac address - must be unique on your network
static byte mymac[] = { 0x70,0x71,0x71,0x2D,0x30,0x31 };

//byte Ethernet::buffer[400]; // udp send and receive buffer
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
unsigned int localPort = 8888;  //local port to listen on

EthernetUDP UdpImuSEND;		//EthernetUDP instance to send IMU data on port 5566
EthernetUDP UdpGpsSEND;		//EthernetUDP instance to send GPS data on port 5555
EthernetUDP UdpRECEIVE;		//EthernetUDP instance to receive AOG data on port 8888

void setup() {
  //set up communication
  //Wire.begin();
  Serial.begin(57600);
  //Serial.println("Serial port initialized");

  // set the data rate for the AltSoftSerial port
  mySerial.begin(9600);

  //set up the pgn for returning IMU data
  IMUtoSend[0] = 0x7F;
  IMUtoSend[1] = 0xEE;

  Ethernet.begin(mymac, myip, myDNS, gwip, mask);
  
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  
  UdpImuSEND.begin(portIMU);
  UdpGpsSEND.begin(portGPS);
  UdpRECEIVE.begin(localPort);
  
/*
 //Code for ethercard module
 if (ether.begin(sizeof Ethernet::buffer, mymac) == 0)
    Serial.println(F("Failed to access Ethernet controller"));

  //set up connection
  ether.staticSetup(myip, gwip, myDNS, mask); 
  ether.printIp("IP:  ", ether.myip);
  ether.printIp("GW:  ", ether.gwip);
  ether.printIp("DNS: ", ether.dnsip);

  //register udpSerialPrint() to port 8888
  ether.udpServerListenOnPort(&udpIMU, 8888);
  
*/
}

void loop() {
  currentTime = millis();
  unsigned int time = currentTime;

  if (currentTime - lastTime >= LOOP_TIME) {
    dT = currentTime - lastTime;
    lastTime = currentTime;
/*
	//CMPS12 - Compass
    Wire.beginTransmission(CMPS12_ADDRESS); //starts communication with CMPS12
    Wire.write(HEADING_16);                  //sends the register we want to start reading from
    Wire.endTransmission();
  
    Wire.requestFrom(CMPS12_ADDRESS, 2);
    while(Wire.available() < 1);
    heading16Hi = Wire.read();
    heading16Lo = Wire.read();
    heading16 = heading16Hi << 8;
    heading16 += heading16Lo;
    Serial.print(heading16); Serial.print("\t");
*/

    delay(1);
    analogRead(dog2Pin); //discard
    delay(1);
    roll = analogRead(dog2Pin);   delay(2);
    roll += analogRead(dog2Pin);   delay(2);
    roll += analogRead(dog2Pin);   delay(2);
    roll += analogRead(dog2Pin);
    roll = roll >> 2; //divide by 4

    //inclinometer goes from -45 to 45 from 0.5 volts to 4.5 volts
    rollK = map(roll, 0, 1023, -900, 900); //20 counts per degree * 16
    rollK *= 1.25;

    //Kalman filter
    Pc = P + varProcess;
    G = Pc / (Pc + varRoll);
    P = (1 - G) * Pc;
    Xp = XeRoll;
    Zp = Xp;
    XeRoll = G * (rollK - Zp) + Xp;

    int temp;
  
    //Vehicle roll --- * 16 in degrees
    temp = XeRoll;
    IMUtoSend[6] = (byte)(temp >> 8);
    IMUtoSend[7] = (byte)(temp);

    //send IMU to AOG
//    ether.sendUdp(IMUtoSend, sizeof(IMUtoSend), portIMU, ipDestination, portDestination);
    if(UdpImuSEND.beginPacket(ipDestination, portDestination)) {
      UdpImuSEND.write(IMUtoSend, sizeof(IMUtoSend));
      UdpImuSEND.endPacket();
    }
  
  } //end of timed loop

	//Read serial GPS data
  int i;
  	while (mySerial.available()) {

	  char c = mySerial.read();

	  if (c == 0x24) {     //if character is '$' start gpsBuffer from beginning
		  newSentence = true;
		  i = 0;
	  }
	  if (c == 0x0d && newSentence) {  //if character is carriage return (\n) build UDP send buffer
		  byte GPStoSend[i + 2];
		  GPStoSend[0] = 0x80;
		  GPStoSend[1] = 0x00;

		  for (int j = 0; j < i; j++) {
			  GPStoSend[j + 2] = (byte)gpsBuffer[j];
		  }

		  udpGpsSend(GPStoSend, sizeof(GPStoSend));

		  i = 0;
		  newSentence = false;

		  //Serial output NMEA strings
		  Serial.write(GPStoSend, sizeof(GPStoSend));
		  Serial.println();
	  }
	  if (newSentence && i < GPS_BUFFER_SIZE) {
		  gpsBuffer[i++] = c;
	  }
  }
} //end of main loop

//callback when received packets
void udpIMU(uint16_t dest_port, uint8_t src_ip[IP_LEN], uint16_t src_port, byte *data, uint16_t len)
{
  /* IPAddress src(src_ip[0],src_ip[1],src_ip[2],src_ip[3]); 
  Serial.print("dPort:");  Serial.print(dest_port);
  Serial.print("  sPort: ");  Serial.print(src_port); 
  Serial.print("  sIP: ");  ether.printIp(src_ip);  Serial.println("  end");*/

  //for (int i = 0; i < len; i++) {
    //Serial.print(data[i],HEX); Serial.print("\t"); } Serial.println(len);

    if (data[0] == 0x7F && data[1] == 0xEF) //Data
    {

    }
}

//UDP function to send GPS data
void udpGpsSend(byte *data, uint8_t len)
{
	if (UdpGpsSEND.beginPacket(ipDestination, portDestination)) {
		UdpGpsSEND.write(data, len);
		UdpGpsSEND.endPacket();
	}
}
