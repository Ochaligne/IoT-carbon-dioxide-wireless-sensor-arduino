//Sensor Libraries and Variables Declaration.
#include <SoftwareSerial.h> //Serial port library.
#include "cozir.h" //CO2 sensor library.
SoftwareSerial nss(0,1); //Rx Tx pin assignement.
COZIR czr(nss); //Pass serial pins to the CoZIR library.
String val= ""; //Holds the string of the value.
int co2 =0; //Holds the actual CO2 value.
double multiplier = 1; //each range of sensor has a different value.
uint8_t buffer[25]; //buffer to read data from serial port.
uint8_t ind =0; //index variable to ititerate over buffer.


//MRF24J40MA Libraries, Pin Assignment and Variables Declaration.
#include <SPI.h> //SPI port library.
#include "mrf24j.h" //MRF24J40MA Library.
const int pin_reset = 6; //Reset pin assignement.
const int pin_cs = 8; //CS pin assignement.
const int pin_interrupt = 2; //Interrupt pin assignement.
const int pin_sclock = 13; //Clock output pin assignement.
Mrf24j mrf(pin_reset, pin_cs, pin_interrupt, pin_sclock); //Pass pins to MRF24J library.
char msg_send[4]; //string storing the data to be sent
long last_time; //
long tx_interval = 1000;

void setup() {
//Sensor Setup 
 Serial.begin(9600); //Start Serial for programme status display.
 Serial1.begin(9600); //Start Serial connection with Sensor. 
 delay(3000);
 //czr.CalibrateFreshAir(); //Uncomment to use autocalibration.
 czr.SetOperatingMode(CZR_POLLING); //Set sensor to polling mode.
 //czr.SetOperatingMode(CZR_STREAMING); //Uncomment to use streaming mode.
 
//RF Module setup
  mrf.reset(); //Perform a hardware reset. 
  mrf.init(); //Initilisation routine of the transceiver.
  mrf.set_promiscuous(true); //Receive any packet on this channel
//Network Setup:
  mrf.set_pan(0xcafe); //Network ID
  mrf.address16_write(0x6001); //Node Address
//Interrupts setup
  attachInterrupt(digitalPinToInterrupt(pin_interrupt), interrupt_routine, LOW); //interrupt 0 equivalent to pin 2(INT0).
  last_time = millis(); //Time to set application start time (post setup).
}
//Interrupt Routine for RF Module
void interrupt_routine() {
    mrf.interrupt_handler(); //mrf24 interrupt routine from library.
}

void loop() {
//--------------------------------------------------//
//Get CO2 reading and package to a variable msg_send.
//--------------------------------------------------//
//Read incoming bytes into a buffer until we get '0x0A'(ASCII value for new-line).
  Serial1.println("Z\r\n"); //Send request command for CO2.
    while(buffer[ind-1] != 0x0A) //While previous character not "new-line".
   {
    if(Serial1.available()) //check if serial is ready.
    {
      buffer[ind] = Serial1.read(); //Characters from serial to buffer.
      ind++; // increase index for each character.
   }
   }
   unpack(); //Once character = '0x0A', unpack the buffer data.
//--------------------------------------------------//
//Package and Send Message with RF Module.
//--------------------------------------------------//
     mrf.check_flags(&rx_handle, &tx_handle); //Check flags to launch appropriate routine.
     unsigned long current_time = millis(); // Get current time.
     if (current_time - last_time > tx_interval) { //Constrain transmission to once every second.
         Serial.println("broadcasting..."); //Print broadcasting state.
         mrf.send16(0x4202, msg_send);  //Send string to defined address .     
    }
//Message is sent, reset the buffer values.    
    ind=0; //Reset the buffer index. 
    val=""; //Reset the value string.
}
void unpack()
{
//--------------------------------------------------//
//Unpack the buffer to get the CO2 reading value.
//--------------------------------------------------//
   for(int i=0; i < ind+1; i++) //Unpacking loop iteration.
   {
   if(buffer[i] == 'z') //once we hit the 'z' character stops the unpacking loop.
   break;
  
   if((buffer[i] != 0x5A)&&(buffer[i] != 0x20)) //Ignore 'Z' and spaces.
    {
   val += buffer[i]-48; //Subtract 48 from each numerical character to get to the actual numerical value.
      }
  }

 co2 = (multiplier * val.toInt()); //val.toInt()now we multiply the value by a factor specific to the sensor as per Software Guide.
 //Print the CO2 value to serial monitor.
 Serial.println( "Co2 = ");
 Serial.print(co2);
 Serial.println(" ppm");
 Serial.println("<----------->");
 delay(1000);
 itoa(co2, msg_send, DEC); //Convert to string array msg_send.
 //Serial.println(msg_send); // Uncomment to print the value to be sent.
 }
//--------------------------------------------------//
//Reception of packet magement routine for RF Module.
//--------------------------------------------------//
void rx_handle() { //Print data from TX FIFO stored in buffers.
    Serial.print("received a packet ");Serial.print(mrf.get_rxinfo()->frame_length, DEC);Serial.println(" bytes long");
    
    if(mrf.get_bufferPHY()){
      Serial.println("Packet data (PHY Payload):"); //Print all characters in the data.
      for (int i = 0; i < mrf.get_rxinfo()->frame_length; i++) { //iterate over each character in the buffer until the end of the frame.
          Serial.print(mrf.get_rxbuf()[i]);
      }
    }
    
    Serial.println("\r\nASCII data (relevant data):"); //Print the message send (CO2 value).
    for (int i = 0; i < mrf.rx_datalength(); i++) {//iterate over each character in the buffer for the length of the data buffer.
        Serial.write(mrf.get_rxinfo()->rx_data[i]);
    }
    
    Serial.print("\r\nLQI/RSSI="); //Print signal related information (LQI and RSSI).
    Serial.print(mrf.get_rxinfo()->lqi, DEC);
    Serial.print("/");
    Serial.println(mrf.get_rxinfo()->rssi, DEC);
}

void tx_handle() { //Confirmation of good transmission of packet.
    if (mrf.get_txinfo()->tx_ok) {
        Serial.println("TX went ok, got ack");
    } else {
        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    }

}
