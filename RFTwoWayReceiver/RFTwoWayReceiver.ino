

/**
 * Example code for using a microchip mrf24j40 module to send and receive
 * packets using plain 802.15.4
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
 */
 
#include <SPI.h>  // SPI library
#include "mrf24j.h" // RF module library.

const int pin_reset = 6;
const int pin_cs = 8; // default CS pin on ATmega8/168/328
const int pin_interrupt = 2; // default interrupt pin on ATmega8/168/328
const int pin_sclock = 13; // default interrupt pin on ATmega8/168/328

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt, pin_sclock);

long last_time;
long tx_interval = 1000;

void setup() {
  Serial.begin(9600); 
  
  mrf.reset();
  mrf.init();
  //Network Setup:
  //mrf.set_channel(12); // 802.15.4 channel number (11 to 26) 
  mrf.set_pan(0xcafe); // network ID
  mrf.address16_write(0x4202);// Node Address
  
  // uncomment if you want to receive any packet on this channel
  mrf.set_promiscuous(true);
  
  // uncomment if you want to enable PA/LNA external control
  //mrf.set_palna(true);

   // uncomment if you want to buffer all PHY Payload
  //mrf.set_bufferPHY(true);
  
  pinMode(pin_interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_interrupt), interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
  last_time = millis();
  //interrupts();
}

void interrupt_routine() {
    mrf.interrupt_handler(); // mrf24 object interrupt routine
}

void loop() {
     mrf.check_flags(&rx_handle, &tx_handle);
     Serial.println("Listening");
     delay(1000); 
     //Serial.println(mrf.get_pan());
     //Serial.println(mrf.address16_read());
}

void rx_handle() {
    Serial.print("received a packet ");Serial.print(mrf.get_rxinfo()->frame_length, DEC);Serial.println(" bytes long");
    
    if(mrf.get_bufferPHY()){
      Serial.println("Packet data (PHY Payload):");
      for (int i = 0; i < mrf.get_rxinfo()->frame_length; i++) {
          Serial.print(mrf.get_rxbuf()[i]);
      }
    }
    
    Serial.println("\r\nASCII data (relevant data):");
    for (int i = 0; i < mrf.rx_datalength(); i++) {
        Serial.write(mrf.get_rxinfo()->rx_data[i]);
    }
    
    Serial.print("\r\nLQI/RSSI=");
    Serial.print(mrf.get_rxinfo()->lqi, DEC);
    Serial.print("/");
    Serial.println(mrf.get_rxinfo()->rssi, DEC);
    }

void tx_handle() {
    if (mrf.get_txinfo()->tx_ok) {
        Serial.println("TX went ok, got ack");
    } else {
        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    }
}
