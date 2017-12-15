#include <Arduino.h>
#include <platform.h>

#include <OneWireHub.h>
#include "DS2431.h"

#define IN_PIN 2               // Interrupt off of D2 for arduino nano
volatile int val = 0;                 // digital value read from interrupt pin

unsigned long last_time = 0;
volatile int edge_count = 0;
float freq = 0;  

// One-Wire Slave
constexpr uint8_t pin_onewire{ PD6 };
auto hub = OneWireHub(pin_onewire);
auto ds2431 = DS2431(DS2431::family_code, 0x00, 0x00, 0x31, 0x24, 0xDA, 0x00);
byte mem[4];

void setup() {
  noInterrupts();           // disable all interrupts

  /* One-wire slave set-up*/
  hub.attach(ds2431);
  //mem[0]=244;
  //ds2431.writeMemory(mem, 4);
  
  /* interrupt set-up */
  Serial.begin(9600);                   // baud rate: 9600 for serial communication (debug)
  Serial.println("Initializing Pin Modes and Interrupt Handler");
  pinMode(IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IN_PIN), count, RISING);
  interrupts();             // enable all interrupts
  
}
void loop() {
    hub.poll(); // check what's going on in the one-wire bus.
    unsigned long elapsed_time;
    unsigned long current_time = millis();
    elapsed_time = current_time - last_time;
  if (elapsed_time >= 5000) {
    int temp_count = edge_count;
    edge_count -= temp_count;
    last_time = current_time;
    
    //edge_count = 0;
    //last_time = current_time;
    
    freq = ((float)temp_count/(float)elapsed_time)*1000; // get frequency in Hz
    Serial.print("Elapsed Time:  \t");
    Serial.println(elapsed_time);
    Serial.print("counts:  \t");
    Serial.println(temp_count);
     Serial.print("counts:  \t");
    Serial.print(freq);
    Serial.println(" Hz");
    
    // put frequency in memory buffer and write to virtual ds2431 memory
    int freq_int = ((int)freq)*100; 
    mem[0]= (freq_int & 0xFF);
    mem[1]= ((freq_int >> 8) & 0xFF);
    ds2431.writeMemory(mem, 4);

    //int[] myInts = (int[])(mem);
    
  }
}

void count() {
  edge_count++;
}
