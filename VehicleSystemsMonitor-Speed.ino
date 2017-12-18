#include <Arduino.h>
#include <platform.h>

#include <OneWireHub.h>
#include "DS2431.h"

#define ONEWIRE PB0
#define OC1A PB1
#define IN_PIN PB2               // Interrupt off of D2 for arduino nano
volatile int val = 0;            // digital value read from interrupt pin

unsigned long last_time = 0;
volatile int edge_count = 0;
float freq = 0;

// One-Wire Slave
constexpr uint8_t pin_onewire{ ONEWIRE };
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
    //Serial.begin(9600);                   // baud rate: 9600 for serial communication (debug)
    //Serial.println("Initializing Pin Modes and Interrupt Handler");
    pinMode(IN_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IN_PIN), count, RISING);

    setupOutputFreq();

    analogReference(DEFAULT);

    interrupts();             // enable all interrupts

}

void setupOutputFreq()
{
    // For ATTINY85
    // === Setup of the CLK for freq output === 
    // freq generation on OC1A - Clk 16 MHz
    pinMode(OC1A, OUTPUT);
    // reset settings of Timer/Counter register 1
    TCCR1 &= ~((1 << CTC1) | (1 << PWM1A) | (1 << COM1A1) | (1 << COM1A0) | (1 << CS13) | (1 << CS12) | (1 << CS11) | (1 << CS10));


    // set waveform generation mode to CTC (Clear Counter on Match)
    TCCR1 |= (1 << CTC1);

    TCCR1 |= (0 << PWM1A);

    // set compare match output A to toogle
    TCCR1 |= (0 << COM1A1) | (1 << COM1A0);

    // set clock select to CK/8192 (from prescaler)
    TCCR1 |= (1 << CS13) | (1 << CS12) | (1 << CS11) | (0 << CS10);

    // synchronous clocking
    PLLCSR &= ~(1 << PCKE);


    setOutputSpeed(0);
}

void setOutputSpeed(double speed)
{
    double hertz = speed / 3.65;

    double read = analogRead(0);
    double adj = read / 512.0;

    setOutputFrequency(hertz * (0.8 + (adj * 0.2)));
}

void setOutputFrequency(float hertz)
{
    // interrupt frequency (Hz) = (Arduino clock speed 16,000,000Hz) / (prescaler * (compare match register + 1))
    // compare match register = [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1

    uint8_t reg;

    if (hertz > 100)
        hertz = 100;

    if (hertz <= 0)
        reg = (uint8_t)-1;  // slowest
    else
    {
        reg = (F_CPU / (long)((float)8192 * hertz) - 1) / 2;
    }
    OCR1C = reg;
    //Serial.println(hertz);
    //Serial.println(reg);
}

int speedI = 0;

void loop() {
    hub.poll(); // check what's going on in the one-wire bus.
    unsigned long elapsed_time;
    unsigned long current_time = millis();
    elapsed_time = current_time - last_time;
    if (elapsed_time >= 5000) {

        speedI++;
        if (speedI > 4)
            speedI = 0;
        float speed = speedI * 20;
        //Serial.print("Setting speed to ");
        //Serial.print(speed);
        //Serial.println("mph");
        setOutputSpeed(speed);

        int temp_count = edge_count;
        edge_count -= temp_count;
        last_time = current_time;

        //edge_count = 0;
        //last_time = current_time;

        freq = ((float)temp_count / (float)elapsed_time) * 1000; // get frequency in Hz
        //Serial.print("Elapsed Time:  \t");
        //Serial.println(elapsed_time);
        //Serial.print("counts:  \t");
        //Serial.println(temp_count);
        //Serial.print("counts:  \t");
        //Serial.print(freq);
        //Serial.println(" Hz");

        // put frequency in memory buffer and write to virtual ds2431 memory
        int freq_int = ((int)freq) * 100;
        mem[0] = (freq_int & 0xFF);
        mem[1] = ((freq_int >> 8) & 0xFF);
        ds2431.writeMemory(mem, 4);

        //int[] myInts = (int[])(mem);

    }
}

void count() {
    edge_count++;
}
