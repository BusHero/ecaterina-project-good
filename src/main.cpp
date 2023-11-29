#include <Arduino.h>
#include <MIDI.h>
#include <Multiplexer4067.h>
#include <Thread.h>
#include <ThreadController.h>

MIDI_CREATE_DEFAULT_INSTANCE();

#define N_MUX 2 //* number of multiplexers
//* Define s0, s1, s2, s3, and x pins
#define s0 3
#define s1 4
#define s2 5
#define s3 6
#define x1 A0 // analog pin of the first mux
#define x2 A1 // analog pin of the second mux...

Multiplexer4067 mux[N_MUX] =
    {
        Multiplexer4067(s0, s1, s2, s3, x2), //*
        Multiplexer4067(s0, s1, s2, s3, x1)  //*
};

const int N_BUTTONS = 6 + 5;                                 //* total numbers of buttons
const int N_BUTTONS_ARDUINO = 5;                             //* number of buttons connected straight to the Arduino (in order)
const int BUTTON_ARDUINO_PIN[N_BUTTONS] = {7, 8, 9, 10, 11}; //* pins of each button connected straight to the Arduino

#define USING_BUTTON_CC_N 1

int BUTTON_CC_N[N_BUTTONS] = {80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90}; // Add the CC NUMBER of each button you want

const int N_BUTTONS_PER_MUX[N_MUX] = {0, 6};

const int BUTTON_MUX_PIN[N_MUX][16] =
    {
        //* pin of each button of each mux in order
        {},
        {5, 4, 3, 2, 1, 0}
    };

int buttonMuxThreshold = 500;

int buttonCurrentState[N_BUTTONS] = {0}; // stores the button current value
int buttonPreviousState[N_BUTTONS] = {0}; // stores the button previous value

byte pin13index = 12; //* put the index of the pin 13 of the buttonPin[] array if you are using, if not, comment

// debounce
unsigned long lastDebounceTime[N_BUTTONS] = {0}; // the last time the output pin was toggled
unsigned long debounceDelay = 5;                 //* the debounce time; increase if the output flickers

// velocity
byte velocity[N_BUTTONS] = {127};

// POTENTIOMETERS
const int N_POTS = 16 + 10 + 0;                 //* total numbers of pots (slide & rotary). Number of pots in the Arduino + number of pots on multiplexer 1 + number of pots on multiplexer 2...
const int N_POTS_ARDUINO = 0;                   //* number of pots connected straight to the Arduino
const int POT_ARDUINO_PIN[N_POTS_ARDUINO] = {}; //* pins of each pot connected straight to the Arduino

#define USING_CUSTOM_CC_N 1 //* comment if not using CUSTOM CC NUMBERS, uncomment if using it.

int POT_CC_N[N_POTS] = {13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 
                        29, 30, 31, 12, 33, 34, 35, 36, 37, 38}; // Add the CC NUMBER of each pot you want

const int N_POTS_PER_MUX[N_MUX] = {16, 10}; //* number of pots in each multiplexer (in order)
const int POT_MUX_PIN[N_MUX][16] =
    {
        //* pins of each pot of each mux in the order you want them to be

        {15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}, // pins of the first mux
        {15, 14, 13, 12, 11, 10, 9, 8, 7, 6}, // pins of the second mux

};

int potentiometersCurrentState[N_POTS] = {0}; // Current state of the pot
int potentiometerPreviosState[N_POTS] = {0};  // Previous state of the pot
int potentiometerVariable = 0;                // Difference between the current and previous state of the pot

int potentiometerMidiCurrentState[N_POTS] = {0};
int potentiometerMidiPreviosState[N_POTS] = {0};

const int TIMEOUT = 300;                  //* Amount of time the potentiometer will be read after it exceeds the variableThreshold
const int variableThreshold = 10;         //* Threshold for the potentiometer signal variation
boolean potentiometerMoving = true;       // If the potentiometer is moving
unsigned long previousTime[N_POTS] = {0}; // Previously stored time
unsigned long timer[N_POTS] = {0};        // Stores the time that has elapsed since the timer was reset

byte MIDI_CH = 1; //* MIDI channel to be used
byte NOTE = 36;   //* Lowest NOTE to be used - if not using custom NOTE NUMBER
byte CC = 1;      //* Lowest MIDI CC to be used - if not using custom CC NUMBER

Thread threadPotentiometers; // thread to control the pots
ThreadController cpu;        // thread master, where the other threads will be added

void potentiometers();
void buttons();
void encoders();

void setup()
{
    // 31250 for MIDI class compliant | 115200 for Hairless MIDI
    Serial.begin(115200);

    for (int i = 0; i < N_BUTTONS_ARDUINO; i++)
    {
        pinMode(BUTTON_ARDUINO_PIN[i], INPUT_PULLUP);
    }

    for (int i = 0; i < N_MUX; i++)
    {
        mux[i].begin();
    }

    pinMode(x1, INPUT_PULLUP);
    pinMode(x2, INPUT_PULLUP);

    threadPotentiometers.setInterval(10);
    threadPotentiometers.onRun(potentiometers);
    cpu.add(&threadPotentiometers);
}

void loop()
{
    cpu.run();
    buttons();
}

void buttons()
{
    // read pins from arduino
    for (int i = 0; i < N_BUTTONS_ARDUINO; i++)
    {
        buttonCurrentState[i] = !(digitalRead(BUTTON_ARDUINO_PIN[i]));
    }

    int nButtonsPerMuxSum = N_BUTTONS_ARDUINO; // offsets the buttonCurrentState at every mux reading

    // read the pins from every mux
    for (int j = 0; j < N_MUX; j++)
    {
        for (int i = 0; i < N_BUTTONS_PER_MUX[j]; i++)
        {
            buttonCurrentState[i + nButtonsPerMuxSum] = mux[j].readChannel(BUTTON_MUX_PIN[j][i]);
            // Scale values to 0-1
            if (buttonCurrentState[i + nButtonsPerMuxSum] > buttonMuxThreshold)
            {
                buttonCurrentState[i + nButtonsPerMuxSum] = LOW;
            }
            else
            {
                buttonCurrentState[i + nButtonsPerMuxSum] = HIGH;
            }
        }
        nButtonsPerMuxSum += N_BUTTONS_PER_MUX[j];
    }

    for (int i = 0; i < N_BUTTONS; i++)
    {
        if ((millis() - lastDebounceTime[i]) > debounceDelay)
        {
            if (buttonPreviousState[i] != buttonCurrentState[i])
            {
                lastDebounceTime[i] = millis();

                if (buttonCurrentState[i] == LOW)
                {

                    velocity[i] = 127; // if button is pressed velocity is 127
                }
                else
                {

                    velocity[i] = 0; // if button is released velocity is 0
                }

                MIDI.sendControlChange(BUTTON_CC_N[i], velocity[i], MIDI_CH); // note, velocity, channel

                buttonPreviousState[i] = buttonCurrentState[i];
            }
        }
    }
}

void potentiometers()
{
    for (int i = 0; i < N_POTS_ARDUINO; i++)
    {
        potentiometersCurrentState[i] = analogRead(POT_ARDUINO_PIN[i]);
    }

    int nPotsPerMuxSum = N_POTS_ARDUINO;
// TODO: fix this
    for (int j = 0; j < N_MUX; j++)
    {
        for (int i = 0; i < N_POTS_PER_MUX[j]; i++)
        {
            potentiometersCurrentState[i + nPotsPerMuxSum] = mux[j].readChannel(POT_MUX_PIN[j][i]);
        }
        nPotsPerMuxSum += N_POTS_PER_MUX[j];
    }

    for (int i = 0; i < N_POTS; i++)
    {
        potentiometerMidiCurrentState[i] = map(potentiometersCurrentState[i], 0, 1023, 0, 127); 

        potentiometerVariable = abs(potentiometersCurrentState[i] - potentiometerPreviosState[i]); 

        if (potentiometerVariable > variableThreshold)
        {                               
            previousTime[i] = millis();
        }

        timer[i] = millis() - previousTime[i];

        if (timer[i] < TIMEOUT)
        {
            potentiometerMoving = true;
        }
        else
        {
            potentiometerMoving = false;
        }

        if (potentiometerMoving == true)
        {
            if (potentiometerMidiPreviosState[i] != potentiometerMidiCurrentState[i])
            {
                MIDI.sendControlChange(POT_CC_N[i], potentiometerMidiCurrentState[i], MIDI_CH); // CC number, CC value, midi channel - custom cc
                potentiometerPreviosState[i] = potentiometersCurrentState[i];                   // Stores the current reading of the potentiometer to compare with the next
                potentiometerMidiPreviosState[i] = potentiometerMidiCurrentState[i];
            }
        }
    }
}