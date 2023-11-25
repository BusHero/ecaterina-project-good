#include <Arduino.h>
#include <MIDI.h>
#include <Multiplexer4067.h>
#include <timer-api.h>

MIDI_CREATE_DEFAULT_INSTANCE();

#define NUMBER_OF_MUX 2
#define s0 3
#define s1 4
#define s2 5
#define s3 6
#define x1 A0 // analog pin of the first mux
#define x2 A1 // analog pin of the second mux...

Multiplexer4067 multiplexor[NUMBER_OF_MUX] =
{
    Multiplexer4067(s0, s1, s2, s3, x2),
    Multiplexer4067(s0, s1, s2, s3, x1)
};

const int NUMBER_OF_BUTTONS = 6 + 5;                                 // total numbers of buttons
const int NUMBER_OF_BUTTONS_CONENTED_TO_ARDUINO = 5;                 // number of buttons connected straight to the Arduino (in order)
const int BUTTON_ARDUINO_PIN[NUMBER_OF_BUTTONS] = {7, 8, 9, 10, 11}; // pins of each button connected straight to the Arduino

int BUTTON_CC_N[NUMBER_OF_BUTTONS] = {80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90}; // Add the CC NUMBER of each button you want

const int N_BUTTONS_PER_MUX[NUMBER_OF_MUX] = {0, 6};

const int BUTTON_MUX_PIN[NUMBER_OF_MUX][16] =
    {
        {},
        {5, 4, 3, 2, 1, 0}
    };

int buttonMuxThreshold = 500;

int buttonCurrentState[NUMBER_OF_BUTTONS] = {0};    // stores the button current value
int buttonPreviousState[NUMBER_OF_BUTTONS] = {0};   // stores the button previous value

// debounce
unsigned long lastDebounceTime[NUMBER_OF_BUTTONS] = {0};    // the last time the output pin was toggled
unsigned long debounceDelay = 5;                            // the debounce time; increase if the output flickers

// velocity
uint8_t velocity[NUMBER_OF_BUTTONS] = {127};

// POTENTIOMETERS
const int NUMBER_OF_POTENTIOMETERS = 16 + 10 + 0;                 //* total numbers of pots (slide & rotary). Number of pots in the Arduino + number of pots on multiplexer 1 + number of pots on multiplexer 2...
const int NUMBER_OF_POTENTIOMETERS_ARDUINO = 0;                   //* number of pots connected straight to the Arduino
const int ARDUINO_NUMBER_OF_POTENTIOMETER_PINS[NUMBER_OF_POTENTIOMETERS_ARDUINO] = {}; //* pins of each pot connected straight to the Arduino

int POT_CC_N[NUMBER_OF_POTENTIOMETERS] =
    {14, 15, 5,  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, // Add the CC NUMBER of each pot you want
     74, 29, 73, 71, 30, 31, 32, 33, 34, 7};

const int NUMBER_OF_POTENTIOMETERS_PER_MUX[NUMBER_OF_MUX] = {16, 10}; //* number of pots in each multiplexer (in order)
const int POT_MUX_PIN[NUMBER_OF_MUX][16] =
    {
        {15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}, // pins of the first multiplexor
        {15, 14, 13, 12, 11, 10, 9, 8, 7, 6},                   // pins of the second multiplexor
};

int potentiometersCurrentState[NUMBER_OF_POTENTIOMETERS] = {0}; // Current state of the pot
int potentiometerPreviousState[NUMBER_OF_POTENTIOMETERS] = {0}; // Previous state of the pot

int potentiometerVariable = 0;                                  // Difference between the current and previous state of the pot

int potentiometerMidiCurrentState[NUMBER_OF_POTENTIOMETERS] = {0};
int potentiometerMidiPreviosState[NUMBER_OF_POTENTIOMETERS] = {0};

const int TIMEOUT = 300;            // Amount of time the potentiometer will be read after it exceeds the variableThreshold
const int variableThreshold = 10;   // Threshold for the potentiometer signal variation
bool potentiometerMoving = true;    // If the potentiometer is moving

unsigned long previousTime[NUMBER_OF_POTENTIOMETERS] = {0}; // Previously stored time
unsigned long timer[NUMBER_OF_POTENTIOMETERS] = {0};        // Stores the time that has elapsed since the timer was reset

uint8_t MIDI_CH = 1; //* MIDI channel to be used

void buttons();
void potentiometers();

void setup()
{
    // 31250 for MIDI class compliant | 115200 for Hairless MIDI
    Serial.begin(115200);

    for (int i = 0; i < NUMBER_OF_BUTTONS_CONENTED_TO_ARDUINO; i++)
    {
        pinMode(BUTTON_ARDUINO_PIN[i], INPUT_PULLUP);
    }

    for (int i = 0; i < NUMBER_OF_MUX; i++)
    {
        multiplexor[i].begin();
    }

    pinMode(x1, INPUT_PULLUP);
    pinMode(x2, INPUT_PULLUP);

    timer_init_ISR_100Hz(TIMER_DEFAULT);
}

void loop()
{
    buttons();
}

void timer_handle_interrupts(int timer)
{
    potentiometers();
}

void buttons()
{
    // read pins from arduino
    for (int i = 0; i < NUMBER_OF_BUTTONS_CONENTED_TO_ARDUINO; i++)
    {
        buttonCurrentState[i] = !(digitalRead(BUTTON_ARDUINO_PIN[i]));
    }

    int numberOfButtonsPerMuxSum = NUMBER_OF_BUTTONS_CONENTED_TO_ARDUINO; // offsets the buttonCurrentState at every multiplexor reading

    // read the pins from every multiplexor
    for (int j = 0; j < NUMBER_OF_MUX; j++)
    {
        for (int i = 0; i < N_BUTTONS_PER_MUX[j]; i++)
        {
            buttonCurrentState[i + numberOfButtonsPerMuxSum] = multiplexor[j].readChannel(BUTTON_MUX_PIN[j][i]);
            // Scale values to 0-1
            if (buttonCurrentState[i + numberOfButtonsPerMuxSum] > buttonMuxThreshold)
            {
                buttonCurrentState[i + numberOfButtonsPerMuxSum] = LOW;
            }
            else
            {
                buttonCurrentState[i + numberOfButtonsPerMuxSum] = HIGH;
            }
        }
        numberOfButtonsPerMuxSum += N_BUTTONS_PER_MUX[j];
    }

    for (int i = 0; i < NUMBER_OF_BUTTONS; i++)
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
    for (int i = 0; i < NUMBER_OF_POTENTIOMETERS_ARDUINO; i++)
    {
        potentiometersCurrentState[i] = analogRead(ARDUINO_NUMBER_OF_POTENTIOMETER_PINS[i]);
    }

    int nPotsPerMuxSum = NUMBER_OF_POTENTIOMETERS_ARDUINO;

    for (int j = 0; j < NUMBER_OF_MUX; j++)
    {
        for (int i = 0; i < NUMBER_OF_POTENTIOMETERS_PER_MUX[j]; i++)
        {
            potentiometersCurrentState[i + nPotsPerMuxSum] = multiplexor[j].readChannel(POT_MUX_PIN[j][i]);
        }
        nPotsPerMuxSum += NUMBER_OF_POTENTIOMETERS_PER_MUX[j];
    }

    for (int i = 0; i < NUMBER_OF_POTENTIOMETERS; i++)
    {
        potentiometerMidiCurrentState[i] = map(potentiometersCurrentState[i], 0, 1023, 0, 127); 

        potentiometerVariable = abs(potentiometersCurrentState[i] - potentiometerPreviousState[i]); 

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
                potentiometerPreviousState[i] = potentiometersCurrentState[i];                   // Stores the current reading of the potentiometer to compare with the next
                potentiometerMidiPreviosState[i] = potentiometerMidiCurrentState[i];
            }
        }
    }
}