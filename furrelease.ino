#include <ardumidi.h>

#define MIDI_E5 76
#define MIDI_DS5 75
#define MIDI_B4 71
#define MIDI_D5 74
#define MIDI_C5 72
#define MIDI_A4 69
#define MIDI_C4 60
#define MIDI_E4 64
#define MIDI_GS4 615
#define MIDI_B4 71
#define MIDI_C5 72


int note_on = 0;
int melody[] = {MIDI_E5, MIDI_DS5, MIDI_E5, MIDI_DS5, MIDI_E5, MIDI_B4, MIDI_D5, MIDI_C5, MIDI_A4, MIDI_C4, MIDI_E4, MIDI_A4, MIDI_B4, MIDI_E4, MIDI_GS4, MIDI_B4, MIDI_C5};
int noteDurations[] = {8, 8, 8, 8, 8, 8, 8, 8, 2, 8, 8, 8, 8, 8, 8, 8, 2};
int melodyLength = sizeof(melody) / sizeof(int);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  for (int i = 0; i < melodyLength; i++) {
    int noteDuration = 1000 / noteDurations[i];
    midi_note_on(0, melody[i], 127);
    delay(noteDuration);
    midi_note_off(0, melody[i], 127);
    delay(noteDuration / 2);
  }
}