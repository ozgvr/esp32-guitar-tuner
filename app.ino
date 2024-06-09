#include <arduinoFFT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SAMPLES         4096
#define SAMPLING_FREQ   20000
#define AUDIO_IN_PIN    34
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

unsigned int sampling_period_us;
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long newTime;

int maxADC = 0;
double lastFrequency = 0;
int lastNoteIndex = 0;
int lastNoteDiff = 0;


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

const char *noteNames[] = {"A","A#","B","C","C#","D","D#","E","F","F#","G","G#"};
const double noteFrequencies[] = {
  55.00,  // A1
  58.27,  // A#1
  61.74,  // B1
  65.41,  // C2
  69.30,  // C#2
  73.42,  // D2
  77.78,  // D#2
  82.41,  // E2
  87.31,  // F2
  92.50,  // F#2
  98.00,  // G2
  103.83, // G#2
  110.00, // A2
  116.54, // A#2
  123.47, // B2
  130.81, // C3
  138.59, // C#3
  146.83, // D3
  155.56, // D#3
  164.81, // E3
  174.61, // F3
  185.00, // F#3
  196.00, // G3
  207.65, // G#3
  220.00, // A3
  233.08, // A#3
  246.94, // B3
  261.63, // C4
  277.18, // C#4
  293.66, // D4
  311.13, // D#4
  329.63, // E4
  349.23, // F4
  369.99, // F#4
  392.00, // G4
  415.30, // G#4
  440.00, // A4
  466.16, // A#4
  493.88, // B4
  523.25, // C5
  554.37, // C#5
  587.33, // D5
  622.25, // D#5
  659.26, // E5
  698.46, // F5
  739.99, // F#5
  783.99, // G5
  830.61, // G#5
  880.00, // A5
  932.33, // A#5
  987.77, // B5
  1046.50, // C6
  1108.73, // C#6
  1174.66, // D6
  1244.51, // D#6
  1318.51, // E6
  1396.91, // F6
  1479.98, // F#6
  1567.98, // G6
  1661.22, // G#6
  1760.00, // A6
};

const int getClosestNoteIndex(double frequency) {
  double minDiff = abs(frequency - noteFrequencies[0]);
  int closestIndex = 0;
  
  for (int i = 1; i < sizeof(noteFrequencies) / sizeof(noteFrequencies[0]); i++) {
    double diff = abs(frequency - noteFrequencies[i]);
    if (diff < minDiff) {
      minDiff = diff;
      closestIndex = i;
    }
  }

  return closestIndex;
}

const double getNoteDiff(double frequency, int closestIndex) {
  double noteGap;
  double frequencyGap;

  if(frequency<noteFrequencies[closestIndex]){
    noteGap = noteFrequencies[closestIndex]-noteFrequencies[closestIndex-1];
  }else{
    noteGap = noteFrequencies[closestIndex]-noteFrequencies[closestIndex+1];
  }
  
  frequencyGap = abs(frequency-noteFrequencies[closestIndex])/noteGap*-100;
  return (int)frequencyGap;
}

void displayResult(int noteDiff, double frequency, int noteIndex){
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);

  display.clearDisplay();
  display.drawLine(64,0,(64+(128*noteDiff/100)),0, WHITE);
  display.drawLine(64,1,(64+(128*noteDiff/100)),1, WHITE);
  display.drawLine(64,2,(64+(128*noteDiff/100)),2, WHITE);
  display.drawLine(64,3,(64+(128*noteDiff/100)),3, WHITE);
  display.drawLine(64,4,(64+(128*noteDiff/100)),4, WHITE);
  display.print("  ");
  display.print(frequency);
  display.println("hz");
  display.setTextSize(4);
  display.print("  ");
  display.print(noteNames[noteIndex % 12]);
  display.display(); 
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Set ADC resolution to 12 bits
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();
}

void loop() {

  for (int i = 0; i < SAMPLES; i++) {
    newTime = micros();
    vReal[i] = analogRead(AUDIO_IN_PIN); // A conversion takes about 9.7uS on an ESP32
    vImag[i] = 0;
    while ((micros() - newTime) < sampling_period_us) { /* chill */ }
  }

  FFT.dcRemoval();
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double peak = 0;
  int peakIndex = 0;
  for (int i = 1; i < (SAMPLES / 2); i++) {
    if (vReal[i] > peak) {
      peak = vReal[i];
      peakIndex = i;
    }
  }

  double frequency = peakIndex * ((SAMPLING_FREQ / 2.0) / (SAMPLES / 2.0));
  frequency = frequency/1.925;

  int noteIndex = getClosestNoteIndex(frequency);

  if(frequency==0 || noteIndex==60 || noteIndex==0 ){
    displayResult(lastNoteDiff, lastFrequency, lastNoteIndex);
  }else{
    int noteDiff = getNoteDiff(frequency, noteIndex);
    displayResult(noteDiff, frequency, noteIndex);
    lastNoteDiff = noteDiff;
    lastFrequency = frequency;
    lastNoteIndex = noteIndex;
  }
}
