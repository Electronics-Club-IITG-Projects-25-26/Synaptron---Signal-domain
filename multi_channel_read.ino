/*
   Triple EMG Read (A0 + A1 + A2) — Optimized
*/

const int emgPins[] = {A0, A1, A2};
const int NUM_CH = 3;

const unsigned long SAMPLE_INTERVAL_US = 1000;
const int SEND_EVERY = 1;

static unsigned long lastSampleTime = 0;
static int sampleCount = 0;

void setup() {
  Serial.begin(115200);
  ADCSRA = (ADCSRA & 0xF8) | 0x04;  // prescaler = 16
  for (int i = 0; i < NUM_CH; i++) {
    pinMode(emgPins[i], INPUT);
  }
}

void loop() {
  unsigned long now = micros();
  if (now - lastSampleTime < SAMPLE_INTERVAL_US) return;
  lastSampleTime = now;

  int vals[NUM_CH];
  for (int i = 0; i < NUM_CH; i++) {
    vals[i] = analogRead(emgPins[i]);
  }

  sampleCount++;
  if (sampleCount >= SEND_EVERY) {
    sampleCount = 0;
    // Sends: "val1,val2,val3\n"
    char buf[24];
    int len = snprintf(buf, sizeof(buf), "%d,%d,%d\n", vals[0], vals[1], vals[2]);
    Serial.write((uint8_t*)buf, len);
  }
}