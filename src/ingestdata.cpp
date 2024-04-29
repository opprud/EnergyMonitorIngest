/*
 * Project Energy Monitor Data Ingestion
 * Author: Your Name
 * Date:
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */
#include "Particle.h"
#include "UIDisplay.h"
#include <Debounce.h>
#include <Wire.h>

SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

typedef enum
{
  IDLE,
  CONNECTING,
  CONNECTED,
  SAMPLING,
  WAIT_FOR_EDGE,
  CAPTURE,
  UPLOADING,
  UPDATE_UI
} state_t;

// above states an arrays of const strings for debugging
const char *stateStrings[] = {
    "IDLE",
    "CONNECTING",
    "CONNECTED",
    "SAMPLING",
    "WAIT_FOR_EDGE",
    "CAPTURE",
    "UPLOADING",
    "UPDATE_UI"};

// HW defines
#define BUTTON_A D4
#define BUTTON_B D3
#define BUTTON_C D2
#define ADC_COUNTS (1 << 12) // ADC bits
#define NUM_SAMPLES 80       // number of samples per event
#define SAMPLE_PERIOD 1500   // Sampling interval for each event in milliseconds

// Pin definitions
int LED_B = D9;        // blinker on custom analog front end PCB
int voltagePIN = A2;   // voltage is on A2
int currentPIN = A5;   // current is pn A5
int zeroCrossPIN = S3; // zero cross, digital signal on A5

// prototypes
void sampleData(void);
bool tx_data(String data_in);
void zeroCrossing(void);
void btnHandler(void);
void setGain(uint8_t gainValue);
int setPayloadLabel(String command);
void packetizeData(void);

// Objects
Timer sample(1, sampleData);
SerialLogHandler logHandler(LOG_LEVEL_INFO);
UIDisplay ui;

// Global variables
int32_t voltageADC = 0;
int32_t currentADC = 0;
int payloadBuffer[2][NUM_SAMPLES];
int sampleIndex = 0;
char sampleBuffer[16];
char buf[1024];
String payloadString;
String sampleString;
char payloadLabel[] = "no_Label";
bool sendData = false;
bool firstSample = true;
bool samplingArmed = false;
bool edgeDetected = false;
bool keyCPressed = false;

double filteredCurrent = 0;
double offsetCurrent = 0;
float sumCurrent = 0;
double squareCurrent = 0;
float rmsCurrent = 0;
float instPower = 0;
double CURRENT_CAL = 0.027566;
double gridFreq = 0.0;
double usSample;
double usSampleStart;
unsigned long now;

state_t state, state_next = CONNECTING;

void setup()
{

  pinMode(LED_B, OUTPUT);
  pinMode(BUTTON_C, INPUT_PULLUP);
  attachInterrupt(zeroCrossPIN, zeroCrossing, FALLING);
  attachInterrupt(BUTTON_C, btnHandler, FALLING);

  Serial.begin(115200);
  Log.info("Enery Monitor Ingestion");
  Particle.variable("gridFreq", gridFreq);
  Particle.variable("Sample duration uS", usSample);
  Particle.function("SetLabel", setPayloadLabel);
  Particle.variable("payloadLabel", payloadLabel);
  // Particle.function("SetGain", setGainValue);
  Particle.connect();

  ui.begin();
  setGain(245);
  // sample.start();
}

void fsm(void)
{
  state = state_next;
  switch (state)
  {
  case CONNECTING:
    if (Particle.connected())
    {
      state_next = CONNECTED;
      ui.cloudIsConnected();
    }
    break;

  case CONNECTED:
    delay(3000);
    state_next = IDLE;
    break;

  case IDLE:
    if (keyCPressed == true)
    {
      state_next = SAMPLING;
      keyCPressed = false;
      now = millis();
    }
    break;

  case SAMPLING:
    if (keyCPressed == true)
    {
      state_next = IDLE;
      keyCPressed = false;
      ui.ingestStatus("IDLE", payloadLabel);
    }
    else
    {
      if (millis() - now > SAMPLE_PERIOD)
      {
        ui.ingestStatus("Sampling", payloadLabel);
        state_next = WAIT_FOR_EDGE;
      }
    }
    break;

  case WAIT_FOR_EDGE:
    //arm sampling trigger, set in ISR to oensure allignemt with zero crossing 
    samplingArmed = true;
    if (edgeDetected == true)
    {
      //sample.start(); //startted from ISR
      edgeDetected = false;
      state_next = CAPTURE;
    }
    break;

  case CAPTURE:
    if (sendData == true)
    {
      state_next = UPLOADING;
    }
    break;

  case UPLOADING:
    ui.ingestStatus("Uploading", payloadLabel);
    digitalWrite(LED_B, HIGH);
    packetizeData();
    // Log.info(buf);
    bool result = tx_data(buf);
    Log.info("result: %d", result);
    Log.info("sample time us: %f", usSample);
    sendData = false; 
    state_next = SAMPLING;
    break;

  }

  if (state != state_next)
  {
    // Log.info("State: %d", state);
    // print stateStrings[state] accordingly
    Log.info("State: %s", stateStrings[state]);
  }
}

void packetizeData(void)
{
  memset(buf, 0, sizeof(buf));
  JSONBufferWriter writer(buf, sizeof(buf) - 1);

  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    writer.beginArray();
    writer.value(payloadBuffer[0][i]);
    writer.value(payloadBuffer[1][i]);
    writer.endArray();
  }
}

void loop(void)
{
  fsm();
}

/**
 * @brief collect data an 1Khz called from timner interruot
 *
 */
void sampleData(void)
{
  //usSampleStart = micros();
  voltageADC = analogRead(voltagePIN); // read ADC values
  currentADC = analogRead(currentPIN);
  payloadBuffer[0][sampleIndex] = voltageADC;
  payloadBuffer[1][sampleIndex] = currentADC;
  sampleIndex++;

  filteredCurrent = currentADC - 2048; // 2048 should be replaced by Vref
  squareCurrent = filteredCurrent * filteredCurrent;
  sumCurrent += squareCurrent;

  if (sampleIndex == NUM_SAMPLES) // we are done sampling for this event
  {
    sampleIndex = 0;
    sample.stop();
    rmsCurrent = sqrt(sumCurrent / NUM_SAMPLES);
    rmsCurrent -= 8;
    if (rmsCurrent < 0)
    {
      rmsCurrent = 0;
    }
    rmsCurrent *= CURRENT_CAL;
    sumCurrent = 0;
    sendData = true;
  }
  //else 
  //  usSample =  micros() - usSampleStart;
}

bool tx_data(String data_in)
{
  bool retval = false;
  char labelString[40];
  sprintf(labelString, "edge/ingest/%s", payloadLabel);
  retval = Particle.publish(labelString, data_in,
                            PRIVATE); // manually set appliance label here TODO,
                                      // subscribe to string from console
  return retval;
}

int setPayloadLabel(String command)
{
  strcpy(payloadLabel, command);
  return 1;
}

void btnHandler(void)
{
  // Log.info("Button Pressed");
  // Log.info("sample time us: %f", usSample);
  keyCPressed = true;
}

void zeroCrossing(void)
{
  static unsigned long lastUs, nowUs;
  unsigned long periodUs;
  lastUs = nowUs;
  nowUs = micros();
  periodUs = nowUs - lastUs;
  gridFreq = (double)1000000.0 / periodUs;
  edgeDetected = true;
  //start sampling here to ensure allignment with zero crossing
  if(samplingArmed == true)
  {
    sample.reset();
    samplingArmed = false;
  }
}

void setGain(uint8_t gainValue)
{
  char gainRet = 0;
  Wire.beginTransmission(0x2f);
  Wire.write(0x00);
  Wire.write(gainValue);
  Wire.endTransmission();
  delay(5);
  Wire.requestFrom(0x2f, 1);
  gainRet = Wire.read();
  Wire.endTransmission();
  if (gainRet == gainValue)
  {
    Log.info("SUCCESS SETTING GAIN: %d", gainRet);
  }
  else
  {
    Log.info("FAILED TO SET GAIN: %d", gainRet);
  }
}
