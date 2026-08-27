/*
 * Motor driver for the three wheeled omnidirectional platform.
 *
 * The sketch runs two FreeRTOS tasks so that the network can never hold up the
 * wheels:
 *
 *   ControlTask   core 1, priority 3, every CONTROL_PERIOD_MS
 *                 reads the encoders, filters the wheel velocities, runs one
 *                 PID per wheel and writes the PWM outputs. It touches no
 *                 network code at all: it reads the setpoints from a shared
 *                 variable and drops the measurements into a queue.
 *
 *   NetworkTask   core 0, priority 1
 *                 keeps WiFi and MQTT alive, receives the setpoints, empties
 *                 the telemetry queue onto the broker and drives the LED.
 *                 Everything in this task is allowed to be slow.
 *
 * A slow MQTT publish, a lost broker or a reconnection can no longer stall the
 * control loop, which is what made the wheels stutter and start late.
 *
 * All velocities, commanded and measured, are wheel LINEAR velocities in cm/s.
 *
 * MQTT topics
 *   motor_velocities      in    "<velA>,<velB>,<velC>"   cm/s
 *   motor_telemetry       out   CSV rows, see telemHeader
 *   motor_telemetry_ctrl  in    "1"/"0" stream on/off, "s1"/"s0" serial copy
 *   DriverCheck           out   "OK" when the driver connects
 */

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <WiFi.h>
#include <PubSubClient.h>
#include <PID_v1.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

///////////////////////////////// CONFIGURATION ////////////////////////////////

// pins, wheel A / B / C
const uint8_t encPinA1 = 13, encPinA2 = 14, encPinB1 = 27, encPinB2 = 26, encPinC1 = 25, encPinC2 = 33;
const uint8_t motorFrwdA = 22, motorBkwdA = 23, motorFrwdB = 18, motorBkwdB = 19, motorFrwdC = 17, motorBkwdC = 5;
const uint8_t ledPin = 2;
const uint8_t motorFrwd[3] = {motorFrwdA, motorFrwdB, motorFrwdC};
const uint8_t motorBkwd[3] = {motorBkwdA, motorBkwdB, motorBkwdC};
const uint8_t encPin1[3] = {encPinA1, encPinB1, encPinC1};
const uint8_t encPin2[3] = {encPinA2, encPinB2, encPinC2};

// mechanics
const int   N = 692;             // encoder ticks per revolution
const float diam = 6.0;          // wheel diameter [cm]
const int   filterNumber = 10;   // length of the moving average on the velocity

// control loop
const int CONTROL_PERIOD_MS = 5; // period of ControlTask, also the PID sample time
double Kp = 4.0, Ki = 23, Kd = 0.012;
// MOTOR_MIN_PWM is the duty at which a wheel really starts turning under load.
// Below it the motor only buzzes, so it is used as the lower PID output limit:
// the integral term then never has to climb through that dead zone before the
// wheel reacts to a new command. Calibrate it once - command a small velocity
// and raise it until the wheels start without hesitating.
const int MOTOR_MIN_PWM = 60;
const int MOTOR_MAX_PWM = 255;

// telemetry
const char *velocityTopic = "motor_velocities";
const char *telemetryTopic = "motor_telemetry";
const char *telemetryCtrlTopic = "motor_telemetry_ctrl";
const char *telemHeader = "#seq,t_ms,setA,velA,pwmA,setB,velB,pwmB,setC,velC,pwmC";
const uint16_t TELEM_BUFFER_SIZE = 1024; // MQTT packet buffer (PubSubClient defaults to 256)
const uint8_t  TELEM_DECIMATION = 2;     // log every Nth control cycle -> 100 Hz
const uint8_t  TELEM_BATCH = 8;          // CSV rows per MQTT message
const uint16_t TELEM_MAX_AGE = 200;      // ms, send a partial batch after this long
const uint16_t TELEM_QUEUE_LEN = 200;    // samples buffered while the network is busy

// MQTT broker, editable from the WiFiManager portal
char mqtt_server[40] = "192.168.0.109";
char mqtt_port[6] = "1883";

/////////////////////////////////// STATE //////////////////////////////////////

WiFiManager wm;
WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
WiFiClient espClient;
PubSubClient client(espClient);

// --- shared between the two tasks --------------------------------------------
// the setpoints are written by NetworkTask and read by ControlTask, so they are
// copied under a spinlock; the telemetry travels the other way through a queue
portMUX_TYPE setpointMux = portMUX_INITIALIZER_UNLOCKED;
volatile float setpointCmd[3] = {0, 0, 0};   // cm/s

typedef struct {
  uint32_t seq;
  uint32_t t_ms;
  float set[3];
  float vel[3];
  float pwm[3];
} TelemSample;

QueueHandle_t telemQueue = NULL;
volatile uint32_t telemDropped = 0;          // samples that did not make it out

// --- owned by ControlTask ----------------------------------------------------
volatile long encCount[3] = {0, 0, 0};       // updated in the interrupts
long lastEncCount[3] = {0, 0, 0};
unsigned long lastCalcTime[3] = {0, 0, 0};
float velFilter[3][filterNumber];
double pidInput[3], pidOutput[3], pidSetpointAbs[3];
PID pidA(&pidInput[0], &pidOutput[0], &pidSetpointAbs[0], Kp, Ki, Kd, DIRECT);
PID pidB(&pidInput[1], &pidOutput[1], &pidSetpointAbs[1], Kp, Ki, Kd, DIRECT);
PID pidC(&pidInput[2], &pidOutput[2], &pidSetpointAbs[2], Kp, Ki, Kd, DIRECT);
PID *pids[3] = {&pidA, &pidB, &pidC};

// --- owned by NetworkTask ----------------------------------------------------
int internetConnected = 0, mqttConnected = 0;
int ledChk = 0;
unsigned long lastReconnectAttempt = 0;
int telemetryEnabled = 1;                    // stream the telemetry over MQTT
int telemetryToSerial = 0;                   // print the same rows on the USB port
char telemPayload[TELEM_BUFFER_SIZE];
uint16_t telemLen = 0;
uint8_t telemRows = 0;
unsigned long telemLastFlush = 0;
unsigned long telemLastWarn = 0;

void ControlTask(void *parameter);
void NetworkTask(void *parameter);
float CalcVelocity(int wheel);
void MoveWheel(int wheel, float setpoint, double output);
void PublishTelemetry();
void FlushTelemetry();
void LedStatus();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);
void saveParamCallback();
String getParam(String name);

///////////////////////////////// INTERRUPTS ///////////////////////////////////

void IRAM_ATTR isrA() {
  if (digitalRead(encPinA1) > digitalRead(encPinA2)) encCount[0]++;
  else encCount[0]--;
}

void IRAM_ATTR isrB() {
  if (digitalRead(encPinB1) > digitalRead(encPinB2)) encCount[1]++;
  else encCount[1]--;
}

void IRAM_ATTR isrC() {
  if (digitalRead(encPinC1) > digitalRead(encPinC2)) encCount[2]++;
  else encCount[2]--;
}

/////////////////////////////////// SETUP //////////////////////////////////////

void setup() {
  Serial.begin(115200);

  for (int w = 0; w < 3; w++) {
    pinMode(motorFrwd[w], OUTPUT);
    pinMode(motorBkwd[w], OUTPUT);
    analogWrite(motorFrwd[w], 0);
    analogWrite(motorBkwd[w], 0);
    pinMode(encPin1[w], INPUT_PULLUP);
    pinMode(encPin2[w], INPUT_PULLUP);
    digitalWrite(encPin1[w], HIGH);   // turn pullup resistor on
    digitalWrite(encPin2[w], HIGH);
    for (int i = 0; i < filterNumber; i++) velFilter[w][i] = 0;
  }
  pinMode(ledPin, OUTPUT);
  LedStatus();

  // reset settings - wipe stored credentials for testing
  //wm.resetSettings();
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.setSaveParamsCallback(saveParamCallback);
  wm.setClass("invert"); // use darkmode

  client.setServer(mqtt_server, atoi(mqtt_port));
  client.setCallback(callback);
  if (!client.setBufferSize(TELEM_BUFFER_SIZE)) {
    Serial.println("Could not allocate the telemetry MQTT buffer - telemetry disabled");
    telemetryEnabled = 0;
  }

  // connects with the saved credentials, otherwise opens the configuration AP
  if (!wm.autoConnect("Motor_Driver")) {
    internetConnected = 0;
    Serial.println("Failed to connect");
  }
  else {
    Serial.println("connected...yeey :):):)");
    internetConnected = 1;
  }
  // the ESP32 station defaults to WIFI_PS_MIN_MODEM: the radio then only wakes
  // on the access point beacon, and incoming setpoints are delivered a few
  // hundred ms late
  WiFi.setSleep(false);

  attachInterrupt(encPinA1, isrA, RISING);
  attachInterrupt(encPinB1, isrB, RISING);
  attachInterrupt(encPinC1, isrC, RISING);

  for (int w = 0; w < 3; w++) {
    // the PID library defaults to a 100 ms sample time, twenty times slower
    // than the loop that calls it
    pids[w]->SetSampleTime(CONTROL_PERIOD_MS);
    pids[w]->SetOutputLimits(MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    pids[w]->SetMode(AUTOMATIC);
    lastCalcTime[w] = micros();
  }

  telemQueue = xQueueCreate(TELEM_QUEUE_LEN, sizeof(TelemSample));
  if (telemQueue == NULL) {
    Serial.println("Could not allocate the telemetry queue - telemetry disabled");
    telemetryEnabled = 0;
  }

  xTaskCreatePinnedToCore(ControlTask, "control", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(NetworkTask, "network", 8192, NULL, 1, NULL, 0);
}

void loop() {
  // everything happens in the two tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

///////////////////////////////// CONTROL TASK /////////////////////////////////

void ControlTask(void *parameter) {
  TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  if (period == 0) period = 1;
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t seq = 0;
  uint8_t decimation = 0;

  for (;;) {
    vTaskDelayUntil(&lastWake, period);   // fixed period, whatever the network is doing

    float setpoint[3];
    portENTER_CRITICAL(&setpointMux);
    setpoint[0] = setpointCmd[0];
    setpoint[1] = setpointCmd[1];
    setpoint[2] = setpointCmd[2];
    portEXIT_CRITICAL(&setpointMux);

    float vel[3];
    for (int w = 0; w < 3; w++) {
      vel[w] = CalcVelocity(w);
      // the PID works on magnitudes, the sign of the setpoint picks the direction
      pidInput[w] = fabs(vel[w]);
      pidSetpointAbs[w] = fabs(setpoint[w]);
      if (setpoint[w] == 0) {
        // stopped: the integral is reset to the lower output limit, so the next
        // command acts at once instead of ramping up from zero
        pids[w]->SetMode(MANUAL);
        pidOutput[w] = 0;
        pids[w]->SetMode(AUTOMATIC);
      }
      else {
        pids[w]->Compute();
      }
      MoveWheel(w, setpoint[w], pidOutput[w]);
    }

    if (++decimation >= TELEM_DECIMATION) {
      decimation = 0;
      if (telemQueue != NULL) {
        TelemSample sample;
        sample.seq = seq++;
        sample.t_ms = millis();
        for (int w = 0; w < 3; w++) {
          sample.set[w] = setpoint[w];
          sample.vel[w] = vel[w];
          sample.pwm[w] = (setpoint[w] < 0) ? -pidOutput[w] : pidOutput[w];
        }
        // never wait on the queue: a dropped sample is better than a late wheel
        if (xQueueSend(telemQueue, &sample, 0) != pdTRUE) telemDropped++;
      }
    }
  }
}
///////////////////////////////////////////////////////////////////////////////
// Wheel linear velocity in cm/s, averaged over the last filterNumber samples.
float CalcVelocity(int wheel) {
  unsigned long now = micros();
  double dt = (now - lastCalcTime[wheel]) / 1000000.0;
  lastCalcTime[wheel] = now;
  if (dt <= 0.0 || dt > 0.5) dt = CONTROL_PERIOD_MS / 1000.0;   // first call, or a stall

  long count = encCount[wheel];
  long delta = count - lastEncCount[wheel];
  lastEncCount[wheel] = count;

  double freq = delta / dt;                          // encoder ticks per second
  double omega = ((2.0 * 3.14159265) / N) * freq;    // rad/s
  double linear = (diam / 2.0) * omega;              // cm/s

  for (int i = 0; i < filterNumber - 1; i++) velFilter[wheel][i] = velFilter[wheel][i + 1];
  velFilter[wheel][filterNumber - 1] = linear;

  float media = 0;
  for (int i = 0; i < filterNumber; i++) media += velFilter[wheel][i];
  return media / filterNumber;
}
///////////////////////////////////////////////////////////////////////////////
void MoveWheel(int wheel, float setpoint, double output) {
  if (setpoint == 0) {
    analogWrite(motorFrwd[wheel], 0);
    analogWrite(motorBkwd[wheel], 0);
    return;
  }
  int pwm = (int)output;
  if (pwm < 0) pwm = 0;
  if (pwm > MOTOR_MAX_PWM) pwm = MOTOR_MAX_PWM;

  if (setpoint > 0) {
    analogWrite(motorBkwd[wheel], 0);
    analogWrite(motorFrwd[wheel], pwm);
  }
  else {
    analogWrite(motorFrwd[wheel], 0);
    analogWrite(motorBkwd[wheel], pwm);
  }
}

///////////////////////////////// NETWORK TASK /////////////////////////////////

void NetworkTask(void *parameter) {
  unsigned long lastLedTime = 0;

  for (;;) {
    if ((WiFi.status() != WL_CONNECTED) || (WiFi.localIP().toString() == "0.0.0.0")) {
      internetConnected = 0;
    }
    else {
      internetConnected = 1;
    }

    if (!client.connected()) reconnect();
    client.loop();
    PublishTelemetry();

    if (millis() - lastLedTime >= 330) {
      LedStatus();
      lastLedTime = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(2));   // yield; incoming messages are read within 2 ms
  }
}
///////////////////////////////////////////////////////////////////////////////
// Empties the telemetry queue into MQTT packets of TELEM_BATCH rows.
void PublishTelemetry() {
  TelemSample sample;

  while (telemQueue != NULL && xQueueReceive(telemQueue, &sample, 0) == pdTRUE) {
    char row[128];
    int n = snprintf(row, sizeof(row),
                     "%lu,%lu,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f\n",
                     (unsigned long)sample.seq, (unsigned long)sample.t_ms,
                     sample.set[0], sample.vel[0], sample.pwm[0],
                     sample.set[1], sample.vel[1], sample.pwm[1],
                     sample.set[2], sample.vel[2], sample.pwm[2]);
    if (n <= 0) continue;
    if (n >= (int)sizeof(row)) n = sizeof(row) - 1;    // snprintf truncated the row

    if (telemetryToSerial) Serial.print(row);
    if (!telemetryEnabled) continue;

    if (telemLen + n >= (int)sizeof(telemPayload) - 1) FlushTelemetry();
    memcpy(telemPayload + telemLen, row, n);
    telemLen += n;
    telemPayload[telemLen] = 0;
    telemRows++;
    if (telemRows >= TELEM_BATCH) FlushTelemetry();
  }

  if (telemRows > 0 && (millis() - telemLastFlush) >= TELEM_MAX_AGE) FlushTelemetry();
}
///////////////////////////////////////////////////////////////////////////////
void FlushTelemetry() {
  telemLastFlush = millis();
  if (telemLen == 0) return;

  // dropping the batch is fine, the control task is not waiting for it
  if (!client.connected() ||
      !client.publish(telemetryTopic, (uint8_t *)telemPayload, telemLen, false)) {
    telemDropped += telemRows;
  }
  telemLen = 0;
  telemRows = 0;

  if (telemDropped > 0 && (millis() - telemLastWarn) >= 5000) {
    telemLastWarn = millis();
    Serial.print("Telemetry samples dropped so far: "); Serial.println(telemDropped);
  }
}
///////////////////////////////////////////////////////////////////////////////
void callback(char *topic, byte *payload, unsigned int length) {
  // telemetry control channel: "1"/"0" start-stop the MQTT stream,
  // "s1"/"s0" start-stop the copy printed on the USB serial port
  if (strcmp(topic, telemetryCtrlTopic) == 0) {
    if (length > 0) {
      if ((char)payload[0] == 's') {
        telemetryToSerial = (length > 1 && (char)payload[1] == '1') ? 1 : 0;
        Serial.print("Telemetry on serial: "); Serial.println(telemetryToSerial);
      }
      else {
        telemetryEnabled = ((char)payload[0] == '1') ? 1 : 0;
        telemLen = 0;
        telemRows = 0;
        if (telemetryEnabled) client.publish(telemetryTopic, telemHeader);
        Serial.print("Telemetry over MQTT: "); Serial.println(telemetryEnabled);
      }
    }
    return;
  }

  // "<velA>,<velB>,<velC>" in cm/s
  char message[64];
  if (length >= sizeof(message)) length = sizeof(message) - 1;
  memcpy(message, payload, length);
  message[length] = 0;

  float values[3] = {0, 0, 0};
  int index = 0;
  char *ptr = strtok(message, ",");
  while (ptr != NULL && index < 3) {
    values[index++] = atof(ptr);
    ptr = strtok(NULL, ",");
  }
  if (index < 3) return;   // ignore a truncated message instead of driving on it

  portENTER_CRITICAL(&setpointMux);
  setpointCmd[0] = values[0];
  setpointCmd[1] = values[1];
  setpointCmd[2] = values[2];
  portEXIT_CRITICAL(&setpointMux);
}
///////////////////////////////////////////////////////////////////////////////
// Non blocking: one attempt every 2 s. The wheels keep running while the broker
// is away and a failure never restarts the board.
void reconnect() {
  if (client.connected()) {
    mqttConnected = 1;
    return;
  }
  mqttConnected = 0;
  if (millis() - lastReconnectAttempt < 2000) return;
  lastReconnectAttempt = millis();

  Serial.print("Connecting to MQTT... ");
  if (client.connect("ESP32_clientID")) {
    mqttConnected = 1;
    Serial.println("connected");
    client.publish("DriverCheck", "OK");
    client.subscribe(velocityTopic);
    client.subscribe(telemetryCtrlTopic);
    // the header tells a logger started later what the columns are
    client.publish(telemetryTopic, telemHeader);
    telemLen = 0;
    telemRows = 0;
    LedStatus();
  }
  else {
    Serial.println("failed, retrying in 2 seconds");
  }
}
///////////////////////////////////////////////////////////////////////////////
void saveParamCallback() {
  Serial.println("[CALLBACK] saveParamCallback fired");
  String server_temp = getParam("mqtt_server");
  String port_temp = getParam("mqtt_port");
  server_temp.toCharArray(mqtt_server, server_temp.length() + 1);
  port_temp.toCharArray(mqtt_port, port_temp.length() + 1);
  Serial.print("PARAM mqtt_server = " + server_temp);
  Serial.println("PARAM mqtt_port = " + port_temp);
}
///////////////////////////////////////////////////////////////////////////////
String getParam(String name) {
  //read parameter from server, for customhmtl input
  String value;
  if (name == "mqtt_server") value = custom_mqtt_server.getValue();
  else { if (name == "mqtt_port") value = custom_mqtt_port.getValue(); }
  return value;
}
///////////////////////////////////////////////////////////////////////////////
void LedStatus() {
  if ((internetConnected >= 1) && (mqttConnected >= 1)) {
    if (ledChk < 3) {
      digitalWrite(ledPin, LOW);
      ledChk++;
    }
    if (ledChk >= 3) {
      digitalWrite(ledPin, HIGH);
      ledChk = 0;
    }
  }
  if ((internetConnected == 1) && (mqttConnected == 0)) {
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
}
