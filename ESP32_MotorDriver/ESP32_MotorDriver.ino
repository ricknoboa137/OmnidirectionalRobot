// Motor driver used to repeat the original velocity controller experiment.
// The control law is the ORIGINAL one, unchanged: the PID keeps the library
// default 100 ms sample time and the default 0-255 output limits, and the
// velocity is computed over the nominal sampleTime window. The only addition
// is the telemetry that records the same three signals that used to be
// printed on the serial port, so the run can be logged and plotted.
// The single line that is not from the original firmware is the
// WiFi.setSleep(false) call in setup(), marked below.
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <WiFi.h>
#include <PubSubClient.h>
#include <PID_v1.h>


char *strings[3];
float velocity_values[3]={0,0,0};
char *ptr = NULL;
uint32_t x=0;
char mqtt_server[40]= "192.168.0.109";
char mqtt_port[6] = "1883";

WiFiManager wm;
WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
WiFiClient espClient;
PubSubClient client(espClient);

const uint8_t encPinA1 = 13, encPinA2 = 14, encPinB1 = 27, encPinB2 = 26, encPinC1 = 25, encPinC2 = 33;
const uint8_t motorFrwdA = 22, motorBkwdA = 23, motorFrwdB = 18, motorBkwdB = 19, motorFrwdC = 17, motorBkwdC = 5;
const uint8_t ledPin = 2;
volatile long Encodervalue=0, LastEncodervalue=0, DeltaEncodervalue=0;
volatile long EncodervalueA=0, LastEncodervalueA=0, DeltaEncodervalueA=0;
volatile long EncodervalueB=0, LastEncodervalueB=0, DeltaEncodervalueB=0;
volatile long EncodervalueC=0, LastEncodervalueC=0, DeltaEncodervalueC=0;
volatile unsigned prevSampTimeA=0, currSampTimeA=0, deltSampTimeA=0;
volatile unsigned prevSampTimeB=0, currSampTimeB=0, deltSampTimeB=0;
volatile unsigned prevSampTimeC=0, currSampTimeC=0, deltSampTimeC=0;
float velVect[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ;
float velVectA[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ;
float velVectB[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ;
float velVectC[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ;
int filterNumber =10;
int cont = 0;
int internetConnected =0, mqttConnected =0;
volatile unsigned prevLedTime=0, currLedTime=0;
int sampleTime = 5, ledChk=0;;
int N = 692; // ticks per rev 
float diam = 6.0, velA, velB, velC;
double freqA=0;
double wA=0;
double vA=0;
double Setpoint, Input, Output;
double SetpointB, InputB, OutputB;
double SetpointC, InputC, OutputC;
double absSetpointA, absSetpointB, absSetpointC;
double Kp=4.0, Ki= 23, Kd=0.012;
PID myPID(&Input, &Output, &absSetpointA, Kp, Ki, Kd, DIRECT);
PID myPIDB(&InputB, &OutputB, &absSetpointB, Kp, Ki, Kd, DIRECT);
PID myPIDC(&InputC, &OutputC, &absSetpointC, Kp, Ki, Kd, DIRECT);
//////////////////////////////////TELEMETRY/////////////////////////////////////
// Streams the measured wheel velocities (plus setpoints and PID outputs) back
// over MQTT so the host can store them in a log file and plot the controller
// response. One sample is taken every control cycle (sampleTime ms) and the
// samples are sent in batches, to keep the packet rate low without losing time
// resolution.
const char *telemetryTopic = "motor_telemetry";          // ESP32 -> host, CSV rows
const char *telemetryCtrlTopic = "motor_telemetry_ctrl"; // host -> ESP32, "1"/"0"/"s1"/"s0"
const uint16_t TELEM_BUFFER_SIZE = 1024;  // MQTT packet buffer (PubSubClient defaults to only 256)
const uint8_t TELEM_BATCH = 8;            // rows per MQTT message ( 8 * 5 ms = one packet every 40 ms )
const uint16_t TELEM_MAX_AGE = 200;       // ms, send a partial batch after this long
int telemetryEnabled = 1;                 // stream over MQTT
int telemetryToSerial = 0;                // mirror the same CSV rows on the USB serial port
char telemPayload[TELEM_BUFFER_SIZE];
uint16_t telemLen = 0;
uint8_t telemRows = 0;
uint32_t telemSeq = 0;                    // row counter, lets the logger spot dropped packets
uint32_t telemDropped = 0;
unsigned long telemLastFlush = 0;
unsigned long telemLastWarn = 0;
// column layout of every telemetry row, published on each MQTT (re)connection
const char *telemHeader = "#seq,t_ms,setA,velA,pwmA,setB,velB,pwmB,setC,velC,pwmC";
void LogVelocities();
void FlushTelemetry();
////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////INTERRUPCTIONS///////////////////////////////
void IRAM_ATTR isrA() {
 if (digitalRead(encPinA1)> digitalRead(encPinA2))
  EncodervalueA++;
 else
  EncodervalueA--;
 //Serial.println(EncodervalueA);
}


void IRAM_ATTR isrB() {
 if (digitalRead(encPinB1)> digitalRead(encPinB2))
  EncodervalueB++;
 else
  EncodervalueB--;

}

void IRAM_ATTR isrC() {
 if (digitalRead(encPinC1)> digitalRead(encPinC2))
  EncodervalueC++;
 else
  EncodervalueC--;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(115200);
    pinMode(motorFrwdA, OUTPUT);
    pinMode(motorBkwdA, OUTPUT);
    pinMode(motorFrwdB, OUTPUT);
    pinMode(motorBkwdB, OUTPUT);
    pinMode(motorFrwdC, OUTPUT);
    pinMode(motorBkwdC, OUTPUT);
    pinMode(ledPin, OUTPUT);
    analogWrite(motorFrwdA, 0);
    analogWrite(motorBkwdA, 0);
    analogWrite(motorFrwdB, 0);
    analogWrite(motorBkwdB, 0);
    analogWrite(motorFrwdC, 0);
    analogWrite(motorBkwdC, 0);
    LedStatus();
    //WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    
    // reset settings - wipe stored credentials for testing
    // these are stored by the esp library
    //wm.resetSettings();
    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    wm.addParameter(&custom_mqtt_server);
    wm.addParameter(&custom_mqtt_port);
    wm.setSaveParamsCallback(saveParamCallback);
    wm.setClass("invert"); // use darkmode

    client.setServer(mqtt_server, atoi(mqtt_port));
    client.setCallback(callback);
    // enlarge the MQTT packet buffer so a batch of telemetry rows fits in one message
    if (!client.setBufferSize(TELEM_BUFFER_SIZE)) {
      Serial.println("Could not allocate the telemetry MQTT buffer - telemetry disabled");
      telemetryEnabled = 0;
    }


    bool res;
    res = wm.autoConnect("Motor_Driver"); // password protected ap ,"password"

    if(!res) {
        internetConnected = 0;
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :):):)");
        // NOT IN THE ORIGINAL FIRMWARE. The ESP32 station defaults to
        // WIFI_PS_MIN_MODEM, so incoming setpoints wait for the next access
        // point beacon. This only changes when a command is delivered, not
        // how the wheel responds to it. Comment this line out to run exactly
        // the original firmware.
        WiFi.setSleep(false);
        internetConnected = 1;
        reconnect(); //connect MQTT        
    }
    client.subscribe("motor_velocities");
    client.subscribe(telemetryCtrlTopic);
    pinMode(encPinA1, INPUT_PULLUP);
    pinMode(encPinA2, INPUT_PULLUP);
    pinMode(encPinB1, INPUT_PULLUP);
    pinMode(encPinB2, INPUT_PULLUP);
    pinMode(encPinC1, INPUT_PULLUP);
    pinMode(encPinC2, INPUT_PULLUP);
    digitalWrite(encPinA1, HIGH); //turn pullup resistor on
    digitalWrite(encPinA2, HIGH); 
    digitalWrite(encPinB1, HIGH); //turn pullup resistor on
    digitalWrite(encPinB2, HIGH); 
    digitalWrite(encPinC1, HIGH); //turn pullup resistor on
    digitalWrite(encPinC2, HIGH); 
    attachInterrupt(encPinA1, isrA, RISING);
    attachInterrupt(encPinB1, isrB, RISING);
    attachInterrupt(encPinC1, isrC, RISING);
    prevLedTime = millis();
    currSampTimeA = millis();
    //Setpoint = 0;
    myPID.SetMode(AUTOMATIC);
    myPIDB.SetMode(AUTOMATIC);
    myPIDC.SetMode(AUTOMATIC);
    //delay(200);
    

}

void loop() {
    // put your main code here, to run repeatedly:  
  if ((WiFi.status() != WL_CONNECTED) || (WiFi.localIP().toString() == "0.0.0.0")) {
    internetConnected =0;
    Serial.println("ROUTER DISCONNECTED !!!!");
  }
  else{
    internetConnected =1;
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  if (millis()-currLedTime >= 330)
 {
    LedStatus();
    currLedTime=millis();
 }
 // the MQTT write is done between control ticks so it cannot delay one
 if (telemRows >= TELEM_BATCH || (telemRows > 0 && millis() - telemLastFlush >= TELEM_MAX_AGE)) FlushTelemetry();

 if (millis()-currSampTimeA >= sampleTime)
 {
  
  velA = CalcVelocity(1);
  currSampTimeA = millis();
  velB = CalcVelocity(2);
  currSampTimeB = millis();
  velC = CalcVelocity(3);
  currSampTimeC = millis();  
  Input = abs(velA);
  InputB = abs(velB);
  InputC = abs(velC);
  myPID.Compute();
  myPIDB.Compute();
  myPIDC.Compute();
  MoveWheels(Output,OutputB,OutputC);
  LogVelocities();   // stream velA/velB/velC (+ setpoints and PWM) back to the host
  //Serial.print(velA);Serial.print(",");Serial.print(Setpoint);Serial.print(",");Serial.print(Output/10);Serial.print(",");
  //Serial.print(velB);Serial.print(",");Serial.print(SetpointB);Serial.print(",");Serial.print(OutputB/10);Serial.print(",");
  //Serial.print(velC);Serial.print(",");Serial.print(SetpointC);Serial.print(",");Serial.print(OutputC/10);Serial.println(" ");
  //Serial.println(vA);
  //Serial.print(internetConnected);Serial.print(" , ");Serial.println(mqttConnected);
 }
}


//////////////////////////////////////////////////////////////////////
void saveParamCallback(){
  Serial.println("[CALLBACK] saveParamCallback fired");
  String server_temp = getParam("mqtt_server");
  String port_temp = getParam("mqtt_port");  
  server_temp.toCharArray(mqtt_server, server_temp.length() + 1);
  port_temp.toCharArray(mqtt_port, port_temp.length() + 1);
  Serial.print("PARAM mqtt_server = " + server_temp );
  Serial.println("PARAM mqtt_port = " +  port_temp);

}
/////////////////////////////////////////////////////////////////////////
String getParam(String name){
  //read parameter from server, for customhmtl input
  String value;
  if(name == "mqtt_server") value = custom_mqtt_server.getValue();
  else{   if(name =="mqtt_port") value = custom_mqtt_port.getValue();  }
  return value;
}
//////////////////////////////////////////////////////////////////
void callback(char* topic, byte* payload, unsigned int length) {
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
  //Serial.print("Message arrived on topic: ");
  //Serial.println(topic);
  //Serial.print("Message: ");
  //String inMessage;
  char message[200]=" ";
  if (length > sizeof(message) - 1) length = sizeof(message) - 1;  // keep the copy inside the buffer
  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
    //inMessage+=(char)payload[i];
    message[i]=(char)payload[i];
  }
  //Serial.println("");
  
  byte index = 0;
  ptr = strtok(message, ",");  // delimiter
  while (ptr != NULL)
   {
      strings[index] = ptr;
      index++;
      ptr = strtok(NULL, ",");
   }
  for (int n = 0; n < index; n++)
   {
      //Serial.print(n);
      //Serial.print("  ");
      //Serial.println(strings[n]);
      velocity_values[n]=atof(strings[n]);
   }
  Setpoint=velocity_values[0];
  SetpointB=velocity_values[1];
  SetpointC=velocity_values[2];
  absSetpointA = abs(Setpoint);
  absSetpointB = abs(SetpointB);
  absSetpointC = abs(SetpointC);
}
////////////////////////////////////////////////////////////////////////////
void reconnect() {
   int8_t ret;

  // Stop if already connected.
  if (client.connected()) {
    mqttConnected = 1;
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 15;
  while (!client.connected()) {// connect will return 0 for connected
         
       if (client.connect("ESP32_clientID")) {
          mqttConnected = 1;
          LedStatus();
          Serial.println("connected");
          // Once connected, publish an announcement...
          client.publish("DriverCheck", "OK");
          // ... and resubscribe
          client.subscribe("motor_velocities");
          client.subscribe(telemetryCtrlTopic);
          // re-send the CSV header so a logger started later still knows the columns
          client.publish(telemetryTopic, telemHeader);
          telemLen = 0;
          telemRows = 0;
        }
        else {
          mqttConnected = 0;
          Serial.println("Retrying MQTT connection in 2 seconds...");
          LedStatus();
          delay(500);  // wait 2 seconds         
          retries--;
          if (retries == 0) {
            // basically die and wait for WDT to reset me
            //wm.resetSettings();
            break;
          }        
        }       
      }     
  if (!client.connected()) {
      Serial.println("Failed to connect MQTT - restarting ESP!");      
      ESP.restart();
  }
  else {
      mqttConnected = 1;
      Serial.println("connected**");
      
  }
}
///////////////////////////////////////////////////////////////////////
void MoveWheels(int a, int b, int c)
{
  if (Setpoint >= 0)
  {
    if (Setpoint == 0)
    { analogWrite(motorBkwdA, 0); analogWrite(motorFrwdA, 0);
      myPID.SetMode(MANUAL);
      Output = 0;
      myPID.SetMode(AUTOMATIC); }
    else{
    analogWrite(motorBkwdA, 0);
    analogWrite(motorFrwdA, 0);
    analogWrite(motorFrwdA, a);}
    
  }
  else 
  {
    analogWrite(motorFrwdA, 0); 
    analogWrite(motorBkwdA, 0); 
    analogWrite(motorBkwdA, a);
       
  }
  if (SetpointB >= 0)
  {
    if (SetpointB == 0)
    { analogWrite(motorBkwdB, 0); analogWrite(motorFrwdB, 0);
      myPIDB.SetMode(MANUAL);
      OutputB = 0;
      myPIDB.SetMode(AUTOMATIC); }
    else{
      analogWrite(motorBkwdB, 0);
      analogWrite(motorFrwdB, 0);
      analogWrite(motorFrwdB, b);}
    
  }
  else 
  {
    analogWrite(motorFrwdB, 0); 
    analogWrite(motorBkwdB, 0); 
    analogWrite(motorBkwdB, b);       
  }
  if (SetpointC >= 0)
  {
    if (SetpointC == 0)
    {   analogWrite(motorBkwdC, 0); analogWrite(motorFrwdC, 0);
      myPIDC.SetMode(MANUAL);
      OutputC = 0;
      myPIDC.SetMode(AUTOMATIC);}
    else{
        analogWrite(motorBkwdC, 0);
        analogWrite(motorFrwdC, 0);
        analogWrite(motorFrwdC, c);}
    
  }
  else 
  {
    analogWrite(motorFrwdC, 0); 
    analogWrite(motorBkwdC, 0); 
    analogWrite(motorBkwdC, c);
       
  }

}
////////////////////////////////////////////////////////////////////////
//////////////////////CALCULATE_VELOCITY///////////////////////////

float CalcVelocity (int motorLabel)
{

  if (motorLabel==1){
    Encodervalue = EncodervalueA;
    LastEncodervalue = LastEncodervalueA;  
    for(int i=0; i < filterNumber; i++)
    {
      velVect[i]=velVectA[i];
    }
  }
  if (motorLabel==2){
    Encodervalue = EncodervalueB;
    LastEncodervalue = LastEncodervalueB;  
    for(int i=0; i < filterNumber; i++)
    {
      velVect[i]=velVectB[i];
    }
  }
  if (motorLabel==3){
    Encodervalue = EncodervalueC;
    LastEncodervalue = LastEncodervalueC;  
    for(int i=0; i < filterNumber; i++)
    {
      velVect[i]=velVectC[i];
    }
  }

  float media = 0 ;
  DeltaEncodervalue = Encodervalue - LastEncodervalue;
  freqA = (DeltaEncodervalue*1000)/(double) sampleTime;
  wA = ((2*3.141596)/N)*freqA;
  vA = (diam/2)*wA;
  for (int i = 0 ; i < filterNumber -1; i++){
      velVect[i] = velVect[i+1];
  }
  velVect[filterNumber -1 ] = wA;
  for (int i = 0 ; i < filterNumber ; i++){
      media+= velVect[i];
  }
  media = media/filterNumber;
  //velVect[filterNumber -1 ] = media;
  
  if (motorLabel==1){
    for(int i=0; i < filterNumber; i++)
    {
      velVectA[i]=velVect[i];      
    }
    LastEncodervalueA = Encodervalue;
  }
  if (motorLabel==2){
    for(int i=0; i < filterNumber; i++)
    {
      velVectB[i]=velVect[i];      
    }
    LastEncodervalueB = Encodervalue;
  }
  if (motorLabel==3){
    for(int i=0; i < filterNumber; i++)
    {
      velVectC[i]=velVect[i];      
    }
    LastEncodervalueC = Encodervalue;
  }
  return (media);

}

/////////////////////////////////////////////////////////////////////////

void LedStatus()
{
  if ((internetConnected >= 1) && (mqttConnected >= 1 )) 
  {
    if(ledChk < 3 )
    {
      digitalWrite(ledPin, LOW);
      ledChk++;
    }
    if(ledChk >= 3 )
    {
      digitalWrite(ledPin, HIGH);
      ledChk=0;
    }
    
  }
  if ((internetConnected==1) &&(mqttConnected == 0 )) 
  {
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
  
}

/////////////////////////////////////////////////////////////////////////
////////////////////////TELEMETRY / VELOCITY LOG/////////////////////////
// One CSV row per control cycle:
//   seq,t_ms,setA,velA,pwmA,setB,velB,pwmB,setC,velC,pwmC
// velX is the filtered angular velocity in rad/s (signed, from the encoders),
// setX is the commanded velocity, pwmX is the PID output signed by the
// commanded direction, so the response can be plotted straight from the log.

void LogVelocities()
{
  if (!telemetryEnabled && !telemetryToSerial) return;

  char row[112];
  int n = snprintf(row, sizeof(row),
                   "%lu,%lu,%.3f,%.3f,%.1f,%.3f,%.3f,%.1f,%.3f,%.3f,%.1f\n",
                   (unsigned long)telemSeq, (unsigned long)millis(),
                   Setpoint,  (double)velA, (Setpoint  < 0 ? -Output  : Output),
                   SetpointB, (double)velB, (SetpointB < 0 ? -OutputB : OutputB),
                   SetpointC, (double)velC, (SetpointC < 0 ? -OutputC : OutputC));
  telemSeq++;
  if (n <= 0) return;
  if (n >= (int)sizeof(row)) n = sizeof(row) - 1;   // snprintf truncated the row

  if (telemetryToSerial) Serial.print(row);
  if (!telemetryEnabled) return;

  // flush first if this row would not fit in the packet buffer
  if (telemLen + n >= (int)sizeof(telemPayload) - 1) FlushTelemetry();

  memcpy(telemPayload + telemLen, row, n);
  telemLen += n;
  telemPayload[telemLen] = '\0';
  telemRows++;

}
/////////////////////////////////////////////////////////////////////////
void FlushTelemetry()
{
  telemLastFlush = millis();
  if (telemLen == 0) return;

  // drop the batch rather than block the control loop if MQTT is down
  if (!client.connected() || !client.publish(telemetryTopic, (uint8_t *)telemPayload, telemLen, false)) {
    telemDropped += telemRows;
    if (millis() - telemLastWarn >= 5000) {   // do not flood the serial port
      telemLastWarn = millis();
      Serial.print("Telemetry rows dropped so far: "); Serial.println(telemDropped);
    }
  }
  telemLen = 0;
  telemRows = 0;
}
