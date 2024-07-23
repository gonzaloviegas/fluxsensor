#include <Arduino.h>
#include <EEPROM.h>

// pins
const byte FLOW_SENSOR_PIN = 2; // Conectado al pin 2
const byte RELAY_PIN = 6; // Conectado al pin 6
const byte BUTTON_PIN = 8;  // the number of the pushbutton pin  --> Foce START BUTTON


// Constantes para calibrar tiempos de espera, canitdad de litros para activar el relay, etc
const unsigned long RELAY_TIME_MILLIS = 60000; // en milisegundos 60000 = 1 minuto
const unsigned long FLOW_SENSOR_TIMEOFF_MILLIS = 120000; // en milisenduos 60000 = 1 minuto
const unsigned long FLOW_SENSOR_WINDOWS_MEASURE_TIME_MILLIS = 60000; // en milisegundos (60000) //tiempo tomado en total para calcular la flowRate
const float FLOW_SENSOR_TRIGGER_LITERS = 2; //activar relay luego de perder tantos litros
const float FLOW_SENSOR_CALIBRATION_FACTOR = 7.5;   //convertir pulsos en litros

//Variables intial state
unsigned long lastFlowRateTime = 0;
unsigned long lastPulseCount = 0;
volatile int pulseCount = 0;
float flowRate = 0.0;
bool flowSensorActive = true;
float liters = 0.0;
bool ignoreButton = false;
int buttonState = 0;  // variable for reading the pushbutton status

void setup() {
  Serial.begin(9600);
  
  //flow sensor setup
  activateFlowSensor();

  // setup  pushbutton pin as an input:
  pinMode(BUTTON_PIN, INPUT);

  // setup relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Apagar el relé inicialmente (estado apagado)


  lastFlowRateTime = millis();
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(BUTTON_PIN);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (!ignoreButton && buttonState == HIGH) {
    Serial.println("Button Pressed -> Force Start Relay");    // turn LED on:
    buttonTimerInterrupt();
  }

  calculateFlowRate();
  if(flowSensorActive && liters >= FLOW_SENSOR_TRIGGER_LITERS){
    buttonTimerInterrupt();
  }

  if (pulseCount != lastPulseCount ){
    lastPulseCount = pulseCount;
    Serial.println("Pulsos leidos: " + String(pulseCount));
  }
 
  delay(100); //miliseconds
}

void pulseCounter() {
  pulseCount++;
  Serial.print(".");
}

void calculateFlowRate() {
  unsigned long elapsedTime = millis() - lastFlowRateTime;
  if(flowSensorActive && elapsedTime > FLOW_SENSOR_WINDOWS_MEASURE_TIME_MILLIS)    // Only process counters once per minute
  { 
    Serial.println("\nelapsedTime: " + String(elapsedTime) + " FLOW_SENSOR_WINDOWS_MEASURE_TIME_MILLIS:" + String(FLOW_SENSOR_WINDOWS_MEASURE_TIME_MILLIS));
    //deactivateFlowSensor(); // No se si vale la pena bloquear las interrupciones para calcular el flow
        
    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the FLOW_SENSOR_CALIBRATION_FACTOR to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1.0 * FLOW_SENSOR_WINDOWS_MEASURE_TIME_MILLIS / elapsedTime) * pulseCount) / FLOW_SENSOR_CALIBRATION_FACTOR;

    liters += flowRate *  elapsedTime / 60000;
       //activateFlowSensor(); // No se si vale la pena bloquear las interrupciones para calcular el flow
    Serial.println("Pulses: " + String(pulseCount) + " - Flow Rate: " + String(flowRate) + "L/min - Qty:" + String(liters) + "L");
    
    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.  
 
    // Reset the pulse counter so we can start incrementing again
    restartPulseCount();
  }
}

void waitMillis(long waitMillis){
  unsigned long initMillis = millis();
  Serial.println("WaitMillis " + String(waitMillis) + " starts at " + String(initMillis)); 
  while(millis() - initMillis < waitMillis){
    delay(100); //miliseconds
    Serial.print("/");
  }
  Serial.println("\nWaitMillis " + String(waitMillis) + " ends at " + String(millis() - initMillis)); 
}

void activateFlowSensor(){
  Serial.println("activateFlowSensor at " +  String(millis()));
  pinMode(FLOW_SENSOR_PIN, INPUT);
  //digitalWrite(FLOW_SENSOR_PIN, HIGH); //descomentado
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING); //FALLING
  flowSensorActive = true;
  restartPulseCount();
  Serial.println("Flow Sensor is ON");
}

void deactivateFlowSensor(){
  Serial.println("deactivateFlowSensor at " +  String(millis()));
  detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
  flowSensorActive = false;
  restartPulseCount();
  Serial.println("Flow Sensor is OFF");
}

void restartPulseCount(){
  Serial.println("restartPulseCount");
  pulseCount = 0;
  liters = 0.0;
  lastFlowRateTime = millis();
}


void buttonTimerInterrupt() {
  Serial.println("buttonTimerInterrupt - Start Relay and wait " + String(RELAY_TIME_MILLIS) + " while flow sensor and button are deactivated");    // turn LED on:
  startRelay();
  deactivateFlowSensor();
  // Inicializar tiempo de encendido despues de presionar el boton
  waitMillis(RELAY_TIME_MILLIS);
  Serial.println("buttonTimerInterrupt - Stop Relay and wait " + String(FLOW_SENSOR_TIMEOFF_MILLIS) + " before activate button and flow sensor again");    // turn LED on:
  stopRelay();
  // Inicializar tiempo de encendido despues de presionar el boton
  waitMillis(FLOW_SENSOR_TIMEOFF_MILLIS);
  activateFlowSensor();
  Serial.println("\nbuttonTimerInterrupt - Ready again\n");    // turn LED on:
}

void startRelay(){
  //Luego de presionar el boton, debe qeudar prendido X tiempo
  Serial.println("startRelay at " +  String(millis()));    // turn LED on:
  digitalWrite(RELAY_PIN, LOW); //Prender
  ignoreButton = true;
}

void stopRelay(){
  Serial.println("stopRelay at " +  String(millis()));    // turn LED on:
  digitalWrite(RELAY_PIN, HIGH); // Apagar el relé después de 1 minuto
  ignoreButton = false; //habilitar el botón nuevamente
}