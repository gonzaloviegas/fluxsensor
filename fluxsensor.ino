#include <Arduino.h>
#include <EEPROM.h>
#include <TimerOne.h> 

// Pin del sensor de flujo
const byte flowSensorPin = 2; // Conectado al pin 2
volatile int pulseCount = 0;
float flowRate = 0.0;
float calibrationFactor = 7.5;
bool flowSensorActive = true;


// constants won't change. They're used here to set pin numbers:
const byte buttonPin = 8;  // the number of the pushbutton pin
const byte relayPin = 6; // Conectado al pin 6

// variables will change:
bool ignoreButton = false;
int buttonState = 0;  // variable for reading the pushbutton status

unsigned long relayTimeOnMs= 10000000; // en microsegundos (60000000)
unsigned long flowSensorOfTimeMs= 10000000; // en microsegundos (60000000)
unsigned long measureTimeWindow = 5000; // en milisegundos (60000) //tiempo tomado en total para calcular la flowRate
unsigned long lastFlowRateTime;
float liters = 0.0;
float triggerRelayAfterLiters = 2; //activar relay luego de perder tantos litros

void setup() {
  Serial.begin(9600);
  
  //flow sensor setup
  activateFlowSensor();

  // setup  pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  // setup relay
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // Apagar el relé inicialmente (estado apagado)

  // Inicializar tiempo de encendido despues de presionar el boton
  Timer1.initialize(relayTimeOnMs);  // Configura TimerOne para el tiempo de encendido del relé
  Timer1.attachInterrupt(buttonTimerInterrupt);  // Asocia la interrupción con la función timerInterrupt
  Timer1.stop();

  lastFlowRateTime = millis();
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (!ignoreButton && buttonState == HIGH) {
    Serial.println("Button HIGH");    // turn LED on:
    startRelay();
  }

  calculateFlowRate();
  if(flowSensorActive && liters >= triggerRelayAfterLiters){
    startRelay();
  }
 
  delay(100); //miliseconds
}

void pulseCounter() {
  pulseCount++;
  Serial.print("p");
}

void calculateFlowRate() {
  unsigned long elapsedTime = millis() - lastFlowRateTime;
  if(flowSensorActive && elapsedTime > measureTimeWindow)    // Only process counters once per minute
  { 
    Serial.println("\nelapsedTime: " + String(elapsedTime) + " measureTimeWindow:" + String(measureTimeWindow));
    //deactivateFlowSensor(); // No se si vale la pena bloquear las interrupciones para calcular el flow
        
    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1.0 * measureTimeWindow / elapsedTime) * pulseCount) / calibrationFactor;

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

void activateFlowSensor(){
  Timer1.stop();
  pinMode(flowSensorPin, INPUT);
  //digitalWrite(flowSensorPin, HIGH); //descomentado
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, RISING); //FALLING
  liters = 0.0;
  flowSensorActive = true;
  restartPulseCount();
  Serial.println("Flow Sensor ON");
}

void deactivateFlowSensor(){
  detachInterrupt(digitalPinToInterrupt(flowSensorPin));
  flowSensorActive = false;
  restartPulseCount();
  Serial.println("Flow Sensor OFF");
}

void restartPulseCount(){
  pulseCount = 0;
  lastFlowRateTime = millis();
}


void buttonTimerInterrupt() {
  Serial.println("Relay time is done");    // turn LED on:
  stopRelay();
  // Inicializar tiempo de encendido despues de presionar el boton
  Timer1.initialize(flowSensorOfTimeMs);  // Configura TimerOne para el tiempo de encendido del relé
  Timer1.attachInterrupt(activateFlowSensor);  // Asocia la interrupción con la función timerInterrupt
  //activateFlowSensor();
}

void startRelay(){
    //Luego de presionar el boton, debe qeudar prendido X tiempo
    digitalWrite(relayPin, HIGH); //Prender
    ignoreButton = true;
    deactivateFlowSensor();
        // Inicializar tiempo de encendido despues de presionar el boton
    Timer1.initialize(relayTimeOnMs);  // Configura TimerOne para el tiempo de encendido del relé
    Timer1.attachInterrupt(buttonTimerInterrupt);  // Asocia la interrupción con la función timerInterrupt
    Timer1.start();  // Inicia el temporizador cuando se presiona el botón
}

void stopRelay(){
  Timer1.stop();  // Detén el temporizador cuando ha pasado el tiempo de encendido del relé
  digitalWrite(relayPin, LOW); // Apagar el relé después de 1 minuto
  ignoreButton = false; //habilitar el botón nuevamente
}
