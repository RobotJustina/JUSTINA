/*
 * This firmware was designef for:
 *  - interact with torso_node
 *  - read the torso height by sharp sensor
 *  - move the torso motor when required
*/
#include <Servo.h>
#include "settings.h"


//FIXME: find outliers with a big less meassure

struct data{
  int raw_values[MAX_ELEM];
  int tope;
  int base;
  int cant_elem;
};

data values;
boolean update_pose;
byte goal_pose;
Servo motor_tronco;
double promMain;
bool outlayer;
byte cm_fixed;
volatile byte paro_state = LOW;
long LAST_STOP_MGS_TIME;

void addValue (int value)
{
  values.tope = (values.tope + 1) % MAX_ELEM;
  values.raw_values[values.tope] = value;
  if (values.tope == values.base) values.base = (values.base + 1) % MAX_ELEM;
  values.cant_elem = values.cant_elem + 1;
  if (values.cant_elem > MAX_ELEM) values.cant_elem = MAX_ELEM;
}

double getPromDist ()
{
  double prom = 0;
  int cant_elem = 0;
  int iter = values.base;
  while ( cant_elem < values.cant_elem)
  {
    prom = prom + values.raw_values[iter];
    iter = (iter + 1) % MAX_ELEM;
    cant_elem++;
  }
  return prom/cant_elem;    
}


void setup() {
  Serial.begin(BAUD);
  values.tope      = 0;
  values.base      = 0;  
  values.cant_elem = 0;
  update_pose = false;  
  LAST_STOP_MGS_TIME = 0;
  //motor pins
  motor_tronco.attach(PIN_MOTOR_PWM);
  motor_tronco.write(MOTOR_STOP);

  //init sensor values - be sure 
  for (int i = 0; i < MAX_ELEM; i++)
  {
    int raw_value = analogRead(PIN_DIST_SENSOR);
    addValue (raw_value);
  }
  promMain = getPromDist();
  outlayer = false;
  cm_fixed = sharp2cm (promMain);

  //interrupt from boton de paro
  paro_state = LOW;
  pinMode(PARO_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PARO_BUTTON_PIN), paro_interrupt, LOW);
  
  //debug
  pinMode(13, OUTPUT);
  digitalWrite (13, LOW);
}


void loop() {
  leer_serial();         // read message
  int raw_value = analogRead(PIN_DIST_SENSOR);
  
  //This is in order to protect something in the sensor line
  if ( abs( promMain - raw_value ) < OUTLAYER_LIMIT )
  {
    addValue (raw_value);  
    promMain = getPromDist(); 
    outlayer = false;    
  }else{    
    stop_motor();
    outlayer = true;
  }

  //Serial.print(sharp2cm (promMain)); Serial.print ("  ");
  //Serial.println(outlayer);
  
  if (update_pose and !outlayer)  
  {
    byte cm = sharp2cm (promMain);
    if ( goal_pose > cm) {
      move_motor_up();
    }
    else
    {
      move_motor_down();        
    }
 
    if ( abs( cm - goal_pose ) < MOTOR_THRESHOLD_IN)
    {
      stop_motor();
      update_pose = false;  
      cm_fixed = cm;      
    }
   } 

  if (  paro_state  && ( (millis() - LAST_STOP_MGS_TIME) > STOP_MESG_TIMEOUT )  )
  {
    sendMsg (MY_ID, MOD_SYSTEM, OP_STOP, NULL, 0);
    LAST_STOP_MGS_TIME = millis(); 
  }
  //Serial.print("paro buton = ");Serial.println(paro_state);    
  
}

void paro_interrupt()
{
  paro_state = HIGH;
}
