/*
 * This firmware was designef for:
 *  - interact with torso_node
 *  - read the torso height by sharp sensor
 *  - move the torso motor when required
*/
#include <Servo.h>
#include "settings.h"


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
byte paro_state      = LOW;
byte paro_state_old  = LOW;
bool paro_state_change = false;
int paro_msg_counter;
long LAST_STOP_MGS_TIME;
long LAST_PARO_BUTON_CHECK;
int paro_button_counter_low;
int paro_button_counter_high;
int old_value;
int new_value;

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

void check_paro_buton(int new_paro_value, int old_paro_value)
{
  if (new_paro_value == 0 && new_paro_value == old_paro_value && paro_button_counter_low <= 50)
  {
    paro_button_counter_low++;  
    paro_button_counter_high = 0;
  }else if (new_paro_value == 1 && new_paro_value == old_paro_value && paro_button_counter_high <= 50)
  {
    paro_button_counter_low = 0;
    paro_button_counter_high++;
  }

  if (paro_button_counter_low > 50)
  {
    paro_state_old = paro_state;
    paro_state = HIGH; //true, because we have a stop message
  }else if (paro_button_counter_high > 50)
  {
    paro_state_old = paro_state;
    paro_state = LOW;  //false, because the button is nos pressed
  }

  if (paro_state_old != paro_state) 
  {
    paro_state_change = true;
    paro_msg_counter = MAX_MSG_TO_SEND;
  }
  
}


void setup() {
  Serial.begin(BAUD);
  values.tope      = 0;
  values.base      = 0;  
  values.cant_elem = 0;
  update_pose = false;  
  LAST_STOP_MGS_TIME = 0;
  LAST_PARO_BUTON_CHECK = 0;
  paro_msg_counter = 0;
  //motor pins
  motor_tronco.attach(PIN_MOTOR_PWM);
  motor_tronco.write(MOTOR_STOP);
  paro_button_counter_low  = 0;
  paro_button_counter_high = 0;
  old_value = 0;
  new_value = 0;
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
  
  //debug
  pinMode(13, OUTPUT);
  digitalWrite (13, LOW);
}


void loop() {
  leer_serial();         // read message
  int raw_value = analogRead(PIN_DIST_SENSOR);
  
  //check paro buton ever PARO_BUTON_CHECK_TOUT miliseconds
  if (millis() - LAST_PARO_BUTON_CHECK > PARO_BUTON_CHECK_TOUT)
  {
      old_value = new_value;
      new_value = digitalRead(PARO_BUTTON_PIN);
      check_paro_buton(new_value, old_value);
      LAST_PARO_BUTON_CHECK = millis();
  }


  
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

  if ( paro_state_change && (millis() - LAST_STOP_MGS_TIME) > STOP_MESG_TIMEOUT )
  {
    sendMsg (MY_ID, MOD_SYSTEM, OP_STOP, &paro_state, 1);
    LAST_STOP_MGS_TIME = millis(); 
    if (paro_msg_counter > 0)  paro_msg_counter--;
    if (paro_msg_counter == 0) paro_state_change = false;
    //Serial.print("paro buton = ");Serial.println(paro_state);        
  }
  
}
