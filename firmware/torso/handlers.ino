
///////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              SYSTEM                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////////

void handleSystem (int opcode, byte* parameters, int largo) {
  switch (opcode) {
  case OP_PING: 
    {
      sendMsg (MY_ID, MOD_SYSTEM, OP_PING, NULL, 0);
      break;  
    }
  case OP_VERSION: 
    {
      parameters[0]=VERSION;
      sendMsg (MY_ID, MOD_SYSTEM, OP_VERSION, parameters, 1);
      break;  
    }
  case OP_LISTMOD: 
    {
      break;  
    }
  }
}

// error reporting
void error (char* text) {
  sendMsg (MY_ID, MOD_SYSTEM, OP_ERROR, (byte*)text, strlen(text));   
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              SENSORS                                              //
///////////////////////////////////////////////////////////////////////////////////////////////////////

void handleSensors (int opcode, byte* parameters, int largo) { 
  switch (opcode) {
  case OP_SETDISTMAX:   // 2 bytes, unidades y centésimas 
     
      break;
    
  case OP_SETDISTMIN:   // 2 bytes, unidades y centésimas
    
      break;
    
  case OP_SETTARGETVEL:   
   
      break;
   
  case OP_GETDISTMAX:
  
     
      break;  
    
  case OP_GETDISTMIN: 
   
     
      break;  
    
  case OP_GETTARGETVEL:   
   
     
      break;
   
  case OP_GETCURRENTDIST:       
      if (update_pose)
      {
        double prom = getPromDist();
        byte cm = sharp2cm (prom);
        sendMsg (MY_ID, MOD_SENSORS, OP_GETCURRENTDIST, &cm, 1);
      }else
      {
        sendMsg (MY_ID, MOD_SENSORS, OP_GETCURRENTDIST, &cm_fixed, 1);
      }
      break;      
   
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              MOTORS                                              //
///////////////////////////////////////////////////////////////////////////////////////////////////////

void handleMotors (int opcode, byte* parameters, int largo) { 
  switch (opcode) {
    
    case OP_SETTORSOPOSE:   // 2 bytes, unidades y centesimas 
        goal_pose = parameters[0];
        if ( abs( sharp2cm ( getPromDist() ) - parameters[0] ) > MOTOR_THRESHOLD_IN)
        {
          update_pose = true;
        }
        sendMsg (MY_ID, MOD_MOTORS, OP_SETTORSOPOSE, &goal_pose, 1);      
        break; 
      
    case OP_CALIBRATE:   // 2 bytes, unidades y centesimas
      for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        motor_tronco.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
      for (int pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
        motor_tronco.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
      motor_tronco.write(90);
      delay (3000);      
      sendMsg (MY_ID, MOD_MOTORS, OP_CALIBRATE, NULL, 0);      
      break;    
      
   case OP_GOUP:
     move_motor_up();
     delay(1000);
     stop_motor();   
     sendMsg (MY_ID, MOD_MOTORS, OP_GOUP, NULL, 0);      
     break;
     
   case OP_GODOWN:
     move_motor_down();
     delay(1000);
     stop_motor();   
     sendMsg (MY_ID, MOD_MOTORS, OP_GODOWN, NULL, 0);      
     break;  
     
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           COMMON CODE                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////////

void parse_msg (byte largo) {
  
  byte id = msg_buffer [0];
  byte module = msg_buffer [1];
  byte opcode = msg_buffer [2];
  
  if (id == MY_ID || id == BROADCAST){
    switch (module) {
      case MOD_SYSTEM:
        handleSystem (opcode, &msg_buffer[3], largo-3);
        break;
      case MOD_SENSORS:
        handleSensors (opcode, &msg_buffer[3], largo-3);
        break;   
      case MOD_MOTORS:
        handleMotors (opcode, &msg_buffer[3], largo-3);
        break;    
    }
  }

}






