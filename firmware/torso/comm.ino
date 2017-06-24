
///////////////////////////////////////////////////////////////////////////////////////////////////////
//                                               COMM                                                //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// nota: SYNC y ESCAPE deben ser ambos > 127
// nota: el largo no puede ser == SYNC ni == ESCAPE (todo lo demás SÍ puede ser == SYNC o == ESCAPE)

#define SYNC            254                  // byte de comienzo de los mensajes seriales
#define ESCAPE          253                  // byte de desambiguación en los mensajes seriales
#define BROADCAST       255                  // broadcast

// estados de la máquina de recepción
enum ESTADO { BUSCAR, LEER_LARGO, LEER_MSG, CHECKSUM }; 

byte msg_buffer [256];                 // buffer fijo para todo el sistema de mensajes

void leer_serial () {                  // máquina de estados que recibe y decodifica los mensajes Seriales 
  
  // variables de control
  static byte largo, cont, checksum;
  static byte anterior = 0;    
  static ESTADO estado = BUSCAR;
    
  while (Serial.available() > 0) {                                                     // si hay al menos 1 byte en el Serial...
    byte b = Serial.read();                                                            // ...lo lee
    if (b==SYNC && ((anterior!=ESCAPE && estado==BUSCAR) || anterior<128)) {         // chequea la presencia de un "TRUE SYNC"
      estado = LEER_LARGO; 
    } else {
      switch (estado) {
        case LEER_LARGO:
          largo = b;                                                        // largo no puede ser > 255 
          checksum = 0;                                                     // se prepara para leer el cuerpo del mensaje
          cont = 0;
          estado = LEER_MSG; 
          break;
        case LEER_MSG:
          msg_buffer [cont++] = b;                                          // va llenando el buffer
          checksum += b;                                                    // va efectuando la suma comprobatoria
          if (cont == largo) {                                              // ...hasta que termina el cuerpo del mensaje
            checksum &= 127;                                                // "recorta" el checksum
            estado = CHECKSUM;
          }
          break;
        case CHECKSUM:
          if (b == checksum) {                                              // comprueba el checksum                                    
            largo = unescape (largo);                                       // esta rutina saca los ESCAPEs. El resultado queda en las mismas variables buffer y largo           
            parse_msg (largo);                                              // acción                                        
          } 
          estado = BUSCAR;                                                  // estado inicial
          break;  
      }
    }
    anterior = b;                                                           // se prepara para procesar el siguiente byte
  } 
}


void sendMsg (byte id, byte module, byte opcode, byte *data, byte largo) {      // formatea, encapsula y manda los mensajes al Serial (largo es el largo de la data solamente)
  
  // primera parte: formatea el mensaje
  byte mensaje [largo+3];
  memcpy (mensaje+3, data, largo);
  largo += 3;
  mensaje [0] = id;
  mensaje [1] = module;
  mensaje [2] = opcode;
    
  // segunda parte: escapea  
  byte salida [largo*2];                                         // crea un array suficientemente largo para contener el string de salida
  byte pos = 0;                                                  // posición en el array de salida
  for (byte i=0; i<largo; i++) {                                 // recorre el array de entrada
    if (mensaje[i] != ESCAPE && mensaje[i] != SYNC) {            // si no es un byte especial...
      salida[pos++] = mensaje[i];                                // ...lo mete como viene en el array de salida e incrementa posición
    } else {                                                     // de lo contrario...
      salida[pos++] = ESCAPE;                                    // mete primero un ESCAPE e incrementa
      salida[pos++] = mensaje[i];                                // y luego mete el byte e incrementa
    }
  }   
  
  // tercera parte: encapsula y manda al Serial
  Serial.write (SYNC);                                  // escribe un SYNC
  Serial.write (pos);                                   // escribe el largo
  byte checksum = 0;                                    // inicializa el acumulador de suma comprobatoria
  for (byte i=0; i<pos; i++) {                          // recorre el array de salida
    Serial.write (salida[i]);                           // escribe el byte
    checksum += salida[i];                              // suma
  }
  Serial.write (checksum & 127);                        // escribe la suma comprobatoria recortada

}

byte unescape (byte largo) {                         // saca los ESCAPES y devuelve el mensaje original
  byte i=0, pos=0;                                       // inicializa posición del array de entrada y salida (que son el mismo)
  do {
    if (msg_buffer [i] == ESCAPE) {i++;}                 // si encuentra un ESCAPE, lo saltea
    msg_buffer [pos++] = msg_buffer [i++];               // copia el byte correcto a la nueva posición
  } while (i < largo);
  return pos;                                            // devuelve el nuevo largo
}



