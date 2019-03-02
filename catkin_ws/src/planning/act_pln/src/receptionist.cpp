#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaAudio.h"
#include "justina_tools/JustinaRepresentation.h"
#include "justina_tools/JustinaTasks.h"
#include "std_msgs/Bool.h"
#include "string"

enum STATES{
    SM_INIT,
    SM_WAIT_FOR_PERSON_ENTRANCE,
    
};


#define SM_INIT 0
#define SM_WAIT_FOR_INIT_COMMAND 10
#define SM_SEARCH_WAVING 20
#define SM_CLOSE_TO_GUEST 30
#define SM_RecognizeGuest 40
#define SM_ReturnSearchWaving 50
#define SM_GoCoatRack 60
#define SM_SearchTaxiDriver 70
#define SM_CLOSE_TO_TAXI_DRIVER 80

int main(int argc, char **argv){


}
