#include "ros/ros.h"

#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaRepresentation.h"


#include <vector>
#include <ctime>
#include <map>


enum STATE{
    SM_Initial,
    SM_RecognizeFace,
    SM_RecognizeGesture,
    SM_Final
};

vision_msgs::VisionFaceObjects recognizeFaces (float timeOut, bool &recognized)
{
    recognized = false;
    int previousSize = 20;
    int sameValue = 0;
    boost::posix_time::ptime curr;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff;
    vision_msgs::VisionFaceObjects lastRecognizedFaces;

    do
    {
        lastRecognizedFaces = JustinaVision::getFaces();
        
        if(previousSize == 1)
            sameValue ++;
        
        if (sameValue == 3)
            recognized = true;

        else
        {
            previousSize = lastRecognizedFaces.recog_faces.size();
            recognized = false;
        }

        curr = boost::posix_time::second_clock::local_time();
        ros::spinOnce();
    }while(ros::ok() && (curr - prev).total_milliseconds()< timeOut && !recognized);

    std::cout << "recognized:" << recognized << std::endl;
    return lastRecognizedFaces;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "take_bag_from_hand_test");
    ros::NodeHandle n;
    
    STATE nextState = SM_Initial;
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
    bool success = false;
    bool leftArm = false;
    ros::Rate rate(10);
    std::string lastRecoSpeech;
    std::string stopRecog = "stop follow me";
    std::vector<std::string> validCommandsStop;
    validCommandsStop.push_back(stopRecog);
    vision_msgs::VisionFaceObjects faces;
    bool recog =false;
    int contChances=0;
    

    while(ros::ok() && !success){

        switch(nextState){
            case SM_Initial:
                std::cout << "State machine: SM_Initial" << std::endl;
                JustinaManip::startHdGoTo(0.0, 0.0);
                JustinaHRI::waitAfterSay("Please put in front of me to see your face", 3000);
                ros::Duration(4.0).sleep();
                nextState = SM_RecognizeFace;
            break;

            case SM_RecognizeFace:
                std::cout << "State machine: SM_RecognizeFace" << std::endl;
                while(!recog && contChances < 3)
                {
                    faces = recognizeFaces (10000, recog);
                    JustinaVision::startFaceRecognition(false);
                    contChances++;
                }

                if(faces.recog_faces.size()==0)
                {
                    JustinaHRI::say("Sorry, I cannot see anybody in front of me");
                    ros::Duration(1.5).sleep();
                    nextState = SM_Final;
                    break;
                }
                else{
                    JustinaManip::startHdGoTo(0.0, -0.4);
                    JustinaHRI::say("Ready, now wait for the next instruction");
                    ros::Duration(2.0).sleep();
                    nextState =SM_RecognizeGesture;
                }
            break;

            case SM_RecognizeGesture:
                std::cout << "State machine: SM_RecognizeGesture" << std::endl;
                if(JustinaTasks::graspBagHand(faces.recog_faces[0].face_centroid, leftArm))
                    std::cout << "test succesfully" << std::endl;
                else
                {
                    JustinaHRI::say("sorry i can not see your hand");
                    ros::Duration(1.0).sleep();
                }
                nextState=SM_Final;
            break;

            case SM_Final:
                std::cout << "State machine: SM_RecognizeGesture" << std::endl;
                JustinaHRI::say("i have finished the test");
                ros::Duration(1.0).sleep();
                success=true;
            break;
        }
        rate.sleep();
        ros::spinOnce();
    }
    return success;
}
