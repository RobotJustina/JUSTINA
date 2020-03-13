#include <iostream>
#include <sstream>

#include "ros/ros.h"
//#include "knowledge_msgs/PlanningCmdClips.h"
//#include "knowledge_msgs/StrQueryKDB.h"
#include "knowledge_msgs/AsyncSpeech.h"

//#include "justina_tools/JustinaNavigation.h"
//#include "justina_tools/JustinaKnowledge.h"
//#include "justina_tools/JustinaRepresentation.h"
#include "justina_tools/JustinaHRI.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

using namespace boost::algorithm;

enum STATE{
    SM_INIT,
    SM_EMPTY,
    SM_SEND_SPEECH
};

//struct for queue
typedef struct elemenQueue{
    std::string *dato;
    int *time;
    int *limit_time;
    int *ros_time;
    struct elemenQueue *siguiente;
}elemento;

typedef struct elementUbi{
    elemento *inicio;
    elemento *ultimo;
    int tam;
}Queue;


bool busy = false;
Queue *tas;

void callbackSpeechState(){
    std::cout << "Speech status" << std::endl;

}

void inicializaQ(){
	if((tas = (Queue*)malloc(sizeof(Queue)))==NULL)
        	return;
	tas->inicio = NULL;
	tas->ultimo = NULL;
	tas->tam = 0;
}

void callbackInsertAsyncSpeech(const knowledge_msgs::AsyncSpeech::ConstPtr& msg){

    std::cout << "CALLBACK SET SPEECH" << std::endl;
	
    elemento *newelemento;

	if((newelemento=(elemento*)malloc(sizeof(elemento))) == NULL)
		return;
	if((newelemento->time=(int*)malloc(sizeof(int))) == NULL)
		return;
	if((newelemento->ros_time=(int*)malloc(sizeof(int))) == NULL)
		return;
	if((newelemento->limit_time=(int*)malloc(sizeof(int))) == NULL)
		return;

	newelemento->dato = new std::string(msg->speech);
	newelemento->time[0] = msg->time;
    newelemento->ros_time[0] = msg->ros_time;
    newelemento->limit_time[0] = msg->limit_time;
	newelemento->siguiente = NULL;
	if(tas->ultimo !=NULL)
		tas->ultimo->siguiente = newelemento;
	tas->ultimo = newelemento;
	if(tas->inicio == NULL)
		tas->inicio = newelemento;
	tas->tam++;
    /*if(!spgenbusy){
        std_msgs::String msg;
        msg.data = "finish_spgen";
        pubSpGenBusy.publish(msg);
    }*/

}

void popQ(){
	elemento *sup_elemento;
	if (tas->tam == 0)
		return;

    ros::Time time;
	sup_elemento = tas->inicio;
	tas->inicio = tas->inicio->siguiente;

    time = ros::Time::now();
    //std::cout << "Actual Time: " << time.sec << " Limit time: " << sup_elemento->ros_time[0] + sup_elemento->limit_time[0] << std::endl; 
    if(time.sec < sup_elemento->ros_time[0] + sup_elemento->limit_time[0]){
    	//bbros_bridge::Default_ROS_BB_Bridge srv;
	//srv.request.parameters = sup_elemento->dato[0];
        //boost::this_thread::sleep(boost::posix_time::milliseconds(sup_elemento->time[0]));
	//srv.request.timeout = 10000;
	//cltSpgSay.call(srv);
        
        JustinaHRI::waitAfterSay(sup_elemento->dato[0], 10000, sup_elemento->time[0]);
    }
    else{
        //std::cout << "Out of Time, Not speech: " << sup_elemento->dato[0] << std::endl;
        std_msgs::String msg;
        msg.data = "finish_spgen";
        //pubSpGenBusy.publish(msg);
        busy = false;
    }

	delete sup_elemento->dato;//free(sup_elemento->dato);
	free(sup_elemento);
	if(tas->inicio == NULL)
		tas->ultimo = NULL;
	tas->tam--;
	return;

}

void callbackBusy(const std_msgs::String::ConstPtr& msg){
    busy = (msg->data == "start_spgen") ? true:false;
}

int main(int argc, char ** argv) {
	std::cout << "Supervisor Node" << std::endl;
	ros::init(argc, argv, "supervisor");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	
    ros::Subscriber subBBBusy = nh.subscribe("/SpGenBusy", 1, callbackBusy);
    ros::Subscriber subAsyncSpeech = nh.subscribe("/AsyncSpeech", 1, callbackInsertAsyncSpeech);
	
        JustinaHRI::setNodeHandle(&nh);
   
        int nextState = SM_INIT;
    

	while (ros::ok()) {
        
        switch(nextState)
        {
            case SM_INIT:
                std::cout << "Test" << std::endl;
                busy = false;
                inicializaQ();
                nextState = SM_EMPTY;
                break;
            case SM_EMPTY:
                //std::cout << "Test empty" << std::endl;
                nextState = (busy) ? SM_EMPTY:SM_SEND_SPEECH;
                break;
            case SM_SEND_SPEECH:
                //std::cout << "test for send speech" << std::endl;
                if(tas->tam > 0){
                    //mandar un speech async
                    busy = true;
                    popQ();
                }
                nextState = SM_EMPTY;
                break;
        }  
		rate.sleep();
		ros::spinOnce();
	}
	
	return 1; 
}
