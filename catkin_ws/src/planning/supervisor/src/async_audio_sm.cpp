#include <iostream>
#include <sstream>

#include "ros/ros.h"
//#include "knowledge_msgs/PlanningCmdClips.h"
//#include "knowledge_msgs/StrQueryKDB.h"

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
//ros::ServiceClient srvCltQueryKDB;

//ros::Publisher robot_pose_pub;

/*void callbackCmdNavigation(
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << "--------- Supervisor Command Navigation ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	if(msg->name == "goto"){
    		std::map<std::string, std::vector<float> > locations;
        std::vector<std::string> tokens;
        std::string str = msg->params;
        split(tokens, str, is_any_of(" "));

		std::cout << "Location: " << tokens[1] << std::endl;

    		JustinaKnowledge::getKnownLocations(locations);
		if(locations.find(tokens[1]) == locations.end())
		{
			std::cout << "MvnPln.->Cannot get close to \"" << tokens[1] << "\". It is not a known location. " << std::endl;
		}
		sx = locations.find(tokens[1])->second[0];
		sy = locations.find(tokens[1])->second[1];

		std::cout << "Location: " << sx << ", " << sy << std::endl;
	}*/

	/*std_msgs::Bool rp;
	rp.data = 1;

	robot_pose_pub.publish(rp);*/
//}

/*void callbackCmdResponse(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << "------ Supervisor Command Finish -------" << std::endl;

	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;
	std::cout << "successful:" << msg->successful << std::endl;

	float d;
	std::string kdb_loc, sup_loc;
	std::stringstream ss;
    std::vector<std::string> tokens;
    std::string str = msg->params;
    split(tokens, str, is_any_of(" "));

	if(msg->name == "goto"){
		JustinaNavigation::getRobotPose(x, y, theta);
		std::cout << "Robot pose: " << x << ", " << y << ", " << theta << std::endl;
		JustinaNavigation::getRobotPoseFromOdom(x, y, theta);
		std::cout << "Robot pose from Odom: " << x << ", " << y << ", " << theta << std::endl;
		std::cout << "Location: " << sx << ", " << sy << std::endl;

		d = pow(x - sx, 2) + pow(y - sy, 2);

		std::cout << "Distance: " << d << std::endl;

		sup_loc = "null";

		if(d < 0.25)
			sup_loc = tokens[1];
                    
		ss.str("");
                ss << "(assert (get_robot_location 1))";
                JustinaRepresentation::strQueryKDB(ss.str(), kdb_loc, 1000);

		if(kdb_loc == sup_loc){
			std::cout << "Supervisor and Kdb have the same information" << std::endl;
			std::cout << kdb_loc << std::endl;
			std::cout << sup_loc << std::endl;
		}
		else{
			std::cout << "Exist a difference between supervisor and kdb" << std::endl;
			std::cout << kdb_loc << std::endl;
			std::cout << sup_loc << std::endl;
		}
	}
}*/

void inicializaQ(){
	if((tas = (Queue*)malloc(sizeof(Queue)))==NULL)
        	return;
	tas->inicio = NULL;
	tas->ultimo = NULL;
	tas->tam = 0;
}

void callbackPushQ(){

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

	/*newelemento->dato = new std::string(dato);
	newelemento->time[0] = time;
    newelemento->ros_time[0] = ros_time;
    newelemento->limit_time[0] = limit_time;
	newelemento->siguiente = NULL;
	if(tas->ultimo !=NULL)
		tas->ultimo->siguiente = newelemento;
	tas->ultimo = newelemento;
	if(tas->inicio == NULL)
		tas->inicio = newelemento;
	tas->tam++;
    if(!spgenbusy){
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
    /*if(time.sec < sup_elemento->ros_time[0] + sup_elemento->limit_time[0]){
    	bbros_bridge::Default_ROS_BB_Bridge srv;
	srv.request.parameters = sup_elemento->dato[0];
        boost::this_thread::sleep(boost::posix_time::milliseconds(sup_elemento->time[0]));
	srv.request.timeout = 10000;
	cltSpgSay.call(srv);
    }
    else{
        //std::cout << "Out of Time, Not speech: " << sup_elemento->dato[0] << std::endl;
        std_msgs::String msg;
        msg.data = "finish_spgen";
        pubSpGenBusy.publish(msg);
    }*/

	delete sup_elemento->dato;//free(sup_elemento->dato);
	free(sup_elemento);
	if(tas->inicio == NULL)
		tas->ultimo = NULL;
	tas->tam--;
	return;

}

int main(int argc, char ** argv) {
	std::cout << "Supervisor Node" << std::endl;
	ros::init(argc, argv, "supervisor");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	
	//ros::Subscriber subCmdNavigation = nh.subscribe("/planning_clips/cmd_goto", 1, callbackCmdNavigation);
	
	//ros::Subscriber subCmdResponse = nh.subscribe("/planning_clips/command_response", 1, callbackCmdResponse);
	//robot_pose_pub = nh.advertise<std_msgs::Bool>("/supervisor/robot_pose", 1);
	
	//JustinaNavigation::setNodeHandle(&nh);
    //	JustinaKnowledge::setNodeHandle(&nh);
	//JustinaRepresentation::setNodeHandle(&nh);
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
                std::cout << "Test empty" << std::endl;
                nextState = (busy) ? SM_EMPTY:SM_SEND_SPEECH;
                break;
            case SM_SEND_SPEECH:
                std::cout << "test for send speech" << std::endl;
                nextState = SM_EMPTY;
                break;
            
        }  
		rate.sleep();
		ros::spinOnce();
	}
	
	return 1; 
}
