#include "justina_tools/JustinaAudio.h"


bool JustinaAudio::is_node_set = false;
ros::Subscriber JustinaAudio::subAngle;
ros::Subscriber JustinaAudio::subFlag;
ros::Publisher JustinaAudio::pubStart;
ros::ServiceClient JustinaAudio::servAng;
float ArrAngle[1] = {0};
bool ProcessTerminate = false;



bool JustinaAudio::setNodeHandle(ros::NodeHandle* nh)
{	
    if(JustinaAudio::is_node_set)
        return true;
    if(nh == 0)
        return false;


    subAngle = nh->subscribe("/audio_source/angles", 1, &JustinaAudio::arrayCallback);
    subFlag = nh->subscribe("/audio_source/flag", 1, &JustinaAudio::flagCallback);
    pubStart = nh->advertise<std_msgs::Bool>("/audio_source/start", 1);
    servAng = nh->serviceClient<audio_msgs::srvAngle>("audio_source/srv_Angle");

}

bool JustinaAudio::isProcessTerminate(){
	return ProcessTerminate;
}

float JustinaAudio::getAudioSource()
{
	//std::thread t1(task1);
	//t1.join();
	audio_msgs::srvAngle arg;
	servAng.call(arg);
	float angulo =  arg.response.Angle;
	return angulo;
}

// void JustinaAudio::task1()
// {
//     while (ProcessTerminate == false){
// 		sleep(1);
// 	}
// }


bool JustinaAudio::startSimpleAudioSource()
{
	ProcessTerminate = false;

	std_msgs::Bool msg;
    msg.data = true;
	pubStart.publish(msg);

	return true;
}



void JustinaAudio::arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	
	std::cout << "XXX  Regreso de angulo de python  XXX" << std::endl;
	ProcessTerminate = true;
	int i = 0;
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
		ArrAngle[i] = *it;
		i++;
	}
	
	return;
}



void JustinaAudio::flagCallback(const std_msgs::Bool::ConstPtr& flag)
{
	
	//ProcessTerminate = flag->data;

	return;
}
