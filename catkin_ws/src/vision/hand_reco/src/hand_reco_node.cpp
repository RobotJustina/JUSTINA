/*#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"*/

#include "std_msgs/Bool.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/VisionObjectList.h"

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ros/ros.h"
#include "pcl_conversions/pcl_conversions.h"
#include "justina_tools/JustinaTools.h"

#define MINAREA 10000
#define MINDISTANCE 40
using namespace std;
using namespace cv;

ros::ServiceClient cltRgbdRobot;
ros::Subscriber subEnableRecognizeTopic;
ros::Publisher pubRecognizedHands;

bool enableHandDetection=false;

void callback_subEnableRecognizeTopic(const std_msgs::Bool::ConstPtr& msg);
void callback_pubRecognizedHands();

int main(int argc, char** argv)
{
	cout << "Initializing..." << endl;

	// Initializing ROS node
	ros::init(argc, argv, "hand_reco_node");
	ros::NodeHandle n;

	subEnableRecognizeTopic = n.subscribe("/vision/hand_reco/enableRecognizeTopic", 1, callback_subEnableRecognizeTopic);

	pubRecognizedHands = n.advertise<vision_msgs::VisionObjectList>("/vision/hand_reco/recognizedHands",1);

	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

	ros::Rate loop(10);
	cout << "Starting ros spin once..." << endl;
	// Principal loop
	char keyStroke = 0;
	while(ros::ok())
	{
		// ROS
		ros::spinOnce();
		loop.sleep();

		if( waitKey(5) == 'q' )
			break;
		callback_pubRecognizedHands();
	}
	destroyAllWindows();
	return 0;
}

void callback_subEnableRecognizeTopic(const std_msgs::Bool::ConstPtr& msg){
	enableHandDetection=msg->data;
}

void callback_pubRecognizedHands(){

	if(!enableHandDetection)
		return;

	vision_msgs::VisionObjectList handList;
	string msg;
	int i,j;
	int cn;
	int defNum;
	int hullcount;
	int con;
	double area;
	double max_area;
	double cDistX,cDistY,cDistZ;
	CvSize tSize;
	IplImage* img;
	IplImage* gray;
	CvSeq* first_contour;
	CvSeq* maxitem;
	CvSeq* it;
	CvSeq* tContour;
	CvSeq* hull;
	CvSeq* defects;
	CvMemStorage* storage;
	CvMemStorage* storage1;
	CvMemStorage* storage2;
	CvPoint point;
	CvPoint fPoint;
	CvPoint* p;
	CvConvexityDefect* defectArray; 
	CvFont font;

	Mat bgrImage;
	Mat xyzCloud;

	point_cloud_manager::GetRgbd srv;
	if(!cltRgbdRobot.call(srv))
	  {
	    cout << "ObjDetector.->Cannot get point cloud" << endl;
	    return;
	  }
	JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, bgrImage, xyzCloud);

	tSize.width=bgrImage.cols;
	tSize.height=bgrImage.rows;

	img = cvCreateImage(tSize,8, 3 );

	img->imageData = (char *) bgrImage.data;
//	imshow("img",bgrImage);

	//Imagen escala de grises 8 bits 
    	gray = cvCreateImage(cvSize(tSize.width/2,tSize.height/2),8, 1 );
	//Loop para dibujo de ventanas

		//Region de interes (mitad de la imagen central)
		cvSetImageROI(img, cvRect(tSize.width/4,tSize.height/4,tSize.width/2,tSize.height/2));
		//cvSetImageROI(img, cvRect(1,1,(tSize.width)-1,(tSize.height)-1));
		//Imagen color a escala de grises
		cvCvtColor(img,gray,CV_BGR2GRAY);
		//cvNamedWindow("Grayscale",CV_WINDOW_AUTOSIZE);
		//cvShowImage("Grayscale",gray);
		//Filtro de suavizado
		cvSmooth(gray,gray,CV_BLUR,(12,12),0);
		//cvNamedWindow("Blur",CV_WINDOW_AUTOSIZE);
		//cvShowImage("Blur",gray);
		//Binarizado
		cvThreshold(gray,gray,0,255,(CV_THRESH_BINARY_INV+CV_THRESH_OTSU));
		//cvNamedWindow( "Bin",CV_WINDOW_AUTOSIZE);
		//cvShowImage( "Bin",gray);
		//--
		storage = cvCreateMemStorage();
		first_contour = NULL;
		maxitem=NULL;
		//Busqueda en contornos sobre binarizado
		cn=cvFindContours(gray,storage,&first_contour,sizeof(CvContour),CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE,cvPoint(0,0));
		max_area=0.0;
		it=0;
		if(cn>0){
			for(it=first_contour;it!=NULL;it=it->h_next){
				//busca en cada contorno encontrado el area absoluta (0)
			    	area=fabs(cvContourArea(it,CV_WHOLE_SEQ,0));
			    	if(area>max_area){
			    		max_area=area;
			    		maxitem=it;
			    	}
			}
			//Si el area del contorno es la "maxima"
			if(max_area>MINAREA){
				storage1 = cvCreateMemStorage();
				storage2 = cvCreateMemStorage(0);
				//secuencia de puntos
				tContour = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvContour),sizeof(CvPoint), storage1);
				//Almacena los puntos del contorno
				for(i=0;i<(maxitem->total);i++){
					p = CV_GET_SEQ_ELEM( CvPoint, maxitem, i );
            				fPoint.x = p->x;
					fPoint.y = p->y;
					cvSeqPush( tContour, &fPoint );
				}
				//Cierre convexo sobre la secuencia, debido al 2o argumento
				//en 0, la salida va a hull
				hull = cvConvexHull2( tContour, 0, CV_CLOCKWISE, 0 );
        			hullcount = hull->total;
				//Los indices que no generan cierre sobre el area almacenado en storage2
        			defects= cvConvexityDefects(tContour,hull,storage2  );
				//Cuenta sobre los defectos, se marcan con una linea
        			for(i=1;i<=hullcount;i++){
           				point = **CV_GET_SEQ_ELEM( CvPoint*, hull, i );
            				//cvLine( img, fPoint, point, CV_RGB( 255, 0, 0 ), 1, CV_AA, 0 );
            				fPoint = point;
        			}
				//mientras sigan existiendo defectos se comprueban
				for(;defects;defects = defects->h_next){  
					defNum = defects->total;
					if(defNum == 0)  
						continue;  
					defectArray = (CvConvexityDefect*)malloc(sizeof(CvConvexityDefect)*defNum);  
					//Se copia la secuencia en un arreglo operable
					cvCvtSeqToArray(defects,defectArray, CV_WHOLE_SEQ); 
					con=0;
					cDistX=0;
					cDistY=0;
					for(i=0;i<defNum;i++)  {
						if(defectArray[i].depth>MINDISTANCE){
							con=con+1;
							/*cDistX=cDistX+((defectArray[i].end->x)-(defectArray[i].start->x))/2;
							cDistY=cDistX+((defectArray[i].end->y)-(defectArray[i].start->y))/2;*/
							cDistX=(cDistX+defectArray[i].start->x)/2;
							cDistY=(cDistY+defectArray[i].start->y)/2;
							//cout << "Distance from center to defect " << i << ": " << defectArray[i].depth << endl;
							cvLine(img,*(defectArray[i].start), *(defectArray[i].depth_point),CV_RGB(255,255,0),1, CV_AA, 0 );  
							cvCircle(img,*(defectArray[i].depth_point), 5, CV_RGB(0,0,255),0, 8,0);  
							cvCircle(img,*(defectArray[i].start), 5, CV_RGB(0,255,0), 0, 8,0); 
							cvLine(img,*(defectArray[i].depth_point), *(defectArray[i].end),CV_RGB(0,255,255),1, CV_AA, 0 );  
							cvDrawContours(img,defects,CV_RGB(0,0,0),CV_RGB(255,0,0),-1,CV_FILLED,8);
						}
					}
					//cDist=cDist/con;
					if(con==1)
						msg="2: ";
					else if(con==2)
						msg="3: ";
					else if(con==3)
						msg="4: ";
					else if(con==4)
						msg="5: ";
					else
						msg="0: ";
					if(con>=2 && con<=4){
							CvPoint circle;
							circle.x=cDistX;
							circle.y=cDistY;
							cvCircle(img,circle, 5, CV_RGB(255,0,255),0, 8,0); 
						cDistX=cDistX+tSize.width/2;
						cDistY=cDistY+tSize.height/2;
						//cvCircle(img,cDist, 5, CV_RGB(0,255,0), 0, 8,0); 
						vision_msgs::VisionObject hando;
						std::stringstream sop;
						sop << "hand_" << j;
						hando.id = sop.str();
						hando.pose.position.x = cDistX;
						hando.pose.position.y = cDistY;
						Point3f pz=xyzCloud.at<Point3f>(cDistX,cDistY);
						cDistZ=pz.z;/*
						if(cDistZ<0.1)
							cDistZ=0;*/
						hando.pose.position.z = cDistZ;
						handList.ObjectList.push_back(hando);
						cout << msg << ": ( " << cDistX << " , " << cDistY << " , " << cDistZ << " )" << endl;
					}
					// Limpia memoria
					cvResetImageROI(img);
					free(defectArray);  
				} 
				//cvReleaseMemStorage(&storage1);
				//cvReleaseMemStorage(&storage2);
			}
		}
		//cvReleaseMemStorage( &storage );
		
		bgrImage = cvarrToMat(img);
		imshow("img",bgrImage);
		//cvNamedWindow("img",CV_WINDOW_AUTOSIZE);
		//cvShowImage("img",img);

	//cvReleaseCapture(&cam);
	//cvDestroyAllWindows();
	pubRecognizedHands.publish( handList );
	
}
