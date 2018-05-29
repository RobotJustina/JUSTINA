
#include "obj_reco_plastic_tray/ControlerPlasticTrayFinder.h"

using namespace cv;
using namespace std;



int main(int argc, char** argv) {


	ros::init(argc, argv, "obj_reco_plastic_tray");
    ros::ServiceClient cltRgbdRobot;
    ros::NodeHandle nodeHandle("~");
    ros::Rate loopRate(30);

    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    
    bool debug;

    
    debug = true;
    ControlerPlasticTrayFinder controler(nodeHandle, debug);


    /*cltRgbdRobot = nodeHandle.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/rgbd_wrt_kinect");

    ros::Rate loopRate(20);
    while (ros::ok()) {
        point_cloud_manager::GetRgbd srv;
        if(cltRgbdRobot.call(srv))
        {
            JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, bgrImg, xyzCloud);

            imshow("Depth", xyzCloud);
            imshow("RGB", bgrImg);

        }
        else
                    std::cout << "Cannot get point cloud" << std::endl;

        ros::spinOnce();
        loopRate.sleep();
    }*/

    return 0;
}
