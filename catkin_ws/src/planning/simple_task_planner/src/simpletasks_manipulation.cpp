#include "simple_task_planner/simpletasks.h"

bool SimpleTasks::placeObject(std::string &t_placeLocation, int t_armToUse)
{
    int maxAttemptPerTask = 3;
    int navTimeout = 120000;
    std::string armDropPosition("drop");
    std::string armStdByPosition("standby");
    int armTimeout = 10000;

    //move the robot to the location where the object will be droped
    bool navSuccess=false;
    int navAttempts = 0;
    while(!navSuccess && navAttempts < maxAttemptPerTask)
    {
       navAttempts = (!(navSuccess = 
                   m_navTasks.syncGetClose(t_placeLocation, navTimeout))) ? 
               navAttempts : navAttempts+1;
    }
    //if the navigation stage was not successfull, then return a fail state
    if(!navSuccess)
    {
        return false;
    }

    //TODO: Align the robot to the dest drop place (trash bin, furniture, 
    //person, etc)

    bool armDropSuccess = false;
    if(t_armToUse == 0)
    {
        //TODO: Instead of move the arm to a predef position. An arm-pose 
        //vector must be calculated based on the free space located at the item
        //where the object will be placed.
        if((armDropSuccess = m_rightArmTasks->syncArmGoTo(armDropPosition, 
                        armTimeout)))
        {
            //open the gripper and wait
            m_rightArmStatus->setTorqueGrip(0.1);
            //TODO:: command to open the gripper and wait for the task end
            ros::Duration d(2);
            d.sleep();

            //return the arm to the standby position
            m_rightArmTasks->syncArmGoTo(armStdByPosition, armTimeout);
            
            //close the gripper and wait
            m_rightArmStatus->setTorqueGrip(-0.1);
            //TODO:: command to open the gripper and wait for the task end
            d.sleep();
        }
    }
    else
    {
        if((armDropSuccess = m_leftArmTasks->syncArmGoTo(armDropPosition,
                        armTimeout)))
        {
            //open the gripper
            m_leftArmStatus->setTorqueGrip(0.1);
            //TODO:: command to open the gripper and wait for the task end
            ros::Duration d(2);
            d.sleep();

            //return the arm to the standby position
            m_leftArmTasks->syncArmGoTo(armStdByPosition, armTimeout);

            //close the gripper
            m_leftArmStatus->setTorqueGrip(-0.1);
            //TODO:: command to open the gripper and wait for the task end
            d.sleep();
        }
    }

    return armDropSuccess;
}

bool SimpleTasks::dropObject(std::string &t_dropLocation, int t_armToUse)
{
    int maxAttemptPerTask = 3;
    int navTimeout = 120000;
    std::string armDropPosition("drop");
    int armTimeout = 10000;

    //move the robot to the location where the object will be droped
    bool navSuccess=false;
    int navAttempts = 0;
    while(!navSuccess && navAttempts < maxAttemptPerTask)
    {
       navAttempts = (!(navSuccess = 
                   m_navTasks.syncGetClose(t_dropLocation, navTimeout))) ? 
               navAttempts : navAttempts+1;
    }
    //if the navigation stage was not successfull, then return a fail state
    if(!navSuccess)
    {
        return false;
    }

    //TODO: Align the robot to the dest drop place (trash bin, furniture, 
    //person, etc)

    bool armDropSuccess = false;
    if(t_armToUse == 0)
    {
        if((armDropSuccess = m_rightArmTasks->syncArmGoTo(armDropPosition, 
                        armTimeout)))
        {
            //open the gripper and wait
            m_rightArmStatus->setTorqueGrip(0.1);
            //TODO:: command to open the gripper and wait for the task end
            ros::Duration d(2);
            d.sleep();
        }
    }
    else
    {
        if((armDropSuccess = m_leftArmTasks->syncArmGoTo(armDropPosition,
                        armTimeout)))
        {
            //open the gripper
            m_leftArmStatus->setTorqueGrip(0.1);
            //TODO:: command to open the gripper and wait for the task end
            ros::Duration d(2);
            d.sleep();
        }
    }


    return armDropSuccess;
}
