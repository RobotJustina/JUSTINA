import rosgraph
import time
import subprocess

def main():
    
    run = False
    while(True):
        if rosgraph.is_master_online():
            print 'ROS MASTER is Online'
            if not run:
                run = True
                subprocess.call(['roslaunch', 'facenet_ros', 'facenet.launch'])
        else:
            print 'ROS MASTER is Offline'
            if run:
                run = False
                subprocess.call(['rosnode', 'kill', '/vision/facenet'])
        time.sleep(1)
        
if __name__ == "__main__":
    main()
