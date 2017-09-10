import paramiko
import time
import threading

def ssh_exec_thread(ssh_object, command):
    stdin, stdout, stderr = ssh_object.exec_command(command, get_pty=True)
    print('test')
    for i in stdout.readlines():
    	print(i),
    for i in stderr.readlines():
    	print(i),

#set up paramiko ssh client
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.MissingHostKeyPolicy())
#set up connection to robot
ssh.connect('10.0.0.21', username='pi', password='raspberry')
#setup scp
scp = ssh.open_sftp() 

#send control file
localFile = 'RobotControl.py'
remoteFile = '/home/pi/catkin_ws/src/robot_control/src/RobotControl.py'
#transfer file
scp.put(localFile, remoteFile) 
scp.put('params.yaml','/home/pi/catkin_ws/src/robot_control/src/params.yaml')
#close scp
scp.close()

#stdin, stdout, stderr = ssh.exec_command('nohup roslaunch robot_launch robot.launch >/dev/null 2>&1 &') 
#time.sleep(10)
#stdin, stdout, stderr = ssh.exec_
#PATH=$PATH:/opt/ros/indigo/bin;
pre_command = """
source ~/.bashrc;
which roslaunch;
PATH=/opt/ros/indigo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games;
export PYTHONPATH='/home/pi/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/indigo/lib/python2.7/dist-packages';
export PKG_CONFIG_PATH='/home/pi/catkin_ws/devel/lib/pkgconfig:/opt/ros/indigo/lib/pkgconfig';
export CMAKE_PREFIX_PATH='/home/pi/catkin_ws/devel:/opt/ros/indigo';
export XDG_RUNTIME_DIR='/run/user/1000';
export ROS_ETC_DIR='/opt/ros/indigo/etc/ros';
export ROS_WORKSPACE='/home/pi/catkin_ws';
export LD_LIBRARY_PATH='/home/pi/catkin_ws/devel/lib:/opt/ros/indigo/lib';
export ROS_ROOT='/opt/ros/indigo/share/ros';
export ROS_PACKAGE_PATH='/home/pi/catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks'
export ROS_MASTER_URI='http://localhost:11311'

env;
"""
command = pre_command + 'roslaunch robot_control robot_control.launch'


thread = threading.Thread(target=ssh_exec_thread, args=(ssh, command))
thread.start()
#    ...do something else...
thread.join()