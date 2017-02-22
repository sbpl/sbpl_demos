to run the Demos, you need to run the follow scripts:

On the PR2:
  after 'robot start' and pr2_dashboard, you need to bring up the 2D nav and map packages

  To use with pre-defined map of SBPL:

  Terminal 1) roslaunch pr2_2dnav pr2_2dnav.launch
  Terminal 2) roscd sbpl_demos/data
  Terminal 2) rosrun map_server map_server sbpl_full_lab.yaml
  
  To use kinect sensor:

  Terminal 3) roslaunch sbpl_demos pr2_openni_head.launch

 On the local computer:

 Setup chrony by syncing your computer to alanbase

 Add "server alanbase minpoll 0 maxpoll 5 maxdelay 0.05" to your /etc/chrony/chrony.conf
 restart chrony with "invoke-rc.d chrony restart"

 Check time offset with 
 "ntpdate -q alanbase"

On Local machine:

For now, source with pr2master

1) roslaunch groovy_indigo_moveit_wrapper i2g_pr2_follow_trajectory_wrappers.launch
2) roslaunch pr2_moveit_config bringup_pr2.launch 
3) roslaunch sbpl_demos pr2_indiv.launch
4) To run demos, "rosrun sbpl_demos pr2_simple_grasping_demo"