===============================================
Human tracking via OpenNi (for Saphari project)
===============================================


Actually RoboSherlock installation
----------------------------------

Use branch `saphariNew', and follow general instructions `here <overview_rs>`

- Running on leela, i.e. on robot Boxy
- Start the kinect on the shoulder::
   
   roslaunch boxy_bringup openni_shoulder_kinect.launch

- Enable depth_registration::
   
   rosrun rqt_reconfigure rqt_reconfigure

- Start RoboSherlock::
   
   rosrun iai_rs_cpp rs_runAE saphariHumans
