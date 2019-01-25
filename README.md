# golden_claw_mqp

Instructions to operate hand: 
*reflex_sf.launch is assuming that the hand is plugged into USB0 (first thing you plug into your USB port)

  -Start up Reflex hand: roslaunch reflex reflex_sf.launch 
  
  -Start up GUI:  rqt --standalone rqt_gui_control
  
  -Start up Glove: 
    plug in Rx Arduino to computer
    in new terminal: 
    *Will probably be USB1 if the reflex hand is on USB0
    
    rosrun rosserial_python serial_node.py /dev/ttyUSB1 
