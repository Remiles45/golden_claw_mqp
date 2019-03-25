/*The below program is designed to work with the Whole_Glove program and relay messages 
 * received from the transmitting Arduino Nano to the robotic hand via ros packages.
 * 
 */

#include <SPI.h>                        //nRF communication
#include <nRF24L01.h>                   //nRF communication
#include <RF24.h>                       //nRF communication
#include <ros.h>                        //ros library
#include <std_msgs/Int16MultiArray.h>   //ros library
#include <std_msgs/MultiArrayLayout.h>
#include<std_msgs/MultiArrayDimension.h>


RF24 radio(9, 10);                // CE, CSN     Identifying which pins to communicate with
long address = 0x555; //address to receive the data from

//Ros Stuff
ros::NodeHandle nh;
std_msgs::Int16MultiArray glove_status_msg;
ros::Publisher glove_data("glove_data", &glove_status_msg);

bool newData = false;
int ReceivedData[5];  //Received Data

int NumSensors = 5; // Number of Sensors

void setup() {
  
  //Set up baud rate and open up reading pipe to receive message
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

//***Ok, so I'm not entirely sure what's going on here ***



  nh.initNode();
  nh.advertise(glove_data);
  //Set up 
  glove_status_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  glove_status_msg.layout.dim[0].label = "Glove Data";
  glove_status_msg.layout.dim[0].size = NumSensors;     
  glove_status_msg.layout.dim[0].stride = 1;
  glove_status_msg.layout.data_offset = 0;
  glove_status_msg.data = (int *)malloc(sizeof(int)*8);
  glove_status_msg.data_length = NumSensors;

  glove_data.publish(&glove_status_msg );  //Attempt to setup publisher in setup not waiting for loop.
  nh.spinOnce();                      //^^
  
}

void loop() {
  nh.spinOnce();
  getData();
  nh.spinOnce();
  showData();
  delay(100);
}

//Below is a function that checks if there is data avaiable from the recently opened pipe
//and if there is data, it saves the data to int_arr_msg and changes the newData boolean to true.
void getData() {
    if ( radio.available() ) {
      Serial.println("radio is available");
        nh.spinOnce();
        radio.read(&ReceivedData, sizeof(ReceivedData));
        newData = true;
    }
}

//If the newData is true then the function showData runs, displaying each part of the received data 
//array.
void showData() {
    if (newData == true) {
       
       //Publish the Received Data and send ROS package to computer
      
       glove_status_msg.data[0] = ReceivedData[0]; //Thumb
       glove_status_msg.data[1] = ReceivedData[1]; //Index
       glove_status_msg.data[2] = ReceivedData[2]; //Middle
       glove_status_msg.data[3] = ReceivedData[3]; //Index-Middle Capacitor Sensor
       glove_status_msg.data[4] = ReceivedData[4]; //Wool Sensor
       nh.spinOnce();
       glove_data.publish(&glove_status_msg );
      
//       
//        Serial.print("Index: ");
//        Serial.println(ReceivedData[0]);
//        Serial.print("Middle ");
//        Serial.println(ReceivedData[1]);
//        Serial.print("Thumb ");
//        Serial.println(ReceivedData[2]);
//        Serial.print("Cap Sensor");
//        Serial.println(ReceivedData[3]); 
//        Serial.print("Wool Sensor ");
//        Serial.println(ReceivedData[4]);  
//        
        
        newData = false;
        
    }
    
 // The below code may publish to ros that no message was received. Needed testing. For now, commented.
   // std_msgs::String msg;
   // std::stringstream ss;
   // ss << "No message" << count;
   // msg.data = ss.str();
}  
