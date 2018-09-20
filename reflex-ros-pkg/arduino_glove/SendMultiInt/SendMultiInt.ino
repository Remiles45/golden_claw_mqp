/*
 * rosserial: Send multiple int to a topic
 */

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>


const int FLEX_PIN = A0; // Pin connected to voltage divider output
const int POTENTIOMETER_PIN = A1; // Pin connected to voltage divider output
const int LIGHT_PIN = A2; // Pin connected to voltage divider output

ros::NodeHandle  nh;

std_msgs::Int16MultiArray int_arr_msg;
ros::Publisher chatter("chatter", &int_arr_msg);

int send_value = 0;

void setup()
{
  nh.initNode();
  int_arr_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  int_arr_msg.layout.dim[0].label = "height";
  int_arr_msg.layout.dim[0].size = 3;
  int_arr_msg.layout.dim[0].stride = 1;
  int_arr_msg.layout.data_offset = 0;
  int_arr_msg.data = (int *)malloc(sizeof(int)*8);
  int_arr_msg.data_length = 3;
  nh.advertise(chatter);
  pinMode(FLEX_PIN, INPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);
  pinMode(LIGHT_PIN, INPUT);
}
//ref: https://answers.ros.org/question/37185/how-to-initialize-a-uint8multiarray-message/
void loop()
{
  int_arr_msg.data[0] = analogRead(FLEX_PIN);
  int_arr_msg.data[1] = analogRead(POTENTIOMETER_PIN);
  int_arr_msg.data[2] = analogRead(LIGHT_PIN);
  chatter.publish( &int_arr_msg );
  nh.spinOnce();
  delay(50);
  
}
