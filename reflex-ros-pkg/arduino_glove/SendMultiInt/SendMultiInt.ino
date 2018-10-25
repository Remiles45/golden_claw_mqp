/*
 * rosserial: Send multiple int to a topic
 */

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>


const int F1_PIN = A0; // Pin connected to voltage divider output
const int F2_PIN = A1; // Pin connected to voltage divider output
const int F3_PIN = A2; // Pin connected to voltage divider output

ros::NodeHandle  nh;

std_msgs::Int16MultiArray int_arr_msg;
ros::Publisher glove_data("glove_data", &int_arr_msg);

int send_value = 0;
int curr_f1_data;
int range;

int maximum = 0;
int minimum = 10000;
  
void setup()
{
  Serial.begin(9600);
  
  nh.initNode();
  int_arr_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  int_arr_msg.layout.dim[0].label = "height";
  int_arr_msg.layout.dim[0].size = 3;
  int_arr_msg.layout.dim[0].stride = 1;
  int_arr_msg.layout.data_offset = 0;
  int_arr_msg.data = (int *)malloc(sizeof(int)*8);
  int_arr_msg.data_length = 3;
  nh.advertise(glove_data);
  pinMode(F1_PIN, INPUT);
  pinMode(F2_PIN, INPUT);
  pinMode(F3_PIN, INPUT);
}
//ref: https://answers.ros.org/question/37185/how-to-initialize-a-uint8multiarray-message/
void loop()
{
  curr_f1_data = analogRead(F1_PIN);
  if (curr_f1_data > maximum){
    maximum = curr_f1_data;
  }
  if (curr_f1_data < minimum){
    minimum = curr_f1_data;
  }
  int_arr_msg.data[0] = curr_f1_data;//analogRead(F1_PIN);
  int_arr_msg.data[1] = maximum - minimum;//analogRead(F2_PIN);
  int_arr_msg.data[2] = 0;//analogRead(F3_PIN);
  glove_data.publish( &int_arr_msg );
  nh.spinOnce();
  delay(50);

//  Serial.println("test");//int(maximum-minimum));
  
}
