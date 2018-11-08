#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle  nh;
std_msgs::Int16MultiArray int_arr_msg;
ros::Publisher glove_data("glove_data", &int_arr_msg);

const int F1_PIN = A0; // Pin connected to voltage divider output
const int F2_PIN = A1; // Pin connected to voltage divider output
const int F3_PIN = A2; // Pin connected to voltage divider output

int curr_f1_data;
int curr_f2_data;
int curr_f3_data;


int maximumF1 = 0;
int minimumF1 = 10000;
int f1_data;

int maximumF2 = 0;
int minimumF2 = 10000;
int f2_data;

int maximumF3 = 0;
int minimumF3 = 10000;
int f3_data;

const byte address[6] = "00001";

void setup() {
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

void loop() {
  // put your main code here, to run repeatedly:
{
  curr_f1_data = analogRead(F1_PIN);
  if (curr_f1_data > maximumF1){
    maximumF1 = curr_f1_data;
  }
  if (curr_f1_data < minimumF1){
    minimumF1 = curr_f1_data;
  }
  curr_f2_data = analogRead(F2_PIN);
  if (curr_f2_data > maximumF2){
    maximumF2 = curr_f2_data;
  }
  if (curr_f2_data < minimumF2){
    minimumF2 = curr_f2_data;
  }
  curr_f3_data = analogRead(F3_PIN);
  if (curr_f3_data > maximumF3){
    maximumF3 = curr_f3_data;
  }
  if (curr_f3_data < minimumF3){
    minimumF3 = curr_f3_data;
  }
}

  
  f1_data = map(curr_f1_data,minimumF1, maximumF1,0,200);
  f2_data = map(curr_f2_data,minimumF2, maximumF2,0,200);
  f3_data = map(curr_f3_data,minimumF3, maximumF3,0,200);
  int_arr_msg.data[0] = f1_data;
  int_arr_msg.data[1] = f2_data;
  int_arr_msg.data[2] = f3_data;
  glove_data.publish( &int_arr_msg );
  nh.spinOnce();
  delay(500);
}
