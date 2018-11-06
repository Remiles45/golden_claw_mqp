#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int F1_PIN = A0; // Pin connected to voltage divider output
const int F2_PIN = A1; // Pin connected to voltage divider output
const int F3_PIN = A2; // Pin connected to voltage divider output

int curr_f1_data;
int curr_f2_data;
int curr_f3_data;


int maximumF1 = 0;
int minimumF1 = 10000;
int rangeF1;
int mappedValueF1; 

int maximumF2 = 0;
int minimumF2 = 10000;
int rangeF2;
int mappedValueF2;

int maximumF3 = 0;
int minimumF3 = 10000;
int rangeF3;
int mappedValueF3;

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  


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
  rangeF1 = (maximumF1 - minimumF1);
  rangeF2 = (maximumF2 - minimumF2);
  rangeF3 = (maximumF3 - minimumF3);
  //value between 0 and 1
  mappedValueF1 = rangeF1 / maximumF1;
  mappedValueF2 = rangeF2 / maximumF2;
  mappedValueF3 = rangeF3 / maximumF3;
  curr_f1_data = analogRead(F1_PIN);
  
  const char text[] = "Sup?";
  radio.write(&text, sizeof(text));
  Serial.println(text);
  delay(500);
  
  //Serial.println(curr_f1_data);
  /*
  Serial.print("Maximum: ");
  Serial.println(maximum);
  Serial.print("Minimum: ");
  Serial.println(minimum);
  Serial.print("Range: ");
  Serial.println(range);
   */
  

}
