#include <SoftwareSerial.h>
#include "LoRa_E22.h"

SoftwareSerial myserial(10,11); 
LoRa_E22 E22(&myserial);
 
typedef  struct {
  float irtifa, lan, lon, ivme_x, ivme_y, ivme_z, gyro_x, gyro_y, gyro_z;
  int parasut_1, parasut_2;
  //float sicaklik,nem,basinc; sıcaklık nem basınç verileri sadece faydalı yükte
} Signal;
Signal data;
 
void setup() {
  Serial.begin(9600);
  E22.begin();
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(7, LOW);
  digitalWrite(6, LOW);
  Serial.print("asd");
  delay(500);
}
 
void loop() {
  while (E22.available()) {
    ResponseStructContainer rsc = E22.receiveMessage(sizeof(Signal));
    data = *(Signal*) rsc.data;
    Serial.print("irtifa: ");
    Serial.println(data.irtifa);
    Serial.print("lan: ");
    Serial.println(data.lan);
    Serial.print("lon: ");
    Serial.println(data.lon);
    Serial.print("ivme_x: ");
    Serial.println(data.ivme_x);
    Serial.print("ivme_y: ");
    Serial.println(data.ivme_y);
    Serial.print("ivme_z: ");
    Serial.println(data.ivme_z);
    Serial.print("gyro_x: ");
    Serial.println(data.gyro_x);
    Serial.print("gyro_y: ");
    Serial.println(data.gyro_y);
    Serial.print("gyro_z: ");
    Serial.println(data.gyro_z);
    
    Serial.println();
    rsc.close();
  }
}
