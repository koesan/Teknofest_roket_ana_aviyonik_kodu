#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include "LoRa_E22.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "MPU9250.h"
#define SEALEVELPRESSURE_HPA (1013.25)
#include <SD.h>
#include <Servo.h>

// ivme sensörü hata verebilir kütüphaneyi değiştirin veya kütühhane sindeki mpu.h dosyasına açın ve i2c pin yapısını 0x69 dan 0x68 yapın 

//dosya yazma
File myFile;

//delay süresi
int delay_time =10, lora_delay = 20;

//ateşleme 
bool ilk_ates = false, ikinci_ates = false, firlatma = false;

//kalman genel değişkenler
float kalman_new = 0, cov_new = 0, kalman_gain = 0, kalman_calculated = 0;

//KALMAN irtifa
float yukseklik_kalman_old = 0 , yukseklik_cov_old = 0;

//servo 
Servo servopin1, servopin2;

// ivme sensörü eksik
MPU9250 IMU(Wire, 0x68);
float roll, accelScale = 9.81 / 16384.0, gyroScale = 1.0 / 131.0, rad2deg = 180.0 / PI;

//basınç sensörü
Adafruit_BME280 bme;
float ilk_yukseklik = 0, yukseklik = 0;

//gps sensörü
SoftwareSerial portgps(14, 15);
TinyGPSPlus gps;

//lora sensörü
SoftwareSerial portlora(14, 15);
LoRa_E22 e22 (&portlora);
typedef  struct {
  float irtifa, gps_irtifa, lan, lon, ivme_x, ivme_y, ivme_z, gyro_x, gyro_y, gyro_z,aci;
  int parasut_1 =0 , parasut_2 = 0; 
  //float sicaklik,nem,basinc; sıcaklık nem basınç verileri sadece faydalı yükte
} Signal;
Signal data;

void setup(){
  
  Serial.begin(9600);
  Wire.begin();
 
  //buzzer
  pinMode(7,OUTPUT);
  delay(500);
  
  //servo
  servopin1.attach(1);
  servopin1.attach(2);
  
  //lora 
  e22.begin();
  delay(500);

  //basınç sensörü
  bme.begin(); 
  if(!bme.begin()){
    Serial.println("bme280 calismadi!");
  }
  else{
    ilk_yukseklik = bme.readAltitude(1013.25);
  }
  delay(500);
  
  //gps
  portgps.begin(9600);
  delay(500);

  //ivme
  IMU.begin();
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);

  delay(500);

  //sd kart
  /*
  SD.begin(4)
  if (!SD.begin(4)) {
    Serial.println("sd kart hatası!");
    sensorler = false;
  }
 */

}
void loop(){
  irtifa();
  ivme();
  //kurtarma algoritmasını çalıştırır
  if(firlatma == false){

    if(yukseklik > 100){
      firlatma = true;
    }
  }
  //roket yerden 100m yükseldiğinde kurtarma algoritması çalışır
  if(firlatma == true){  
    
    if(ilk_ates == false){
      
      //sürüklenme paraşüt açma fonk
      if(yukseklik >= 3000 ){// açı verilerinide ekle 
        tone(7,3000);
        delay(2000);
        noTone(7);
        servopin1.write(180);
        ilk_ates = true;
        data.parasut_1 = 1;
      }
    }

    if(ilk_ates == true){
      if(yukseklik <= 550){
        tone(7,3000);
        delay(2000);
        noTone(7);
        servopin2.write(180);
        ikinci_ates = true;
        data.parasut_2 = 1;
      }
    }
  }
  // verileri yer istasyonuna iletir
  veri_ilet();
  
}

//gps delay fonksiyonu
static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do{
    while (portgps.available())
      gps.encode(portgps.read());
  } while (millis() - start < ms);
}

//basınç sensöründen yükseklik verisi alınır
float irtifa(){
  yukseklik = bme.readAltitude(1013.25);
  yukseklik = yukseklik - ilk_yukseklik;
  yukseklik = kalman_filter(yukseklik);
  data.irtifa = yukseklik;
  //data.nem = bme.readHumidity(); // faydali yük
  //data.sicaklik = bme.readTemperature(); // faydali yük
  //data.basinc = bme.readPressure() / 100.0F; // faydali yük
}

//ivme sensöründen gereken veriler alınır
float ivme(){
  IMU.readSensor();
  data.ivme_x = IMU.getAccelX_mss();
  data.ivme_y = IMU.getAccelY_mss();
  data.ivme_z = IMU.getAccelZ_mss();
  data.gyro_x = IMU.getGyroX_rads();
  data.gyro_y = IMU.getGyroY_rads();
  data.gyro_z = IMU.getGyroZ_rads();
  roll = (atan2(data.ivme_y * accelScale, data.ivme_z * accelScale)) *rad2deg;
  data.aci = roll;
}

void veri_ilet(){

  //bme280
  irtifa();
  delay(delay_time);

  //gps
  portgps.listen();
  data.gps_irtifa = gps.altitude.meters();
  data.lan = gps.location.lat();
  data.lon = gps.location.lng();
  smartDelay(delay_time);

  //ivme 
  ivme();
  delay(delay_time);
 
  //lora veri ilet
  portlora.listen();
  ResponseStatus rs = e22.sendFixedMessage(0, 1, 10, &data, sizeof(Signal));
  delay(lora_delay);

  /*
  //sd kart veri yazma
  myFile = SD.open("veri.txt", FILE_WRITE);
  if (myFile) {
    int a = 10;
    // &data daki verileri kaydedecek mi?
    myFile.println(a);
    // bunu kadldır daha sonra
     myFile.close();
  }
  delay(delay_time);
  */
}

float kalman_filter(float input){
    
  kalman_new = yukseklik_kalman_old; 
  cov_new = yukseklik_cov_old + 0.50;
  kalman_gain = cov_new / (cov_new + 0.9);
  
  kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new));
  
  cov_new = (1 - kalman_gain) * yukseklik_cov_old;
  yukseklik_cov_old = cov_new;
  
  yukseklik_kalman_old = kalman_calculated;

  return kalman_calculated;
}
