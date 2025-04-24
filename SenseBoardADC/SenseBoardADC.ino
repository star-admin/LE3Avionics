#include <ADS126X.h>
#include <SPI.h>

ADS126X adc; // start the class

#define DRDY_PIN 7  // brown 15
// #define AINCOM 3
// #define AVSS 5
#define DOUT 41 // purple 14
#define DIN 5 //yellow 7
#define SCLK 13 // green 13
#define CS 40 // blue 6
int i = 0; // ADS126X pin AIN0, for positive input
int pos_pin = 0; // ADS126X pin AIN0, for positive input
//int pos_pin = ADS126X_AIN0;

int neg_pin = 0xA; // ADS126X pin AIN1, for negative input
//int neg_pin = ADS126X_AIN1;

unsigned long totalReadTime = 0;
unsigned long minReadTime = ULONG_MAX;
unsigned long maxReadTime = 0;
int readCount = 0;
unsigned long readTime = 0;
unsigned long start_time = 0;


void setup() {
  Serial.begin(115200);
  Serial.println("Here:");
  SPI.begin(SCLK, DOUT,DIN,CS);
 
  adc.begin(CS); // setup with chip select pin
  adc.startADC1(); // start conversion on ADC1
  adc.setRate(ADS126X_RATE_38400);


  pinMode(DRDY_PIN, INPUT);
 
  Serial.println("Reading Voltages:");
}


void loop() {

  
  // REAL FOR LOOP
  for(int i=0; i<10; i++) {

    totalReadTime += readTime;
    readCount++;
    start_time = micros();
    unsigned long readTime = 0;
    
    long volt_reader = adc.readADC1(i, neg_pin); // read the voltage
    float voltage = (float)(volt_reader) * 3.3 / 8388608.0; // 2^23 = 8388608

    while (digitalRead(DRDY_PIN) == HIGH) {
      // wait
    }
    delayMicroseconds(150); 
    //if drdy doesnt work, use this and comment out drdy


    
    volt_reader = adc.readADC1(i, neg_pin); // read the voltage
    Serial.print(i);
    Serial.print(": ");
    Serial.println(volt_reader); // send voltage through serial
    voltage = (float)(volt_reader) * 3.3 / 8388608.0; // 2^23 = 8388608
    Serial.print(i);
    Serial.print(": ");
    Serial.print(voltage, 6);
    Serial.println(" V");

    while (digitalRead(DRDY_PIN) == HIGH) {
        // wait
      }

    delayMicroseconds(150);
    //if drdy doesnt work, use this and comment out drdy

    readTime = micros() - start_time;
    Serial.println("\nTiming statistics:");
    Serial.print("Current read time: ");
    Serial.print(readTime);
    Serial.println(" µs");

    // Calculate effective sampling rate
    float samplesPerSecond = 10000000.0 / readTime;
    Serial.print("Effective sampling rate: ");
    Serial.print(samplesPerSecond, 2);
    Serial.println(" samples per second");

    Serial.print("Total samples read: ");
    Serial.println(readCount * 10);
  
  
    
  }

}