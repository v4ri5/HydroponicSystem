#include <DallasTemperature.h>
#include <OneWire.h>
#include <U8G2.h>

#define TEMP_PIN 12

unsigned long lastRead = 0;



//One wire object
OneWire oneWire(TEMP_PIN);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  onewire.reset();
  if(oneWire.read() == 0){
    Serial.println("Device present");
  }
}
