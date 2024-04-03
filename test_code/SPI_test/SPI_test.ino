#include <SPI.h>
//assign SPI pis
int ss = 10;

SPISettings spi_setting(1000000, MSBFIRST, SPI_MODE0);

void setup() {
  pinMode(ss,OUTPUT);
  digitalWrite(ss,HIGH);
  SPI.begin();
}

void loop() {
  SPI.beginTransaction(spi_setting);
  digitalWrite(ss,LOW);
  int encoderData = SPI.transfer(0x00);
  digitalWrite(ss,HIGH);
  SPI.endTransaction();
  Serial.println("Encoder data: " + String(encoderData));
}