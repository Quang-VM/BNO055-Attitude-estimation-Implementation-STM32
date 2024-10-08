 #include <Arduino.h>
void setup() {
  Serial.begin(115200);
  
  // Define the LED pin as Output (for ESP32, let's use GPIO2)
  
  pinMode(2, OUTPUT);
  Serial.println("ESP32 UART Receiver");
  Serial.println("-----------------------------");
}

void loop() {
  //digitalWrite(2, LOW); 
  // Wait until something is received
  while(!Serial.available());
  //digitalWrite(2, HIGH); 
  // Read the data
  char in_read = Serial.read();
  // Print the data
  Serial.print(in_read);
}