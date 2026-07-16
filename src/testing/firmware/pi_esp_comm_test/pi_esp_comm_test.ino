#define LED 2

String incoming = "";

void setup() {
  pinMode(LED, OUTPUT);

  // Hardware UART0
  Serial.begin(115200);

  delay(1000);

  Serial.println("ESP_READY");
}


void loop() {

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {

      incoming.trim();

      if (incoming == "PING") {

        Serial.println("PONG");

      } 
      else if (incoming.startsWith("CMD")) {

        digitalWrite(LED, !digitalRead(LED));

        Serial.print("ACK,");
        Serial.println(incoming);

      }
      else {

        Serial.print("ERR,UNKNOWN,");
        Serial.println(incoming);

      }

      incoming = "";

    } 
    else {
      incoming += c;
    }
  }


  // optional heartbeat
  static unsigned long last = 0;

  if (millis() - last > 5000) {
    Serial.println("ESP_ALIVE");
    last = millis();
  }
}



//haqq 1 - pi to ESP

// #define RXD2 15
// #define TXD2 23

// void setup() {
//   Serial.begin(115200);                          
//   Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);   
//   Serial.println("ESP32 listener ready");
// }

// void loop() {
//   if (Serial2.available()) {
//     String line = Serial2.readStringUntil('\n');
//     line.trim();
//     Serial.println("RX: " + line);
//   }
// }


//haqq 2 - ESP to pi

// #define RXD2 15
// #define TXD2 14

// void setup() {
//   Serial.begin(115200);                           // USB debug
//   Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);   // to Pi
//   Serial.println("ESP32 sender ready");
// }

// int counter = 0;

// void loop() {
//   String msg = "Hello from ESP32 #" + String(counter++);
//   Serial2.println(msg);
//   Serial.println("Sent: " + msg);  // mirror to USB for debugging
//   delay(1000);
// }

// // #define LED 2

// String incoming = "";

// void setup() {
//   pinMode(LED, OUTPUT);

//   // Hardware UART0
//   Serial.begin(115200);

//   delay(1000);

//   Serial.println("ESP_READY");
// }


// void loop() {

//   while (Serial.available()) {
//     char c = Serial.read();

//     if (c == '\n') {

//       incoming.trim();

//       if (incoming == "PING") {

//         Serial.println("PONG");

//       } 
//       else if (incoming.startsWith("CMD")) {

//         digitalWrite(LED, !digitalRead(LED));

//         Serial.print("ACK,");
//         Serial.println(incoming);

//       }
//       else {

//         Serial.print("ERR,UNKNOWN,");
//         Serial.println(incoming);

//       }

//       incoming = "";

//     } 
//     else {
//       incoming += c;
//     }
//   }


//   // optional heartbeat
//   static unsigned long last = 0;

//   if (millis() - last > 5000) {
//     Serial.println("ESP_ALIVE");
//     last = millis();
//   }
// }



// //haqq 1 - pi to ESP

// // #define RXD2 15
// // #define TXD2 23

// // void setup() {
// //   Serial.begin(115200);                          
// //   Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);   
// //   Serial.println("ESP32 listener ready");
// // }

// // void loop() {
// //   if (Serial2.available()) {
// //     String line = Serial2.readStringUntil('\n');
// //     line.trim();
// //     Serial.println("RX: " + line);
// //   }
// // }


// //haqq 2 - ESP to pi

// // #define RXD2 15
// // #define TXD2 14

// // void setup() {
// //   Serial.begin(115200);                           // USB debug
// //   Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);   // to Pi
// //   Serial.println("ESP32 sender ready");
// // }

// // int counter = 0;

// // void loop() {
// //   String msg = "Hello from ESP32 #" + String(counter++);
// //   Serial2.println(msg);
// //   Serial.println("Sent: " + msg);  // mirror to USB for debugging
// //   delay(1000);
// // }

// #define RXD2 16
// #define TXD2 17

// HardwareSerial TestSerial(2);

// String received = "";

// void setup() {
//   Serial.begin(115200);   // USB serial monitor

//   TestSerial.begin(
//     115200,
//     SERIAL_8N1,
//     RXD2,
//     TXD2
//   );

//   delay(1000);

//   Serial.println("ESP32 UART2 Loopback Test");
//   Serial.println("Connect GPIO17 TX2 -> GPIO16 RX2");
// }

// void loop() {

//   // Send test message every second
//   static unsigned long lastSend = 0;

//   if (millis() - lastSend > 1000) {
//     lastSend = millis();

//     String msg = "Hello UART " + String(millis());

//     TestSerial.println(msg);

//     Serial.print("Sent: ");
//     Serial.println(msg);
//   }


//   // Read anything coming back through RX
//   while (TestSerial.available()) {
//     char c = TestSerial.read();

//     if (c == '\n') {
//       Serial.print("Received: ");
//       Serial.println(received);
//       received = "";
//     }
//     else {
//       received += c;
//     }
//   }
// }