/*  AR4 Annin Robot Control Software Arduino Nano sketch
    Copyright (c) 2021, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be
   free of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          you must give appropriate credit and indicate if changes were made.
   You may do so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
          Selling AR2 software, robots, robot parts, or any versions of robots
   or software based on this work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY DIRECT,
   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com

    Log:

*/

#include <Servo.h>

// Firmware version
const char* VERSION = "0.0.1";

String inData;

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;

const int Input2 = 2;
const int Input3 = 3;
const int Input4 = 4;
const int Input5 = 5;
const int Input6 = 6;
const int Input7 = 7;

const int Output8 = 8;
const int Output9 = 9;
const int Output10 = 10;
const int Output11 = 11;
const int Output12 = 12;
const int Output13 = 13;

void setup() {
  // run once:
  Serial.begin(9600);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  pinMode(Input2, INPUT_PULLUP);
  pinMode(Input3, INPUT_PULLUP);
  pinMode(Input4, INPUT_PULLUP);
  pinMode(Input5, INPUT_PULLUP);
  pinMode(Input6, INPUT_PULLUP);
  pinMode(Input7, INPUT_PULLUP);

  pinMode(Output8, OUTPUT);
  pinMode(Output9, OUTPUT);
  pinMode(Output10, OUTPUT);
  pinMode(Output11, OUTPUT);
  pinMode(Output12, OUTPUT);
  pinMode(Output13, OUTPUT);

  servo0.attach(A0);
  servo1.attach(A1);
  servo2.attach(A2);
  servo3.attach(A3);
  servo4.attach(A4);
  servo5.attach(A5);
  servo6.attach(A6);
  servo7.attach(A7);

  // Make servo0 (the servo gripper) go to an arbitrary initial position,
  // otherwise it goes to some unknow position beyond the acceptable range
  servo0.write(30);
}

void loop() {
  // start loop
  while (Serial.available() > 0) {
    char recieved = Serial.read();
    inData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n') {
      String function = inData.substring(0, 2);

      if (function == "ST") {
        Serial.println(String(VERSION));
      }

      //-----COMMAND TO MOVE SERVO-----
      if (function == "SV") {
        int SVstart = inData.indexOf('V');
        int POSstart = inData.indexOf('P');
        int servoNum = inData.substring(SVstart + 1, POSstart).toInt();
        int servoPOS = inData.substring(POSstart + 1).toInt();
        if (servoNum == 0) {
          servo0.write(servoPOS);
        }
        if (servoNum == 1) {
          servo1.write(servoPOS);
        }
        if (servoNum == 2) {
          servo2.write(servoPOS);
        }
        if (servoNum == 3) {
          servo3.write(servoPOS);
        }
        if (servoNum == 4) {
          servo4.write(servoPOS);
        }
        if (servoNum == 5) {
          servo5.write(servoPOS);
        }
        if (servoNum == 6) {
          servo6.write(servoPOS);
        }
        if (servoNum == 7) {
          servo7.write(servoPOS);
        }
        Serial.println("Done");
      }

      //-----COMMAND TO READ SERVO POSITION-----
      if (function == "SP") {
        int SPstart = inData.indexOf('P');
        int servoNum = inData.substring(SPstart + 1).toInt();
        if (servoNum == 0) {
          Serial.println(servo0.read());
        }
        if (servoNum == 1) {
          Serial.println(servo1.read());
        }
        if (servoNum == 2) {
          Serial.println(servo2.read());
        }
        if (servoNum == 3) {
          Serial.println(servo3.read());
        }
        if (servoNum == 4) {
          Serial.println(servo4.read());
        }
        if (servoNum == 5) {
          Serial.println(servo5.read());
        }
        if (servoNum == 6) {
          Serial.println(servo6.read());
        }
        if (servoNum == 7) {
          Serial.println(servo7.read());
        }
      }

      //-----COMMAND IF INPUT THEN JUMP-----
      if (function == "JF") {
        int IJstart = inData.indexOf('X');
        int IJTabstart = inData.indexOf('T');
        int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
        if (digitalRead(IJInputNum) == HIGH) {
          Serial.println("T");
        }
        if (digitalRead(IJInputNum) == LOW) {
          Serial.println("F");
        }
      }

      //-----COMMAND SET OUTPUT ON-----
      if (function == "ON") {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, HIGH);
        Serial.println("Done");
      }
      //-----COMMAND SET OUTPUT OFF-----
      if (function == "OF") {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, LOW);
        Serial.println("Done");
      }
      //-----COMMAND TO WAIT INPUT ON-----
      if (function == "WI") {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();
        while (digitalRead(InputNum) == LOW) {
          delay(100);
        }
        Serial.println("Done");
      }
      //-----COMMAND TO WAIT INPUT OFF-----
      if (function == "WO") {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();

        // String InputStr =  String("Input" + InputNum);
        // uint8_t Input = atoi(InputStr.c_str ());
        while (digitalRead(InputNum) == HIGH) {
          delay(100);
        }
        Serial.println("Done");
      }
      //-----COMMAND ECHO TEST MESSAGE-----
      if (function == "TM") {
        String echo = inData.substring(2);
        Serial.println(echo);
      }

      inData = "";  // Clear received buffer
    }
  }
}
