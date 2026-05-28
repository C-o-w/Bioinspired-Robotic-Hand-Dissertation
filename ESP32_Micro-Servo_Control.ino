#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>  //ESP32 bluetooth library https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial/examples

//Bluetooth Setup debug text from the library
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pca9685(0x40);

// Safer calibrated range for SG90 servos
#define SERVOMIN 150
#define SERVOMAX 500

// Limit motion range to avoid mechanical strain
#define ANGLE_MIN 20
#define ANGLE_MAX 160

//Command Setup
String InputCommand;                //Full command inputted
int CommandIndexA;                  //Position of first colon
int CommandIndexB;                  //Position of second colon
String CurrentCommand;              //First section of the inputted string (before the first colon)
String CommandInputA;               //Second section of the inputted string (between the colons)
String CommandInputB;               //Third section of the inputted strong (after the last colon)
float CommandIntA;                  //Used if the first input is an int
float CommandIntB;                  //Used if the second input is an int
String OutputString = ("INITIAL");  //Used for data output

//Variables to declare which outputs to use. Bluetooth much cleaner than serial due to problems with ledc
int BluetoothEnabled = 1;
int SerialEnabled = 1;
int DebugEnabled = 1;
int VerboseEnabled = 1;

int DebugInt;    //For passing ints to debug when needed
int DebugFloat;  //For passing floats to debug when needed

const int servoPins[] = {
  0, 1, 2, 3, 4,
  5, 6, 7, 8, 9,
  10, 11, 12, 13, 14
};

int servoCurrentAngles[] = {
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0
};

const int numServos = sizeof(servoPins) / sizeof(servoPins[0]);

String commandQueue[30];
int queueStart = 0;
int queueEnd = 0;

bool queueEmpty() {
  return queueStart == queueEnd;
}

bool queueFull() {
  return ((queueEnd + 1) % 30) == queueStart;
}

void enqueueCommand(String cmd) {

  if (!queueFull()) {

    commandQueue[queueEnd] = cmd;

    queueEnd = (queueEnd + 1) % 30;
  }
}

String dequeueCommand() {

  if (!queueEmpty()) {

    String cmd = commandQueue[queueStart];

    queueStart = (queueStart + 1) % 30;

    return cmd;
  }

  return "";
}


void setup() {

  Serial.begin(115200);

  Wire.begin();  // ESP32 I2C

  pca9685.begin();
  pca9685.setPWMFreq(50);

  SerialBT.begin("Fingers");  //Bluetooth device name

  delay(1000);

  PrintMessageLn("Servo system ready");
}


void PrintMessage(String Message) {  //Used to send outputs over both serial and IR, used to make other sections cleaner
  if (SerialEnabled == 1) {
    Serial.print(Message);
  }
  if (BluetoothEnabled == 1) {  //Doesnt print bluetooth message, only adds to buffer. Will be printed once the PrintMessageLn() command is used. Is due to library problems
    uint8_t buf[Message.length()];
    memcpy(buf, Message.c_str(), Message.length());
    SerialBT.write(buf, Message.length());
  }
}

void PrintMessageLn(String MessageLn) {  //Used to send outputs over both serial and IR, used to make other sections cleaner
  if (SerialEnabled == 1) {
    Serial.println(MessageLn);
  }
  if (BluetoothEnabled == 1) {  //Assembles the message, writes it to buffer then sends buffer
    uint8_t buf[MessageLn.length()];
    memcpy(buf, MessageLn.c_str(), MessageLn.length());
    SerialBT.write(buf, MessageLn.length());
    SerialBT.println();
  }
}

void Debug(String Message, int Verbose) {  //Used to send outputs over both serial and IR, used to make other sections cleaner, separated from PrintMessageLn to minimise interaction with debug
  if (DebugEnabled == 1) {
    if (Verbose == 1) {
      if (VerboseEnabled == 1) {
        if (SerialEnabled == 1) {
          Serial.print(Message);
        }
        if (BluetoothEnabled == 1) {  //Assembles the message, writes it to buffer then sends buffer
          uint8_t buf[Message.length()];
          memcpy(buf, Message.c_str(), Message.length());
          SerialBT.write(buf, Message.length());
        }
      }
    } else {
      if (SerialEnabled == 1) {
        Serial.print(Message);
      }
      if (BluetoothEnabled == 1) {  //Doesnt print bluetooth message, only adds to buffer. Will be printed once the PrintMessageLn() command is used. Is due to library problems
        uint8_t buf[Message.length()];
        memcpy(buf, Message.c_str(), Message.length());
        SerialBT.write(buf, Message.length());
      }
    }
  }
}

void DebugLn(String MessageLn, int Verbose) {  //Used to send outputs over both serial and IR, used to make other sections cleaner, separated from PrintMessageLn to minimise interaction with debug
  if (DebugEnabled == 1) {
    if (Verbose == 1) {
      if (VerboseEnabled == 1) {
        if (SerialEnabled == 1) {
          PrintMessageLn(MessageLn);
        }
        if (BluetoothEnabled == 1) {  //Assembles the message, writes it to buffer then sends buffer
          uint8_t buf[MessageLn.length()];
          memcpy(buf, MessageLn.c_str(), MessageLn.length());
          SerialBT.write(buf, MessageLn.length());
          SerialBT.println();
        }
      }
    } else {
      if (SerialEnabled == 1) {
        PrintMessageLn(MessageLn);
      }
      if (BluetoothEnabled == 1) {  //Assembles the message, writes it to buffer then sends buffer
        uint8_t buf[MessageLn.length()];
        memcpy(buf, MessageLn.c_str(), MessageLn.length());
        SerialBT.write(buf, MessageLn.length());
        SerialBT.println();
      }
    }
  }
}

//CHECKCOMMAND AND READCOMMAND ARE COMPLETELY UNNECESSARY TO THE FUNCTIONALITY OF THE DEVICE, PURELY DEBUG
void CheckCommand() {

  while (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');

    cmd.trim();

    if (cmd.length() > 0) {

      enqueueCommand(cmd);
    }
  }

  while (SerialBT.available()) {

    String cmd = SerialBT.readStringUntil('\n');

    cmd.trim();

    if (cmd.length() > 0) {

      enqueueCommand(cmd);
    }
  }
}

void ReadCommand() {  //Detects the markers in the code and separates the sections, then decides which function to trigger.
  PrintMessageLn(InputCommand);
  CommandIndexA = InputCommand.indexOf(":");  //Error with detecting commands without any colons at the end
  if (CommandIndexA > -1) {
    CommandIndexB = InputCommand.indexOf(":", (CommandIndexA + 1));
    CurrentCommand = InputCommand.substring(0, CommandIndexA);
  } else {
    CurrentCommand = String(InputCommand);
    CommandIndexB = -1;
  }
  if (CommandIndexA > -1) {  //
    if (CommandIndexB > -1) {
      CommandInputA = InputCommand.substring((CommandIndexA + 1), (CommandIndexB));
      CommandInputB = InputCommand.substring((CommandIndexB + 1));
    } else {
      CommandInputA = InputCommand.substring((CommandIndexA + 1));
      CommandInputB = "0";
    }
  } else {
    CommandInputA = InputCommand.substring((CommandIndexA + 1));
    CommandInputB = "0";
  }
  DebugLn("Current Command:", 0);
  DebugLn(String(CurrentCommand), 0);
  DebugLn("Command Inputs:", 0);
  DebugLn(String(CommandInputA), 0);
  DebugLn(String(CommandInputB), 0);                     //Debug output
  if (CurrentCommand == "1" || CurrentCommand == "1") {  //Code to decide which command has been inputted
    finger1(ANGLE_MIN, ANGLE_MAX);
    delay(200);
    finger1(ANGLE_MAX, ANGLE_MIN);
  } else if (CurrentCommand == "2" || CurrentCommand == "2") {  //Code to decide which command has been inputted
    finger2(ANGLE_MIN, ANGLE_MAX);
    delay(200);
    finger2(ANGLE_MAX, ANGLE_MIN);
  } else if (CurrentCommand == "3" || CurrentCommand == "3") {  //Code to decide which command has been inputted
    finger3(ANGLE_MIN, ANGLE_MAX);
    delay(200);
    finger3(ANGLE_MAX, ANGLE_MIN);
  } else if (CurrentCommand == "4" || CurrentCommand == "4") {  //Code to decide which command has been inputted
    finger4(ANGLE_MIN, ANGLE_MAX);
    delay(200);
    finger4(ANGLE_MAX, ANGLE_MIN);
  } else if (CurrentCommand == "5" || CurrentCommand == "5") {  //Code to decide which command has been inputted
    finger5(ANGLE_MIN, ANGLE_MAX);
    delay(200);
    finger5(ANGLE_MAX, ANGLE_MIN);
  } else if (CurrentCommand == "Clench" || CurrentCommand == "clench") {  //Code to decide which command has been inputted
    clench();
  } else if (CurrentCommand == "Unclench" || CurrentCommand == "unclench") {  //Code to decide which command has been inputted
    unclench();
  } else if (CurrentCommand == "ListAngles" || CurrentCommand == "listangles") {  //Code to decide which command has been inputted
    listangles();
  } else if (CurrentCommand == "SERVO" || CurrentCommand == "servo") {

    int servoNum = CommandInputA.toInt();
    int angle = CommandInputB.toInt();

    angle = constrain(angle, ANGLE_MIN, ANGLE_MAX);

    if (servoNum >= 0 && servoNum < numServos) {

      sweepServo(
        servoPins[servoNum],
        servoCurrentAngles[servoNum],
        angle);

      PrintMessageLn(
        "Servo " + String(servoNum) + " swept to " + String(angle));
    }
  } else {
    PrintMessageLn("Unknown Command, did you remember the colons?");
    delay(1000);
  }
}
void setServoAngle(int pin, int angle) {

  int pwm = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  pca9685.setPWM(pin, 0, pwm);
}

void sweepServo(int pin, int startAngle, int endAngle) {

  if (servoCurrentAngles[pin] < endAngle) {

    for (int a = servoCurrentAngles[pin]; a <= endAngle; a++) {
      setServoAngle(pin, a);
      delay(3);
    }
    servoCurrentAngles[pin] = endAngle;
    PrintMessage("Motor ");
    PrintMessage(String(pin));
    PrintMessage(" is at angle: ");
    PrintMessageLn(String(servoCurrentAngles[pin]));

  } else {

    for (int a = servoCurrentAngles[pin]; a >= endAngle; a--) {
      setServoAngle(pin, a);
      delay(3);
    }
    servoCurrentAngles[pin] = endAngle;
    PrintMessage("Motor ");
    PrintMessage(String(pin));
    PrintMessage(" is at angle: ");
    PrintMessageLn(String(servoCurrentAngles[pin]));
  }
}

void loop() {

  CheckCommand();

  if (!queueEmpty()) {

    InputCommand = dequeueCommand();

    CommandIndexA = InputCommand.indexOf(":");

    if (CommandIndexA == -1) {

      InputCommand = InputCommand + ":000:";
    }

    ReadCommand();
  }
}

void finger1(int START, int END) {
  sweepServo(servoPins[0], START, END);
  delay(50);
  sweepServo(servoPins[1], START, END);
  delay(50);
  sweepServo(servoPins[2], START, END);
  delay(50);
}

void finger2(int START, int END) {
  sweepServo(servoPins[3], START, END);
  delay(50);
  sweepServo(servoPins[4], START, END);
  delay(50);
  sweepServo(servoPins[5], START, END);
  delay(50);
}

void finger3(int START, int END) {
  sweepServo(servoPins[6], START, END);
  delay(50);
  sweepServo(servoPins[7], START, END);
  delay(50);
  sweepServo(servoPins[8], START, END);
  delay(50);
}

void finger4(int START, int END) {
  sweepServo(servoPins[9], START, END);
  delay(50);
  sweepServo(servoPins[10], START, END);
  delay(50);
  sweepServo(servoPins[11], START, END);
  delay(50);
}

void finger5(int START, int END) {
  sweepServo(servoPins[12], START, END);
  delay(50);
  sweepServo(servoPins[13], START, END);
  delay(50);
  sweepServo(servoPins[14], START, END);
  delay(50);
}

void clench() {
  finger1(ANGLE_MIN, ANGLE_MAX);
  finger2(ANGLE_MIN, ANGLE_MAX);
  finger3(ANGLE_MIN, ANGLE_MAX);
  finger4(ANGLE_MIN, ANGLE_MAX);
  finger5(ANGLE_MIN, ANGLE_MAX);
}

void unclench() {
  finger1(ANGLE_MAX, ANGLE_MIN);
  finger2(ANGLE_MAX, ANGLE_MIN);
  finger3(ANGLE_MAX, ANGLE_MIN);
  finger4(ANGLE_MAX, ANGLE_MIN);
  finger5(ANGLE_MAX, ANGLE_MIN);
}

void listangles() {
  for (int a = 0; a < 15; a++) {
    PrintMessage("Motor ");
    PrintMessage(String(a));
    PrintMessage(" is at angle: ");
    PrintMessageLn(String(servoCurrentAngles[a]));
    delay(20);
  }
}
