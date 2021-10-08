//Emisora de radiocontrol
//Por Francisco Carabaza

#include "EasyTransfer.h"
EasyTransfer ETin, ETout; //create two objects

const int FRAME_TIME_MS = 20; //frame time (20msec = 50Hz)
const int UPDATE_INDICATOR_FRAME_TIME_MS = 100; // time interval in ms for updating display

unsigned long previousFrameTime;
unsigned long updateIndicatorPreviousFrameTime; // time of display last update

unsigned long time;
char NextionData;   // Nextion data received

int16_t LocalBatteryVoltage;
int16_t RobotBatteryVoltage;

unsigned int Joystick_RX_axis; //RX X axis right joystick (aileron)
unsigned int Joystick_RY_axis; //RY Y axis right joystick (elevator)
unsigned int Joystick_LY_axis; //LY Y axis left joystick (gas)
unsigned int Joystick_LX_axis; //LX X axis left joystick (rudder)
unsigned int Joystick_LZ_axis; //LZ Z axis left joystick rotation
unsigned int Joystick_RZ_axis; //RZ Z axis right joystick rotation
unsigned int Joystick_Left_Button; //LB Button on left joystick
unsigned int Joystick_Right_Button; //RB Button on right joystick
unsigned int capture_offsets = 0; //capture offsets mode

unsigned long currentTime;            //frame time variables
unsigned long previousTime;

byte mode = 1;
byte gait = 2;

struct SEND_DATA_STRUCTURE {
  int16_t RX;
  int16_t RY;
  int16_t RZ;
  int16_t LX;
  int16_t LY;
  int16_t LZ;
  int16_t LB;
  int16_t RB;
  int16_t mode;
  int16_t gait;
  int16_t capture_offsets_remote_order;
};

struct RECEIVE_DATA_STRUCTURE {
  int16_t RobotBatteryVoltage;
};

SEND_DATA_STRUCTURE txdata;
RECEIVE_DATA_STRUCTURE rxdata;

void setup() {
  pinMode(52, INPUT_PULLUP); //botón joystick derecho
  pinMode(53, INPUT_PULLUP); //botón joystick izquierdo

  //initialize serial communications:
  Serial.begin(115200); //Puerto para depurar
  Serial1.begin(115200);//Puerto para el módulo bluetooth
  Serial2.begin(9600);  //Puerto para la pantalla Nextion


  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ETin.begin(details(rxdata), &Serial1); //BT module
  ETout.begin(details(txdata), &Serial1);//BT module
}

void loop() {
  ReadInputs();
  Load_easytransfer_variables();
  ETin.receiveData();
  ReadFromNextion();
  UpdateNextionDisplay();
  if ((millis() - previousTime) > FRAME_TIME_MS)
  {
    previousTime = millis();
    ETout.sendData();
    debug_print();
  }
}

void ReadInputs() {
  Joystick_RX_axis = map(analogRead(A1), 0, 1023, 0, 255);
  Joystick_RY_axis = map(analogRead(A2), 0, 1023, 0, 255);
  Joystick_LY_axis = map(analogRead(A3), 0, 1023, 0, 255);
  Joystick_LX_axis = map(analogRead(A4), 0, 1023, 0, 255);
  Joystick_LZ_axis = map(analogRead(A6), 0, 1023, 0, 255);
  Joystick_RZ_axis = map(analogRead(A5), 0, 1023, 0, 255);
  Joystick_Left_Button = digitalRead(52);
  Joystick_Right_Button = digitalRead(53);
}

void Load_easytransfer_variables()
{
  txdata.RX = Joystick_RX_axis;
  txdata.RY = Joystick_RY_axis;
  txdata.RZ = Joystick_RZ_axis;
  txdata.LX = Joystick_LX_axis;
  txdata.LY = Joystick_LY_axis;
  txdata.LZ = Joystick_LZ_axis;
  txdata.LB = Joystick_Left_Button;
  txdata.RB = Joystick_Right_Button;
  txdata.mode = mode;
  txdata.gait = gait;
  txdata.capture_offsets_remote_order = capture_offsets;
}

void UpdateNextionDisplay()
{
  if ((millis() - updateIndicatorPreviousFrameTime) > UPDATE_INDICATOR_FRAME_TIME_MS)
  {
    updateIndicatorPreviousFrameTime = millis();

    Serial2.print("j0.val=");
    Serial2.print(rxdata.RobotBatteryVoltage);
    Serial2.write(255);
    Serial2.write(255);
    Serial2.write(255);

    Serial2.print("t0.txt=\"");
    Serial2.print(rxdata.RobotBatteryVoltage);
    Serial2.print("%\"");
    Serial2.write(255);
    Serial2.write(255);
    Serial2.write(255);

    LocalBatteryVoltage = map(analogRead(A14), 0, 1023, 0, 100);
    
    Serial2.print("j1.val=");
    Serial2.print(LocalBatteryVoltage);
    Serial2.write(255);
    Serial2.write(255);
    Serial2.write(255);

    Serial2.print("t1.txt=\"");
    Serial2.print(LocalBatteryVoltage);
    Serial2.print("%\"");
    Serial2.write(255);
    Serial2.write(255);
    Serial2.write(255);
    
  }
}

void ReadFromNextion()
{
  if (Serial2.available()) {
    NextionData = Serial2.read(); //Get next character from bluetooth
    Serial.println(NextionData);
    if (NextionData == 'W') { // Walk mode
      mode = 1;
    }
    if (NextionData == 'T') { // Translate and rotate mode
      mode = 2;
    }
    if (NextionData == 'R') { // Rotate mode
      mode = 3;
    }
    if (NextionData == 'O') { // One leg mode
      mode = 4;
    }
    if (NextionData == 'A') { // Tripod gait
      gait = 0;
    }
    if (NextionData == 'B') { // Wave gait
      gait = 1;
    }
    if (NextionData == 'C') { // Ripple gait
      gait = 2;
    }
    if (NextionData == 'D') { // 4Pod gait
      gait = 3;
    }
    if (NextionData == 'K') { // Clear offsets
      capture_offsets = 0;
    }
    if (NextionData == 'J') { // Capture offsets
      capture_offsets = 1;
    }
  }
}

void debug_print()
{
  Serial.print (txdata.RX);
  Serial.print ("\t");
  Serial.print (txdata.RY);
  Serial.print ("\t");
  Serial.print (txdata.RZ);
  Serial.print ("\t");
  Serial.print (txdata.LX);
  Serial.print ("\t");
  Serial.print (txdata.LY);
  Serial.print ("\t");
  Serial.print (txdata.LZ);
  Serial.print ("\t");
  Serial.print (txdata.LB);
  Serial.print ("\t");
  Serial.print (txdata.RB);
  Serial.print ("\t");
  Serial.print (txdata.mode);
  Serial.print ("\t");
  Serial.print (txdata.gait);
  Serial.print ("\t");
  Serial.print (txdata.capture_offsets_remote_order);
  Serial.print ("\t");
  Serial.println (rxdata.RobotBatteryVoltage);
}
