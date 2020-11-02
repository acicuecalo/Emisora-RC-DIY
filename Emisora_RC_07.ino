//Emisora de radiocontrol
//Por Francisco Carabaza
//octubre 2020

//LCD1602_PCF8574_I2C adapter
//https://www.naylampmechatronics.com/blog/35_Tutorial--LCD-con-I2C-controla-un-LCD-con-so.html

/*
  Control de versiones:
  V06 01/nov/2020. Se añade modelo omniwheel
  V07 02/nov/2020. El modelo por defecto se guarda en EEPROM.
*/


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs) 8 canales 22500.
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 10  //set PPM signal output pin on the arduino
//////////////////////////////////////////////////////////////////

/*this array holds the servo values for the ppm signal
  change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];
int modoTX = 0;
int estadoMenu = 0;
int eeAddress = 0;
unsigned long time;
char BluetoothData; // the Bluetooth data received
int Speed = 250; // Motors Speed
//int SensA = A8;    // select the input pin for the current of channel A
//int SensB = A9;    // select the input pin for the current of channel B

LiquidCrystal_I2C lcd(0x27, 16, 2);//Función constructor, crea un objeto de la clase LiquidCrystal_I2C, con dirección, columnas y filas indicadas.

void setup() {
  pinMode(52, INPUT_PULLUP); //botón joystick derecho
  pinMode(53, INPUT_PULLUP); //botón joystick izquierdo
  lcd.init(); // Inicializar el LCD
  lcd.clear(); // borra la pantalla lcd
  lcd.backlight();//Encender la luz de fondo.
  //lcd.noBacklight(); // Esto apaga la luz de fondo.
  //lcd.print("Tiempo conectado:"); // Escribimos el Mensaje en el LCD.
  //lcd.print("Salida PPM"); // Escribimos el Mensaje en el LCD.
  modoTX = 0;

  // initialize serial communications:
  Serial.begin(115200); //Puerto para depurar
  Serial1.begin(115200);//Puerto para el bluetooth HC-06

  //initiallize default ppm value
  for (int i = 0; i < chanel_number; i++) {
    ppm[i] = default_servo_value;
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)

  //lcd.clear(); // borra la pantalla lcd
  //lcd.setCursor(0, 1);// Ubicamos el cursor en la primera posición(columna:0) de la segunda línea(fila:1). Posiciona el cursor del LCD en la posición indicada por col y row(x,y);

  if (digitalRead(53) == LOW) { //Pulsador derecho pulsado, entrada al menú

    lcd.clear(); // borra la pantalla lcd
    lcd.print("Elige modo:");
    lcd.setCursor(0, 1);// Ubicamos el cursor al inicio de la segunda fila
    lcd.print("normal");

    estadoMenu = 0;
    //modoTX = 0 modo normal
    //motoTX = 1 modo elevon
    //modoTX = 2 modo Bluetooth
    //modoTX = 3 modo OmniWheel 3CH

    while (estadoMenu == 0) {

      if (modoTX == 0 && analogRead(A2) < 400) { //stick abajo,
        modoTX = 1; // Modo Elevon desde modo normal
        lcd.clear(); // borra la pantalla lcd
        lcd.print("Elige modo:");
        lcd.setCursor(0, 1);// Ubicamos el cursor al inicio de la segunda fila
        lcd.print("Elevon");
        while (analogRead(A2) < 400 || analogRead(A2) > 600) {} // quieto aquí hasta que se centra el stick
      }

      if (modoTX == 1 && analogRead(A2) > 600) { //stick arriba
        modoTX = 0; // Modo normal desde modo elevon
        lcd.clear(); // borra la pantalla lcd
        lcd.print("Elige modo:");
        lcd.setCursor(0, 1);// Ubicamos el cursor al inicio de la segunda fila
        lcd.print("Normal");
        while (analogRead(A2) < 400 || analogRead(A2) > 600) {} // quieto aquí hasta que se centra el stick
      }

      if (modoTX == 1 && analogRead(A2) < 400) { //stick abajo
        modoTX = 2; //Modo Bluetooth desde modo elevon
        lcd.clear(); // borra la pantalla lcd
        lcd.print("Elige modo:");
        lcd.setCursor(0, 1);// Ubicamos el cursor al inicio de la segunda fila
        lcd.print("Bluetooth");
        while (analogRead(A2) < 400 || analogRead(A2) > 600) {} // quieto aquí hasta que se centra el stick
      }

      if (modoTX == 2 && analogRead(A2) > 600) { //stick arriba
        modoTX = 1; // Modo elevon desde modo bluetooth
        lcd.clear(); // borra la pantalla lcd
        lcd.print("Elige modo:");
        lcd.setCursor(0, 1);// Ubicamos el cursor al inicio de la segunda fila
        lcd.print("Elevon");
        while (analogRead(A2) < 400 || analogRead(A2) > 600) {} // quieto aquí hasta que se centra el stick
      }


      if (modoTX == 2 && analogRead(A2) < 400) { //stick abajo
        modoTX = 3; //Modo OmniWheel 3 canales
        lcd.clear(); // borra la pantalla lcd
        lcd.print("Elige modo:");
        lcd.setCursor(0, 1);// Ubicamos el cursor al inicio de la segunda fila
        lcd.print("OmniWheel 3CH");
        while (analogRead(A2) < 400 || analogRead(A2) > 600) {} // quieto aquí hasta que se centra el stick
      }

      if (modoTX == 3 && analogRead(A2) > 600) { //stick arriba
        modoTX = 2; // Modo bluetooth desde modo OmniWheel
        lcd.clear(); // borra la pantalla lcd
        lcd.print("Elige modo:");
        lcd.setCursor(0, 1);// Ubicamos el cursor al inicio de la segunda fila
        lcd.print("Bluetooth");
        while (analogRead(A2) < 400 || analogRead(A2) > 600) {} // quieto aquí hasta que se centra el stick
      }



      if (analogRead(A1) > 600) { //stick derecha
        lcd.clear(); // borra la pantalla lcd
        lcd.print("Modo");
        lcd.setCursor(0, 1);// Ubicamos el cursor al inicio de la segunda fila
        lcd.print("OK");
        estadoMenu = 1;
        while (analogRead(A1) < 400 || analogRead(A1) > 600) {} // quieto aquí hasta que se centra el stick
        EEPROM.put(eeAddress, modoTX); // Graba el modoTX en la dirección cero de la EEPROM
      }
    }

  }

  lcd.clear(); // borra la pantalla lcd
  lcd.print("Salida PPM");
  lcd.setCursor(0, 1);// Ubicamos el cursor al inicio de la segunda fila

  EEPROM.get( eeAddress, modoTX ); // Lee el modoTX desde la EEPROM
  switch (modoTX) {
    case 0:
      lcd.print("Modo normal");
      break;
    case 1:
      lcd.print("Modo elevon");
      break;
    case 2:
      lcd.print("Modo bluetooth");
      break;
    case 3:
      lcd.print("Modo OmniWheel");
      break;
    default:
      lcd.print("Modo no definido");
      break;
  }


  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

void loop() {

  //read the pushbutton value into a variable
  int sensorValDcho = digitalRead(53);
  int sensorValIzqu = digitalRead(52);

  // read the inputs on analog pins
  int Analog0 = analogRead(A1);
  int Analog1 = analogRead(A2) + 3;
  int Analog2 = analogRead(A3) - 12;
  int Analog3 = analogRead(A4) - 3;
  int Analog4 = analogRead(A6) + 10;
  int Analog5 = analogRead(A5);
  int Analog6 = map(sensorValIzqu, 1, 0, 980, 2020);
  int Analog7 = map(sensorValDcho, 1, 0, 980, 2020);


  switch (modoTX) {
    case 0:   //modo TX normal
      ppm[0] = map(Analog0, 0, 1023, 980, 2020);
      ppm[1] = map(Analog1, 0, 1023, 980, 2020);
      ppm[2] = map(Analog2, 0, 1023, 980, 2020);
      ppm[3] = map(Analog3, 0, 1023, 980, 2020);
      ppm[4] = map(Analog4, 0, 1023, 980, 2020);
      ppm[5] = map(Analog5, 0, 1023, 980, 2020);
      ppm[6] = Analog6;
      ppm[7] = Analog7;
      break;

    case 1: //modo mezcla elevon
      ppm[0] = map(Analog1, 0, 1023, -1000, 1000) + map(Analog0, 0, 1023, -1000, 1000);
      ppm[0] = map(ppm[0], -1000, 1000, 980, 2020);
      ppm[0] = constrain(ppm[0], 980, 2020);

      ppm[1] = map(Analog1, 0, 1023, -1000, 1000) - map(Analog0, 0, 1023, -1000, 1000);
      ppm[1] = map(ppm[1], -1000, 1000, 980, 2020);
      ppm[1] = constrain(ppm[1], 980, 2020);

      ppm[2] = map(Analog2, 0, 1023, 980, 2020);
      ppm[3] = map(Analog3, 0, 1023, 980, 2020);
      ppm[4] = map(Analog4, 0, 1023, 980, 2020);
      ppm[5] = map(Analog5, 0, 1023, 980, 2020);

      ppm[6] = Analog6;
      ppm[7] = Analog7;

      break;

    case 2:  //modo TX Bluetooth
      //Serial1.print("*G" + String(analogRead(SensA)) + "*");
      //Serial1.print("*H" + String(analogRead(SensB)) + "*");

      //Check Bluetooth for new Instructions
      if (Serial1.available()) {

        BluetoothData = Serial1.read(); //Get next character from bluetooth

        if (BluetoothData == 'A') { // Slider Change Speed
          Speed = Serial1.parseInt(); //Read Speed value
        }

        if (BluetoothData == '1') { // Up Button Pressed
          ppm[1] = 1500 + Speed;
        }

        if (BluetoothData == '2') { // Right Button Pressed
          ppm[0] = 1500 + Speed;
        }

        if (BluetoothData == '3') { // Down Button Pressed
          ppm[1] = 1500 - Speed;
        }

        if (BluetoothData == '4') { // Left Button Pressed
          ppm[0] = 1500 - Speed;
        }

        if (BluetoothData == '5') { // Up Right Button Pressed

        }

        if (BluetoothData == '6') { // Down Right Button Pressed

        }

        if (BluetoothData == '7') { // Down Left Button Pressed

        }

        if (BluetoothData == '8') { // Up Left Button Pressed

        }

        if (BluetoothData == '0') { // Release Pad String
          ppm[0] = 1500;
          ppm[1] = 1500;
          ppm[2] = 1500;
          ppm[3] = 1500;
          ppm[4] = 1500;
          ppm[5] = 1500;
          ppm[6] = 1500;
          ppm[7] = 1500;
        }

      }
      break;

    case 3: //modo OmniWheel 3CH
      int ail;
      int ele;
      int rud;
      int gv1 = 42;
      int gv2 = 42;

      //inputs
      ail =  map(Analog0, 0, 1023, -45, 45);
      ele =  map(Analog1, 0, 1023, -45, 45);
      rud =  map(Analog5, 0, 1023, -45, 45);

      //mixes
      ppm[0] = (-gv1 * ail / 100) + (-ele) + (-rud);
      ppm[0] = map(ppm[0], -100, 100, 980, 2020);

      ppm[1] = (-gv2 * ail / 100) + (ele) + (-rud);
      ppm[1] = map(ppm[1], -100, 100, 980, 2020);

      ppm[2] = (ail) + (-rud);
      ppm[2] = map(ppm[2], -100, 100, 980, 2020);

      ppm[3] = 1500;
      ppm[4] = 1500;
      ppm[5] = 1500;
      ppm[6] = 1500;
      ppm[7] = 1500;

      break;

  }

  delay(10);

  // print out mapped values:
  Serial.print(ppm[0]);
  Serial.print(", ");
  Serial.print(ppm[1]);
  Serial.print(", ");
  Serial.print(ppm[2]);
  Serial.print(", ");
  Serial.print(ppm[3]);
  Serial.print(", ");
  Serial.print(ppm[4]);
  Serial.print(", ");
  Serial.print(ppm[5]);
  Serial.print(", ");
  Serial.print(ppm[6]);
  Serial.print(", ");
  Serial.print(ppm[7]);
  Serial.println(", ");

}

ISR(TIMER1_COMPA_vect) { //leave this alone
  static boolean state = true;
  TCNT1 = 0;
  if (state) { //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else { //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
    digitalWrite(sigPin, !onState);
    state = true;
    if (cur_chan_numb >= chanel_number) {
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;//
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}
