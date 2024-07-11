#include <Keypad.h>             // Include biblioteca Keypad pentru tastatura
#include <Servo.h>              // Include biblioteca Servo pentru controlul servo-motorului
#include <TM1637Display.h>      // Include biblioteca TM1637Display pentru afisajul cu 7 segmente
#include <Wire.h>               // Include biblioteca Wire pentru comunicare I2C
#include <LiquidCrystal_I2C.h>  // Include biblioteca LiquidCrystal_I2C pentru ecranul lcd
#include "MAX30105.h"           // Include biblioteca pentru senzorul MAX30105
#include "heartRate.h"          // Include biblioteca pentru măsurarea ritmului cardiac
#include <SoftwareSerial.h>     // Include biblioteca SoftwareSerial pentru comunicare serială suplimentară
#include <dht11.h>              // Include biblioteca pentru senzorul DHT11

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Keypad
const byte ROWS = 4;       // Definește numărul de rânduri pentru tastatură
const byte COLS = 3;       // Definește numărul de coloane pentru tastatură
char keys[ROWS][COLS] = {  // Definește layout-ul tastelor
  { '1', '2', '3' },
  { '4', '5', '6' },
  { '7', '8', '9' },
  { '*', '0', '#' }
};
byte rowPins[ROWS] = { 35, 37, 39, 41 };                                 // Definește pinii pentru rândurile tastaturii
byte colPins[COLS] = { 43, 45, 47 };                                     // Definește pinii pentru coloanele tastaturii
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);  // Creează un obiect Keypad


Servo myservo;  // Creează un obiect Servo pentru a controla servo-motorul
int pos = 0;    // Inițializează poziția servo-motorului la 0

// TM1637 Display
#define CLK 25                    // Definește pinul pentru clock al afisajului
#define DIO 24                    // Definește pinul pentru date al afisajului
TM1637Display display(CLK, DIO);  // Creează un obiect TM1637Display

#define COUNTDOWN_DURATION_DEFAULT_HOURS 0    // Durata implicită a contorului în ore
#define COUNTDOWN_DURATION_DEFAULT_MINUTES 1  // Durata implicită a contorului în minute

int countdownHours = 0;        // Inițializează orele contorului la 0
int countdownMinutes = 0;      // Inițializează minutele contorului la 0
int countdownSeconds = 0;      // Inițializează secundele contorului la 0
bool countdownActive = false;  // Inițializează starea contorului la inactiv

unsigned long previousMillis = 0;  // Timpul anterior pentru a urmări trecerea timpului
const long interval = 1000;        // Intervalul de timp pentru actualizarea contorului (1000 ms)

char enteredTime[5] = "0000";  // Buffer pentru stocarea timpului introdus
int enteredIndex = 0;          // Indexul pentru buffer

int motor = 53;
int buzzer = 8;
int module = 0;
int vital = 0;

int redLedB = 6;
int blueLedB = 5;

#define DHT11PIN 26  // Definește pinul digital 7 pentru senzorul DHT11

dht11 DHT11;  // Creează un obiect de tip dht11

// Inițializează senzorul de ritm cardiac
MAX30105 particleSensor;

// Variabile pentru măsurarea ritmului cardiac
const byte RATE_SIZE = 4;  // Mărimea array-ului pentru ritm cardiac (mai mare pentru mediere mai bună, 4 este optim)
byte rates[RATE_SIZE];     // Array de ritmuri cardiace
byte rateSpot = 0;         // Poziția curentă în array-ul de ritmuri cardiace
long lastBeat = 0;         // Momentul în care a avut loc ultima bătaie

float beatsPerMinute = 0;  // Valoarea curentă a BPM (bătăi pe minut)
int beatAvg = 0;           // Valoarea medie a BPM

int sensorValue;  // Declară variabila pentru valoarea senzorului

int Vibration_signal = 27;  // Definește pinul digital de intrare pentru semnalul senzorului de vibrații
int Sensor_State = 1;       // Variabilă pentru a stoca starea senzorului de vibrații

const int mq2Pin = A2;  // Pin analogic conectat la senzorul MQ2

int sensorPin = A7;  // Selectează pinul de intrare pentru LDR

int sensorValue1 = 0;  // Variabilă pentru a stoca valoarea venită de la senzor

void setup() {
  pinMode(buzzer, OUTPUT);  // Setează pinul buzzerului ca ieșire
  analogWrite(buzzer, 0);
  lcd.init();           // Initializează LCD-ul
  lcd.backlight();      // Asigură-te că iluminarea de fundal este pornită
  lcd.setCursor(0, 0);  // Setează cursorul la coloana 0, linia 0
  lcd.clear();
  Serial.begin(9600);  // Inițializează comunicarea serială la 9600 baud

  pinMode(Vibration_signal, INPUT);  // Setează pinul senzorului de vibrații ca intrare
  pinMode(redLedB, OUTPUT);
  pinMode(blueLedB, OUTPUT);

  myservo.attach(52);      // Atașează servo-motorul la pinul 52
  myservo.write(0);        // Setează servo-motorul la poziția 0
  pinMode(motor, OUTPUT);  // Setează pinul motorului ca ieșire

  display.setBrightness(0x0f);  // Setează luminozitatea afișajului la maxim
  display.clear();              // Curăță afișajul

  lcd.print("Selecteaza Modul");

  // Inițializează senzorul de ritm cardiac
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {  // Utilizează portul I2C implicit, viteză 400kHz
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    lcd.clear();
    lcd.print("Eroare");
  }

  particleSensor.setup();                     // Configurează senzorul cu setările implicite
  particleSensor.setPulseAmplitudeRed(0x0A);  // Setează LED-ul roșu la intensitate scăzută pentru a indica că senzorul funcționează
  particleSensor.setPulseAmplitudeGreen(0);   // Dezactivează LED-ul verde
}

void loop() {
  char key = keypad.getKey();  //Citeste o tasta de la tastatura

  if (key == '*') {  //Daca tasta * este apasata
    changeModule();  //Executa schimbarea de modul
  }

  if (key == '#') {
    lcd.clear();
    switch (module) {
      case 1:
        lcd.clear();
        lcd.print("Introdu Timpul");
        while (true) {
          key = keypad.getKey();  // Obține din nou cheia în bucla while
          if (key == '*') {
            module = 0;
            resetTimer();
            lcd.clear();
            lcd.print("Selecteaza Modul");
            break;
          }
          if (key != NO_KEY) {                               // Dacă o tastă a fost apăsată
            if (!countdownActive) {                          // Dacă contorul nu este activ
              if (key >= '0' && key <= '9') {                // Dacă tasta este un număr
                if (enteredIndex < 4) {                      // Dacă buffer-ul nu este plin
                  enteredTime[enteredIndex] = key;           // Adaugă tasta în buffer
                  enteredIndex++;                            // Incrementează indexul
                  display.setSegments(encodeEnteredTime());  // Actualizează afișajul
                }
              } else if (key == '#') {  // Dacă tasta este #
                parseEnteredTime();     // Parsează timpul introdus
                startCountdown();       // Pornește contorul
              }
              Serial.print("Key pressed: ");  // Afișează tasta apăsată
              Serial.println(key);
            }
          }

          if (countdownActive) {                               // Dacă contorul este activ
            unsigned long currentMillis = millis();            // Citește timpul curent
            if (currentMillis - previousMillis >= interval) {  // Dacă a trecut un interval de timp
              if (countdownSeconds > 0 || countdownMinutes > 0 || countdownHours > 0) {
                updateCountdown();  // Actualizează contorul
              }
              previousMillis = currentMillis;  // Actualizează timpul anterior
            }
          }
        }
        break;
      case 2:
        lcd.clear();
        while (true) {
          key = keypad.getKey();  // Obține din nou cheia în bucla while
          if (key == '*') {
            module = 0;
            lcd.clear();
            lcd.print("Selecteaza Modul");
            break;
          }

          Sensor_State = digitalRead(Vibration_signal);  // Citește starea senzorului de vibrații
          if (Sensor_State == 1) {                       // Dacă senzorul de vibrații detectează ceva
            Serial.println("Sensing vibration");         // Trimite un mesaj prin serial
            lcd.clear();                                 // Curăță ecranul LCD
            lcd.setCursor(0, 0);                         // Setează cursorul LCD-ului la începutul primei linii
            lcd.print("Cutremur!");                      // Afișează mesajul "Cutremur!" pe LCD
            analogWrite(buzzer, 100);                    // Activează buzzer-ul
            for (int i = 0; i < 10; i++) {
              analogWrite(redLedB, 100);
              delay(50);
              analogWrite(redLedB, 0);
              analogWrite(blueLedB, 100);
              delay(50);
              analogWrite(blueLedB, 0);
            }
            lcd.clear();
            analogWrite(buzzer, 0);  // Dezactivează buzzer-ul după semnalizare
          }
          delay(100);  // Întârziere de 100 milisecunde

          // Citește valoarea analogică de la senzorul MQ2
          sensorValue = analogRead(mq2Pin);
          Serial.println(sensorValue);  // Afișează valoarea citită în consola serială

          // Dacă valoarea senzorului depășește pragul specificat, presupune prezența fumului/gazului
          if (sensorValue > 970) {                  // Ajustează acest prag după necesități
            analogWrite(buzzer, 100);               // Activează buzzer-ul
            lcd.clear();                            // Curăță ecranul LCD
            lcd.setCursor(0, 0);                    // Setează cursorul LCD-ului la începutul primei linii
            lcd.print("Fum/Gaz Detectat");          // Afișează mesajul "Fum/Gaz Detectat" pe LCD
            Serial.println("Smoke/Gas Detected!");  // Afișează mesajul în consola serială
            for (int i = 0; i < 10; i++) {
              analogWrite(redLedB, 100);
              delay(50);
              analogWrite(redLedB, 0);
              analogWrite(blueLedB, 100);
              delay(50);
              analogWrite(blueLedB, 0);
            }
            lcd.clear();
            analogWrite(buzzer, 0);  // Dezactivează buzzer-ul după semnalizare
          }
          delay(100);  // Întârziere de 100 milisecunde

          // Citește valoarea analogică de la LDR
          sensorValue = analogRead(sensorPin);
          Serial.println(sensorValue);  // Afișează valoarea citită în consola serială

          // Dacă valoarea senzorului este sub 50, presupune prezența focului
          if (sensorValue < 50) {
            lcd.clear();                      // Curăță ecranul LCD
            lcd.setCursor(0, 0);              // Setează cursorul LCD-ului la începutul primei linii
            lcd.print("Foc Detectat");        // Afișează mesajul "Foc Detectat" pe LCD
            Serial.println("Fire Detected");  // Afișează mesajul în consola serială
            analogWrite(buzzer, 100);         // Activează buzzer-ul
            delay(100);                       // Întârziere de 100 milisecunde

            for (int i = 0; i < 10; i++) {
              analogWrite(redLedB, 100);
              delay(50);
              analogWrite(redLedB, 0);
              analogWrite(blueLedB, 100);
              delay(50);
              analogWrite(blueLedB, 0);
            }
            lcd.clear();
            analogWrite(buzzer, 0);  // Dezactivează buzzer-ul după semnalizare
          }

          delay(sensorValue);  // Întârziere în funcție de valoarea senzorului de lumină
        }
        break;

      case 3:
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Selecteaza");
        lcd.setCursor(0, 1);
        lcd.print("Vital");
        lcd.setCursor(0, 0);

        while (true) {
          key = keypad.getKey();

          if (key == '*') {
            module = 0;
            vital = 0;
            lcd.clear();
            lcd.print("Selecteaza Modul");
            break;
          }

          if (key == '0') {
            changeVital();
          }

          if (key == '#') {
            switch (vital) {
              case 1:
                while (true) {
                  key = keypad.getKey();
                  if (key == '0') {
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Selecteaza");
                    lcd.setCursor(0, 1);
                    lcd.print("Vital");
                    lcd.setCursor(0, 0);
                    vital = 0;
                    break;
                  }

                  Serial.println();  // Tipărește o linie goală pentru separare

                  int chk = DHT11.read(DHT11PIN);  // Citește datele de la senzorul DHT11 și le stochează în variabila chk

                  Serial.print("Humidity (%): ");            // Tipărește textul "Humidity (%): " pe monitorul serial
                  Serial.println((float)DHT11.humidity, 2);  // Tipărește valoarea umidității citite de senzor, cu 2 zecimale

                  Serial.print("Temperature  (C): ");           // Tipărește textul "Temperature  (C): " pe monitorul serial
                  Serial.println((float)DHT11.temperature, 2);  // Tipărește valoarea temperaturii citite de senzor, cu 2 zecimale

                  lcd.setCursor(0, 0);                  // Setează cursorul LCD-ului la prima coloană (0) și primul rând (0)
                  lcd.print("Umiditate: ");             // Afișează textul "Umiditate: " pe LCD
                  lcd.print((float)DHT11.humidity, 2);  // Afișează valoarea umidității citite de senzor, cu 2 zecimale

                  lcd.setCursor(0, 1);                     // Setează cursorul LCD-ului la prima coloană (0) și al doilea rând (1)
                  lcd.print("Temp:    ");                  // Afișează textul "Temp:    " pe LCD
                  lcd.print((float)DHT11.temperature, 2);  // Afișează valoarea temperaturii citite de senzor, cu 2 zecimale
                  lcd.print("*C");                         // Afișează simbolul gradului Celsius pe LCD
                  delay(500);
                  lcd.clear();
                }
                break;
              case 2:
                while (true) {
                  key = keypad.getKey();
                  if (key == '0') {
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Selecteaza");
                    lcd.setCursor(0, 1);
                    lcd.print("Vital");
                    lcd.setCursor(0, 0);
                    vital = 0;
                    break;
                  }
                  sensorValue = analogRead(3);  // Citește valoarea analogică de la pinul 3
                  Serial.print("AirQua=");
                  Serial.print(sensorValue, DEC);  // Afișează valoarea citită în format zecimal
                  Serial.println(" PPM");

                  lcd.setCursor(0, 0);  // Setează cursorul pe LCD la poziția (0, 0)
                  lcd.print("Calitate Aer:");
                  lcd.setCursor(0, 1);  // Setează cursorul pe LCD la poziția (0, 1)
                  if (sensorValue < 800) {
                    lcd.print("Buna");  // Afișează "Buna" dacă valoarea senzorului este sub 800
                  } else if (sensorValue < 1200) {
                    lcd.print("Medie");  // Afișează "Medie" dacă valoarea senzorului este între 800 și 1200
                  } else {
                    lcd.print("Rea");  // Afișează "Rea" dacă valoarea senzorului este peste 1200
                  }
                  delay(500);   // Așteaptă 5 secunde
                  lcd.clear();  // Șterge ecranul LCD
                }
                break;
              case 3:
                measureHeartRate();  // Măsoară ritmul cardiac

                break;
              default:
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Niciun");
                lcd.setCursor(0, 1);
                lcd.print("Vital");
                lcd.setCursor(0, 0);
                delay(1000);

                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Selecteaza");
                lcd.setCursor(0, 1);
                lcd.print("Vital");
                lcd.setCursor(0, 0);

                break;
            }
          }
        }
        break;


        break;
      default:
        lcd.print("Niciun Modul");
        delay(2000);
        lcd.clear();
        lcd.print("Selecteaza Modul");
        break;
    }
  }
}

void changeModule() {
  lcd.clear();
  module++;
  if (module > 3) module = 1;
  if (module == 1) lcd.print("Dozator");
  if (module == 2) lcd.print("Pericole");
  if (module == 3) lcd.print("Vitale");
}

void changeVital() {
  lcd.clear();
  vital++;
  if (vital > 3) vital = 1;
  if (vital == 1) lcd.print("Temperatura");
  if (vital == 2) lcd.print("Calitate Aer");
  if (vital == 3) lcd.print("Puls");
}

void parseEnteredTime() {
  int enteredValue = atoi(enteredTime);   // Convertește timpul introdus la un număr
  countdownHours = enteredValue / 100;    // Extrage orele
  countdownMinutes = enteredValue % 100;  // Extrage minutele
}

void startCountdown() {

  lcd.clear();
  countdownSeconds = 0;                    // Setează secundele la 0
  countdownActive = true;                  // Setează contorul la activ
  Serial.print("Countdown started for ");  // Afișează mesaj de pornire contor
  Serial.print(countdownHours);
  Serial.print(" hours and ");
  Serial.print(countdownMinutes);
  Serial.println(" minutes.");
}

void updateCountdown() {
  if (countdownSeconds > 0) {  // Dacă secundele sunt mai mari decât 0
    countdownSeconds--;        // Decrementează secundele
  } else {
    if (countdownMinutes > 0 || countdownHours > 0) {
      if (countdownMinutes > 0) {       // Dacă minutele sunt mai mari decât 0
        countdownMinutes--;             // Decrementează minutele
        countdownSeconds = 59;          // Setează secundele la 59
      } else if (countdownHours > 0) {  // Dacă orele sunt mai mari decât 0
        countdownHours--;               // Decrementează orele
        countdownMinutes = 59;          // Setează minutele la 59
        countdownSeconds = 59;          // Setează secundele la 59
      }
    }
  }

  Serial.print("Time remaining: ");  // Afișează timpul rămas
  Serial.print(countdownHours);
  Serial.print(" hours, ");
  Serial.print(countdownMinutes);
  Serial.print(" minutes, ");
  Serial.print(countdownSeconds);
  Serial.println(" seconds");

  if (countdownHours == 0 && countdownMinutes == 0 && countdownSeconds == 0) {  // Dacă contorul a ajuns la 0
    countdownActive = false;                                                    // Setează contorul la inactiv
    Serial.println("Countdown finished!");                                      // Afișează mesaj de finalizare
    pillDispenser();                                                            // Activează distribuitorul de pastile

    countdownMinutes = COUNTDOWN_DURATION_DEFAULT_MINUTES;  // Resetează minutele implicite
    countdownHours = COUNTDOWN_DURATION_DEFAULT_HOURS;      // Resetează orele implicite
    startCountdown();                                       // Repornește contorul
  }

  displayCountdown();  // Actualizează afișajul
}

void displayCountdown() {
  int totalMinutes = countdownHours * 60 + countdownMinutes;     // Calculează totalul de minute
  display.showNumberDecEx(totalMinutes / 60, 0x40, true, 2, 0);  // Afișează orele și minutele
  display.showNumberDec(totalMinutes % 60, true, 2, 2);
}

uint8_t* encodeEnteredTime() {
  static uint8_t data[4];  // Creează un buffer static pentru date
  for (int i = 0; i < 4; i++) {
    data[i] = display.encodeDigit(enteredTime[i] - '0');  // Encodează fiecare cifră
  }
  return data;  // Returnează bufferul
}

void pillDispenser() {
  for (pos = 0; pos <= 30; pos += 1) {  // Mișcă servo-motorul înainte
    myservo.write(pos);
    delay(15);
  }
  delay(250);  // Așteaptă 250 ms

  for (pos = 30; pos >= 0; pos -= 1) {  // Mișcă servo-motorul înapoi
    myservo.write(pos);
    delay(15);
  }

  digitalWrite(motor, HIGH);  // Pornește motorul
  delay(1000);                // Așteaptă 1 secundă
  digitalWrite(motor, LOW);   // Oprește motorul

  delay(1000);  // Așteaptă 1 secundă
}

void resetTimer() {
  countdownHours = 0;       // Inițializează orele contorului la 0
  countdownMinutes = 0;     // Inițializează minutele contorului la 0
  countdownSeconds = 0;     // Inițializează secundele contorului la 0
  countdownActive = false;  // Inițializează starea contorului la inactiv

  display.setBrightness(0x0f);  // Setează luminozitatea afișajului la maxim
  display.clear();              // Curăță afișajul

  enteredIndex = 0;  // Resetează indexul pentru bufferul timpului introdus
  for (int i = 0; i < 4; i++) {
    enteredTime[i] = '0';  // Resetează bufferul timpului introdus
  }
}
void measureHeartRate() {
  lcd.setCursor(0, 0);  // Setează cursorul pe LCD la poziția (0, 0)
  lcd.print("Scanare...");
  while (true) {

    if (vital == 0) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Selecteaza");
      lcd.setCursor(0, 1);
      lcd.print("Vital");
      lcd.setCursor(0, 0);
      break;
    }
    const int numReadings = 300;  // Numărul de citiri
    int validReadings = 0;        // Numărul de citiri valide
    float totalBPM = 0;           // Totalul BPM pentru calcularea mediei

    for (int i = 0; i < numReadings; i++) {   // Buclă pentru citirea datelor de ritm cardiac
      long irValue = particleSensor.getIR();  // Citește valoarea IR de la senzor

      int key = keypad.getKey();
      if (key == '0') {
        vital = 0;
        break;
      }


      if (checkForBeat(irValue) == true) {         // Verifică dacă a fost detectată o bătaie
        long delta = millis() - lastBeat;          // Calculează intervalul de timp dintre bătăi
        lastBeat = millis();                       // Actualizează timpul ultimei bătăi
        beatsPerMinute = 60.0 / (delta / 1000.0);  // Calculează BPM

        if (beatsPerMinute < 255 && beatsPerMinute > 20) {  // Verifică dacă valoarea BPM este în intervalul valid
          rates[rateSpot++] = (byte)beatsPerMinute;         // Stochează valoarea BPM în array
          rateSpot %= RATE_SIZE;                            // Reia array-ul dacă a ajuns la capăt

          totalBPM += beatsPerMinute;  // Adaugă BPM la total pentru calculul mediei
          validReadings++;             // Incrementare număr citiri valide
        }
      }
      delay(20);  // Mică întârziere între citiri
    }

    beatAvg = totalBPM / validReadings;  // Calculează media BPM
    lcd.clear();
    lcd.setCursor(0, 0);  // Setează cursorul pe LCD la poziția (0, 0)
    lcd.print("BPM: ");
    lcd.print(beatAvg);  // Afișează media BPM pe LCD
    delay(500);
  }
}
