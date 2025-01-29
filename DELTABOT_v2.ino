/*
 * Datei: DELTABOT_v2.ino
 * Projektname: DELTABOT
 * Beschreibung: Delta-Roboter mit Sauger
 * Autor: Lasse Timmerhinrich & Isa El-Molla 
 * Version: 2.0
 * Datum: 27.01.2025
 */

/*
###################################################################################################################################################################

Menüführung und Steuerung des Deltabots über den seriellen Monitor der Arduino IDE mit der Baudrate 115200 möglich.
Zu Beginn muss immer eine Initialisierungsfahrt gemacht werden, sonst kann der Handbetrieb und die Kreisfahrt nicht verwendet werden.

Benötigte Zusatzdateien (wenn nicht verlinkt, in Zip-Datei enthalten):
- DELTABOT_v2.h
- AdvancedDeltaKinematics.h (erbt von Deltakinematics-Klasse aus angepasster Deltakinematics Library)

Benötigte Librarys:
- Accelstepper (http://www.airspayce.com/mikem/arduino/AccelStepper/)
- Deltakinematics, mit der zusätzlichen Deklaration "virtual" um die Methode inverse(x0, y0, z0) nach Vererbung zu nutzen
(Original: https://github.com/tinkersprojects/Delta-Kinematics-Library ; angepasste Version in Zip-Datei enthalten)

Zum Handbetrieb:
Der Koordinatenursprung ist in der Mitte auf dem Boden X und Y Richtung am DeltaBot markiert, Z-Koordinate wird nach oben Größer.
Bereich: X: von -80 mm bis +80 mm; Y: von -80 mm bis +80 mm; Z: von 5 mm bis 160 mm (5 mm ist die Position wo der Sauger) einen Abstand von 5 mm zum Boden hat.
Wenn X und Y gleich 0 sind, ist der Sauger in der Mitte positioniert. Der Sauger kann vor jeder Positionseingabe aktiviert/deaktiviert werden.

Zur Kreisfahrt:
Vor jeder Kreisfahrt muss ein Radius angegeben werden (bis max. 80 mm).

###################################################################################################################################################################
*/

#include "DELTABOT_v2.h"

void setup() {
  Serial.begin(115200);

  state = menuMode;  //Schrittkette auf Null setzen

  stepperA.setAcceleration(Acceleration);
  pinMode(enablePinA, OUTPUT);
  digitalWrite(enablePinA, LOW);

  stepperB.setAcceleration(Acceleration);
  pinMode(enablePinB, OUTPUT);
  digitalWrite(enablePinB, LOW);

  stepperC.setAcceleration(Acceleration);
  pinMode(enablePinC, OUTPUT);
  digitalWrite(enablePinC, LOW);

  pinMode(limitSwitchA, INPUT);
  pinMode(limitSwitchB, INPUT);
  pinMode(limitSwitchC, INPUT);

  //Lüfter
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, HIGH);

  //Sauger;
  pinMode(vacuumPin, OUTPUT);

  //interrupt
  attachInterrupt(digitalPinToInterrupt(limitSwitchA), stopMotorA, FALLING);
  attachInterrupt(digitalPinToInterrupt(limitSwitchB), stopMotorB, FALLING);
  attachInterrupt(digitalPinToInterrupt(limitSwitchC), stopMotorC, FALLING);
}

void loop() {
  //Prüft in welchem Zustand die Maschine ist
  switch (state) {
    case (menuMode): menu(); break;
    case (initMode): initialization(); break;
    case (handMode):
      if (isInit) {  //prüfen ob Maschine initialisiert wurde
        manuel();
        break;
      } else {
        Serial.print("Maschine muss initialisiert werden, bitte zuerst eine initialisirungsfahrt durchführen");
        Serial.println();
        state = menuMode;
        break;
      }
    case (circleMode):
      if (isInit) {  //prüfen ob Maschine initialisiert wurde
        circleMotion();
        break;
      } else {
        Serial.print("Maschine muss initialisiert werden, bitte zuerst eine initialisirungsfahrt durchführen");
        Serial.println();
        state = menuMode;
        break;
      }
    default:;
  }
}

void menu() {
  interruptTriggered = false;  //Interrupt erkennung zurücksetzten
  Serial.println();
  Serial.println("----- Menü -----");
  Serial.println("1: Initialisierungsfahrt");
  Serial.println("2: Handbetrieb");
  Serial.println("3: Kreisfahrt");
  Serial.println("----------------");
  Serial.println();

  // Warten auf Eingabe
  while (Serial.available() == 0) {
    // Keine Eingabe, weiter warten
  }
  uint8_t inputMenu = Serial.readString().toInt();
  if (1 <= inputMenu && inputMenu <= 3) {
    state = inputMenu;
  } else {
    Serial.println();
    Serial.print("Falche Eingabe, bitte nochmal versuchen!");
    Serial.println();
  }
}

//---HANDBETRIEB---
void manuel() {
  //Ist-Position ausgeben
  Serial.println("-------------------");
  Serial.println("Aktuelle Position: ");
  Serial.print("x: ");
  Serial.println(delta.x);
  Serial.print("Y: ");
  Serial.println(delta.y);
  Serial.print("Z: ");
  //Z-Koordinate umskalieren, Koordinaten ursprung Boden
  double deltaZ_scal = map(delta.z, -329, -175, 5, 160);
  Serial.println(deltaZ_scal);
  Serial.println("-------------------");
  Serial.println();
  if (vacuumOn) {
    Serial.println("Sauger ist aktiviert.");
    Serial.println();
  } else {
    Serial.println("Sauger ist deaktiviert.");
    Serial.println();
  }
  // um do-while schleife zu verlassen nach Auswahl
  bool eingabe = true;
  //Bediener fragen
  do {
    Serial.println("-------------------------------------");
    Serial.println("1: Sauger aktivieren/deaktivieren");
    Serial.println("2: Position anfahren");
    Serial.println("3: Zurück zum Menü");
    Serial.println("-------------------------------------");
    Serial.println();

    // Warten auf Eingabe
    while (Serial.available() == 0) {
      // Keine Eingabe, weiter warten
      //falls interrupt State ändert, Schleife verlassen
      if (state == menuMode && interruptTriggered) {
        Serial.println();
        Serial.println("Endlage angefahren. Zurück zum Menü...");
        Serial.println();
        break;
      }
    }
    uint8_t inputHand = Serial.readString().toInt();
    //Eingabe prüfen
    if (1 <= inputHand && inputHand <= 3) {
      switch (inputHand) {
        case (1):  //sauger aktivieren/deaktivieren
          Serial.println();
          Serial.println("Sauger wird aktivieren/deaktiviert...");
          Serial.println();
          vacuumOn = !vacuumOn;
          digitalWrite(vacuumPin, vacuumOn);
          break;
        case (2): eingabe = false; break;  //weiter mit manuel()
        case (3):
          state = menuMode;
          eingabe = false;
          return;  // Funktion sofort beenden
          break;
        default:
          break;
      }
    } else {
      Serial.println();
      Serial.print("Falche Eingabe, bitte nochmal versuchen!");
      Serial.println();
    }
    //falls interrupt State ändert, Schleife verlassen
    if (state == menuMode && interruptTriggered) {
      Serial.println();
      Serial.println("Endlage angefahren. Zurück zum Menü...");
      Serial.println();
      break;
    }
  } while (eingabe);

  //---Abfrage Koordinaten---
  Serial.println("Geben Sie die Zielkoordinaten ein: ");
  Serial.println("Bereich X/Y: -80 bis 80  Z: 5 bis 160");
  Serial.print("Für X: ");

  // Warten auf Eingabe
  while (Serial.available() == 0) {
    // Keine Eingabe, weiter warten
    //falls interrupt State ändert, Schleife verlassen
    if (state == menuMode && interruptTriggered) {
      Serial.println();
      Serial.println("Endlage angefahren. Zurück zum Menü...");
      Serial.println();
      break;
    }
  }
  double inputX = Serial.readString().toDouble();

  Serial.println(inputX);
  double targetX = inputX;

  Serial.print("Für Y: ");

  // Warten auf Eingabe
  while (Serial.available() == 0) {
    // Keine Eingabe, weiter warten
    //falls interrupt State ändert, Schleife verlassen
    if (state == menuMode && interruptTriggered) {
      Serial.println();
      Serial.println("Endlage angefahren. Zurück zum Menü...");
      Serial.println();
      break;
    }
  }
  double inputY = Serial.readString().toDouble();

  Serial.println(inputY);
  double targetY = inputY;

  Serial.print("Für Z: ");

  // Warten auf Eingabe
  while (Serial.available() == 0) {
    // Keine Eingabe, weiter warten
    //falls interrupt State ändert, Schleife verlassen
    if (state == menuMode && interruptTriggered) {
      Serial.println();
      Serial.println("Endlage angefahren. Zurück zum Menü...");
      Serial.println();
      break;
    }
  }
  double inputZ = Serial.readString().toDouble();
  //Z-Koordinate umskalieren, Koordinaten ursprung Boden
  double inputZ_scal = map(inputZ, 5, 160, -329, -175);
  Serial.println(inputZ);
  Serial.println();
  double targetZ = inputZ_scal;

  //------Eingabe verarbeiten------
  int8_t result = delta.inverse(targetX, targetY, targetZ);
  //prüfen ob position erreichbar
  if (result == no_error) {
    //errechnete Motorwinkel in Steps umrechnen
    double stepsA = (Multiplier * delta.a) / 360;
    double stepsB = (Multiplier * delta.b) / 360;
    double stepsC = (Multiplier * delta.c) / 360;
    //Winkel in Koordinaten umrechnen
    delta.forward(delta.a, delta.b, delta.c);
    //Stepper Ziel übergeben
    stepperA.moveTo(stepsA);
    stepperB.moveTo(stepsB);
    stepperC.moveTo(stepsC);
    //bediener rückmelden das fahrt läuft
    Serial.println("Fährt zur Position...");
    Serial.println();
    //zur position Fahren
    while (stepperA.distanceToGo() != 0 || stepperB.distanceToGo() != 0 || stepperC.distanceToGo() != 0) {
      stepperA.run();
      stepperB.run();
      stepperC.run();
      //falls interrupt State ändert, Schleife verlassen
      if (state == menuMode && interruptTriggered) {
        Serial.println();
        Serial.println("Endlage angefahren. Zurück zum Menü...");
        Serial.println();
        break;
      }
    }
  } else {
    Serial.println("Fehler: Position nicht erreichbar!");
    Serial.println();
  }
}

//---------Kreisfahrt-----
void circleMotion() {
  double radius = 0;
  bool validRadius = false;

  // Abfrage des Radius
  while (!validRadius) {
    Serial.println("Bitte geben Sie den Radius des Kreises ein (10-80):");

    // Warten auf Eingabe
    while (Serial.available() == 0) {
      // Keine Eingabe, weiter warten
    }

    radius = Serial.readString().toDouble();

    // Überprüfung, ob der Radius im erlaubten Bereich liegt
    if (radius >= 10 && radius <= 80) {
      validRadius = true;
    } else {
      Serial.println("Ungültige Eingabe! Der Radius muss zwischen 10 und 80 liegen. Bitte erneut versuchen.");
    }
  }

  double angleStep = 2 * PI / 1000;  // Kreis in 1000 Schritte aufteilen
  Serial.println("Kreisfahrt läuft...");
  //Berechnung Positionen für den Kreis
  for (int i = 0; i < 1000; ++i) {
    double angle = i * angleStep;
    double x = radius * cos(angle);
    double y = radius * sin(angle);
    double z = -230;  //

    int8_t result = delta.inverse(x, y, z);
    double stepsA = (Multiplier * delta.a) / 360;
    double stepsB = (Multiplier * delta.b) / 360;
    double stepsC = (Multiplier * delta.c) / 360;

    // Position übergeben
    stepperA.moveTo(stepsA);
    stepperB.moveTo(stepsB);
    stepperC.moveTo(stepsC);

    //Position anfahren
    while (stepperA.distanceToGo() != 0 || stepperB.distanceToGo() != 0 || stepperC.distanceToGo() != 0) {
      stepperA.run();
      stepperB.run();
      stepperC.run();
    }
  }
  Serial.println("Kreisfahrt abgeschlossen");
  bool eingabe = true;
  //Bediener Fragen was zutun ist
  do {
    Serial.println("-------------------------------------");
    Serial.println("1: Erneute Kreisfahrt");
    Serial.println("2: Zurück zum Menü");
    Serial.println("-------------------------------------");
    Serial.println();

    // Warten auf Eingabe
    while (Serial.available() == 0) {
      // Keine Eingabe, weiter warten
    }
    uint8_t inputHand = Serial.readString().toInt();
    //Eingabe prüfen
    if (1 <= inputHand && inputHand <= 2) {
      switch (inputHand) {
        case (1): eingabe = false; break;  //weiter mit manuel()
        case (2):
          state = menuMode;
          eingabe = false;
          return;  // Funktion sofort beenden
          break;
        default:
          break;
      }
    } else {
      Serial.println();
      Serial.print("Falche Eingabe, bitte nochmal versuchen!");
      Serial.println();
    }
  } while (eingabe);
}

//---INITIALISIERUNG---

void initialization() {

  if (!homingDone) {
    Serial.println("Initialisierungsfahrt läuft");
    //geschwindigkeit anpassen
    stepperA.setMaxSpeed(SpeedInit);
    stepperB.setMaxSpeed(SpeedInit);
    stepperC.setMaxSpeed(SpeedInit);
    //Postition um in Endschalter rein zu fahren
    if (digitalRead(limitSwitchA)) {  //abfrage ob bereits in Endlage um nicht weiter zu fahren, wenn Endlage angefahren limitswitch = false somit wird keine neue pos. übergeben
      stepperA.moveTo(-150 * Multiplier);
    }
    if (digitalRead(limitSwitchB)) {
      stepperB.moveTo(-150 * Multiplier);
    }
    if (digitalRead(limitSwitchC)) {
      stepperC.moveTo(-150 * Multiplier);
    }
    //fährt solange bis interrupt wegen Endlagen auslöst
    while (stepperA.distanceToGo() != 0 || stepperB.distanceToGo() != 0 || stepperC.distanceToGo() != 0) {
      stepperA.run();
      stepperB.run();
      stepperC.run();
    }
    //wenn Roboter komplett oben ist, neue Nullposition
    if (!digitalRead(limitSwitchA) && !digitalRead(limitSwitchB) && !digitalRead(limitSwitchC)) {
      stepperA.setCurrentPosition(0);
      stepperB.setCurrentPosition(0);
      stepperC.setCurrentPosition(0);
      homingDone = true;
    }
  } else if (!downMoveDone) {
    //nach unten Fahren sodass Arme Parallel zum Boden sind (Home Position)
    if (stepperA.distanceToGo() == 0 && stepperB.distanceToGo() == 0 && stepperC.distanceToGo() == 0) {
      stepperA.moveTo(0.1 * Multiplier);
      stepperB.moveTo(0.1 * Multiplier);
      stepperC.moveTo(0.1 * Multiplier);
    }
    //fährt bis Ziel erreicht wird
    while (stepperA.distanceToGo() != 0 || stepperB.distanceToGo() != 0 || stepperC.distanceToGo() != 0) {
      stepperA.run();
      stepperB.run();
      stepperC.run();
    }
    //neue Null Position entspricht Home
    if (stepperA.distanceToGo() == 0 && stepperB.distanceToGo() == 0 && stepperC.distanceToGo() == 0) {
      stepperA.setCurrentPosition(0);
      stepperB.setCurrentPosition(0);
      stepperC.setCurrentPosition(0);
      downMoveDone = true;

      //geschwindigkeit erhöhen
      stepperA.setMaxSpeed(Speed);
      stepperB.setMaxSpeed(Speed);
      stepperC.setMaxSpeed(Speed);

      isInit = true;  //rückmeldung das Initialisiert wurde
      //Setzt variablen zurück um später eine erneute Init fahrt zu ermöglichen
      homingDone = false;
      downMoveDone = false;
      delta.forward(0, 0, 0);  // um aktuelle Position im Deltakinematics-Objekt zu speichern
      //bestätigung an Bediener
      Serial.println();
      Serial.println("###########################");
      Serial.println("Initialisierung erfolgreich");
      Serial.println("###########################");
      Serial.println();
      state = menuMode;  //Zurück zum Menü
    }
  }
}

//--- ISR INTERRUPT HANDLING ---

void stopMotorA() {
  //prüft ob Motor in richtung Endlagenschalter fährt (neg. = fährt in richtung Endlage)
  if (stepperA.speed() < 0) {
    //Setzt die Zielposition des Steppers als aktuelle position, damit Motor stehen bleibt
    stepperA.setCurrentPosition(stepperA.targetPosition());
  }
  stepperA.run();
  // isInt auf false setzten damit eine Referenzierung wieder notwendig ist
  isInit = false;
  if (state != initMode) {
    state = menuMode;
    interruptTriggered = true;
  }
}
void stopMotorB() {
  if (stepperB.speed() < 0) {
    stepperB.setCurrentPosition(stepperB.targetPosition());
  }
  stepperB.run();
  isInit = false;
  if (state != initMode) {
    state = menuMode;
    interruptTriggered = true;
  }
}
void stopMotorC() {
  if (stepperC.speed() < 0) {
    stepperC.setCurrentPosition(stepperC.targetPosition());
  }
  stepperC.run();
  isInit = false;
  if (state != initMode) {
    state = menuMode;
    interruptTriggered = true;
  }
}
