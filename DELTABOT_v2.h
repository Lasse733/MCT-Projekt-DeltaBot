/*
 * Datei: DELTABOT_v2.h
 * Projektname: DELTABOT
 * Beschreibung: Delta-Roboter mit Sauger
 * Autor: Lasse Timmerhinrich & Isa El-Molla 
 * Version: 2.0
 * Datum: 22.01.2025
 */

#include <DeltaKinematics.h>  //https://github.com/tinkersprojects/Delta-Kinematics-Library  //Methode inverse(x0, y0, z0) auf virtual gesetzt (in der Klasse Deltakinematics angepasst)
#include <AccelStepper.h>     //http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "AdvancedDeltaKinematics.h" //selbsterstellte Klasse die von Deltakinematics erbt

//Geschwindigkeiten
#define SpeedInit 400  //Geschwindigkeit bei der Initialisierung Achtung: zu hohe geschwindigkeiten könnten zu schäden führen
#define Speed 2200     //Geschwindigket
#define Acceleration 2200

//Delta-Roboter-Spezifikationen
#define armLength 100.0   // (rf) Länge der Arme
#define rodLength 230.0   // (re) Länge der Schubstangen
#define bassTri 45.0      // (e) Seitenlänge der Basisplattform
#define platformTri 55.0  // (f) Seitenlänge der beweglichen Plattform \
                          // (b) 354 mm Abstand von Motorwelle zum Boden (für online-Rechner relevant) \
                          // https://www.marginallyclever.com/other/samples/fk-ik-test.html

//Kinematic Objekt erstellen
AdvancedDeltaKinematics delta(armLength, rodLength, bassTri, platformTri);

//Microstepping
#define Microstepping 16                              // 1 1/2 1/4 1/8 1/16 1/32 1/64 1/128 Microstepping // DRV8825 kann nur bis 1/32
#define StepsPerRevolution 200                        //Eine ganze Umdrehung = 200 steps (ohne Microstepping)
#define Multiplier StepsPerRevolution* Microstepping  //Multiplier um Steps zu berechnen

//Motor A
#define StepPinA 54
#define DirPinA 55
#define enablePinA 38

//Motor B
#define StepPinB 60
#define DirPinB 61
#define enablePinB 56

//Motor C
#define StepPinC 46
#define DirPinC 48
#define enablePinC 62

//Endlagenschalter
#define limitSwitchA 2    //Interrupt Pin auf Ramps 1.6 X+
#define limitSwitchB 3    //Interrupt Pin auf Ramps 1.6 X-
#define limitSwitchC 19   //Interrupt Pin auf Ramps 1.6 Z+

//Lüfter
#define fanPin 9

//Sauger
#define vacuumPin 10

//Motor Objekte erstellen
AccelStepper stepperA(AccelStepper::DRIVER, StepPinA, DirPinA);
AccelStepper stepperB(AccelStepper::DRIVER, StepPinB, DirPinB);
AccelStepper stepperC(AccelStepper::DRIVER, StepPinC, DirPinC);

//Globale Variablen
volatile uint8_t state = 0;  //Zustandsautomat
bool vacuumOn = false;
bool homingDone = false;
bool downMoveDone = false;
bool isInit = false;
volatile bool interruptTriggered = false;   // WENN unerwartet ein Endlagenschalter betätigt wurde

//Zuordnung der States in Klartext
enum { menuMode,
       initMode,
       handMode,
       circleMode };

//funktionen
void initialization();
void menu();
void manuel();
void circleMotion();

//für Interrupt
void stopMotorA();
void stopMotorB();
void stopMotorC();