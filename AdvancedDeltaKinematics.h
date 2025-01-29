/*
 * Datei: AdvancedDeltaKinematics.h
 * Projektname: DELTABOT
 * Beschreibung: Delta-Roboter mit Sauger
 * Autor: Lasse Timmerhinrich & Isa El-Molla 
 * Version: 2.0
 * Datum: 22.01.2025
 */

class AdvancedDeltaKinematics : public DeltaKinematics {
public:
    // Konstruktor der abgeleiteten Klasse
    AdvancedDeltaKinematics(double _ArmLength, double _RodLength, double _BassTri, double _PlatformTri)
        : DeltaKinematics(_ArmLength, _RodLength, _BassTri, _PlatformTri) {
        // Zusätzliche Initialisierungen falls nötig
    }
    #define limit_error -3

    // Zusätzliche Grenzen definieren
    double maxX = 80.0;
    double maxY = 80.0;
    double maxZ = -175.0;
    double minX = -80.0;
    double minY = -80.0;
    double minZ = -329.0;

    // Prüfen, ob die Position innerhalb der Grenzen liegt
    bool isWithinLimits(double x0, double y0, double z0) {
        return (x0 >= minX && x0 <= maxX &&
                y0 >= minY && y0 <= maxY &&
                z0 >= minZ && z0 <= maxZ);
    }

    // Überschreiben der inverse Methode mit Grenzprüfung
    int inverse(double x0, double y0, double z0) override {
        // Überprüfung der Grenzen
        if (!isWithinLimits(x0, y0, z0)) {
            Serial.println("Nicht im definierten Bereich (Grenzen in der Klasse AdvancedDeltaKinematics)");
            return limit_error; // Rückgabe eines speziellen Fehlercodes
        }
        
        // Wenn die Position innerhalb der Grenzen liegt, normale inverse Kinematik anwenden
        return DeltaKinematics::inverse(x0, y0, z0);
    }
};