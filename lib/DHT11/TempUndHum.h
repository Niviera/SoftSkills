/*
 * Diese Klasse dient dafür um Daten vom dem DHT11 auszulesen.
 * Error-Codes:
 * 5 = fehler beim Sensor auslesen!
 */
#include "DHT.h"
#include <DHT_U.h>

class TempUndHum
{
private:
    int error;
    float temperatur;
    float humidity;

    DHT_Unified *p_dht;

public:
    /* Funktionen */
    bool readSensor();

    /* Getter & Setter */
    float get_Temp();
    float get_Humi();
    int get_Error();

    /* Konstructor und Destructor */
    TempUndHum(int pin);
    ~TempUndHum();
};
