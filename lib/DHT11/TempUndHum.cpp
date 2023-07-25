#include "TempUndHum.h"

/* Funktionen */

bool TempUndHum::readSensor()
{
    sensors_event_t event;

    this->p_dht->temperature().getEvent(&event);
    this->temperatur = event.temperature;

    this->p_dht->humidity().getEvent(&event);
    this->humidity = event.relative_humidity;

    if (isnan(this->humidity) || isnan(this->temperatur))
    {

        this->error = 10;
        return false;
    }

    return true;
}

/* Getter und Setter */
float TempUndHum::get_Temp()
{
    return this->temperatur;
}

float TempUndHum::get_Humi()
{
    return this->humidity;
}

int TempUndHum::get_Error()
{
    return this->error;
}

/* Konstruktoren */
TempUndHum::TempUndHum(int pin)
{
    DHT_Unified dht(pin, DHT11);
    this->p_dht = &dht;
}

/* Destructor */
TempUndHum::~TempUndHum()
{
}