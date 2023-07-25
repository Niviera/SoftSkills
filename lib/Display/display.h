#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string>

class Display
{
private:
    /* data */
    Adafruit_SSD1306 *pDisplayOLED;
    void createOutputRow(char *pstartMsg, char *poutputMsg, int *plength);

public:
    // Funktionen:
    void printTempeture(float *ptemperatur, std::string upupperRow);
    void writeStatusMSG(std::string message);
    void printCo2(float *co2, std::string upupperRow);
    void printHumi(float *ptemperatur, std::string upupperRow);
    bool connectDisplay();
    int displayXCoordinate(int textLNG);
    // Konstruktoren & Destructoren
    Display(/* args */);
    ~Display();
};
