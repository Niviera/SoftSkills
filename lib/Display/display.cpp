#include "display.h"
#include <stdlib.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define MAXCHARPERROW 10

/* Eine Ziffer ist 12 breit*/

// Funktionen:
int Display::displayXCoordinate(int textLength)
{
    return (128 - (12 * textLength)) / 2;
}

void Display::printTempeture(float *ptemperatur, std::string upupperRow)
{
    /* Prepare Output: */
    int lengthUpperRow = 0;
    char *prow1 = (char *)malloc(sizeof(char) * (MAXCHARPERROW));
    createOutputRow(&upupperRow[0], prow1, &lengthUpperRow);
    pDisplayOLED->clearDisplay();
    /* Obere Zeile */
    pDisplayOLED->setCursor(displayXCoordinate(lengthUpperRow), 12);
    pDisplayOLED->printf(&upupperRow[0]);
    /* Untere Zeil: Celsius angabe */
    pDisplayOLED->setCursor(displayXCoordinate(String(*(ptemperatur), 2U).length() + 1), 36);
    pDisplayOLED->printf("%04.2fC", *(ptemperatur));
    pDisplayOLED->display();

    free(prow1);
}

void Display::printHumi(float *ptemperatur, std::string upupperRow)
{
    /* Prepare Output: */
    int lengthUpperRow = 0;
    char *prow1 = (char *)malloc(sizeof(char) * (MAXCHARPERROW));
    createOutputRow(&upupperRow[0], prow1, &lengthUpperRow);
    pDisplayOLED->clearDisplay();
    /* Obere Zeile */
    pDisplayOLED->setCursor(displayXCoordinate(lengthUpperRow), 12);
    pDisplayOLED->printf(&upupperRow[0]);
    /* Untere Zeil: Celsius angabe */
    pDisplayOLED->setCursor(displayXCoordinate(String(*(ptemperatur), 2U).length() + 1), 36);
    pDisplayOLED->printf("%04.2f %", *(ptemperatur));
    pDisplayOLED->display();

    free(prow1);
}

void Display::printCo2(float *co2, std::string upupperRow)
{
    /* Prepare Output: */
    int lengthUpperRow = 0;
    char *prow1 = (char *)malloc(sizeof(char) * (MAXCHARPERROW));
    createOutputRow(&upupperRow[0], prow1, &lengthUpperRow);
    pDisplayOLED->clearDisplay();
    /* Obere Zeile */
    pDisplayOLED->setCursor(displayXCoordinate(lengthUpperRow), 12);
    pDisplayOLED->printf(&upupperRow[0]);
    /* Untere Zeil: Celsius angabe */
    pDisplayOLED->setCursor(displayXCoordinate(String(*(co2), 2U).length() + 1), 36);
    pDisplayOLED->printf("%03.2fppm", *(co2));
    pDisplayOLED->display();

    free(prow1);
}

void Display::createOutputRow(char *pstartMsg, char *poutputMsg, int *plength)
{
    int i = 0;
    while (pstartMsg[i] != '\0' && i < MAXCHARPERROW)
    {
        *(plength) += 1;
        poutputMsg[i] = pstartMsg[i];
        i++;
    }
    poutputMsg[i] = '\0';
}

void Display::writeStatusMSG(std::string msg)
{
    /*
        ToDo: Check how long message is max 11 characters included '\0' per row!
        i use max two rows!
    */
    char *prow1 = (char *)malloc(sizeof(char) * (MAXCHARPERROW + 1));
    char *prow2 = (char *)malloc(sizeof(char) * (MAXCHARPERROW + 1));
    int lengthUpperRow = 0;
    int lengthbottomRow = 0;
    /* Fill row 1:*/
    createOutputRow(&msg[0], prow1, &lengthUpperRow);
    /* Fill row 2:*/
    if (msg.length() > MAXCHARPERROW)
    {
        createOutputRow(&msg[MAXCHARPERROW], prow2, &lengthbottomRow);
    }
    else
    {
        prow2[0] = '\0';
    }

    /* Display Message */
    pDisplayOLED->clearDisplay();
    pDisplayOLED->setCursor(0, 10);
    pDisplayOLED->printf(prow1);
    pDisplayOLED->printf("\n%s", prow2);
    pDisplayOLED->display();

    /* free occupied Memory */
    free(prow1);
    free(prow2);
}

bool Display::connectDisplay()
{
    /*
        Funktion verbindet mit dem Display
        bei erfolg -> true
        bei misserfolg -> false
        begin para: erster intern oder extern Spannungsversorgung, zweiter Adresse des Pannels

    */
    if (!pDisplayOLED->begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        return false;
    }
    return true;
}

// Konstruktor & Destructor
Display::Display(/* args */)
{

    pDisplayOLED = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
    pDisplayOLED->setTextColor(WHITE);
    pDisplayOLED->setTextSize(2);
}

Display::~Display()
{
    free(pDisplayOLED);
}
