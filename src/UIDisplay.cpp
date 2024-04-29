#include "UIDisplay.h"
#include "Particle.h"
#include "icons.h"
#include "Org_01.h"

char dispTimes[16];

UIDisplay::UIDisplay() 
    : _display(64, 128, &Wire), 
      lampElement(Bitmaps::lamp, 0, 0), 
      blenderElement(Bitmaps::blender, 0, 28), 
      fridgeElement(Bitmaps::fridge, 0, 56), 
      fanElement(Bitmaps::fan, 0, 84) {}

void UIDisplay::begin() {
    if(!_display.begin(0x3C, true)) {
        Log.error("Could not initialize OLED!");
        while (1) {Particle.process();}
    }
    Log.info("OLED begun");
    _display.clearDisplay();
    drawAll(3,4,5, false);
}

void UIDisplay::drawAll(int dsp_ms, int inf_ms, int ano_ms, bool anomaly) {
    sprintf(dispTimes, "D: %i I: %i", dsp_ms, inf_ms);
    lampElement.draw(_display);
    blenderElement.draw(_display);
    fridgeElement.draw(_display);
    fanElement.draw(_display);
    _display.setFont(&Org_01);
    _display.setTextSize(1);
    _display.setTextColor(1);
    _display.setCursor(0, 118);
    _display.println(dispTimes);
    sprintf(dispTimes, "  ANOM: %i", ano_ms);
    _display.setCursor(0, 126);
    _display.println(dispTimes);
    if(anomaly)
    {   _display.clearDisplay();
        _display.drawBitmap(10,20, Bitmaps::anomaly,48,48,1);
        _display.setFont(&Org_01);
        _display.setTextSize(1);
        _display.setTextColor(1);
        _display.setCursor(0, 88);
        _display.println("ANOMALY");
        _display.setCursor(0, 98);
        _display.println("DETECTED!");
    }
    _display.display();
}

void UIDisplay::setLampState(ElementState state) {
    lampElement.setState(state);
    lampElement.draw(_display);
}

void UIDisplay::setBlenderState(ElementState state) {
    blenderElement.setState(state);
    blenderElement.draw(_display);
}

void UIDisplay::setFridgeState(ElementState state) {
    fridgeElement.setState(state);
    fridgeElement.draw(_display);
}

void UIDisplay::setFanState(ElementState state) {
    fanElement.setState(state);
    fanElement.draw(_display);
}

void UIDisplay::debugADC(int32_t voltageRaw, int32_t currentRaw){
    _display.clearDisplay();
    _display.display();
    _display.setFont(&Org_01);
    _display.setTextSize(2);
    _display.setTextColor(1);
    _display.setCursor(0,12);
    _display.println("volt");
    _display.println(voltageRaw);
    _display.println("amp");
    _display.println(currentRaw);
    _display.display();
}

void UIDisplay::ingestStatus(char* state, char* label){
    _display.clearDisplay();
    _display.display();
    _display.setFont(&Org_01);
    _display.setTextSize(1);
    _display.setTextColor(1);
    _display.setCursor(0,12);
    _display.println("State");
    _display.println(state);
    _display.println("Label");
    _display.println(label);
    _display.display();
}

void UIDisplay::cloudIsConnected(void){
    _display.clearDisplay();
    _display.display();
    _display.setFont(&Org_01);
    _display.setTextSize(1);
    _display.setTextColor(1);
    _display.setCursor(0,12);
    _display.println("Connected!");
    _display.display();
}

void UIDisplay::anomalyAlert(void){
    _display.clearDisplay();
    _display.display();
    _display.drawBitmap(12,12, Bitmaps::anomaly,48,48,1);
    _display.display();
}

void UIDisplay::plugMeIn(void){
    _display.clearDisplay();
    _display.display();
    _display.drawBitmap(12,12, Bitmaps::plug,48,48,1);
    _display.display();
}