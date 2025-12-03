#ifndef LEDSFAROIS_H
#define LEDSFAROIS_H

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>


#define canalLedBrancoDir 8
#define canalLedBrancoEsq 9

class Led
{
private:
    bool estadoSetaPisca = 0;
    bool estadoLanterna = 0;
    bool estadoFarol = 0;
    bool estadoLed[6];
    int posicao_led = 0;
    Adafruit_MCP23X17 &mcp;

    uint8_t duty = 0;  // 0 a 255
    const uint16_t periodo = 2000; // 500 Hz = 2000 us

    unsigned long lastMicros = 0;
    bool estado = false; // HIGH ou LOW do PWM

public:
    explicit Led(Adafruit_MCP23X17 &mcp_dev);

    void begin();
    uint8_t ligarFarol(uint8_t);
    uint8_t ligarLanterna(uint8_t);
    uint8_t desligarLed(uint8_t);
    // void desligarLanterna(uint8_t);
    void update();
    uint8_t piscarSeta(uint8_t, unsigned long);
    // void desligarSetas(uint8_t);
};
#endif
