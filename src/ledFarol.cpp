#include "ledFarol.h"

Led::Led(Adafruit_MCP23X17 &mcp_dev) : mcp(mcp_dev) {}

void Led::begin()
{
    for (char i = 10; i < 16; i++)
        mcp.pinMode(i, OUTPUT);
}

void Led::update()
{
    for (char i = 0; i < 6; i++)
        mcp.digitalWrite((i + 10), estadoLed[i]);
}

uint8_t Led::ligarFarol(uint8_t posicao)
{
    estadoFarol = 1;

    switch (posicao)
    {
    case 1:
        estadoLed[2] = true;
        estadoLed[1] = true;
        break;

    case 2:
        estadoLed[2] = true;
        estadoLed[1] = false;
        break;

    case 3:
        estadoLed[1] = true;
        estadoLed[2] = false;
        break;

    default:
        break;
    }

    return estadoFarol;
}

uint8_t Led::ligarLanterna(uint8_t posicao)
{
    estadoLanterna = 1;

    switch (posicao)
    {
    case 1:
        estadoLed[4] = true;
        estadoLed[5] = true;
        break;

    case 2:
        estadoLed[4] = true;
        // estadoLed[5] = false;
        break;

    case 3:
        estadoLed[5] = true;
        // estadoLed[4] = false;
        break;

    default:
        break;
    }

    return estadoLanterna;
}

uint8_t Led::desligarLed(uint8_t posicao)
{
    estadoFarol = 0;

    switch (posicao)
    {
    case 1: // farol
        estadoLed[2] = false;
        estadoLed[1] = false;
        break;

    case 2: // seta
        estadoLed[0] = false;
        estadoLed[3] = false;
        break;

    case 3: // lanterna
        estadoLed[4] = false;
        estadoLed[5] = false;
        break;

    default:
        break;
    }

    return estadoFarol;
}

uint8_t Led::piscarSeta(uint8_t posicao, unsigned long intervalo)
{
    estadoSetaPisca = 1;

    static unsigned long ultimoTempo = 0;
    static bool estadoPisca = false;
    unsigned long agora = millis();

    if (agora - ultimoTempo >= intervalo)
    {
        estadoPisca = !estadoPisca;
        ultimoTempo = agora;
    }
    
    switch (posicao)
    {
    case 1:
        estadoLed[0] = estadoPisca;
        estadoLed[3] = estadoPisca;
        break;
    case 2:
        estadoLed[0] = estadoPisca;
        // estadoLed[3] = false;
        break;
    case 3:
        estadoLed[3] = estadoPisca;
        // estadoLed[0] = false;
        break;
    }

    update();
    
    return estadoSetaPisca;
}
