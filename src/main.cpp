#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Adafruit_MCP23X17.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_PN532.h>
#include <VL53L0X.h>
#include <TFT_eSPI.h>
#include <Bounce2.h>
#include "carrinho.h"
#include "ledFarol.h"
#include "internet.h"
#include "certificados.h"
#include "Motores.h"
#include "imagem.h"

#define AMBOS 1    // Liga os dois leds
#define DIREITA 2  // Liga os led direita
#define ESQUERDA 3 // Liga os led esquerda

#define ENCODER_A 38
#define ENCODER_B 37
#define ENCODER_BTN 19

// 1 desliga farol
// 2 desliga seta
// 3 desliga lanterna

const int mqtt_port = 8883;
const char *mqtt_id = "Panamaaa_esp32";
// const char *mqtt_SUB_dados = "carrinho/dados";
const char *mqtt_SUB_controle = "carrinho/controle";
const char *mqtt_SUB_app = "carrinho/app";
const char *mqtt_SUB_dash = "carrinho/dash";
const char *mqtt_PUB = "carrinho/dados";

// MCP inicializado fora da classe e passado por referência
WiFiClientSecure espClient;
PubSubClient mqtt(espClient);
Adafruit_MCP23X17 mcp;
Adafruit_ADS1115 ads;
Adafruit_PN532 nfc(-1, -1);
Carrinho carrinho(mcp);
Led leds(mcp);
Motores motor;
Preferences prefs;
VL53L0X sensor;
TFT_eSPI tft;
Bounce Encoder_boot = Bounce();

bool atualizacao = 0;
bool atualizacaoApp = 0;
bool atualizacaoDash = 0;

bool carrinhoAtivo = false;

int leitura_motor00 = 0;
int leitura_motor01 = 0;
int leitura_motor02 = 0;
int leitura_motor03 = 0;
int tempMotor00 = 0;
int tempMotor01 = 0;
int tempMotor02 = 0;
int tempMotor03 = 0;

bool estadoFarol = 0;
bool estadoSeta = 0;
bool estadoLanterna = 0;
int posicaoFarol = 0;
int posicaoSeta = 0;
int posicaoLanterna = 0;
int frequenciaPisca = 500;

bool botaoA = 0;
bool botaoAntesA = 0;
bool botaoB = 0;
bool botaoAntesB = 0;
bool botaoC = 0;
bool botaoAntesC = 0;
bool botaoD = 0;
bool botaoAntesD = 0;
bool botaoE = 0;
bool botaoAntesE = 0;
bool botaoF = 0;
bool botaoAntesF = 0;
bool botaoK = 0;
bool botaoAntesK = 0;

int analogX = 0;
int analogY = 0;

int alterarFormato = 0;
int velocidadeCarrinho = 30;
int estadoTick = 2;

bool emCorrida = false;
bool emAtrasoPartida = false;
bool parar_Carrinho = false;

int distancia = 0;

uint8_t estadoFarolApp = 0;
uint8_t estado_LanternaT_esq_dash = 0;
uint8_t estado_LanternaT_dir_dash = 0;
uint8_t estadoFaroDirlDash = 0;
uint8_t estadoSetaApp = 0;
uint8_t estadoFarolDash = 0;
uint8_t estadoSetaDash = 0;

int estadoIntensidade = 0;

float ultimoErroValido = 0.0f;
static constexpr int ERRO_SEM_LINHA = INT16_MAX;
bool INVERTER_OMEGA = true;

bool atualizacaoDisplay = 0;
bool estadoModo = 0;
bool telaCreditos = 0; // ← novo: indica se está na tela de créditos
bool modoEasterEgg = 0;

bool esperandoEaster = false;
unsigned long tempoPressionado = 0;
unsigned long tempo = 4000;
const unsigned long TEMPO_EASTER = 3000;

volatile int movimento = 1; // começa na primeira opção
volatile int acumulador = 0;
volatile bool mudou = false;
volatile uint8_t ultimoEstado = 0;

unsigned long ultimoTempo = 0;
const unsigned long intervalo = 3000;
unsigned long ultimoCheckCartao = 0; // controle do tempo de verificação NFC
const unsigned long intervaloCartao = 500;

enum EstadoCarrinho
{
  NORMAL,
  PARANDO,
  GIRANDO
};

EstadoCarrinho estadoAtual = NORMAL;
unsigned long tempoAcao01 = 2000;
unsigned long tempoAcao02 = 4600;

void conectaMQTT();
void Callback(char *, byte *, unsigned int);
void enviar_mqtt();
void modoSeguidorLinha();
void joystick();
void pararCarrinho();
void displayCarrinho();
void desenhaMenuBase();
void comandosApp();

void IRAM_ATTR encoderISR()
{
  uint8_t estadoAtual = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);

  static const int tabela[4][4] = {
      {0, +1, -1, 0},
      {-1, 0, 0, +1},
      {+1, 0, 0, -1},
      {0, -1, +1, 0}};

  acumulador += tabela[ultimoEstado][estadoAtual];
  ultimoEstado = estadoAtual;

  if (acumulador >= 4)
  {
    movimento = 0;
    mudou = true;
    acumulador = 0;
  }
  else if (acumulador <= -4)
  {
    movimento = 1;
    mudou = true;
    acumulador = 0;
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(8, 9, 400000);
  mcp.begin_I2C(0x20, &Wire);
  leds.begin();
  ads.begin();
  motor.init();
  nfc.begin();
  tft.init();

  conectaWiFi();

  espClient.setCACert(AWS_ROOT_CA);
  espClient.setCertificate(AWS_CERT);
  espClient.setPrivateKey(AWS_KEY);
  mqtt.setBufferSize(2048);
  mqtt.setServer(AWS_BROKER, mqtt_port);
  mqtt.setCallback(Callback);

  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);

  // Inicialização do MCP
  for (uint8_t i = 0; i < 8; i++)
    mcp.pinMode(i, INPUT); // GPIOA0..7 sensores

  mcp.pinMode(8, OUTPUT);
  mcp.digitalWrite(8, HIGH); // liga o sensor de linha

  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);

  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("status: Conectado!!");
  tft.setCursor(10, 20);
  tft.drawRect(0, 50, 1000, 0, TFT_SKYBLUE);
  tft.setCursor(10, 60);
  tft.print(">");
  tft.setCursor(30, 60);
  tft.print("Modo Manual");
  tft.setCursor(30, 85);
  tft.print("Creditos");

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  Encoder_boot.attach(ENCODER_BTN, INPUT_PULLUP);

  ultimoEstado = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

  displayCarrinho();

  // Inicializa os valores neutros do joystick
  prefs.begin("workSpace", true);
  alterarFormato = prefs.getInt("estado_Formato_Salvo", 0);
  estadoSeta = prefs.getBool("estado_Seta_Salva", 0);
  posicaoSeta = prefs.getInt("posicao_Seta_Salva", 0);
  estadoModo = prefs.getBool("estado_Modo_Salvo", 0);
  telaCreditos = prefs.getBool("estado_Creditos_Salvo", 0);
  prefs.end();
  analogX = 9;
  analogY = 9;
  botaoA = botaoB = botaoC = botaoD = botaoE = botaoF = botaoK = 0;

  carrinho.begin();

  while (!sensor.init(0x29))
  {
    unsigned long agora = millis();
    if (agora - ultimoTempo >= intervalo)
    {
      ultimoTempo = agora;
      Serial.println("Tentando reconectar VL53L0X...");
    }
  }

  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(25);
  sensor.setTimeout(100);

  Serial.println("Sistema iniciado, aproxime o cartão para ativar o carrinho.");
}

void loop()
{

  checkWiFi();

  if (!mqtt.connected())
    conectaMQTT();

  mqtt.loop();

  unsigned long agora = millis();

  // 🔹 Se o carrinho não estiver ativo, tenta detectar o cartão
  if (!carrinhoAtivo)
  {
    bool newCard = nfc.inListPassiveTarget();
    if (newCard)
    {
      uint8_t uid[7];
      uint8_t uidLength;

      if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength))
      {
        Serial.print("Cartão detectado! UID: ");
        for (uint8_t i = 0; i < uidLength; i++)
        {
          Serial.print(uid[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        carrinhoAtivo = true;
        ultimoCheckCartao = agora; // marca tempo da última leitura do cartão
        Serial.println("Carrinho ativado!");
      }
    }

    // enquanto não tiver cartão, o carrinho fica parado
    pararCarrinho();
    return;
  }

  // 🔹 A cada 5 segundos, verifica se o cartão ainda está por perto
  if (agora - ultimoCheckCartao >= intervaloCartao)
  {
    ultimoCheckCartao = agora;

    bool cartaoAindaPresente = nfc.inListPassiveTarget();
    if (!cartaoAindaPresente)
    {
      Serial.println("Cartão ausente! Parando o carrinho...");
      pararCarrinho();
      carrinhoAtivo = false;
      return;
    }
  }

  enviar_mqtt();
  Encoder_boot.update();
  leds.update();

  // 🔹 Se o carrinho estiver ativo, lê o sensor de distância normalmente
  uint16_t distancia = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred())
  {
    Serial.println("Timeout na leitura do sensor VL53L0X");
    return;
  }

  // Serial.printf("Distancia: %d mm\n", distancia);

  prefs.begin("workSpace", false);

  if (mudou)
  {
    if (!telaCreditos)
    {
      atualizacaoDisplay = 1;
      mudou = false;
    }

    else
      movimento = 0;
  }

  if (Encoder_boot.fell())
  {
    if (movimento == 1)
    {
      // Troca o modo apenas se a seta estiver no "Modo"
      atualizacaoDisplay = 1;
      estadoModo = !estadoModo;
      Serial.printf("encoder_Boot = %d\n", estadoModo);
      prefs.putBool("estadfo_Modo_Salvo", estadoModo);
    }
    else if (movimento == 0)
    {
      // Entra na tela de Créditos
      atualizacaoDisplay = 1;
      telaCreditos = !telaCreditos;
      Serial.printf("TelaCreditos = %d\n", telaCreditos);
      prefs.putBool("estado_Creditos_Salvo", telaCreditos);
    }
  }

  displayCarrinho();

  joystick();

  if (estadoModo)
  {
    carrinho.tick(estadoTick);

    //     switch (estadoAtual)
    //     {
    //       case NORMAL:
    //       if (distancia <= 200)
    //       {
    //         Serial.println("Obstáculo detectado! Parando...");

    //     pararCarrinho();
    //     estadoAtual = PARANDO;
    //   }
    //   break;

    // case PARANDO:
    //   if (millis() - tempoAcao01 >= 1000) // espera 1s parado
    //   {
    //     Serial.println("Girando meia volta...");
    //     motor.girar_direita(40);  // velocidade do giro
    //     estadoAtual = GIRANDO;
    //   }
    //   break;

    // case GIRANDO:
    //   if (millis() - tempoAcao02 >= 2300)  // TEMPO PARA MEIA VOLTA (ajuste fino)
    //   {
    //     Serial.println("Meia volta concluída!");
    //     motor.parar();
    //     estadoAtual = NORMAL;
    //   }
    //   break;
    // }
  }

  if (estadoFarol)
  {
    leds.ligarFarol(AMBOS);
    leds.ligarLanterna(AMBOS);
  }

  else
  {
    leds.desligarLed(1);
    leds.desligarLed(3);
  }

  if (estadoSeta)
  {
    switch (posicaoSeta)
    {
    case 1:
      leds.piscarSeta(AMBOS, frequenciaPisca);
      break;
    case 2:
      leds.piscarSeta(DIREITA, frequenciaPisca);
      break;
    case 3:
      leds.piscarSeta(ESQUERDA, frequenciaPisca);
      break;

    default:
      break;
    }
  }

  else
    leds.desligarLed(2);

  comandosApp();
}

void Callback(char *topic, byte *payload, unsigned int length)
{
  String msg((char *)payload, length);
  Serial.printf("Mensagem recebida (topico: [%s]): %s\n\r", topic, msg.c_str());

  Serial.println(msg);
  msg.trim();

  JsonDocument doc;
  DeserializationError erro = deserializeJson(doc, msg);

  if (erro)
    Serial.printf("Erro %s no formato json", erro.c_str());

  else
  {

    // carrinho/joystick
    if (!doc["botao0"].isNull())
    {
      botaoA = doc["botao0"];
      atualizacao = 1;
    }

    if (!doc["botao1"].isNull())
    {
      botaoB = doc["botao1"];
      atualizacao = 1;
    }

    if (!doc["botao2"].isNull())
    {
      botaoC = doc["botao2"];
      atualizacao = 1;
    }

    if (!doc["botao3"].isNull())
    {
      botaoD = doc["botao3"];
      atualizacao = 1;
    }

    if (!doc["botao4"].isNull())
    {
      botaoE = doc["botao4"];
      atualizacao = 1;
    }

    if (!doc["botao5"].isNull())
    {
      botaoF = doc["botao5"];
      atualizacao = 1;
    }

    if (!doc["botao6"].isNull())
    {
      botaoK = doc["botao6"];
      atualizacao = 1;
    }

    if (!doc["AnalogX"].isNull())
    {
      analogX = doc["AnalogX"];
      atualizacao = 1;
    }
    if (!doc["AnalogY"].isNull())
    {
      analogY = doc["AnalogY"];
      atualizacao = 1;
    }

    // carrinho/app
    if (!doc["estado_FarolDir_app"].isNull())
    {
      estadoFarolApp = doc["estado_FarolDir_app"];
      atualizacaoApp = 1;
    }

    if (!doc["estado_Seta_app"].isNull())
    {
      estadoSetaApp = doc["estado_Seta_app"];
      atualizacaoApp = 1;
    }

    if (!doc["intensidade"].isNull())
    {
      estadoIntensidade = doc["intensidade"];
      atualizacaoApp = 1;
    }

    // carrinho/dashboard
    if (!doc["estado_LanternaT_esq_dash"].isNull())
    {
      estado_LanternaT_esq_dash = doc["estado_LanternaT_esq_dash"];
      atualizacaoDash = 1;
    }
    if (!doc["estado_LanternaT_dir_dash"].isNull())
    {
      estado_LanternaT_dir_dash = doc["estado_LanternaT_dir_dash"];
      atualizacaoDash = 1;
    }
  }
}

void conectaMQTT()
{
  while (!mqtt.connected())
  {
    Serial.print("Conectando ao AWS Iot Core ...");

    if (mqtt.connect(mqtt_id))
    {
      Serial.println("conectado.");
      mqtt.subscribe(mqtt_SUB_app);
      mqtt.subscribe(mqtt_SUB_dash);
      mqtt.subscribe(mqtt_SUB_controle);

      tft.setTextSize(2);
      tft.setCursor(10, 10);
      tft.print("status: Conectado!!");
    }
    else
    {
      Serial.printf("falhou (%d). Tentando novamente em 5s \n\r", mqtt.state());
      tft.setCursor(10, 10);
      tft.print("status: ");
      tft.println("reconectando...");
      delay(5000);
    }
  }
}

void enviar_mqtt()
{
  leitura_motor00 = ads.readADC_SingleEnded(0); // de 0 atè 3
  leitura_motor01 = ads.readADC_SingleEnded(1); // de 0 atè 3
  leitura_motor02 = ads.readADC_SingleEnded(2); // de 0 atè 3
  leitura_motor03 = ads.readADC_SingleEnded(3); // de 0 atè 3
  tempMotor00 = leitura_motor00 * 0.01875;      // CONVERTER  EM TEMPERATURA DO LM35
  tempMotor01 = leitura_motor01 * 0.01875;      // CONVERTER  EM TEMPERATURA DO LM35
  tempMotor02 = leitura_motor02 * 0.01875;      // CONVERTER  EM TEMPERATURA DO LM35
  tempMotor03 = leitura_motor03 * 0.01875;      // CONVERTER  EM TEMPERATURA DO LM35

  JsonDocument doc;

  doc["estadoFormato"] = alterarFormato;
  doc["estado_Seta"] = estadoSeta;
  doc["estado_Farol"] = estadoFarol;
  doc["estado_Lanterna"] = estadoLanterna;
  doc["velocidade_carrinho"] = velocidadeCarrinho;
  doc["sensor_distancia"] = distancia;
  doc["temperatura_Motor00"] = tempMotor00;
  doc["temperatura_Motor01"] = tempMotor01;
  doc["temperatura_Motor02"] = tempMotor02;
  doc["temperatura_Motor03"] = tempMotor03;

  static unsigned long tempoAntes = 0;
  unsigned long tempoAgora = millis();
  if (tempoAgora - tempoAntes > 2000)
  {
    String msg;
    serializeJson(doc, msg);

    mqtt.setBufferSize(msg.length() + 100);
    mqtt.publish(mqtt_PUB, msg.c_str());
    if (mqtt.publish(mqtt_PUB, msg.c_str()))
    {
      Serial.print("📤 MQTT enviado: ");
      Serial.println(msg);
    }
    else
    {
      Serial.println("❌ Falha ao publicar MQTT");
    }
    tempoAntes = tempoAgora;
  }
}

void joystick()
{
  if (atualizacao)
  {
    prefs.begin("workSpace", false);

    if (botaoA && !botaoAntesA)
    {
      prefs.putInt("estado_Formato_Salvo", ++alterarFormato);

      if (alterarFormato >= 2)
        alterarFormato = 2;
    }

    else if (botaoC && !botaoAntesC)
    {
      prefs.putInt("estado_Formato_Salvo", --alterarFormato);

      if (alterarFormato <= 0)
        alterarFormato = 0;
    }

    else if (botaoB && !botaoAntesB)
    {
      estadoSeta = !estadoSeta;
      posicaoSeta = 2;

      Serial.printf("estadoSeta = %d\t", estadoSeta);
      Serial.printf("posicaoSeta = %d\n", posicaoSeta);
      // prefs.putBool("estado_Seta_Salvo", estadoSeta);
      // prefs.putInt("posicao_Seta_Salvo", posicaoSeta);
    }

    else if (botaoD && !botaoAntesD)
    {
      estadoSeta = !estadoSeta;
      posicaoSeta = 3;

      Serial.printf("estadoSeta = %d\t", estadoSeta);
      Serial.printf("posicaoSeta = %d\n", posicaoSeta);
      // prefs.putBool("estado_Seta_Salvo", estadoSeta);
      // prefs.putInt("posicao_Seta_Salvo", posicaoSeta);
    }

    else if (botaoD && !botaoAntesD)
    {
      estadoSeta = !estadoSeta;
      posicaoSeta = 2;

      Serial.printf("estadoSeta = %d\t", estadoSeta);
      Serial.printf("posicaoSeta = %d\n", posicaoSeta);
      // prefs.putBool("estado_Seta_Salvo", estadoSeta);
      // prefs.putInt("posicao_Seta_Salvo", posicaoSeta);
    }

    else if (botaoE && !botaoAntesE)
    {
      estadoModo = !estadoModo;
      atualizacaoDisplay = 1;
      Serial.printf("Modo = %s\n", estadoModo ? "Auto" : "Manual");
      prefs.putBool("estado_Modo_Salvo", estadoModo);
    }

    if (botaoF && !botaoAntesF)
    {
      estadoFarol = !estadoFarol;
      estadoLanterna = !estadoLanterna;
      Serial.printf("estadoFarol = %d\t", estadoFarol);
      Serial.printf("estadoLanterna = %d\n", estadoLanterna);
    }

    if (botaoK) // botão está pressionado
    {
      if (!esperandoEaster)
      {
        // começou a segurar agora
        esperandoEaster = true;
        tempoPressionado = millis();
      }
      else if (millis() - tempoPressionado >= TEMPO_EASTER)
      {
        // segurou 3 segundos → ativa o easter egg
        modoEasterEgg = true; // ou toggle: = !modoEasterEgg;
        atualizacaoDisplay = 1;
      }
    }
    // else
    // {
    //   // botão solto → reseta a contagem
    //   esperandoEaster = false;
    // }

    botaoAntesA = botaoA;
    botaoAntesB = botaoB;
    botaoAntesC = botaoC;
    botaoAntesD = botaoD;
    botaoAntesE = botaoE;
    botaoAntesF = botaoF;
    // botaoAntesK = botaoK;

    prefs.end();

    // if (estadoSeta)
    // {
    //   switch (posicaoSeta)
    //   {
    //   case 1:
    //     leds.piscarSeta(AMBOS, frequenciaPisca);
    //     break;
    //   case 2:
    //     leds.piscarSeta(DIREITA, frequenciaPisca);
    //     break;
    //   case 3:
    //     leds.piscarSeta(ESQUERDA, frequenciaPisca);
    //     break;

    //   default:
    //     break;
    //   }
    //}

    // if (estadoFarol)
    // {
    //   leds.ligarFarol(AMBOS);
    //   leds.ligarLanterna(AMBOS);
    // }

    // else
    //   leds.desligarLed(2);

    // Serial.printf("Formato = %d\n", alterarFormato);

    switch (alterarFormato)
    {
    case 0: // Formato Padrão
      if (analogX == 9 && analogY > 15)
        motor.avancar(velocidadeCarrinho);

      else if (analogX == 9 && analogY < 5)
      {
        motor.para_traz(velocidadeCarrinho);
        leds.piscarSeta(AMBOS, 1000);
      }

      else if (analogX > 15 && analogY == 9)
        motor.esquerda(velocidadeCarrinho);

      else if (analogX < 5 && analogY == 9)
        motor.direita(velocidadeCarrinho);

      else if (analogX > 15 && analogY > 15)
        motor.avancar_esquerda(velocidadeCarrinho);

      else if (analogX < 5 && analogY > 15)
        motor.avancar_direita(velocidadeCarrinho);

      else if (analogX > 15 && analogY < 5)
      {
        motor.para_traz_esquerda(velocidadeCarrinho);
        leds.piscarSeta(AMBOS, 1000);
      }

      else if (analogX < 5 && analogY < 5)
      {
        motor.para_traz_direita(velocidadeCarrinho);
        leds.piscarSeta(AMBOS, 1000);
      }

      else
        motor.parar();
      break;

    case 1: // Formato Circular
      if (analogX > 15 && analogY == 9)
        motor.girar_direita(velocidadeCarrinho);

      else if (analogX < 5 && analogY == 9)
        motor.girar_esquerda(velocidadeCarrinho);

      else if (analogX > 15 && analogY > 15)
        motor.curva_direita_frente(velocidadeCarrinho);

      else if (analogX < 5 && analogY > 15)
        motor.curva_esquerda_frente(velocidadeCarrinho);

      else if (analogX > 15 && analogY < 5)
      {
        motor.curva_direita_traz(velocidadeCarrinho);
        leds.piscarSeta(AMBOS, 1000);
      }

      else if (analogX < 5 && analogY < 5)
      {
        motor.curva_esquerda_traz(velocidadeCarrinho);
        leds.piscarSeta(AMBOS, 1000);
      }

      else
        motor.parar();
      break;

    case 2: // Formato Arco
      if (analogX > 15 && analogY > 15)
        motor.arco_lateral_direita_frente(velocidadeCarrinho);

      else if (analogX < 5 && analogY > 15)
        motor.arco_lateral_esquerda_frente(velocidadeCarrinho);

      else if (analogX > 15 && analogY < 5)
        motor.arco_lateral_esquerda_traz(velocidadeCarrinho);

      else if (analogX < 5 && analogY < 5)
        motor.arco_lateral_direita_traz(velocidadeCarrinho);

      else
        motor.parar();
      break;

    default:
      break;
    }

    prefs.end();
    atualizacao = 0;
  }
}

void pararCarrinho()
{
  parar_Carrinho = true;
  emCorrida = false;
  emAtrasoPartida = false;
  carrinho.controlarRodas(0.0f, 0.0f, 0.0f);
  Serial.println(">> PAROU (botao)");
}

void desenhaMenuBase()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("status: Conectado!!");
  tft.setCursor(10, 20);
  tft.drawRect(0, 50, 1000, 0, TFT_SKYBLUE);
  tft.setCursor(30, 60);
  tft.print("Modo Manual");
  tft.setCursor(30, 85);
  tft.print("Creditos");
}

void displayCarrinho()
{
  if (atualizacaoDisplay)
  {

    desenhaMenuBase();
    tft.fillRect(0, 50, 240, 200, TFT_BLACK);

    // Desenha a seta na opção selecionada
    if (movimento == 1)
    {
      tft.setCursor(10, 60);
      tft.print(">");
    }
    else
    {
      tft.setCursor(10, 85);
      tft.print(">");
      modoEasterEgg = false;
    }

    // Primeira opção: muda o texto conforme o modo
    tft.setCursor(30, 60);
    if (estadoModo)
      tft.print("Modo Auto");
    else
      tft.print("Modo Manual");

    // Segunda opção
    tft.setCursor(30, 85);
    tft.print("Creditos");

    if (telaCreditos)
    {
      tft.pushImage(0, 0, 240, 240, grupo);
      tft.setCursor(10, 40);
      tft.setTextColor(TFT_BLACK);
      tft.print("> Voltar");
      Serial.println("Entrou nos Créditos");
    }

    if (modoEasterEgg)
      tft.pushImage(0, 0, 240, 240, easter_egg);

    atualizacaoDisplay = 0;
  }
}

void comandosApp()
{
  if (atualizacaoApp)
  {
    if (estadoSetaApp > 0)
      leds.piscarSeta(estadoSetaApp, frequenciaPisca);

    else
      leds.desligarLed(2);

    if (estadoFarolApp > 0)
    {
      leds.ligarFarol(estadoFarolApp);
      leds.setIntensidade(estadoIntensidade);
    }
  }

  if (atualizacaoDash)
  {
    if (estado_LanternaT_esq_dash)
      leds.ligarLanterna(2);

    else
      leds.desligarLed(3);

    if (estado_LanternaT_dir_dash)
      leds.ligarLanterna(3);

    else
      leds.desligarLed(3);
  }

  else
    return;

  atualizacaoApp = 0;
  atualizacaoDash = 0;
}