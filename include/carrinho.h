#pragma once
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

class Carrinho {
public:
  // Construtor recebe um MCP23X17 já inicializado fora da classe
  explicit Carrinho(Adafruit_MCP23X17& mcp_dev);

  // Ciclo de vida
  void begin();
    void tick(int estadoTick);

  // APIs públicas para os alunos
  void entrarCalibracao();                 // força o modo de calibração
  void iniciarSeguirLinha();               // força entrar em seguir linha
  void seguirLinhaStep(float kp, float ki, float kd, float vyPercent);                  // executa um passo de seguidor com PID
  uint8_t lerLinhaMascara();               // lê A0..A7 do MCP
  float lerErro();                         // lê máscara e retorna erro
  float calcularErroMascara(uint8_t m);    // calcula o erro a partir da máscara
  void controlarRodas(float vy, float vx, float omega); // percentuais -100..+100
    // pid
  float pidAtualizar(float erro, float dt, float kp, float ki, float kd);
    float calcularDt();

    
private:
  // estados
  enum Estado : uint8_t { CALIBRACAO = 0, ESPERANDO_LARGADA = 1, CORRIDA = 2 };

  // pinos motores
  static constexpr int pinM0Dir = 18;
  static constexpr int pinM0Esq = 3;
  static constexpr int pinM1Dir = 10;
  static constexpr int pinM1Esq = 46;
  static constexpr int pinM2Dir = 13;
  static constexpr int pinM2Esq = 14;
  static constexpr int pinM3Dir = 12;
  static constexpr int pinM3Esq = 11;

  // PWM
  static constexpr uint32_t Freq_PWM = 20000;
  static constexpr uint8_t  Resol_PWM = 8;

  // seguidor
  static constexpr bool PRETO_BIT_1 = true; // se seu sensor ativa em baixo, mude para false
  static constexpr int  ERRO_SEM_LINHA = INT16_MAX;

  // ajustes
  bool INVERTER_OMEGA = true;
  bool invertMotor[4] = {false,false,false,false};

  // estado e tempo
  Estado   estado = CALIBRACAO;
  uint32_t tPrevMicros = 0;
  uint32_t tPrevLog    = 0;
  uint32_t LOG_MS      = 100;
  bool     LOG_ATIVO   = true;
 
  // buffers
  char  cmdBuf[48] = {};
  float ultimoErroValido = 0.0f;

  // PID
  float kp = 6.0f, ki = 0.6f, kd = 0.6f;
  float vyPercent = 20.0f;
  float omegaMax  = 40.0f;
  float erroAnterior = 0.0f;
  float integralAcumulada = 0.0f;

  // LUTs
  uint8_t dutyLUT[101] = {};
  uint8_t dutyAtual[4][2] = {};
  int8_t  erroLUT[256] = {};

  // mapeamentos
  uint8_t chMotor[4][2] = {{0,1},{2,3},{4,5},{6,7}};
  uint8_t pinMotor[4][2] = {
    {pinM0Esq, pinM0Dir},
    {pinM1Esq, pinM1Dir},
    {pinM2Esq, pinM2Dir},
    {pinM3Esq, pinM3Dir}
  };

  // referência ao MCP
  Adafruit_MCP23X17& mcp;

  // auxiliares gerais
  void printHelp();
  void processaSerial();
  void logStatus(float erro, float omega);
  static float clampDt(float dt);

  // seguidor
  void   seguidorInitLUT();
  void   seguidorImprimir(uint8_t mascara);

  // motores
  void initDutyLUT();
  void motoresBegin();
  void motoresPararTodos();
  void acionaMotor(int i, int velPct);
  void acionaRodasOminiInt(int vx, int vy, int omega);
  static int  toCent(float v);
  inline uint8_t porcentagemPWM(uint8_t v) { return dutyLUT[v]; }
  inline void writePWM(int m, int lado, uint8_t duty);



  // modos internos
  void modoCalibracao(uint8_t mascara, float erro);
  void modoEsperandoLargada(uint8_t mascara);
  void modoCorrida(uint8_t mascara, float erro, float dt);
};
