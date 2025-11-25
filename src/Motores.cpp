#include "Motores.h"

/**
 * @brief Construtor da classe Motores
 */
Motores::Motores() {}

/**
 * @brief Inicializa os motores
 */
void Motores::init()
{
  for (char i = 0; i < 4; i++)
    for (char j = 0; j < 2; j++)
    {
      pinMode(pinMotor[i][j], OUTPUT);
      ledcSetup(chMotor[i][j], Freq_PWM, Resol_PWM);
      ledcAttachPin(pinMotor[i][j], chMotor[i][j]);
      ledcWrite(chMotor[i][j], 0);
    }
}

int Motores::velocidade(int valor)
{
  return valor != 0 ? map(valor, 0, 100, 150, 255) : 0;
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::avancar(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][0], dutyCicle);
  ledcWrite(chMotor[1][0], dutyCicle);
  ledcWrite(chMotor[2][0], dutyCicle);
  ledcWrite(chMotor[3][0], dutyCicle);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::para_traz(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][1], dutyCicle);
  ledcWrite(chMotor[1][1], dutyCicle);
  ledcWrite(chMotor[2][1], dutyCicle);
  ledcWrite(chMotor[3][1], dutyCicle);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::direita(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][1], dutyCicle);
  ledcWrite(chMotor[1][0], dutyCicle);
  ledcWrite(chMotor[2][0], dutyCicle);
  ledcWrite(chMotor[3][1], dutyCicle);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::esquerda(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][0], dutyCicle);
  ledcWrite(chMotor[1][1], dutyCicle);
  ledcWrite(chMotor[2][1], dutyCicle);
  ledcWrite(chMotor[3][0], dutyCicle);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::avancar_direita(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][0], velocidade(0));
  ledcWrite(chMotor[1][0], dutyCicle);
  ledcWrite(chMotor[2][0], dutyCicle);
  ledcWrite(chMotor[3][0], velocidade(0));
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::avancar_esquerda(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][0], dutyCicle);
  ledcWrite(chMotor[1][0], velocidade(0));
  ledcWrite(chMotor[2][0], velocidade(0));
  ledcWrite(chMotor[3][0], dutyCicle);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::para_traz_direita(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][1], dutyCicle);
  ledcWrite(chMotor[1][0], velocidade(0));
  ledcWrite(chMotor[2][0], velocidade(0));
  ledcWrite(chMotor[3][1], dutyCicle);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::para_traz_esquerda(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][0], velocidade(0));
  ledcWrite(chMotor[1][1], dutyCicle);
  ledcWrite(chMotor[2][1], dutyCicle);
  ledcWrite(chMotor[3][0], velocidade(0));
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::girar_direita(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][0], dutyCicle);
  ledcWrite(chMotor[1][1], dutyCicle);
  ledcWrite(chMotor[2][0], dutyCicle);
  ledcWrite(chMotor[3][1], dutyCicle);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::girar_esquerda(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][1], dutyCicle);
  ledcWrite(chMotor[1][0], dutyCicle);
  ledcWrite(chMotor[2][1], dutyCicle);
  ledcWrite(chMotor[3][0], dutyCicle);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::curva_direita_frente(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);
  int dutyCicleMetade = velocidade(velocidadePercentual / 2);

  ledcWrite(chMotor[0][0], dutyCicle);
  ledcWrite(chMotor[1][0], dutyCicleMetade);
  ledcWrite(chMotor[2][0], dutyCicle);
  ledcWrite(chMotor[3][0], dutyCicleMetade);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::curva_esquerda_frente(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);
  int dutyCicleMetade = velocidade(velocidadePercentual / 2);

  ledcWrite(chMotor[0][0], dutyCicleMetade);
  ledcWrite(chMotor[1][0], dutyCicle);
  ledcWrite(chMotor[2][0], dutyCicleMetade);
  ledcWrite(chMotor[3][0], dutyCicle);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::curva_direita_traz(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);
  int dutyCicleMetade = velocidade(velocidadePercentual / 2);

  ledcWrite(chMotor[0][1], dutyCicle);
  ledcWrite(chMotor[1][1], dutyCicleMetade);
  ledcWrite(chMotor[2][1], dutyCicle);
  ledcWrite(chMotor[3][1], dutyCicleMetade);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::curva_esquerda_traz(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);
  int dutyCicleMetade = velocidade(velocidadePercentual / 2);

  ledcWrite(chMotor[0][1], dutyCicleMetade);
  ledcWrite(chMotor[1][1], dutyCicle);
  ledcWrite(chMotor[2][1], dutyCicleMetade);
  ledcWrite(chMotor[3][1], dutyCicle);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::arco_lateral_direita_frente(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][0], dutyCicle);
  ledcWrite(chMotor[1][1], dutyCicle);
  ledcWrite(chMotor[2][0], velocidade(0));
  ledcWrite(chMotor[3][0], velocidade(0));
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::arco_lateral_direita_traz(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][0], velocidade(0));
  ledcWrite(chMotor[1][0], velocidade(0));
  ledcWrite(chMotor[2][0], dutyCicle);
  ledcWrite(chMotor[3][1], dutyCicle);
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::arco_lateral_esquerda_frente(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][1], dutyCicle);
  ledcWrite(chMotor[1][0], dutyCicle);
  ledcWrite(chMotor[2][0], velocidade(0));
  ledcWrite(chMotor[3][0], velocidade(0));
}

/**
 * @param velocidadePercentual numero da velocidade do carro (de 0 a 100)
 */
void Motores::arco_lateral_esquerda_traz(int velocidadePercentual)
{
  int dutyCicle = velocidade(velocidadePercentual);

  ledcWrite(chMotor[0][0], velocidade(0));
  ledcWrite(chMotor[1][0], velocidade(0));
  ledcWrite(chMotor[2][1], dutyCicle);
  ledcWrite(chMotor[3][0], dutyCicle);
}

void Motores::parar()
{
  for (char i = 0; i < 4; i++)
    for (char j = 0; j < 2; j++)
      ledcWrite(chMotor[i][j], 0);
}