#ifndef MOTORES_H
#define MOTORES_H
#define pinM0Dir 18
#define pinM0Esq 3
#define pinM1Dir 10
#define pinM1Esq 46
#define pinM2Dir 13
#define pinM2Esq 14
#define pinM3Dir 12
#define pinM3Esq 11
#define Freq_PWM 20000
#define Resol_PWM 8

#include <Arduino.h>

class Motores
{
    private:
        const uint8_t pinMotor[4][2] = {
    {pinM0Esq, pinM0Dir},
    {pinM1Esq, pinM1Dir},
    {pinM2Esq, pinM2Dir},
    {pinM3Esq, pinM3Dir}};

    const uint8_t chMotor[4][2] = {
  //frente  traz
  // (0)    (1)
     {0,     1},
     {2,     3},
     {4,     5},  
     {6,     7}};

     int velocidade(int valor);

    public: 
        Motores();

        void init();

        // Formato Padrão
        void avancar(int velocidadePercentual); 
        void para_traz(int velocidadePercentual); 
        void direita(int velocidadePercentual); 
        void esquerda(int velocidadePercentual); 
        void avancar_direita(int velocidadePercentual);
        void avancar_esquerda(int velocidadePercentual);
        void para_traz_direita(int velocidadePercentual);
        void para_traz_esquerda(int velocidadePercentual);

        // Formato Circular
        void girar_direita(int velocidadePercentual);
        void girar_esquerda(int velocidadePercentual);
        void curva_direita_frente(int velocidadePercentual);
        void curva_esquerda_frente(int velocidadePercentual);
        void curva_direita_traz(int velocidadePercentual);
        void curva_esquerda_traz(int velocidadePercentual);

        // Formato Arco
        void arco_lateral_direita_frente(int velocidadePercentual);
        void arco_lateral_esquerda_frente(int velocidadePercentual);
        void arco_lateral_direita_traz(int velocidadePercentual);
        void arco_lateral_esquerda_traz(int velocidadePercentual);

        void parar();
};










#endif