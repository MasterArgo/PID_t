#ifndef PID_T_H
#define PID_T_H

#include <float.h>

// -----------------------------
// ENUMS DE CONFIGURAÇÃO
// -----------------------------

// Estado do controlador
enum ModoPID { MANUAL, AUTOMATICO };

// Sentido de funcionamento
enum SentidoPID { DIRETO, INVERSO };

// Tipo de cálculo
enum TipoPID { ABSOLUTO, INCREMENTAL };

// Estilo de anti-windup
enum EstiloPID { CLASSICO, PERSONALIZADO };


// -----------------------------
// CLASSE PRINCIPAL PID_t
// -----------------------------
class PID_t {
private:
    // -----------------------------
    // CONSTANTES DE AJUSTE
    // -----------------------------
    double Kp;   // Ganho proporcional
    double Ki;   // Ganho integral
    double Kd;   // Ganho derivativo

    // -----------------------------
    // CONTROLE DE TEMPO
    // -----------------------------
    unsigned long tempo;         // Intervalo de execução (ms)
    unsigned long ultimo_tempo;  // Última execução

    // -----------------------------
    // VARIÁVEIS INTERNAS DE CÁLCULO
    // -----------------------------
    double ultimo_err;     // Último erro (para derivativo)
    double dif_err_der;    // Diferença de erro (derivativo)
    double soma_err_int;   // Soma acumulada do erro (integral)

    // -----------------------------
    // LIMITES
    // -----------------------------
    double minimo;         // Limite mínimo da saída
    double maximo;         // Limite máximo da saída
    double integral_min;   // Limite mínimo da integral
    double integral_max;   // Limite máximo da integral

    // -----------------------------
    // VARIÁVEIS DE PROCESSO
    // -----------------------------
    double input;          // Valor medido (feedback)
    double output;         // Saída calculada
    double setpoint;       // Alvo desejado

    // -----------------------------
    // CONFIGURAÇÕES DE MODO
    // -----------------------------
    SentidoPID sentido;    // Direto ou inverso
    ModoPID modo;          // Manual ou automático
    TipoPID tipo;          // Absoluto ou incremental
    EstiloPID estilo;      // Anti-windup clássico ou personalizado

    // -----------------------------
    // FUNÇÕES AUXILIARES INTERNAS
    // -----------------------------
    double integral(double err, double dt);   // Cálculo do termo integral
    double derivativo(double err, double dt); // Cálculo do termo derivativo
    void ExecucaoLimites();                   // Aplica limites de saída e integral

public:
    // -----------------------------
    // CONSTRUTOR / DESTRUTOR
    // -----------------------------
    PID_t();
    ~PID_t();

    // -----------------------------
    // FUNÇÃO PRINCIPAL
    // -----------------------------
    void Compute();   // Executa o cálculo PID

    // -----------------------------
    // CONFIGURAÇÃO DE CONSTANTES
    // -----------------------------
    void DefConstantes(double Kp_inicial, double Ki_inicial, double Kd_inicial);

    // -----------------------------
    // CONFIGURAÇÕES DE OPERAÇÃO
    // -----------------------------
    void DefIntervalo(unsigned long tempo);           // Intervalo de execução
    void DefModo(ModoPID novo_modo);                  // Define modo
    void DefTipo(TipoPID novo_tipo);                  // Define tipo
    void DefSentido(SentidoPID novo_sentido);         // Define sentido
    void DefEstilo(EstiloPID novo_estilo);            // Define estilo anti-windup
    void DefLimitesSaida(double func_min, double func_max);     // Limites da saída
    void DefLimitesIntegral(double func_min, double func_max);  // Limites da integral
    void DefSetpoint(double func_setpoint);           // Define alvo
    void DefInput(double func_input);                 // Define entrada
    double LerOutput();                               // Lê saída atual

    // -----------------------------
    // UTILITÁRIOS
    // -----------------------------
    void Reset();   // Reseta variáveis internas
};

// Função utilitária externa
void troca(double &a, double &b);

#endif
