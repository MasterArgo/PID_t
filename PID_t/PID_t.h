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

// Nível de precisão
enum PrecisaoPID { PRECISO, NORMAL };

// -----------------------------
//  STRUCT PARA LOGS
// -----------------------------
struct Logs_t {
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
    PrecisaoPID precisao;  // micros() ou millis()
};

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
    PrecisaoPID precisao;  // micros() ou millis()

    // -----------------------------
    // FUNÇÕES AUXILIARES INTERNAS
    // -----------------------------
    inline double integral(double err, double dt);   // Cálculo do termo integral
    inline double derivativo(double err, double dt); // Cálculo do termo derivativo
    inline void ExecucaoLimites();                   // Aplica limites de saída e integral

public:
    // -----------------------------
    // CONSTRUTOR / DESTRUTOR
    // -----------------------------
    PID_t();
    ~PID_t();

    // -----------------------------
    // FUNÇÃO PRINCIPAL
    // -----------------------------
    double Compute();   // Executa o cálculo PID

    // -----------------------------
    // CONFIGURAÇÃO DE CONSTANTES
    // -----------------------------
    void DefConstantes(double Kp_inicializador, double Ki_inicializador, double Kd_inicializador);

    // -----------------------------
    // CONFIGURAÇÕES DE OPERAÇÃO
    // -----------------------------
    void DefIntervalo(unsigned long tempo);           // Intervalo de execução
    void DefModo(ModoPID novo_modo);                  // Define modo
    void DefTipo(TipoPID novo_tipo);                  // Define tipo
    void DefSentido(SentidoPID novo_sentido);         // Define sentido
    void DefEstilo(EstiloPID novo_estilo);            // Define estilo anti-windup
    void DefPrecisao(PrecisaoPID nova_precisao);      // Define a precisão do tempo de amostragem
    void DefLimitesSaida(double func_min, double func_max);     // Limites da saída
    void DefLimitesIntegral(double func_min, double func_max);  // Limites da integral
    void DefSetpoint(double func_setpoint);           // Define alvo
    void DefInput(double func_input);                 // Define entrada
    double LerOutput();                               // Lê saída atual

    // -----------------------------
    // UTILITÁRIOS
    // -----------------------------
    void Reset();   // Reseta variáveis internas

    // -----------------------------
    // LOGS
    // -----------------------------
    void Logs(Logs_t &log);
};

// Função utilitária externa
void troca(double &a, double &b);

#endif
