#ifndef PID_C_H
#define PID_C_H

#include <float.h>

// Estado Desligado/Ligado
enum ModoPID{MANUAL, AUTOMATICO};

// Sentido de funcionamento
enum SentidoPID{DIRETO, INVERSO};

// Tipo de funcionamento
enum TipoPID{ABSOLUTO, INCREMENTAL};

// Estilo de anti-windup
enum EstiloPID{CLASSICO, PERSONALIZADO};

// Classe PID, o coração dessa biblioteca
class PID_t{
    private:
        // Constantes de PID
        double Kp;
        double Ki;
        double Kd;

        // Tempo
        unsigned long tempo;
        unsigned long ultimo_tempo;

        // Variáveis auxiliares
        double ultimo_err;
        double dif_err_der;
        double soma_err_int;

        // Limites
        double minimo;
        double maximo;
        double integral_min;
        double integral_max;

        // Alvo
        double input;
        double output;
        double setpoint;

        // Sentido (Direto/Inverso)
        SentidoPID sentido;

        // Modo (Manual/Automático)
        ModoPID modo;

        // Tipo (Absoluto/Incremental)
        TipoPID tipo;

        // Estilo (Classico/Personalizado)
        EstiloPID estilo;

        // Funções auxiliares
        double integral(double err, double dt);
        double derivativo(double err, double dt);

    public:
        // Construtor e destrutor
        PID_t();
        ~PID_t();

        // Funções principais
        void Compute();
        void DefConstantes(double Kp_inicializador, double Ki_inicializador, double Kd_inicializador);

        // Configurações
        void DefIntervalo(unsigned long tempo);
        void DefSetpoint(double func_setpoint);
        void DefInput(double func_input);
        double LerOutput();
        void DefModo(ModoPID novo_modo);
        void DefTipo(TipoPID novo_tipo);
        void DefSentido(SentidoPID novo_sentido);
        void DefEstilo(EstiloPID novo_estilo);
        void DefLimitesSaida(double func_min, double func_max);
        void DefLimitesIntegral(double func_min, double func_max);

        // Utilitários
        void Reset();
        void ExecucaoLimites();
};

void troca(double &a, double &b);

#endif
