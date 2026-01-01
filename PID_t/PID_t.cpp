/****************************************************************************************
	* Biblioteca de PID - Versão 2.0.0
	* em C++
	* Por Icaro Razera, icarorazera@gmail.com
	* Biblioteca pensada para plataforma Arduino IDE
****************************************************************************************/

#include "PID_c.h"
#include <Arduino.h>

/*Compute()******************************************************************************
	* É quem executa o PID de fato
	* Avalia o sentido e executa se estiver em AUTOMATICO
****************************************************************************************/

void PID_t::Compute(){
    if(modo == AUTOMATICO){
        double err = setpoint - input;
        unsigned long tempo_decorrido = millis() - ultimo_tempo;
        double output_aux;

        if(tempo_decorrido >= tempo){
            double tempo_segundos;
            ultimo_tempo = millis();
            tempo_segundos = tempo_decorrido / 1000.0;

            if(sentido == INVERSO){
                err = -err;
            }

            // Cálculo PID
            output_aux = Kp * err + Ki * integral(err, tempo_segundos) + Kd * derivativo(err, tempo_segundos);
            output = (tipo == ABSOLUTO)? output_aux : output + output_aux; //seria melhor que: (tipo == ABSOLUTO)? output = output_aux : output += output_aux; ?

            // Atualiza o erro
            ultimo_err = err;

            //Aplica os limites
            ExecucaoLimites();
        }
    }

    return;
}

/*integral(...)**************************************************************************
	* Função auxiliar para calcular o termo integral
	* Se estiver muito próxima do alvo, zera o somatório dos erros
	* Limita o somatório a valores máximos e mínimos
****************************************************************************************/

double PID_t::integral(double err, double dt){
    // Evita overshoot em alguns tipos de sistema
    // Em sistemas pouco ruidosos, a resposta é mais limpa
    // Por outro lado, em sistemas com oscilação no setpoint,
    // isso pode atrapalhar a suavidade da resposta
    if(err * ultimo_err <= 0 && estilo == PERSONALIZADO){
        soma_err_int = 0;
    }

    // Se saída saturou, não acumula integral
    if(output >= maximo && err > 0){
        // já está no máximo e erro positivo -> não acumula
        return soma_err_int;
    }else if(output <= minimo && err < 0){
        // já está no mínimo e erro negativo -> não acumula
        return soma_err_int;
    }


    soma_err_int += err * dt;

    if(soma_err_int > integral_max){
        soma_err_int = integral_max;
    }else if(soma_err_int < integral_min){
        soma_err_int = integral_min;
    }

    return soma_err_int;
}

/*derivativo(...)************************************************************************
	* Função auxiliar para calcular o termo derivativo
****************************************************************************************/

double PID_t::derivativo(double err, double dt){
    if(dt == 0){
        return 0;
    }

    return (err - ultimo_err) / dt;
}

/*PID_t**********************************************************************************
	* Lista de inicialização da variável do tipo PID_t
	* O tempo padrão é de 200 ms
	* Começa ligado
	* Outras variáveis podem ser modificadas depois
****************************************************************************************/

PID_t::PID_t()
    : Kp(0), Ki(0), Kd(0),
      tempo(200), //intervalo padrão de funcionamento do PID
      ultimo_err(0),
      soma_err_int(0),
      dif_err_der(0),
      ultimo_tempo(millis()),
      minimo(-FLT_MAX),
      maximo(FLT_MAX),
      sentido(DIRETO),
      modo(AUTOMATICO),
      tipo(INCREMENTAL),
      estilo(PERSONALIZADO),
      integral_min(-FLT_MAX),
      integral_max(FLT_MAX) {}

/*DefConstantes(...)*********************************************************************
    * Função para inicializar as constantes Kp, Ki, Kd
****************************************************************************************/

PID_t::~PID_t(){
    // Por enquanto, nada...
}

void PID_t::DefConstantes(double Kp_inicializador, double Ki_inicializador, double Kd_inicializador){
    Kp = Kp_inicializador;
    Ki = Ki_inicializador;
    Kd = Kd_inicializador;

    return;
}
/*DefIntervalo(...)**********************************************************************
	* Função para modificar o intervalo entre cada execução do PID
****************************************************************************************/

void PID_t::DefIntervalo(unsigned long tempo_func){
    tempo = tempo_func;

    return;
}

void PID_t::DefSetpoint(double func_setpoint){
    setpoint = func_setpoint;

    return;
}

void PID_t::DefInput(double func_input){
    input = func_input;

    return;
}

double PID_t::LerOutput(){
    return output;
}

/*DefModo(...)***************************************************************************
	* Função para alternar entre AUTOMATICO e MANUAL
	* AUTOMATICO executa o PID
	* MANUAL pausa o PID
	* Exemplo: pid.DefModo(MANUAL);
****************************************************************************************/

void PID_t::DefModo(ModoPID novo_modo){
    modo = novo_modo;

    return;
}

/*DefTipo(...)***************************************************************************
	* Função para alternar entre ABSOLUTO e INCREMENTAL
	* ABSOLUTO executa o PID sem depender da saída (output) anterior
	* INCREMENTAL executa o PID dependendo da saída (output) anterior
	* Exemplo: pid.DefTipo(INCREMENTAL);
****************************************************************************************/

void PID_t::DefTipo(TipoPID novo_tipo){
    tipo = novo_tipo;

    return;
}

/*DefSentido(...)************************************************************************
	* Função para alternar entre DIRETO e INVERSO
    * DIRETO: saída aumenta com erro positivo
    * INVERSO: saída diminui com erro positivo
    * Exemplo: pid.DefSentido(DIRETO);
****************************************************************************************/

void PID_t::DefSentido(SentidoPID novo_sentido){
    sentido = novo_sentido;

    return;
}

void PID_t::DefEstilo(EstiloPID novo_estilo){
    estilo = novo_estilo;

    return;
}

/*DefinaLimitesSaida(...)****************************************************************
	* Função para modificar os limites máximo e mínimo do PID
****************************************************************************************/

void PID_t::DefLimitesSaida(double func_min, double func_max){
    troca(func_min, func_max);

    minimo = func_min;
    maximo = func_max;

    return;
}

/*Reset(...)*****************************************************************************
	* Função para zerar o erro da integral
	* Evita saturação
****************************************************************************************/

void PID_t::Reset(){
    soma_err_int = 0;

    return;
}

/*ExecucaoLimites(...)*********************************************************************
	* Função para verificar se a saída está dentro dos limites
****************************************************************************************/

void PID_t::ExecucaoLimites(){
    if(output < minimo){
        output = minimo;
    }else if(output > maximo){
        output = maximo;
    }

    return;
}


/*DefinaLimitesIntegral(...)*************************************************************
	* Função para determinar os limites do somatório dos erro * dt
****************************************************************************************/

void PID_t::DefLimitesIntegral(double func_min, double func_max){
    troca(func_min, func_max);

    integral_max = func_max;
    integral_min = func_min;

    return;
}


// Só uma função auxiliar
void troca(double &a, double &b){
    if(a > b){
        double temp = a;
        a = b;
        b = temp;
    }

    return;
}

