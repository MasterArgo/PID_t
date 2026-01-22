/****************************************************************************************
 * Biblioteca PID_t - Versão 2.2.2
 * Linguagem: C++
 * Autor: Ícaro Razera (icarorazera@gmail.com)
 * Plataforma alvo: Arduino IDE
 *
 * Descrição:
 *   Implementação de um controlador PID flexível e didático.
 *   - Suporte a modos: MANUAL / AUTOMATICO
 *   - Sentido: DIRETO / INVERSO
 *   - Tipo: ABSOLUTO / INCREMENTAL
 *   - Estilo de anti-windup: CLASSICO / PERSONALIZADO
 *
 * Histórico:
 *   v1.0.0 - Primeira versão em linguagem C
 *   v2.0.0 - Reescrita em C++ com suporte a estilos de anti-windup
 *   v2.0.1 - Reorganização do método compute() e chamada inline das auxiliares internas.
 *            A função compute agora retorna o output.
 *   v2.1.2 - Adição de um modo mais preciso, usando micros(). Otimização do método
 *            compute().
 *   v2.2.2 - Adição de uma estrutura para coletar logs
 *
 * Licença:
 *   Este código é licenciado sob a MIT License.
 *   Consulte o arquivo LICENSE para mais detalhes.
 *   
 * GitHub:
 *   https://github.com/MasterArgo/PID_t
 ****************************************************************************************/

#include "PID_t.h"
#include <Arduino.h>

/*----------------------------------------------------------------------------------------
 * PID_t()
 *
 * Construtor da classe PID_t.
 *
 * Inicialização padrão:
 *   - Intervalo de execução: 200 ms
 *   - Modo: AUTOMATICO (controlador já começa ativo)
 *   - Constantes Kp, Ki, Kd: inicializadas em 0.0
 *   - Sentido: DIRETO
 *   - Tipo: INCREMENTAL
 *   - Estilo de anti-windup: PERSONALIZADO
 *   - Limites de saída: [-FLT_MAX, FLT_MAX]
 *   - Limites da integral: [-FLT_MAX, FLT_MAX] (sem restrição inicial)
 *
 * Observações:
 *   - Após a criação, o usuário deve configurar as constantes Kp, Ki, Kd
 *     usando DefConstantes().
 *   - Outras variáveis (setpoint, input, limites, estilo, etc.) podem ser
 *     ajustadas conforme necessário no setup().
 *---------------------------------------------------------------------------------------*/

PID_t::PID_t()
    : Kp(0), Ki(0), Kd(0),
      tempo(200), //intervalo padrão de funcionamento do PID
      ultimo_err(0),
      soma_err_int(0),
      dif_err_der(0),
      minimo(-FLT_MAX),
      maximo(FLT_MAX),
      sentido(DIRETO),
      modo(AUTOMATICO),
      tipo(INCREMENTAL),
      precisao(NORMAL),
      ultimo_tempo(precisao == PRECISO ? micros() : millis()),
      estilo(PERSONALIZADO),
      integral_min(-FLT_MAX),
      integral_max(FLT_MAX) {}

/*----------------------------------------------------------------------------------------
 * ~PID_t()
 *
 * Destrutor da classe PID_t.
 *
 * Funcionamento:
 *   - Atualmente não realiza nenhuma ação específica, pois a classe não aloca
 *     recursos dinâmicos (como memória via new ou ponteiros).
 *   - É mantido para garantir consistência da interface e permitir futuras
 *     expansões, caso seja necessário liberar recursos externos ou realizar
 *     alguma limpeza.
 *
 * Observações:
 *   - Em aplicações simples no Arduino, geralmente não há necessidade de lógica
 *     no destrutor, já que os objetos são desalocados automaticamente ao fim
 *     da execução.
 *   - A presença explícita do destrutor facilita manutenção e deixa claro que
 *     não há tarefas pendentes de liberação.
 *---------------------------------------------------------------------------------------*/

PID_t::~PID_t(){
    // Por enquanto, nada...
}

/*----------------------------------------------------------------------------------------
 * Compute()
 *
 * Função principal do controlador PID.
 * Executa o cálculo da saída com base no setpoint e no input definidos.
 *
 * Fluxo:
 *  1. Verifica se o modo está em AUTOMATICO.
 *  2. Calcula o erro (setpoint - input).
 *  3. Avalia o tempo decorrido desde a última execução.
 *  4. Se o tempo mínimo definido já passou:
 *      - Converte tempo para segundos.
 *      - Inverte o erro caso o sentido seja INVERSO.
 *      - Calcula os termos proporcional, integral e derivativo.
 *      - Atualiza a saída:
 *          • ABSOLUTO: substitui pela nova saída calculada.
 *          • INCREMENTAL: soma a variação calculada à saída anterior.
 *      - Atualiza o último erro para uso futuro.
 *      - Aplica os limites de saída e integral (anti-windup).
 *
 * Observações:
 *  - A função só atua em modo AUTOMATICO.
 *  - O intervalo de execução é controlado por DefIntervalo().
 *  - O sentido (DIRETO/INVERSO) define se o erro aumenta ou diminui a saída.
 *  - Os limites garantem que a saída e a integral não ultrapassem valores válidos.
 *
 * Retorno:
 *  - Retorna o Output.
 *---------------------------------------------------------------------------------------*/

double PID_t::Compute(){
    if(modo == AUTOMATICO){
        unsigned long agora = (precisao == PRECISO ? micros() : millis());
        unsigned long tempo_decorrido = agora - ultimo_tempo;

        if(tempo_decorrido >= tempo){
            double output_aux;
            double err = setpoint - input;

            ultimo_tempo = agora;

            // Cálculo PID
            output_aux = Kp * err + Ki * integral(err, tempo_decorrido) + Kd * derivativo(err, tempo_decorrido);
            output = (tipo == ABSOLUTO)? output_aux : output + output_aux;

            // Atualiza o erro
            ultimo_err = err;

            //Aplica os limites
            ExecucaoLimites();
        }
    }

    return output;
}

/*----------------------------------------------------------------------------------------
 * DefConstantes(...)
 *
 * Define os ganhos do controlador PID:
 *   - Kp: ganho proporcional
 *   - Ki: ganho integral
 *   - Kd: ganho derivativo
 *
 * Parâmetros:
 *   Kp_inicializador - constante proporcional
 *   Ki_inicializador - constante integral
 *   Kd_inicializador - constante derivativa
 *
 * Funcionamento:
 *   - Esses valores determinam a intensidade da resposta do controlador.
 *   - Kp controla a reação imediata ao erro.
 *   - Ki acumula o erro ao longo do tempo (corrige offset).
 *   - Kd reage à taxa de variação do erro (suaviza oscilações).
 *   - A função apenas armazena os valores; o cálculo é feito em Compute().
 *
 * Exemplo de uso:
 *   MeuPID.DefConstantes(1.0, 2.0, 0.1);
 *
 * Observação:
 *   Ajustar corretamente Kp, Ki e Kd é essencial para estabilidade.
 *   Valores muito altos podem causar oscilação; muito baixos podem deixar o sistema lento.
 *---------------------------------------------------------------------------------------*/

void PID_t::DefConstantes(double Kp_inicializador, double Ki_inicializador, double Kd_inicializador){
    constexpr int AJUSTE_MICROS = 1000000;
    constexpr int AJUSTES_MILLIS = 1000;
    short ajuste = (precisao == PRECISO ? AJUSTE_MICROS : AJUSTES_MILLIS);

    Kp = Kp_inicializador;
    // As constantes foram ajustadas na inicialização para que a conversão de ms para s não
    // precisasse ser feita a cada execução do compute()
    Ki = Ki_inicializador / ajuste;
    Kd = Kd_inicializador * ajuste;

    if(sentido == INVERSO){
        Kp = -Kp;
        Ki = -Ki;
        Kd = -Kd;
    }

    return;
}

/*----------------------------------------------------------------------------------------
 * DefIntervalo(...)
 *
 * Define o intervalo mínimo de tempo (em milissegundos) entre cada execução do cálculo PID.
 *
 * Parâmetros:
 *   tempo_func - valor em milissegundos que representa o período de amostragem do controlador.
 *
 * Funcionamento:
 *   - O valor armazenado em 'tempo' será usado dentro da função Compute().
 *   - Compute() só executará o cálculo se o tempo decorrido desde a última execução
 *     for maior ou igual a 'tempo'.
 *   - Isso evita que o PID seja recalculado em intervalos muito curtos, garantindo
 *     estabilidade e reduzindo sobrecarga de CPU.
 *
 * Exemplo de uso:
 *   MeuPID.DefIntervalo(100);   // Executa o PID a cada 100 ms
 *
 * Observação:
 *   O valor escolhido deve ser compatível com a dinâmica do sistema físico.
 *   Intervalos muito pequenos podem gerar ruído; intervalos muito grandes podem
 *   deixar o controle lento.
 *---------------------------------------------------------------------------------------*/

void PID_t::DefIntervalo(unsigned long tempo_func){
    tempo = tempo_func;

    return;
}


/*----------------------------------------------------------------------------------------
 * DefModo(...)
 *
 * Alterna o modo de operação do controlador PID.
 *
 * Parâmetros:
 *   novo_modo - pode ser AUTOMATICO ou MANUAL
 *
 * Funcionamento:
 *   - AUTOMATICO: o controlador executa os cálculos PID normalmente dentro da função Compute().
 *   - MANUAL: o controlador é pausado; Compute() não altera a saída.
 *             Nesse modo, o usuário pode definir manualmente o valor de 'output'.
 *
 * Exemplo de uso:
 *   pid.DefModo(MANUAL);      // pausa o PID e permite controlar a saída manualmente
 *   pid.DefModo(AUTOMATICO);  // ativa o PID para calcular a saída automaticamente
 *
 * Observações:
 *   - Útil para iniciar o sistema em MANUAL e só depois habilitar o PID em AUTOMATICO.
 *   - Também pode ser usado para testes ou situações em que o controle automático
 *     precisa ser temporariamente desativado.
 *---------------------------------------------------------------------------------------*/


void PID_t::DefModo(ModoPID novo_modo){
    modo = novo_modo;

    return;
}

/*----------------------------------------------------------------------------------------
 * DefTipo(...)
 *
 * Define o tipo de cálculo do controlador PID.
 *
 * Parâmetros:
 *   novo_tipo - pode ser ABSOLUTO ou INCREMENTAL
 *
 * Funcionamento:
 *   - ABSOLUTO: a saída calculada substitui diretamente o valor anterior.
 *               Exemplo: output = novo_valor;
 *   - INCREMENTAL: a saída calculada é somada ao valor anterior, representando
 *                  apenas a variação necessária.
 *               Exemplo: output = output + delta;
 *
 * Exemplo de uso:
 *   pid.DefTipo(ABSOLUTO);     // saída sempre recalculada do zero
 *   pid.DefTipo(INCREMENTAL);  // saída ajustada incrementalmente
 *
 * Observações:
 *   - O modo ABSOLUTO é mais intuitivo e comum em sistemas simples.
 *   - O modo INCREMENTAL pode ser útil em sistemas discretos ou quando se deseja
 *     aplicar apenas correções relativas.
 *---------------------------------------------------------------------------------------*/

void PID_t::DefTipo(TipoPID novo_tipo){
    tipo = novo_tipo;

    return;
}

/*----------------------------------------------------------------------------------------
 * DefSentido(...)
 *
 * Define o sentido de atuação do controlador PID.
 *
 * Parâmetros:
 *   novo_sentido - pode ser DIRETO ou INVERSO
 *
 * Funcionamento:
 *   - DIRETO: erro positivo aumenta a saída.
 *   - INVERSO: erro positivo diminui a saída (erro é invertido internamente).
 *
 * Exemplo de uso:
 *   pid.DefSentido(DIRETO);   // saída cresce quando input < setpoint
 *   pid.DefSentido(INVERSO);  // saída diminui quando input < setpoint
 *
 * Observações:
 *   - Útil para adaptar o controlador ao sistema físico.
 *   - Exemplo: em um aquecedor, sentido DIRETO aumenta potência quando a temperatura
 *     está abaixo do setpoint. Já em um sistema de resfriamento, pode ser necessário
 *     usar INVERSO.
 *---------------------------------------------------------------------------------------*/

void PID_t::DefSentido(SentidoPID novo_sentido){
    sentido = novo_sentido;

    return;
}

/*----------------------------------------------------------------------------------------
 * DefEstilo(...)
 *
 * Define o estilo de anti-windup (controle da saturação da integral).
 *
 * Parâmetros:
 *   novo_estilo - pode ser CLASSICO ou PERSONALIZADO
 *
 * Funcionamento:
 *   - CLASSICO: aplica anti-windup tradicional, limitando a integral de forma simples.
 *   - PERSONALIZADO: permite maior flexibilidade, aplicando limites definidos pelo usuário
 *                    via DefLimitesIntegral().
 *
 * Exemplo de uso:
 *   pid.DefEstilo(CLASSICO);       // usa anti-windup padrão
 *   pid.DefEstilo(PERSONALIZADO);  // usa limites definidos pelo usuário
 *
 * Observações:
 *   - O estilo CLASSICO é suficiente para a maioria dos sistemas.
 *   - O estilo PERSONALIZADO é útil quando se deseja controlar mais precisamente
 *     o comportamento da integral, evitando saturação ou resposta lenta.
 *---------------------------------------------------------------------------------------*/

void PID_t::DefEstilo(EstiloPID novo_estilo){
    estilo = novo_estilo;

    return;
}

/*----------------------------------------------------------------------------------------
 * DefPrecisao(...)
 *
 * Define a precisão da amostragem.
 *
 * Parâmetros:
 *   nova_precisao - pode ser NORMAL ou PRECISO
 *
 * Funcionamento:
 *   - NORMAL: usa a função millis() para as amostragens, que retorna o tempo com precisão
 *             de 1 milissegundo.
 *   - PRECISO: usa a função micros() para as amostragens, que retorna o tempo com precisão
               de 4 microssegundos.
 *
 * Exemplo de uso:
 *   pid.DefPrecisao(NORMAL);
 *   pid.DefPrecisao(PRECISO);
 *
 * Observações:
 *   - O estilo NORMAL é suficiente para a maioria dos sistemas.
 *   - O estilo PRECISO é útil quando se deseja controlar mais precisamente
 *     o tempo de execução do PID, garantindo intervalos regulares de cálculo.
 *     Em microcontroladores baseados em ARM, como o ESP32, o uso de micros()
 *     é eficiente e permite maior resolução temporal sem penalidade significativa.
 *     Já em placas Arduino clássicas (AVR), o uso de micros() é mais custoso,
 *     sendo recomendado utilizar millis() quando a resolução de 1 ms for suficiente.
 *     Dessa forma, o estilo PRECISO deve ser escolhido apenas quando a aplicação
 *     exigir alta precisão temporal, como em controle de motores ou medições rápidas.
 *---------------------------------------------------------------------------------------*/

void PID_t::DefPrecisao(PrecisaoPID nova_precisao){
    precisao = nova_precisao;

    return;
}

/*----------------------------------------------------------------------------------------
 * DefLimitesSaida(...)
 *
 * Define os limites mínimo e máximo da saída do controlador PID.
 *
 * Parâmetros:
 *   func_min - valor mínimo permitido para a saída
 *   func_max - valor máximo permitido para a saída
 *
 * Funcionamento:
 *   - Os valores são armazenados em 'minimo' e 'maximo'.
 *   - A função troca(func_min, func_max) garante que o menor valor fique em 'minimo'
 *     e o maior em 'maximo', mesmo que sejam passados invertidos.
 *   - Durante o cálculo em Compute(), a saída é limitada dentro desse intervalo.
 *
 * Exemplo de uso:
 *   pid.DefLimitesSaida(0, 255);   // saída limitada entre 0 e 255
 *
 * Observações:
 *   - Útil para sistemas com restrições físicas (ex.: PWM de 0–255).
 *   - Evita saturação ou valores inválidos na saída.
 *---------------------------------------------------------------------------------------*/


void PID_t::DefLimitesSaida(double func_min, double func_max){
    troca(func_min, func_max);

    minimo = func_min;
    maximo = func_max;

    return;
}

/*----------------------------------------------------------------------------------------
 * DefLimitesIntegral(...)
 *
 * Define os limites mínimo e máximo para o termo integral do controlador PID.
 *
 * Parâmetros:
 *   func_min - limite inferior da integral
 *   func_max - limite superior da integral
 *
 * Funcionamento:
 *   - Os valores são armazenados em 'integral_min' e 'integral_max'.
 *   - A função troca(func_min, func_max) garante que o menor valor fique em 'integral_min'
 *     e o maior em 'integral_max'.
 *   - Durante o cálculo, o somatório do erro * dt é limitado dentro desse intervalo
 *     para evitar saturação (anti-windup).
 *
 * Exemplo de uso:
 *   pid.DefLimitesIntegral(-100, 100);   // integral limitada entre -100 e 100
 *
 * Observações:
 *   - Essencial para evitar que o termo integral cresça indefinidamente.
 *   - Valores muito restritos podem reduzir a ação corretiva da integral.
 *---------------------------------------------------------------------------------------*/

void PID_t::DefLimitesIntegral(double func_min, double func_max){
    troca(func_min, func_max);

    integral_max = func_max;
    integral_min = func_min;

    return;
}

/*----------------------------------------------------------------------------------------
 * DefSetpoint(...)
 *
 * Define o valor alvo (setpoint) do controlador PID.
 *
 * Parâmetros:
 *   func_setpoint - valor desejado para o processo controlado
 *
 * Funcionamento:
 *   - O valor é armazenado em 'setpoint'.
 *   - Durante o cálculo em Compute(), o erro é obtido como (setpoint - input).
 *
 * Exemplo de uso:
 *   pid.DefSetpoint(100.0);   // define alvo de 100 unidades
 *
 * Observações:
 *   - O setpoint pode ser alterado dinamicamente durante a execução.
 *   - É o valor que o sistema tentará alcançar e manter.
 *---------------------------------------------------------------------------------------*/

void PID_t::DefSetpoint(double func_setpoint){
    setpoint = func_setpoint;

    return;
}

/*----------------------------------------------------------------------------------------
 * DefInput(...)
 *
 * Define o valor de entrada (feedback) do controlador PID.
 *
 * Parâmetros:
 *   func_input - valor medido do processo
 *
 * Funcionamento:
 *   - O valor é armazenado em 'input'.
 *   - Durante o cálculo em Compute(), o erro é obtido como (setpoint - input).
 *
 * Exemplo de uso:
 *   pid.DefInput(sensorValue);   // define entrada a partir de leitura de sensor
 *
 * Observações:
 *   - Deve ser atualizado a cada ciclo com o valor real do sistema.
 *   - Representa a variável controlada (ex.: temperatura, velocidade, posição).
 *---------------------------------------------------------------------------------------*/

void PID_t::DefInput(double func_input){
    input = func_input;

    return;
}

/*----------------------------------------------------------------------------------------
 * LerOutput()
 *
 * Retorna o valor atual da saída calculada pelo controlador PID.
 *
 * Retorno:
 *   output - valor da saída após o último cálculo em Compute()
 *
 * Funcionamento:
 *   - O valor armazenado em 'output' é retornado.
 *   - Esse valor deve ser aplicado ao atuador (ex.: PWM, motor, válvula).
 *
 * Exemplo de uso:
 *   double saida = pid.LerOutput();
 *   analogWrite(pinPWM, saida);
 *
 * Observações:
 *   - LerOutput() não recalcula nada, apenas retorna o último valor.
 *   - Para atualizar a saída, é necessário chamar Compute() antes.
 *---------------------------------------------------------------------------------------*/

double PID_t::LerOutput(){
    return output;
}

/*----------------------------------------------------------------------------------------
 * Reset(...)
 *
 * Zera o acumulador da integral (soma_err_int).
 *
 * Funcionamento:
 *   - Remove qualquer valor acumulado no termo integral.
 *   - Útil para evitar saturação (windup) quando o sistema muda de estado
 *     ou quando se deseja reiniciar o controlador.
 *
 * Exemplo de uso:
 *   pid.Reset();   // limpa o termo integral
 *
 * Observações:
 *   - Pode ser chamado em situações de reset do sistema ou troca de setpoint.
 *   - Não afeta os termos proporcional ou derivativo.
 *---------------------------------------------------------------------------------------*/

void PID_t::Reset(){
    soma_err_int = 0;

    return;
}

/*----------------------------------------------------------------------------------------
 * ExecucaoLimites(...)
 *
 * Aplica os limites definidos para a saída do controlador PID.
 *
 * Funcionamento:
 *   - Se 'output' for menor que 'minimo', ajusta para 'minimo'.
 *   - Se 'output' for maior que 'maximo', ajusta para 'maximo'.
 *   - Garante que a saída esteja sempre dentro da faixa permitida.
 *
 * Exemplo de uso:
 *   pid.DefLimitesSaida(0, 255);
 *   pid.Compute();
 *   pid.ExecucaoLimites();   // saída ficará entre 0 e 255
 *
 * Observações:
 *   - É chamado automaticamente dentro de Compute().
 *   - Evita valores inválidos ou saturação do atuador.
 *---------------------------------------------------------------------------------------*/

inline void PID_t::ExecucaoLimites(){
    if(output < minimo){
        output = minimo;
    }else if(output > maximo){
        output = maximo;
    }

    return;
}

/*----------------------------------------------------------------------------------------
 * integral(...)
 *
 * Calcula o termo integral do controlador PID.
 *
 * Parâmetros:
 *   err - erro atual (setpoint - input)
 *   dt  - tempo decorrido em segundos
 *
 * Funcionamento:
 *   - Se o erro muda de sinal (cruzou o setpoint) e o estilo for PERSONALIZADO,
 *     zera o acumulador da integral (anti-overshoot).
 *   - Se a saída já está saturada (no máximo ou mínimo), não acumula integral
 *     para evitar windup.
 *   - Caso contrário, acumula err * dt em soma_err_int.
 *   - Aplica limites definidos em integral_min e integral_max.
 *
 * Observações:
 *   - Em sistemas pouco ruidosos, melhora a resposta.
 *   - Em sistemas com setpoint oscilante, pode reduzir suavidade.
 *---------------------------------------------------------------------------------------*/

inline double PID_t::integral(double err, double dt){
    // Evita overshoot em alguns tipos de sistema
    // Em sistemas pouco ruidosos, a resposta é mais limpa
    // Por outro lado, em sistemas com oscilação no setpoint,
    // isso pode atrapalhar a suavidade da resposta
    if(estilo == PERSONALIZADO && err * ultimo_err <= 0){
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

    // Limita o somatório
    if(soma_err_int > integral_max){
        soma_err_int = integral_max;
    }else if(soma_err_int < integral_min){
        soma_err_int = integral_min;
    }

    return soma_err_int;
}

/*----------------------------------------------------------------------------------------
 * derivativo(...)
 *
 * Calcula o termo derivativo do controlador PID.
 *
 * Parâmetros:
 *   err - erro atual (setpoint - input)
 *   dt  - tempo decorrido em segundos
 *
 * Funcionamento:
 *   - Se dt == 0, retorna 0 (evita divisão por zero).
 *   - Caso contrário, calcula a variação do erro:
 *       (err - ultimo_err) / dt
 *
 * Observações:
 *   - O termo derivativo ajuda a suavizar oscilações.
 *   - Pode amplificar ruídos se o sistema tiver medições instáveis.
 *---------------------------------------------------------------------------------------*/

inline double PID_t::derivativo(double err, double dt){
    if(dt == 0){
        return 0;
    }

    return (err - ultimo_err) / dt;
}

void PID_t::Logs(Logs_t &log){
    log.dif_err_der = dif_err_der;
    log.estilo = estilo;
    log.input = input;
    log.integral_max = integral_max;
    log.integral_min = integral_min;
    log.Kd = Kd;
    log.Ki = Ki;
    log.Kp = Kp;
    log.maximo = maximo;
    log.minimo = minimo;
    log.modo = modo;
    log.output = output;
    log.precisao = precisao;
    log.sentido = sentido;
    log.setpoint = setpoint;
    log.soma_err_int = soma_err_int;
    log.tempo = tempo;
    log.tipo = tipo;
    log.ultimo_err = ultimo_err;
    log.ultimo_tempo = ultimo_tempo;

    return;
}

/*----------------------------------------------------------------------------------------
 * troca(...)
 *
 * Função auxiliar para garantir que dois valores fiquem em ordem crescente.
 *
 * Parâmetros:
 *   a - primeiro valor
 *   b - segundo valor
 *
 * Funcionamento:
 *   - Se 'a' for maior que 'b', troca os valores.
 *   - Usada em funções de limites para garantir que o menor valor seja
 *     armazenado como mínimo e o maior como máximo.
 *
 * Exemplo de uso:
 *   double min = 10, max = 5;
 *   troca(min, max);   // agora min = 5, max = 10
 *
 * Observações:
 *   - Simples utilitário.
 *   - Evita erros de configuração quando os limites são passados invertidos.
 *---------------------------------------------------------------------------------------*/

void troca(double &a, double &b){
    if(a > b){
        double temp = a;
        a = b;
        b = temp;
    }

    return;
}

// Fim


