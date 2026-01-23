O que é?

A PID_t é uma biblioteca que implementa um controle PID (Proporcional-Integral-Derivativo). Esse tipo de controle é usado para ajustar automaticamente uma saída (exemplos: intensidade de LED/lâmpada, rotação de um motor, temperatura de um forno, etc.) de forma a seguir um valor desejado (o setpoint).


Como usar?

1. Crie um objeto PID
	ex.:

	#include "PIDController.h"	

	PIDController MeuPID;
	
2. Defina os ganhos (Kp, Ki, Kd)
	- Use o método setTunings(Kp, Ki, Kd) para atribuir.
	ex.:

	...

	MeuPID.setTunings(1.0, 2.0, 0.1);

	//ou alternativamente...:

	double Kp_MeuPID = 1.0;
	double Ki_MeuPID = 2.0;
	double Kd_MeuPID = 0.1;

	MeuPID.setTunings(Kp_MeuPID, Ki_MeuPID, Kd_MeuPID);

3. Se quiser um tempo próprio, defina o intervalo de funcionamento do PID
	- Use o método setSampleTime(...) para isso.

	ex.:

	...

	MeuPID.setSampleTime(10); //o intervalo de execução do PID é de 10 ms

4. Se quiser, escolha o modo de funcionamento. Por padrão, o objeto é inicializado no modo INCREMENTAL. Há dois modos disponíveis para uso: INCREMENTAL e ABSOLUTE.
	ABSOLUTE:
	    - A saída é substituída apenas pelo valor recém calculado; ou seja, depende apenas dos termos P, I, D.
	    - Se o erro variar muito de uma leitura para a outra, a saída pode ser muito brusca.
	    - É como se você olhasse o erro e decidisse imediatamente "a saída deve ser X".

	INCREMENTAL:
	    - A saída é substituída usando a saída anterior somada com o valor recém calculado.
	    - Isso suaviza a resposta, porquanto a saída evolui gradualmente.
	    - É como se você olhasse o erro e decidisse "vou mexer um pouquinho a saída, somando ou subtraindo, até chegar lá"

	ex.:

	...

	MeuPID.setType(INCREMENTAL);

5. Se quiser, escolha o sentido de funcionamento. Por padrão, o objeto é inicializado no sentido DIRETO. Há dois sentidos disponíveis: DIRECT e REVERSE
	DIRECT:
	    - Quando o erro aumenta, o controlador aumenta a saída 
	    - Normalmente usado em LED, motores aquecedores...

	REVERSE:
	    - Quando o erro aumenta, o controlador diminui a saída
	    - Normalmente usado em freio, válvula...

	ex.:

	...

	MeuPID.setDirection(DIRECT);

6. Se quiser, escolha o estilo de funcionamento. Por padrão, o objeto é inicializado no estilo PERSONALIZADO. Há dois estilos disponíveis: CLASSIC e CUSTOM
	CLASSIC:
	    - É o método tradicional de anti-windup
	    - Quando a saída chega ao limite (máximo ou mínimo), a integral para de acumular. Isso evita que o controlador exagere na correção.

	CUSTOM:
	    - Além de aplicar o método CLASSICO, ele zera o termo integral quando há mudança de sinal entre o erro atual e o erro anterior. Quando isso acontece, significa que o sistema cruzou o setpoint.
	    - É uma forma simples proposta por mim para diminuir overshoot.

	ex.:

	...
	
	MeuPID.setAntiOvershootStyle(CUSTOM);

7. Se quiser, defina os limites de saída. O atuador na prática atua sobre um intervalor de valores. O Arduino por exemplo só aceita valores PWM de 0 a 255. Dessa forma, o limite força 255 caso o PID calcule 300 ou força 0 se o PID calcular -50.

	ex.:

	...

	MeuPID.setOutputLimits(0, 255);

8. Se quiser, defina os limites da integração. Isso faz com que a integral não cresça indefinidamente, o controlador responde mais rápido, e reduz atrasos por excesso de integral acumulada.

	ex.:

	...

	MeuPID.setintegralLimits(-100,100);


9. Existe um método para ligar e desligar o controle PID. Use AUTOMATIC quando quiser ligar o controle e MANUAL quando quiser desligar. Quando o objeto é criado, o controle já vem ligado por padrão.

	ex.:

	...

	MeuPID.setMode(AUTOMATICO);

Até então, tem-se algo parecido com isso:

#include "PIDController.h"

// Declaração do objeto de controle PID
PIDController MeuPID;

// Declaração das constantes de PID
double Kp_MeuPID = 1.0;
double Ki_MeuPID = 2.0;
double Kd_MeuPID = 0.1;

void setup(){
	Serial.begin(115200);

	// Configuração do controle PID
	// Note que algumas coisas não precisam ser declaradas caso 
	// sejam iguais ao que já vem por padrão
	// Não esqueça de declarar as constantes. Elas são inicializadas em zero
	MeuPID.setTunings(Kp_MeuPID, Ki_MeuPID, Kd_MeuPID);
	MeuPID.setSampleTime(10);
	MeuPID.setType(INCREMENTAL);		// igual ao padrão
	MeuPID.setDirection(DIRECT);		// igual ao padrão
	MeuPID.setStyle(CUSTOM);			// igual ao padrão
	MeuPID.setOutputLimits(0, 255);
	MeuPID.setIntegrallimits(-100,100);	// às vezes não é ncessário
	MeuPID.setMode(AUTOMATIC);		// igual ao padrão
}

10. Com toda essa preparação, falta falar sobre a entrada (input), o alvo (setpoint) e a saída (output). A entrada é normalmente a leitura de algum sensor em que se deseja um controle PID. O método usado para atribuir esse valor deve ser chamado a cada ciclo de execução, já que o output depende do input.

	ex.:

	...
	
	MeuPID.setInput(leitura_sensor);

11. O setpoint pode ser algo fixo (então declarado no setup) ou algo que queira ser ajustado durante a execução do código. 

	ex.:

	...

	MeuPID.setSetpoint(leitura_potenciometro);

12. O output é justamente a saída calculada.

	ex.:

	...

	saida = MeuPID.getOutput();

No fim, o esqueleto será algo parecido com isto:

#include "PIDController.h"

// Declaração dos pinos do Arduino
const int sensor = A0;
const int potenciometro = A1;
const int porta_saida = 3;

// Declaração do objeto de controle PID
PIDController MeuPID;

// Declaração das constantes de PID
double Kp_MeuPID = 1.0;
double Ki_MeuPID = 2.0;
double Kd_MeuPID = 0.1;

void setup(){
	Serial.begin(115200);

	// Configuração do controle PID
	// Note que algumas coisas não precisam ser declaradas caso 
	// sejam iguais ao que já vem por padrão
	// Não esqueça de declarar as constantes. Elas são inicializadas em zero
	MeuPID.setTunings(Kp_MeuPID, Ki_MeuPID, Kd_MeuPID);
	MeuPID.setSampleTime(10);
	MeuPID.setType(INCREMENTAL);		// igual ao padrão
	MeuPID.setDirection(DIRECT);		// igual ao padrão
	MeuPID.setStyle(CUSTOM);			// igual ao padrão
	MeuPID.setOutputLimits(0, 255);
	MeuPID.setIntegrallimits(-100,100);	// às vezes não é ncessário
	MeuPID.setMode(AUTOMATIC);		// igual ao padrão
}

void loop(){
	// Realização das leituras
	int leitura_sensor = analogRead(sensor);
	int leitura_potenciometro = analogRead(potenciometro);
	
	// Padronização das leituras para algo dentro do intervalo do sinal PWM
	// analogRead retorna um valor de 0 a 1023
	int leitura_sensor_padronizada = map(leitura_sensor, 0, 1023, 0, 255);
	int leitura_potenciometro_padronizada = map(leitura_potenciometro, 0, 1023, 0, 255);

	// Hora de definir o input do PID nesse ciclo
	MeuPID.setInput(leitura_sensor_padronizada);

	// Agora, hora de definir o alvo nesse ciclo
	MeuPID.setSetpoint(leitura_potenciometro_padronizada);

	// Com esses dois definidos, podemos executar os cálculos
	MeuPID.Compute();

	// O resultado foi escrito no membro output da instância MeuPID. Vamos pegá-lo
	double valor_saida = MeuPID.getOutput();

	// Finalmente, escreveremos a saída na devida porta
	analogWrite(porta_saida, valor_saida);

	// Caso queira uma estrutura para debug...
	Serial.print("Setpoint: ");
	Serial.print(leitura_potenciometro_padronizada);
	Serial.print(" | Input: ");
	Serial.print(leitura_sensor_padronizada);
	Serial.print(" | Output: ");
	Serial.println(valor_saida);
}

// Como resultado da versão v2.2.1, o método Compute() retorna o output,
// então pode ser feito:
// double valor_saida = MeuPID.Compute(); 
