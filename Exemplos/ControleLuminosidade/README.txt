Objetivo:
Este projeto tem como finalidade demonstrar o funcionamento de um controlador PID (Proporcional–Integral–Derivativo) aplicado ao controle da intensidade luminosa de um LED. A ideia é criar um sistema de realimentação em que a luz emitida pelo LED é medida por um fotoresistor e ajustada automaticamente para se manter próxima de um valor desejado (setpoint), definido por um potenciômetro.

Componentes utilizados:
- 1 LED -> atuador que emite luz e será controlado via PWM.
- 1 resistor de 330 Ω -> limita a corrente do LED, protegendo-o contra sobrecarga.
- 1 fotoresistor (LDR) -> sensor que mede a intensidade da luz ambiente (incluindo a luz do LED).
- 2 resistores de 5.1 kΩ -> usados em conjunto com o LDR e o potenciômetro para formar divisores de tensão, permitindo que o Arduino leia valores analógicos proporcionais à luminosidade e ao setpoint.
- 1 potenciômetro -> define o nível de luminosidade desejado (setpoint).
- 1 Arduino Nano -> microcontrolador responsável por executar o algoritmo PID e gerar o sinal PWM para o LED.
- 7 fios macho–macho e 2 fios macho–fêmea -> realizam as conexões entre os componentes e a placa.

Funcionamento do sistema:
- O potenciômetro fornece um valor analógico que representa o nível de luminosidade desejado.
- O fotoresistor mede a intensidade da luz real no ambiente, incluindo a emitida pelo LED.
- O Arduino compara o valor medido com o valor desejado (setpoint).
- O PID calcula a diferença (erro) e ajusta a saída PWM do LED:
- P (Proporcional) -> corrige de acordo com a magnitude do erro.
- I (Integral) -> acumula o erro ao longo do tempo, garantindo que o sistema alcance o setpoint.
- D (Derivativo) -> reage à variação do erro, suavizando oscilações.
- O LED aumenta ou diminui sua intensidade até que a leitura do fotoresistor se aproxime do valor definido pelo potenciômetro.

Objetivo pedagógico:
O projeto mostra, de forma prática e visual, como um controlador PID pode ser aplicado em sistemas de controle realimentados. Ele ajuda a entender conceitos como:
- Feedback (realimentação de um sistema).
- Controle automático de variáveis físicas (neste caso, luminosidade).
- Ajuste de parâmetros PID (Kp, Ki, Kd) e seus efeitos na estabilidade e resposta do sistema.

