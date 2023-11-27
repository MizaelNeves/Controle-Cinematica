# README

## Descrição do Projeto

Este projeto consiste em um programa desenvolvido em C++ para o ambiente Arduino. O código é projetado para controlar um manipulador robótico com três juntas, utilizando motores de passo e sensores para realizar movimentos precisos. A cinemática direta e inversa é implementada para calcular as posições do manipulador com base nos ângulos das juntas.

## Requisitos do Projeto

- Placa Arduino compatível
- Motores de passo (3 unidades)
- Sensores de limite para cada junta
- Biblioteca AccelStepper para controle de motores de passo

## Configuração do Hardware

- Conecte os motores de passo às portas especificadas no código (STEP e DIR).
- Conecte os sensores de limite às portas especificadas no código (limitSwitchX, limitSwitchY, limitSwitchZ).

## Configuração do Software

1. Instale a biblioteca AccelStepper no seu ambiente de desenvolvimento Arduino.

2. Carregue o código fornecido no Arduino IDE.

3. Configure as conexões de hardware de acordo com a descrição no código.

4. Faça o upload do código para a placa Arduino.

## Utilização do Programa

- Certifique-se de que a placa Arduino está devidamente conectada.

- Abra o Monitor Serial para interagir com o programa.

- Envie comandos através do Monitor Serial, como descrito no código:

  - Comandos incluem movimentos manuais, simulações automáticas e rotinas específicas.

## Licença

Este projeto é distribuído sob a licença [MIT](LICENSE). Sinta-se à vontade para usar, modificar e distribuir conforme necessário.
