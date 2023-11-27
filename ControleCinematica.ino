#include <AccelStepper.h>
#include <math.h>

// Configuração dos motores de passo
AccelStepper motor1(1, 2, 5); // (Type:driver, STEP, DIR)
AccelStepper motor2(1, 3, 6);
AccelStepper motor3(1, 4, 7);

// Configuração dos sensores
#define limitSwitchX 9  // Sensor junta 1
#define limitSwitchY 10 // Sensor junta 2
#define limitSwitchZ 11 // Sensor junta 3

// Variáveis para os ângulos das juntas e seus respectivos limites
float theta1 = 0; // Ângulo da junta 1
float limiteTheta1 = 120;
float limiteMinTheta1 = -140;

float theta2 = 0; // Ângulo da junta 2
float limiteTheta2 = 180;
float limiteMinTheta2 = 0;

float theta3 = 0; // Ângulo da junta 3
float limiteTheta3 = 90;
float limiteMinTheta3 = -90;

// Variáveis para coordenadas
float X = 0; // Posição em X
float Y = 0; // Posição em Y
float Z = 0; // Posição em Z

// Comprimentos dos braços
float L1 = 0;      // Comprimento elo 1
float d1 = 291.00; // Deslocamento elo 1
float L2 = 141.50; // Comprimento elo 2
float L3 = 150.00; // Comprimento elo 3

// Define a quantidade de passos para 1° do motor
float revolucaoMotor = (200.00 / 360.00) * 8.00; // PassosPorRevolução * ConfigMicroPassos
float reducaoMotor1 = 25.00;                     // 25:1
float reducaoMotor2 = (230.00 / 11.00);          // 230:11
float reducaoMotor3 = (45.00 / 8.00);            // 45:8

String comando;

// Função para configurar inicialização
void setup()
{
    pinMode(limitSwitchX, INPUT_PULLUP);
    pinMode(limitSwitchY, INPUT_PULLUP);
    pinMode(limitSwitchZ, INPUT_PULLUP);

    // Inicialize os motores de passo com velocidade máxima e aceleração
    configurarMotores();

    // Inicialize a comunicação serial
    Serial.begin(115200);
    Serial.println("Iniciando...");

    homing();
}

// Função principal do programa
void loop()
{
    if (Serial.available() > 0)
    {
        comando = Serial.readStringUntil('\n'); // Lê a string até encontrar uma quebra de linha
        char tipoComando = comando.charAt(0);   // Obtém o primeiro caractere para determinar o tipo de comando

        switch (tipoComando)
        {
        case 'D':
            processarComandoD(comando);
            break;
        case 'I':
            processarComandoI(comando);
            break;
        case 'H':
            Serial.println("Home");
            homing();
            break;
        case 'A':
            Serial.println("Simulação Automática");
            processarSimulacaoAutomatica();
            break;
        case 'M':
            Serial.println("Simulação Manual");
            processarSimulacaoManual();
            break;
        default:
            Serial.println("Comando desconhecido.");
            break;
        }
    }
}

// Função para configurar os motores de passo
void configurarMotores()
{
    motor1.setMaxSpeed(4000);
    motor1.setAcceleration(2000);
    motor2.setMaxSpeed(4000);
    motor2.setAcceleration(2000);
    motor3.setMaxSpeed(1000);
    motor3.setAcceleration(500);
}

// Função para realizar o procedimento de homing
void homing()
{
    homingStepper(&motor1, limitSwitchX, -21000);
    homingStepper(&motor2, limitSwitchY, -1100);
    homingStepper(&motor3, limitSwitchZ, 3412.5);

    cinematicaDireta(0, 0, 0);
}

// Função para realizar homing em um motor de passo
void homingStepper(AccelStepper *motor, int limitSwitch, long homePosition)
{
    while (digitalRead(limitSwitch) == 1)
    {
        motor->setSpeed(-2000);
        motor->runSpeed();
        motor->setCurrentPosition(homePosition);
    }
    delay(20);

    motor->moveTo(0);
    while (motor->currentPosition() != 0)
    {
        motor->run();
    }
}

// Função para processar comando de movimento absoluto
void processarComandoD(String comando)
{
    if (comando.indexOf("T1:") != -1 && comando.indexOf("T2:") != -1 && comando.indexOf("T3:") != -1)
    {
        theta1 = comando.substring(comando.indexOf("T1:") + 3, comando.indexOf("T2:")).toFloat();
        theta2 = comando.substring(comando.indexOf("T2:") + 3, comando.indexOf("T3:")).toFloat();
        theta3 = comando.substring(comando.indexOf("T3:") + 3).toFloat();

        if (!validaAngulos(theta1, theta2, theta3))
        {
            Serial.println("Os ângulos devem ser menores que os limites estabelecidos!");
            return;
        }

        cinematicaDireta(theta1, theta2, theta3);
    }
    else
    {
        Serial.println("Formato da string D inválido! Use o formato: D T1:xx T2:yy T3:zz");
    }
}

// Função para calcular a cinemática direta e atualizar as posições dos motores de passo
void cinematicaDireta(float t1, float t2, float t3)
{
    long passosTheta1 = t1 * revolucaoMotor * reducaoMotor1;
    long passosTheta2 = t2 * revolucaoMotor * reducaoMotor2;
    long passosTheta3 = t3 * revolucaoMotor * reducaoMotor3;

    t1 = radians(t1);
    t2 = radians(t2);
    t3 = radians(t3);

    float X = cos(t1) * (L2 * cos(t2) + L3 * cos(t2 + t3));
    float Y = sin(t1) * (L2 * cos(t2) + L3 * cos(t2 + t3));
    float Z = d1 + L2 * sin(t2) + L3 * sin(t2 + t3);

    Serial.print("X: ");
    Serial.print(X);
    Serial.print("   Y: ");
    Serial.print(Y);
    Serial.print("   Z: ");
    Serial.println(Z);

    movimentaMotor(passosTheta1, passosTheta2, passosTheta3);
}

// Função para processar comando de movimento relativo
void processarComandoI(String comando)
{
    if (comando.indexOf("X") != -1 && comando.indexOf("Y") != -1 && comando.indexOf("Z") != -1)
    {
        X = comando.substring(comando.indexOf("X") + 1, comando.indexOf("Y")).toFloat();
        Y = comando.substring(comando.indexOf("Y") + 1, comando.indexOf("Z")).toFloat();
        Z = comando.substring(comando.indexOf("Z") + 1).toFloat();

        cinematicaInversa(X, Y, Z);
    }
    else
    {
        Serial.println("Formato da string I inválido! Use o formato: I Xnn Ynn Znn");
    }
}

// Função para calcular a cinemática inversa e atualizar as posições dos motores de passo
void cinematicaInversa(float X, float Y, float Z)
{
    float D = pow((Z - d1), 2) + pow(X, 2) + pow(Y, 2);
    theta3 = acos((D - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3));
    theta2 = atan(((Z - d1) * (L2 + L3 * cos(theta3) - D * (L3 * sin(theta3)))) / (D * (L2 * L3 * cos(theta3)) + (Z - d1) * (L3 * sin(theta3))));

    theta1 = atan(Y / X);

    if (isnan(theta1))
        theta1 = 0;
    if (isnan(theta2))
        theta2 = 0;
    if (isnan(theta3))
        theta3 = 0;

    theta1 = degrees(theta1);
    theta2 = degrees(theta2);
    theta3 = degrees(theta3);

    AnguloIncrementalTheta1();
    AnguloIncrementalTheta2();
    anguloIncrementalTheta3();

    Serial.print("T1: ");
    Serial.print(theta1);
    Serial.print("   T2: ");
    Serial.print(theta2);
    Serial.print("   T3: ");
    Serial.println(theta3);

    if (!validaAngulos(theta1, theta2, theta3))
    {
        Serial.println("Os ângulos devem ser menores que os limites estabelecidos!");
        return;
    }

    Serial.print("X: ");
    Serial.print(X);
    Serial.print("   Y: ");
    Serial.print(Y);
    Serial.print("   Z: ");
    Serial.println(Z);

    // Converte os ângulos para o número de passos
    long passosTheta1 = theta1 * revolucaoMotor * reducaoMotor1;
    long passosTheta2 = theta2 * revolucaoMotor * reducaoMotor2;
    long passosTheta3 = theta3 * revolucaoMotor * reducaoMotor3;

    movimentaMotor(passosTheta1, passosTheta2, passosTheta3);
}

// Função para processar simulação automática
void processarSimulacaoAutomatica()
{
    int posicao = 1;
    bool parar = false;

    while (!parar)
    {
        parar = simulacao(posicao);
        posicao += 1;
    }
    Serial.println("Simulação Automática encerrada.");
}

// Função para processar simulação manual
void processarSimulacaoManual()
{
    cinematicaInversa(291.50, 0.00, 291.00);

    bool parar = false;
    int posicao = 1;

    Serial.println("Insira o comando 'N' para a PRÓXIMA posição ou 'S' para PARAR a simulação.");

    while (!parar)
    {
        if (Serial.available() > 0)
        {
            String comandoManual = Serial.readStringUntil('\n');
            char tipoComando = comandoManual.charAt(0);

            switch (tipoComando)
            {
            case 'S':
                parar = true;
                break;
            case 'N':
                parar = simulacao(posicao);
                posicao += 1;
                break;
            default:
                Serial.println("Comando desconhecido! 'S' - PARAR; 'N' - PRÓXIMO");
                break;
            }
        }
    }
    Serial.println("Simulação Manual encerrada.");
}

// Função para realizar simulação de posição
bool simulacao(int posicao)
{
    switch (posicao)
    {
    case 1:
        cinematicaInversa(0.00, 291.50, 291.00);
        return false;
        break;
    case 2:
        cinematicaInversa(0.00, 0.00, 582.50);
        return false;
        break;
    case 3:
        cinematicaInversa(141.50, 0.00, 441.00);
        return false;
        break;
    case 4:
        cinematicaInversa(0.00, 141.50, 441.00);
        return false;
        break;
    case 5:
        cinematicaInversa(-150.00, 0.00, 432.50);
        return false;
        break;
    case 6:
        cinematicaInversa(-223.30, -187.37, 291.00);
        return false;
        break;
    case 7:
        cinematicaInversa(-145.75, 252.45, 291.00);
        return false;
        break;
    case 8:
        cinematicaInversa(-70.75, 122.54, 141.00);
        return false;
        break;
    case 9:
        cinematicaInversa(-108.40, -90.54, 441.00);
        return false;
        break;
    case 10:
        cinematicaInversa(-108.40, -90.54, 141.00);
        return false;
        break;
    default:
        // Ação padrão para caracteres desconhecidos
        cinematicaInversa(291.50, 0.00, 291.00);
        return true;
        break;
    }
}

// Função para validar os ângulos
bool validaAngulos(float anguloT1, float anguloT2, float anguloT3)
{
    return (anguloT1 >= limiteMinTheta1 && anguloT1 <= limiteTheta1) &&
           (anguloT2 >= limiteMinTheta2 && anguloT2 <= limiteTheta2) &&
           (anguloT3 >= limiteMinTheta3 && anguloT3 <= limiteTheta3);
}

// Função para movimentar os motores de passo
void movimentaMotor(long passosTheta1, long passosTheta2, long passosTheta3)
{
    motor1.moveTo(passosTheta1);
    motor2.moveTo(passosTheta2);
    motor3.moveTo(passosTheta3);

    while (motor1.currentPosition() != passosTheta1 || motor2.currentPosition() != passosTheta2 || motor3.currentPosition() != passosTheta3)
    {
        motor1.run();
        motor2.run();
        motor3.run();
    }
}

// Função para verificar o quadrante e ajustar o ângulo incremental
void AnguloIncrementalTheta1()
{
    float LT = L2 + L3;
    if (LT > X && X > 0)
    {
        if (0 > Y && Y > -LT && 0 >= theta1 && theta1 >= -90) // Q1 da posição
        {
            // Serial.println("Q3 --> Q1 ");
            theta1 += 180;
        }
        else if (LT >= Y && Y > 0 && 90 >= theta1 && theta1 >= 0) // Q2 da posição
        {
            // Serial.println("Q4 --> Q2 ");
            theta1 -= 180;
        }
    }
    else if (0 > X && X >= -LT)
    {
        if (LT > Y && Y > 0 && 0 >= theta1 && theta1 >= -90) // Q3 da posição
        {
            // Serial.println("Q1 --> Q3 ");
            theta1 += 180;
        }
        else if (0 > Y && Y >= -LT && 90 >= theta1 && theta1 >= 0) // Q4 da posição
        {
            // Serial.println("Q2 --> Q4 ");
            theta1 -= 180;
        }
    }
}

// Função para ajustar o ângulo incremental de Theta2
void anguloIncrementalTheta2()
{
    if (theta3 == 90 && theta2 >= -91.00 && theta2 <= -89.00 && X < 0)
    {
        // Serial.println("Acrescimo +180 em T2");
        theta2 += 180;
    }
    else if ((theta3 == 0 && X == 0 && Y == 0) || (theta3 == 90 && Y == 0) || (theta3 == 90 && X == 0))
    {
        // Serial.println("Acrescimo +90 em T2");
        theta2 += 90;
    }
    else if (theta2 >= -91.00 && theta2 <= -89.00 && X < 0)
    {
        // Serial.println("Acrescimo +90 em T2");
        theta2 += 90;
    }
}

// Função para ajustar o ângulo incremental de Theta3
void anguloIncrementalTheta3()
{
    if (theta3 <= 91.00 && theta3 >= 89.00 && Z < 291)
        theta3 -= 180;
}
