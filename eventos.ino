#include <Ultrasonic.h>
#include <VarSpeedServo.h>
#define echo 2  //PINO DIGITAL UTILIZADO PELO HC-SR04 ECHO(RECEBE)
#define trig 3  //PINO DIGITAL UTILIZADO PELO HC-SR04 TRIG(ENVIA)
#define s0 33   // Pino 33 do arduino conectado ao pino S0 do TCS230
#define s1 34   // Pino 34 do arduino conectado ao pino S1 do TCS230
#define s2 32   // Pino 32 do arduino conectado ao pino S2 do TCS230
#define s3 31   // Pino 31 do arduino conectado ao pino S3 do TCS230
#define out 30  // Pino 30 do arduino conectado ao pino OUT do TCS230
#define IN1 5   //Pino IN1 da ponte h
#define IN2 4   //Pino IN2 da ponte h
#define vel 7   //controle de velocidade da esteira

// create servo object to control a servo
VarSpeedServo servo1;  //frente trás
VarSpeedServo servo2;  //esquerda direita
VarSpeedServo servo3;  //cima baixo
VarSpeedServo servo4;  // garra

Ultrasonic ultrasonic(trig, echo);  //INICIALIZANDO OS PINOS DO ULTRASSÔNICO

//variáveis auxiliares
long distancia;
int pos;
int red = 0;
int green = 0;
int blue = 0;
int control = 0;

/* limites de ângulo dos servos
    int RIG = 40; 
    int LEF = 120;
    int FRO = 10;
    int BAC = 80;
    int GAR = 80;
    int GAR2 = 160;
    int UP = 100;
    int DOW = 35;
*/

void setup() {
  pinMode(s0, OUTPUT);  // Pino S0 configurado como saida
  pinMode(s1, OUTPUT);  // Pino S1 configurado como saida
  pinMode(s2, OUTPUT);  // Pino S2 configurado como saida
  pinMode(s3, OUTPUT);  // Pino S3 configurado como saida
  pinMode(out, INPUT);
  pinMode(echo, INPUT);   //DEFINE O PINO COMO ENTRADA (RECEBE)
  pinMode(trig, OUTPUT);  //DEFINE O PINO COMO SAIDA (ENVIA)
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(vel, OUTPUT);
  Serial.begin(115200);
  servo1.attach(9);
  servo2.attach(8);
  servo3.attach(10);
  servo4.attach(11);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
}
void loop() {
  servo1.write(80);
  servo2.write(75);
  servo3.write(150);
  servo4.write(80);
  hcsr04();

  if (distancia >= 1 && distancia <= 6) { //ultrassônico detectar peça

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    for (control = 255; control >= 0; control--) {
      analogWrite(vel, control);
    }

    leitura_cores();

  } else { //peça não detectada
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    for (control = 0; control <= 255; control++) {
      analogWrite(vel, control);
    }
  }
}
float hcsr04() {                         //função de leitura do ultrassônico
  distancia = (ultrasonic.Ranging(CM));  //VARIÁVEL GLOBAL RECEBE O VALOR DA DISTÂNCIA MEDIDA
  Serial.print("Distancia ");            //IMPRIME O TEXTO NO MONITOR SERIAL
  Serial.print(distancia);
  Serial.println("cm");  //IMPRIME O TEXTO NO MONITOR SERIAL
  delay(100);            //INTERVALO DE 500 MILISSEGUNDOS
  return distancia;
}

void leitura_cores() {

  digitalWrite(s2, LOW);  // Pino S2 em nivel baixo
  digitalWrite(s3, LOW);  // Pino S3 em nivel baixo
  delay(50);
  red = pulseIn(out, LOW);

  // Configura a leitura para os fotodiodos Green (Verde)
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delay(50);

  // Lê a frequencia de saída do fotodiodo verde
  green = pulseIn(out, LOW);

  // Configura a leitura para os fotodiodos Blue (Azul)
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  delay(50);

  // Lê a frequencia de saída do fotodiodo azul
  blue = pulseIn(out, LOW);

  // Analisa se a menor frequencia é vermelha
  if (red < green && red < blue) {
    posicao1();
  }

  // Analisa se a menor frequencia é verde
  if (green < red && green < blue) {
    posicao2();
  }

  // Analisa se a menor frequencia é azul
  if (blue < red && blue < green) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    for (control = 0; control <= 255; control++) {
      analogWrite(vel, control);
    }
    Serial.println("Azul");
  }

  delay(1000);  // Aguarda 1 segundo e reinicia leitura
}

void posicao1() {  //peça vermelha(lado direito)
  pos = 0;

  //braço move para frente
  for (pos = 80; pos >= 40; pos -= 2) {
    servo1.write(pos);
    delay(20);
  }

  //braço desce
  for (pos = 150; pos <= 155; pos++) {
    servo3.write(pos);
    delay(20);
  }

  //fechar garra
  for (pos = 80; pos <= 160; pos++) {
    servo4.write(pos);
    delay(20);
  }
  //delay(50);

  //braço sobe
  for (pos = 155; pos >= 140; pos--) {
    servo3.write(pos);
    delay(20);
  }

  //braço move para trás
  for (pos = 40; pos <= 80; pos += 2) {
    servo1.write(pos);
    delay(20);
  }

  //Braço vai para direita
  for (pos = 75; pos >= 40; pos -= 2) {
    servo2.write(pos);
    delay(20);
  }

  //abrir garra
  for (pos = 160; pos >= 80; pos -= 2) {
    servo4.write(pos);
    delay(20);
  }
}

void posicao2() {  //peça verde(lado esquerdo)
  pos = 0;

  //braço move para frente
  for (pos = 80; pos >= 40; pos -= 2) {
    servo1.write(pos);
    delay(20);
  }
  delay(100);

  //braço desce
  for (pos = 150; pos <= 155; pos++) {
    servo3.write(pos);
    delay(20);
  }
  delay(100);

  //fechar garra
  for (pos = 80; pos <= 160; pos++) {
    servo4.write(pos);
    delay(20);
  }

  //braço sobe
  for (pos = 155; pos >= 140; pos--) {
    servo3.write(pos);
    delay(20);
  }

  //braço move para trás
  for (pos = 40; pos <= 80; pos += 2) {
    servo1.write(pos);
    delay(20);
  }

  //Braço vai para esquerda
  for (pos = 75; pos <= 120; pos += 2) {
    servo2.write(pos);
    delay(20);
  }

  for (pos = 140; pos <= 150; pos++) {
    servo3.write(pos);
    delay(20);
  }

  //abrir garra
  for (pos = 160; pos >= 80; pos -= 2) {
    servo4.write(pos);
    delay(20);
  }
}