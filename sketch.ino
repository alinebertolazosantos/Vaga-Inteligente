//Inclusão das blibliotecas utilizadas
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <BlynkSimpleEsp32.h>
#include <PubSubClient.h>

//Definição da conexão do Blynk
#define BLYNK_TEMPLATE_ID "TMPL2NMTPu_nN"
#define BLYNK_TEMPLATE_NAME "Vaga inteligente"
#define BLYNK_AUTH_TOKEN "haPXyseb_Md3xn_vNXASsjIquht0dwTO"

//Conexão com a internet própria do Wokwi
char auth[] = BLYNK_AUTH_TOKEN;
const char *SSID = "Wokwi-GUEST"; 
const char *PASS = "";      

//Variaveis para conexão com o servidor
WiFiClient wifiCliente;
PubSubClient cliente;

//Variaveis utilizadas com o blynk
BlynkTimer timer;
int value0, value1, value2, value3;

//Definição dos pinos do primeiro sensor
#define trig1 25
#define echo1 26

//Definição dos leds do primeiro sensor 
#define vagaAberta1 2 //Led utilizado para quando não há veiculos no raio de alcance definido do sensor
#define vagaOcupada1 4 //Led utilizado para quando há veiculos no raio de alcance do definido sensor

//Definição dos pinos do segundo sensor
#define trig2 32
#define echo2 33

//Definição dos leds do segundo sensor 
#define vagaAberta2 15 //Led utilizado para quando não há veiculos no raio de alcance definido do sensor
#define vagaOcupada2 13 //Led utilizado para quando há veiculos no raio de alcance do definido sensor

//Definir a maxima distancia para ser considerada estacionada nos sensores
#define distanciaMaxima 10.00 //sensor 1

//Definição das tarefas
void task_sensor1(void *parametros);
void task_sensor2(void *parametros);
void task_estado_vaga1(void *parametros); 
void task_estado_vaga2(void *parametros); 
void task_blynk(void *parametros);

//Definição das filas utilizadas (queues)
QueueHandle_t xQueue_sensor1, xQueue_sensor2, xQueue_vagaAberta, xQueue_vagaOcupada;

//Definição da variavel do semaforo
SemaphoreHandle_t xSerial_semaphore;

void setup() {
  //Iniciar serial
  Serial.begin(9600);

  //Iniciar o blynk
  Blynk.begin(auth, SSID, PASS);   

  // Iniciar conexao wifi
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(SSID);
  WiFi.begin(SSID, PASS);

  // Verificar conexao wifi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Exibir mensagem de sucesso
  Serial.println("");
  Serial.println("***WiFi connected. ***");

  //Iniciar e configurar os Pinos do LEDS do sensor 1
  pinMode(vagaAberta1, OUTPUT);
  digitalWrite(vagaAberta1, LOW);
  pinMode(vagaOcupada1, OUTPUT);
  digitalWrite(vagaOcupada1, LOW);

  //Configuração dos pinos do sensor de movimento 1 
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  digitalWrite(trig1, LOW);

  //Iniciar e configurar os Pinos do LEDS do sensor 1
  pinMode(vagaAberta2, OUTPUT);
  digitalWrite(vagaAberta2, LOW);
  pinMode(vagaOcupada2, OUTPUT);
  digitalWrite(vagaOcupada2, LOW);

  //Configuração dos pinos do sensor de movimento
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  digitalWrite(trig2, LOW);

   while (!Serial) {
     //Garante que só rode se a serial estiver funcionando
    ; 
  }

  //Criação das filas 
  xQueue_sensor1 = xQueueCreate( 1, sizeof( float ) );
  xQueue_sensor2 = xQueueCreate( 1, sizeof( float ) );
  xQueue_vagaAberta = xQueueCreate( 1, sizeof( float ) );
  xQueue_vagaOcupada = xQueueCreate( 1, sizeof( float ) );

  //Criação do semaforo
  xSerial_semaphore = xSemaphoreCreateMutex();

  //Verificação de que o semaforo foi criado
   if (xSerial_semaphore == NULL)
  {
    Serial.println("Erro: não é possivel criar o semaforo");
    while (1); 
  }

  //Criação das tarefas
  xTaskCreate(
    task_sensor1
    , "sensor1"
    , 4096
    , NULL
    , 5
    , NULL
  );
 xTaskCreate(
    task_sensor2
    , "sensor2"
    , 4096
    , NULL
    , 5
    , NULL
  );
   xTaskCreate(
    task_estado_vaga1
    , "estado_vaga1"
    , 4096 
    , NULL
    , 3
    , NULL
  );
   xTaskCreate(
    task_estado_vaga2
    , "estado_vaga2"
    , 4096
    , NULL
    , 3
    , NULL
  );

    xTaskCreatePinnedToCore(
    task_blynk,
    "task_blynk",
    4096,
    NULL,
    4,
    NULL,
    PRO_CPU_NUM);
    
}

void loop() {
    // Tudo é executado nas tarefas. Há nada a ser feito aqui.
}

void task_sensor1(void *parametros){
  (void) parametros;
  unsigned long tempo = 0.0;
  float distancia = 0.0;

  while (1)
  {
    //Recebe as informações do sensor através do trig
    digitalWrite(trig1, HIGH);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    digitalWrite(trig1, LOW);

    //Receber o tempo da distancia encontrada através do echo
    tempo = pulseIn(echo1, HIGH);

    //Transformar o tempo em distancia
    distancia = tempo / 58;
   
   // Envia a distancia para as tarefas a partir de filas
   xQueueOverwrite(xQueue_sensor1, (void *)&distancia); 

  //Enviar informações do sensor 1 para o Blynk
   Blynk.virtualWrite(V4, distancia);

  //Para fins de teste de ocupação de stack, printa na serial o high water mark
   xSemaphoreTake(xSerial_semaphore, portMAX_DELAY );

  //Libera o semáforo da tarefa
  xSemaphoreGive(xSerial_semaphore);
  }
}

void task_sensor2 (void *parametros){
 (void) parametros;
  unsigned long tempo;
  float distancia;

 while (1)
  {
    //Recebe as informações do sensor através do trig
    digitalWrite(trig2, HIGH);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    digitalWrite(trig2, LOW);

    //Receber o tempo da distancia encontrada através do echo
    tempo = pulseIn(echo2, HIGH);

    //Transformar o tempo em distancia
    distancia = tempo / 58;
   
   // Envia a distancia para as tarefas a partir de filas
   xQueueOverwrite(xQueue_sensor2, (void *)&distancia); 

  //Enviar informações do sensor 1 para o Blynk
   Blynk.virtualWrite(V5, distancia);

  //Para fins de teste de ocupação de stack, printa na serial o high water mark
   xSemaphoreTake(xSerial_semaphore, portMAX_DELAY );

  //Libera o semáforo da tarefa
  xSemaphoreGive(xSerial_semaphore);
  }

}

BLYNK_WRITE(V0)
{
  value0 = param.asInt();
  digitalWrite(vagaAberta1, value0);
}
BLYNK_WRITE(V1)
{
  value1 = param.asInt();
  digitalWrite(vagaOcupada1, value1);
}
void task_estado_vaga1(void *parametros){
  (void) parametros;
  float distancia_recebida = 0.00;

  while (1)
  {
  //Recebe o conteudo da fila do sensor 1
  xQueueReceive(xQueue_sensor1,  (void *)&distancia_recebida, portMAX_DELAY);

  //Caso a distancia recebida seja menor que a distancia definida 10
  if (distancia_recebida >= distanciaMaxima) {
    //Ascende o led que representa vaga aberta do primeiro sensor
    digitalWrite(vagaAberta1, HIGH); 

    //Desligar o led que representa vaga ocupada do primeiro sensor
    digitalWrite(vagaOcupada1, LOW);

    Serial.println("Vaga 01 - Primeiro andar DISPONÍVEL");
  
    //Envia a mensagem para o pino digital do Blynk para ascender o led
    Blynk.virtualWrite(V0, HIGH); 

  //Envia a mensagem para o pino digital do Blynk para desligar o led
    Blynk.virtualWrite(V1, LOW); 


  } 
  //Caso não seja
  else {
    //Desligar o led que representa vaga aberta do primeiro sensor
    digitalWrite(vagaAberta1, LOW); 

    //Ascende o led que representa vaga ocupada do primeiro sensor
    digitalWrite(vagaOcupada1, HIGH);



    //Envia a mensagem para o pino digital do Blynk para ascender o led
    Blynk.virtualWrite(V0, LOW); 

    //Envia a mensagem para o pino digital do Blynk para desligar o led
    Blynk.virtualWrite(V1, HIGH); 

    Serial.println("Vaga 01 - Primeiro andar OCUPADA");
  }
   //Para fins de teste de ocupação de stack, printa na serial o high water mark
   xSemaphoreTake(xSerial_semaphore, portMAX_DELAY );

  //Libera o semáforo da tarefa
  xSemaphoreGive(xSerial_semaphore);
  }
}

BLYNK_WRITE(V2)
{
  value2 = param.asInt();
  digitalWrite(vagaAberta2, value2);
}
BLYNK_WRITE(V3)
{
  value3 = param.asInt();
  digitalWrite(vagaOcupada2, value3);
}
void task_estado_vaga2(void *parametros){
 (void) parametros;
  float distancia_recebida = 0.00;

  while (1)
  {
  //Recebe o conteudo da fila do sensor 1
  xQueueReceive(xQueue_sensor2,  (void *)&distancia_recebida, portMAX_DELAY);

  //Caso a distancia recebida seja menor que a distancia definida 10
  if (distancia_recebida >= distanciaMaxima) {
    //Ascende o led que representa vaga aberta do primeiro sensor
    digitalWrite(vagaAberta2, HIGH); 

    //Desligar o led que representa vaga ocupada do primeiro sensor
    digitalWrite(vagaOcupada2, LOW);
 
    Serial.println("Vaga 02 - Primeiro andar DISPONÍVEL");

    //Envia a mensagem para o pino digital do Blynk para ascender o led
    Blynk.virtualWrite(V2, HIGH); 

  //Envia a mensagem para o pino digital do Blynk para desligar o led
    Blynk.virtualWrite(V3, LOW); 

  } 
  //Caso não seja
  else {
    //Desligar o led que representa vaga aberta do primeiro sensor
    digitalWrite(vagaAberta2, LOW); 

    //Ascende o led que representa vaga ocupada do primeiro sensor
    digitalWrite(vagaOcupada2, HIGH);

    //Envia a mensagem para o pino digital do Blynk para ascender o led
    Blynk.virtualWrite(V2, LOW); 

    //Envia a mensagem para o pino digital do Blynk para desligar o led
    Blynk.virtualWrite(V3, HIGH); 

    Serial.println("Vaga 02 - Primeiro andar OCUPADA");
  }
   //Para fins de teste de ocupação de stack, printa na serial o high water mark
   xSemaphoreTake(xSerial_semaphore, portMAX_DELAY );

  //Libera o semáforo da tarefa
  xSemaphoreGive(xSerial_semaphore);
  }
}

//Task de controle do Blynk
void task_blynk(void *arg){
  while (1) {
    Blynk.run();
    timer.run();
    vTaskDelay(100);

  //Para fins de teste de ocupação de stack, printa na serial o high water mark
  xSemaphoreTake(xSerial_semaphore, portMAX_DELAY );

  //Libera o semáforo da tarefa
  xSemaphoreGive(xSerial_semaphore);
  }
}


