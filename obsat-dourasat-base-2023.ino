//----- Declaração das Bibliotecas Principais do Satélite -----//

#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiMulti.h>
#include <TinyGPS.h>
#include <LoRa.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "math.h"

//----- Definição das constantes de cálculo -----//

#define PATM 101325 // Pressão no nível do mar
#define GRAV 6.67408e-11 // Contante gravitacional
#define RAIO 6375109.0 // Raio da terra
#define MASS 5.972e24 // Massa da terra

#define MPU6050_DEVICE_ID 0x68 // Define o valor correto do valor MPU6050_WHO_AM_I

//----- Pinos de Controle do LoRa -----//
/*
#define ss 5 // CHIP SELECT do LoRa - RA02
#define rst 14 // RESET do LoRa - RA02
#define dio0 2 // DIGITAL I/O do LoRa - RA02
*/

/**
 * Enumeração dos estados do satélite
 */
enum EstadoMissao{
  INICIALIZANDO, // Estado de inicialização da missão
  TESTE,         // Estado de teste do satelite
  SUBIDA,        // Estado de ascenção do satelite no balão atmosférico
  EMERGENCIA,    // Estado de emergência que protege o satelite contra destruição
  EXECUCAO,      // Estado em que o satélite está posicionado e começa a receber as coordenadas do caminhão
  DESCIDA,       // Estado de queda controlada do satélite
  RESGATE        // Estado que o satélite sinaliza para ser resgatado
};

EstadoMissao estado;     // Declara o estado dos periféricos
File myFile;             // Delaração do arquivo que será salvo no cartão SD no sistema de arquivos FAT
Adafruit_MPU6050 mpu;    // Declara o sensor Acelerômetro e Giroscópio
Adafruit_BMP085 bmp_ext; // Declara o sensor de Pressão Barométrica
WiFiMulti wifiMulti;     // Declara a biblioteca de multiplas conexões WiFi

const int CS = 2;        // Declara o pino do Chip Select do cartão SD
const char* ssid = "StussiMi9Pro"; // Declara o nome da rede WiFi que irá se conectar
const char* password_wifi = "1234567890"; // Declara a senha da rede WiFi que irá se conectar

String serverName = "https://obsat.org.br/teste_post/envio.php"; // URL do servidor que irá enviar a requisição POST com os dados
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

double angulo_atual; // Declaração do angulo relativo atual em variável global
double posicao_atual[3]; // Declaração da posição relativa atual em variavel global
double velocidade_atual[3]; // Declaração da velocidade relativa atual em variável global
double velocidade[3]; // Declaração da velocidade anterior relativa em variável global
unsigned long tempo; // Declaração do tempo anterior

bool status_sd;  // Status do cartão SD
bool status_mpu; // Status do sensor acelerômetro
bool status_bmp;  // Status do sensor de pressão

bool status_msg;  // Status do envio da mensagem
bool status_wifi;  // Status da conexão wifi

void WriteFile(const char * path, const char * message){
  myFile = SD.open(path, FILE_WRITE);
  if (myFile) {
    Serial.printf("Writing to %s ", path);
    myFile.println(message);
    myFile.close(); // close the file:
    Serial.println("completed.");
  } 
  else {
    Serial.println("error opening file ");
    Serial.println(path);
  }
}

void ReadFile(const char * path){
  // open the file for reading:
  myFile = SD.open(path);
  if (myFile) {
     Serial.printf("Reading file from %s\n", path);
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close(); // close the file:
  } 
  else {
    Serial.println("error opening test.txt");
  }
}

/**
 * Calcula a altitude teorica usando a pressão absoluta em Pascal
 */
double altitude_teorica(double pressao){
  return (log(PATM)-log(pressao))/1.15e-4;
}

/**
 * Converte a pressão teórica em altitude relativa em metros
 */
double pressao_teorica(double altitude){
  return PATM/exp(1.15e-4*altitude);
}

/**
 * Calcula a gravidade teórica pela altitude
 */
double gravidade_teorica(double altitude){
  return (GRAV*MASS)/((RAIO-altitude)*(RAIO-altitude));
}

double getTemperatura(){
  // TODO: retornar a temperatura
}

double getHumidade(){
  // TODO: retorna a humidade
}

double getPressao(){
  // TODO: retorna a pressão
}

double getLuminosidade(){
  // TODO: retorna a luminosidade
}

double getGiroscopio(int eixo){
  // TODO: retorna a angulação
}

double getAceleracao(int eixo){
  // TODO: retorna a aceleração
}

double getMagnetismo(int eixo){
  // TODO: retorna a força magnética
}

/**
 * Rotina de inicialização do satélite
 */
void setup() {

  //----- Inicialização das Interfaces de Comunicação Seriais -----//
  
  Serial.begin(115200);   // Inicia a comunicação serial 1 que comunica com o conversor USB/Serial
  Serial2.begin(115200);   // Inicia a comunicação serial 2 que comunica com o módulo GSM/GPS/GPRS

  //----- Verificação Inicial do WiFi -----//
  
  wifiMulti.addAP(ssid, password_wifi); // Conectando à rede WiFi
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.print("Falha ao conectar à rede: ");
    Serial.print(ssid);
    Serial.print(" com a senha: ");
    Serial.print(password_wifi);
  }
  Serial.print("Conectado à rede WiFi com IP Address: ");
  Serial.println(WiFi.localIP());
  status_msg = false;

  //----- Verificação inicial do Cartão SD -----//
  
  status_sd = SD.begin(CS);
  if (!status_sd) {
    Serial.println("Falha no módulo do cartão SD!");
    delay(100);
  }

  //----- Verificação inicial do Giroscópio e Acelerômetro -----//
  
  status_mpu = mpu.begin(0x68);
  if (!status_mpu) {
    Serial.println("Falha no módulo MPU6050!");
    delay(100);
  } else {
    //----- Ranges possíveis para acelerômetro -----//
    //MPU6050_RANGE_2_G
    //MPU6050_RANGE_4_G
    //MPU6050_RANGE_8_G
    //MPU6050_RANGE_16_G
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    //----- Ranges possíveis para giroscópio -----//
    //MPU6050_RANGE_250_DEG
    //MPU6050_RANGE_500_DEG
    //MPU6050_RANGE_1000_DEG
    //MPU6050_RANGE_2000_DEG
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    //----- Possíveis bandas de filtragem -----//
    //MPU6050_BAND_260_HZ
    //MPU6050_BAND_184_HZ
    //MPU6050_BAND_94_HZ
    //MPU6050_BAND_44_HZ
    //MPU6050_BAND_21_HZ
    //MPU6050_BAND_10_HZ
    //MPU6050_BAND_5_HZ
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  }

  //----- Verificação inicial do Sensor de Pressão -----//
  
  status_bmp = bmp_ext.begin(0x77);
  if (!status_bmp) {
    Serial.println("Falha no módulo BMP180!");
    delay(100);
  }

  //----- Inicialização do Módulo LoRa Ai Thinker -----//
  
  /*
  LoRa.setPins(ss, rst, dio0);          // definindo os pinos de controle do LoRa
  cubeSat.setLed(L1, HIGH);             // indicador LED da inicialização do LoRa
  Serial.print("Inicialização LoRa ");  // Aviso de inicialização do LoRa
  while (!LoRa.begin(62.5E3)) {          // Enquanto o LoRa não inicia o programa permanece no ciclo
    Serial.print(".");                  // Mensagem de espera
    delay(500);                         // Espera de meio segundo
  }
  Serial.println("");                   // Encerramento da mensagem de espera
  LoRa.setSyncWord(0xF3);               // Mensagem de sincronização
  cubeSat.setLed(L1, LOW);              // desligar LED indicador de inicializador do LoRa
  */

  //----- Inicialização das Variáveis Globais -----//

  estado = INICIALIZANDO;  // Seta o estado inicial como iniciando a missão
  angulo_atual = 0;        // zerando o angulo atual
  posicao_atual[0] = 0;    // zerando a posição atual em x
  posicao_atual[1] = 0;    // zerando a posição atual em y
  posicao_atual[2] = 0;    // zerando a posição atual em z
  velocidade_atual[0] = 0; // zerando a velocidade atual em x
  velocidade_atual[1] = 0; // zerando a velocidade atual em y
  velocidade_atual[2] = 0; // zerando a velocidade atual em z
  velocidade[0] = 0;       // zerando a velocidade anterior em x
  velocidade[1] = 0;       // zerando a velocidade anterior em y
  velocidade[2] = 0;       // zerando a velocidade anterior em z
  tempo = micros();        // definindo o offset do tempo inicial

  //pinMode(27, OUTPUT);   // Pino definido como output
  //digitalWrite(18, LOW); // Definir para nível lógico baixo
  //digitalWrite(18, HIGH);// Definir para nível lógico alto
}

/**
 * Rotina principal do firmware do satélite
 */
void loop() {
  //---------------------------------------------------------------------------------
  //                  Rotina de verificação de situação atual
  //---------------------------------------------------------------------------------
  sensors_event_t a, g, temp;
  double temperatura; // Temperatura: °C
  double humidade = getHumidade(); // Umidade: %HR
  double pressao = getPressao(); // Pressão: pa
  double luminosidade = getLuminosidade(); // Luminosidade: %
  double aceleracao[3]; // Aceleração nos eixos XYZ
  double rotacao[3]; // Rotação nos eixos XYZ
  double magnetometro[3]; // Magnetismo nos eixos XYZ
  for(int i = 0; i < 3; i++){
    aceleracao[i] = getAceleracao(i);    // m/s^2
    rotacao[i] = getGiroscopio(i);        // graus/s
    magnetometro[i] = getMagnetismo(i);   // uT
  }
  double altitude = altitude_teorica(pressao); // Altitude: m
  unsigned long t_atual = micros(); // Tempo atual da execução do arduino em us
  unsigned long t_delta; // Delta de tempo
  if(t_atual > tempo) t_delta = t_atual-tempo; else t_delta = t_atual+((unsigned long)4294967295-tempo); // Delta tempo entre os ciclos da máquina de estados
  tempo = t_atual; // Atualização do tempo
  double gravidade_local = gravidade_teorica(altitude); // Gravidade teórica local
  double v_delta[3]; // Declaração do delta de velocidades
  for(int i = 0; i < 3; i++){
    velocidade_atual[i] = aceleracao[i]*t_delta; // Velocidade relativa atual
    v_delta[i] = velocidade_atual[i]-velocidade[i]; // Delta de velocidade
    velocidade[i] = velocidade_atual[i]; // Atualizando a velocidade
    posicao_atual[i] = v_delta[i]*t_delta; // Posição relativa atual
  }
  //---------------------------------------------------------------------------------
  //         Rotinas de seleção de estados da missão
  //---------------------------------------------------------------------------------
  switch(estado){
    case INICIALIZANDO: 
      break;
    case TESTE: 
      break;
    case SUBIDA: 
      break;
    case EMERGENCIA: 
      break;
    case EXECUCAO: 
      break;
    case DESCIDA: 
      break;
    case RESGATE: 
      break;
    default: 
      break;
  }
  //---------------------------------------------------------------------------------
  //         Rotinas de reorganização das variáveis incrementais
  //---------------------------------------------------------------------------------
  
  //----- Rotina de Envio de Mensagens Utilizando o Long Range (LoRa) Radio -----//
  /*
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
  */

  //----- Rotina de Envio de Mensagens Utilizando o WiFi -----//
  if(status_msg){
    HTTPClient http;   
     
    http.begin(serverName);  
    http.addHeader("Content-Type", "application/json");         
       
    StaticJsonDocument<384> doc;
  
    doc["equipe"] = 1;
    doc["bateria"] = 100;
    doc["temperatura"] = temperatura;
    doc["pressao"] = pressao;
    
    JsonArray giroscopio = doc.createNestedArray("giroscopio");
    giroscopio.add(rotacao[0]);
    giroscopio.add(rotacao[1]);
    giroscopio.add(rotacao[2]);
    
    JsonArray acelerometro = doc.createNestedArray("acelerometro");
    acelerometro.add(aceleracao[0]);
    acelerometro.add(aceleracao[1]);
    acelerometro.add(aceleracao[2]);
    
    JsonObject payload = doc.createNestedObject("payload");
    payload["nome"] = "dourasat";
    payload["saude"] = "100";
    payload["radiacao"] = "100";
    payload["luv"] = "100";
    payload["temp"] = "100";
    
    String requestBody;
    
    serializeJson(doc, requestBody);
    
    //-------------------------------------------------//
    int httpResponseCode = http.POST(requestBody);
    if(httpResponseCode>0){
      String response = http.getString();                       
      Serial.println(httpResponseCode);   
      Serial.println(response);
    }
    else {
      Serial.printf("Error occurred while sending HTTP POST: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    status_msg = true;
  }
}
