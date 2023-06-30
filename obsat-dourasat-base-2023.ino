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

#define LED 2 // Pino do Led da Placa do ESP32 DevKit

//----- Pinos de Controle do LoRa - Preparado para Telemetria da Fase 4 -----//
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

/**
 * Enumeração direção vertical do satélite
 */
enum DirecaoVertical{
  SUBINDO,          // O satélite está em ascenção
  PARADO,           // O satélite nem está subindo nem descendo
  DESCENDO,         // O satélite está descendo
};

EstadoMissao estado;     // Declara o estado dos periféricos como glogal
DirecaoVertical direcaov;// Declara a direção vertical do satélite como glogal
File arquivo;            // Delaração do arquivo que será salvo no cartão SD no sistema de arquivos FAT como glogal
Adafruit_MPU6050 mpu;    // Declara o sensor Acelerômetro e Giroscópio como glogal
Adafruit_BMP085 bmp_ext; // Declara o sensor de Pressão Barométrica como glogal
WiFiMulti wifiMulti;     // Declara instância da biblioteca de multiplas conexões WiFi como glogal

const int CS = 13;       // Declara o pino do Chip Select do cartão SD
const int BAT = 4;       // Pino da bateria
const char* ssid = "StussiMi9Pro"; // Declara o nome da rede WiFi que irá se conectar
const char* password_wifi = "1234567890"; // Declara a senha da rede WiFi que irá se conectar

String serverName = "https://obsat.org.br/teste_post/envio.php"; // URL do servidor que irá enviar a requisição POST com os dados
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;
int envia;

double altitudeAnterior;  // Altitude do ciclo anterior
double altitudeAcumulada; // Altitude acumulada de vários ciclos
double angulo_atual; // Declaração do angulo relativo atual em variável global
double posicao_atual[3]; // Declaração da posição relativa atual em variavel global
double velocidade_atual[3]; // Declaração da velocidade relativa atual em variável global
double velocidade[3]; // Declaração da velocidade anterior relativa em variável global
unsigned long tempo; // Declaração do tempo anterior
unsigned long contaCiclo; // Contador de ciclos da altitude
unsigned long mensagem_tmp; // Tempo do contador de tempo das mensagens

bool status_sd;  // Status do cartão SD
bool status_mpu; // Status do sensor acelerômetro
bool status_bmp;  // Status do sensor de pressão

bool status_msg;  // Status do envio da mensagem
bool status_wifi;  // Status da conexão wifi

/**
 * Escreve no arquivo especificado a informação desejada
 */
void WriteFile(const char * path, const char * message){
  arquivo = SD.open(path, FILE_WRITE);
  if (arquivo) {                        // Caso o arquivo exista
    Serial.printf("Escrevendo no arquivo: %s - ", path);
    arquivo.println(message);           // Gravando a mensagem no arquivo
    arquivo.close();                    // fechando o arquivo
    Serial.println("Sucesso!");
  } 
  else {
    Serial.printf("Erro ao abrir o arquivo: %s\n",path);
  }
}

/**
 * Lê o conteúdo do arquivo especificado
 */
void ReadFile(const char * path){
  arquivo = SD.open(path);
  if (arquivo) {
     Serial.printf("Lendo: %s ...\n", path);
    while (arquivo.available()) {
      Serial.write(arquivo.read());
    }
    arquivo.close();
  } 
  else {
    Serial.println("Erro na leitura.txt");
  }
}

/**
 * Transmite a mensagem padrão requerida pela OBSAT para o servidor especificado no formato JSON via WiFi utilizando o verbo POST
 */
void TransmissaoWifi(String urlServer, int bat, int tmp, int pres, float rx, float ry, float rz, float ax, float ay, float az, float tens, float corr, float rad, float luv, float temp){
    HTTPClient http;   
     
    http.begin(urlServer);  
    http.addHeader("Content-Type", "application/json");         
       
    StaticJsonDocument<384> doc;
  
    doc["equipe"] = 1;
    doc["bateria"] = bat;
    doc["temperatura"] = tmp;
    doc["pressao"] = pres;
    
    JsonArray giroscopio = doc.createNestedArray("giroscopio");
    giroscopio.add(rx);
    giroscopio.add(ry);
    giroscopio.add(rz);
    
    JsonArray acelerometro = doc.createNestedArray("acelerometro");
    acelerometro.add(ax);
    acelerometro.add(ay);
    acelerometro.add(az);
    
    JsonObject payload = doc.createNestedObject("payload");
    payload["tens"] = tens;
    payload["corr"] = corr;
    payload["rad"] = rad;
    payload["luv"] = luv;
    payload["temp"] = temp;
    
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
      Serial.printf("Erro enviando HTTP POST: %s\n", http.errorToString(httpResponseCode).c_str());
      estado = EMERGENCIA;
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
  return 0;
}

double getHumidade(){
  // TODO: retorna a humidade
  return 0;
}

double getPressao(){
  // TODO: retorna a pressão
  return 0;
}

double getLuminosidade(){
  // TODO: retorna a luminosidade
  return 0;
}

double getGiroscopio(sensors_event_t sensor, int eixo){
  switch(eixo){
    case 0: return sensor.acceleration.x; break;
    case 1: return sensor.acceleration.y; break;
    case 2: return sensor.acceleration.z; break;
    default: return sensor.acceleration.x; break;
  }
}

double getAceleracao(sensors_event_t sensor, int eixo){
  switch(eixo){
    case 0: return sensor.gyro.x; break;
    case 1: return sensor.gyro.y; break;
    case 2: return sensor.gyro.z; break;
    default: return sensor.gyro.x; break;
  }
}

double getMagnetismo(int eixo){
  // TODO: retorna a força magnética
  return 0;
}

void piscaLED(int tempo){
  digitalWrite(LED, HIGH);
  delay(tempo);
  digitalWrite(LED, LOW);
  delay(tempo);
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
    status_wifi = false;
  } else {
    Serial.print("Conectado à rede WiFi com IP Address: ");
    Serial.println(WiFi.localIP());
    status_wifi = true;
  }
  
  status_msg = false;
  envia = 0;

  //----- Verificação inicial do Cartão SD -----//
  
  status_sd = SD.begin(CS);
  if (!status_sd) {
    Serial.println("Falha no módulo do cartão SD!");
    delay(100);
  } else {
    Serial.println("Cartão SD pronto!");
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
    Serial.println("Giroscópio e Acelerômetro prontos!");
  }

  //----- Verificação inicial do Sensor de Pressão -----//
  
  status_bmp = bmp_ext.begin(0x77);
  if (!status_bmp) {
    Serial.println("Falha no módulo BMP180!");
    delay(100);
  } else {
    Serial.println("Barômetro pronto!");
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
  direcaov = PARADO;       // Seta o estado inicial como parado ou estacionado
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
  mensagem_tmp = 0;        // zera o contador de tempo para mandar mensagens
  altitudeAcumulada = 0;   // zera o acumulador de altitude
  contaCiclo = 0;          // zera o contador de ciclos da altitude
  altitudeAnterior = altitude_teorica(bmp_ext.readPressure()); // Define o offset de altitude como a altitude inicial

  pinMode(BAT,INPUT);
  pinMode(LED, OUTPUT);
  //pinMode(27, OUTPUT);   // Pino definido como output
  //digitalWrite(18, LOW); // Definir para nível lógico baixo
  //digitalWrite(18, HIGH);// Definir para nível lógico alto

  digitalWrite(LED, HIGH);
}

/**
 * Rotina principal do firmware do satélite
 */
void loop() {
  //---------------------------------------------------------------------------------
  //                  Rotina de verificação de situação atual
  //---------------------------------------------------------------------------------
  sensors_event_t a, g, temp; // Variáveis de Verificação do Giroscópio/Acelerômetro
  mpu.getEvent(&a, &g, &temp); // Aquisição dos sinais do Giroscópio/Acelerômetro
  double temperatura = bmp_ext.readTemperature(); // Temperatura: °C
  double humidade = getHumidade(); // Umidade: %HR
  double pressao = bmp_ext.readPressure(); // Pressão: pa
  double luminosidade = getLuminosidade(); // Luminosidade: %
  double aceleracao[3]; // Aceleração nos eixos XYZ
  double rotacao[3]; // Rotação nos eixos XYZ
  double magnetometro[3]; // Magnetismo nos eixos XYZ
  double bateria = ((3.3/1024.0)*analogRead(BAT));
  for(int i = 0; i < 3; i++){
    aceleracao[i] = getAceleracao(a, i);    // m/s^2
    rotacao[i] = getGiroscopio(g, i);        // graus/s
    magnetometro[i] = getMagnetismo(i);   // uT
  }
  double altitude = altitude_teorica(pressao); // Altitude: m
  double deltaAltitude = altitude-altitudeAnterior; //define o delta de altitude
  altitudeAnterior = altitude; // Atualiza a altitude anterior
  
  unsigned long t_atual = micros(); // Tempo atual da execução do ESP32 em us
  unsigned long t_delta; // Delta de tempo
  if(t_atual > tempo) t_delta = t_atual-tempo;  // Armazena o Delta tempo entre os ciclos do loop
                 // Delta tempo entre os ciclos da máquina de estados caso
                 // ocorra estouro de pilha do contador de tempo do ESP32.
  else t_delta = t_atual+((unsigned long)4294967295-tempo); 
  tempo = t_atual; // Atualização do tempo
  
  mensagem_tmp += t_delta; // Soma o tempo deccorrido desde a ultima mensagem
  if(mensagem_tmp >= 10000000){   // Caso tenham se passado 10 segundos
    status_msg = false;           // Reset do flag de mensagem da telemetria
    mensagem_tmp = 0;             // Zera o contador de tempo de mensagem
  }

  altitudeAcumulada += deltaAltitude; // Soma a altitude deccorrida desde o ultimo ciclo
  contaCiclo += t_delta;              // Soma o tempo da altitude
  if(altitudeAcumulada >= 20.0 && contaCiclo < 20000000){   // Caso tenham se passado 20 metros ou 50 
    altitudeAcumulada = 0;            // Reset do flag de mensagem da telemetria
    contaCiclo = 0;                   // Zera o contador de ciclos de mensagem
    direcaov = SUBINDO;               // O satélite está subindo
  } else if(altitudeAcumulada <= -20.0 && contaCiclo < 20000000){
    altitudeAcumulada = 0;            // Reset do flag de mensagem da telemetria
    contaCiclo = 0;                   // Zera o contador de ciclos de mensagem
    direcaov = DESCENDO;              // O satélite está descendo
  } else if(contaCiclo >= 20000000){
    altitudeAcumulada = 0;            // Reset do flag de mensagem da telemetria
    contaCiclo = 0;                   // Zera o contador de ciclos de mensagem
    direcaov = PARADO;                // O satélite está parado
  }
  
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
      if(direcaov == SUBINDO) estado = SUBIDA; // Caso o satélite esteja subindo muda para o estado subindo
      else if(direcaov == DESCENDO) estado = DESCIDA; // Caso o satélite esteja descendo muda para o estado descida
      else if(direcaov == PARADO && altitude <= 800) { // Caso o satélite esteja parado no chão ou próximo do chão
        //Rotina de inicialização para o lançamento recolhendo as partes móveis e aguardando a subida
      }
      break;
    case TESTE: 
      //----------------------------------------------------------------------------
      //         Rotina especial de testes do sistema do satélite
      //----------------------------------------------------------------------------
      // A rotina de testes informa todos os 
      // valores da variáveis através da serial
      Serial.print("estado: ");
      Serial.print(estado);
      Serial.print(" alt: ");
      Serial.print(altitude);
      Serial.print(" gravidade_local: ");
      Serial.print(gravidade_local);
      Serial.print(" analog: ");
      Serial.print(analogRead(BAT));
      Serial.print(" bat: ");
      Serial.print(bateria);
      Serial.print(" porcent: ");
      Serial.print(((bateria/3.3)*100.0));
      for(int i = 0; i < 3; i++){
        Serial.print(" eix: ");
        Serial.print(i);
        Serial.print(" r: ");
        Serial.print(rotacao[i]);
        Serial.print(" a: ");
        Serial.print(aceleracao[i]);
      }
      Serial.print(" press: ");
      Serial.print(pressao);
      Serial.print(" temp: ");
      Serial.print(temperatura);
      Serial.print(" T: ");
      Serial.println(t_atual);
      break;
    case SUBIDA: 
      digitalWrite(LED, LOW);

      //----- Rotina de Envio de Mensagens Utilizando o Long Range (LoRa) Radio - Preparado para Telemetria da Fase 4 -----//
      /*
      int packetSize = LoRa.parsePacket();
      if (packetSize) {
        // Recebeu o pacote
        Serial.print("Pacote Recebido! '");
    
        // Pacote Lido
        while (LoRa.available()) {
          Serial.print((char)LoRa.read());
        }
    
        // Mostra RSSI (força do sinal recebido) do pacote
        Serial.print("' com RSSI ");
        Serial.println(LoRa.packetRssi());
      }
      */
      
      //----- Rotina de Envio de Mensagens Utilizando o WiFi -----//
      if((!status_msg) && (status_wifi)){ // Condição para mandar a mensagem a cada 10 Segundos
        
        Serial.println("Enviando MSG WiFi");
    
        TransmissaoWifi(
          serverName,           // Endereço de Envio para o Servidor
          (int)((bateria/3.3)*100.0), // Porcentagem do nível da bateria
          (int)temperatura,     // Temperatura
          (int)pressao,         // Pressão em Pa
          rotacao[0],      // Rotação em X
          rotacao[1],      // Rotação em Y
          rotacao[2],      // Rotação em Z
          aceleracao[0],   // Aceleração em X
          aceleracao[1],   // Aceleração em Y
          aceleracao[2],   // Aceleração em Z
          10,                   // Tensão
          10,                   // Corrente
          0,                    // Radiação
          0,                    // Luminosidade Ultra Violeta
          temp.temperature);    // Temperatura da Célula
        
        status_msg = true;
      }
      if(altitude >= 2000) estado = EXECUCAO;
      break;
    case EMERGENCIA: 
      // Reconfere todos os subsistemas e reinicia o satélite
      Serial.begin(115200);   // Inicia a comunicação serial 1 que comunica com o conversor USB/Serial
      Serial2.begin(115200);   // Inicia a comunicação serial 2 que comunica com o módulo GSM/GPS/GPRS
      wifiMulti.addAP(ssid, password_wifi); // Conectando à rede WiFi
      if (wifiMulti.run() != WL_CONNECTED) status_wifi = false; // Caso não consiga conectar marca a flag falsa
      else status_wifi = true;                                  // Caso contrário marca a flag verdadeira
      status_sd = SD.begin(CS);     // Reinicialização do cartão de memória
      status_mpu = mpu.begin(0x68); // Reinicialização do Acelerômetro/Giroscópio
      if (status_mpu) {
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // Range do Acelerômetro para 2G
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);      // Range do Giroscópio para 250 Deg/s
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);    // Filtro de Banda < 5 Hz (corta baixa)
      }
      status_bmp = bmp_ext.begin(0x77); // Reinicialização do Barômetro
      estado = INICIALIZANDO;  // Estado inicial Reiniciando
      direcaov = PARADO;       // Movimentação parado ou estacionado
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
      mensagem_tmp = 0;        // zera o contador de tempo para mandar mensagens
      altitudeAcumulada = 0;   // zera o acumulador de altitude
      contaCiclo = 0;          // zera o contador de ciclos da altitude
      altitudeAnterior = altitude_teorica(bmp_ext.readPressure()); // Define o offset de altitude como a altitude inicial
      break;
    case EXECUCAO: 
      digitalWrite(LED, LOW);

      //----- Rotina de Envio de Mensagens Utilizando o Long Range (LoRa) Radio - Preparado para Telemetria da Fase 4 -----//
      /*
      int packetSize = LoRa.parsePacket();
      if (packetSize) {
        // Recebeu o pacote
        Serial.print("Pacote Recebido! '");
    
        // Pacote Lido
        while (LoRa.available()) {
          Serial.print((char)LoRa.read());
        }
    
        // Mostra RSSI (força do sinal recebido) do pacote
        Serial.print("' com RSSI ");
        Serial.println(LoRa.packetRssi());
      }
      */
      
      //----- Rotina de Envio de Mensagens Utilizando o WiFi -----//
      if((!status_msg) && (status_wifi)){ // Condição para mandar a mensagem a cada 10 Segundos
        
        Serial.println("Enviando MSG WiFi");
    
        TransmissaoWifi(
          serverName,           // Endereço de Envio para o Servidor
          (int)((bateria/3.3)*100.0), // Porcentagem do nível da bateria
          (int)temperatura,     // Temperatura
          (int)pressao,         // Pressão em Pa
          rotacao[0],      // Rotação em X
          rotacao[1],      // Rotação em Y
          rotacao[2],      // Rotação em Z
          aceleracao[0],   // Aceleração em X
          aceleracao[1],   // Aceleração em Y
          aceleracao[2],   // Aceleração em Z
          10,                   // Tensão
          10,                   // Corrente
          0,                    // Radiação
          0,                    // Luminosidade Ultra Violeta
          temp.temperature);    // Temperatura da Célula
        
        status_msg = true;
      }
      if(altitude >= 2000) estado = EXECUCAO;
      break;
    case DESCIDA:
      // Recolhe todas as partes móveis e se propara para o pouso
      piscaLED(100);
      break;
    case RESGATE: 
      // No resgate aciona as luzes para facilitar a identificação do satélite
      piscaLED(500);
      break;
    default: 
      Serial.println("Erro ao especificar o estado!");
      estado = EMERGENCIA;
      break;
  }
  //---------------------------------------------------------------------------------
  //         Rotinas de reorganização das variáveis incrementais
  //---------------------------------------------------------------------------------
  
  

  
}
