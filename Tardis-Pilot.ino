#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

Servo direcao;
Servo esc;

int pinDirecao = 25;
int pinEsc = 26;

int angleMin = 30;
int angleMax = 150;

int escMin = 1000;
int escMax = 2000;
int escStop = 1500;

int offsetDirecao = -7;

// Aceleração normal
// int aceleracaoFixa = 1350;  // ajusta pra velocidade boa
int aceleracaoFixa = 1750;  // ajusta pra velocidade boa

// Ré (ajusta pra teu ESC — geralmente < 1500)
// int aceleracaoRe = 1650;
int aceleracaoRe = 1350;

// Ultrassônico HC-SR04
#define TRIGGER_PIN 5
#define ECHO_PIN 18
const int ledPin = 2;

#define ULTRA_TIME_THRESHOLD 200
#define ULTRA_BUFFER_WINDOW 10

// Distância pra parar (em cm)
const int distanciaObstaculo = 5;  // para quando < 30cm

// UUIDs BLE
#define SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int steerCommand = 0;  // valor recebido do celular (-100 a +100)

unsigned long tempoObstaculo = 0;
int estadoObstaculo = 0;  // 0 = normal, 1 = parar + LED, 2 = ré, 3 = virar esquerda

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Celular conectado! Modo autônomo ativado.");
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Celular desconectado! Parando tudo.");
    esc.writeMicroseconds(escStop);
    direcao.write(90 + offsetDirecao);
    BLEDevice::startAdvertising();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();

    if (value.length() > 0) {
      int received = (unsigned char)value[0];
      steerCommand = received - 100;  // -100 a +100
      Serial.print("Comando recebido: ");
      Serial.println(steerCommand);
    }
  }
};

long lerDistancia(uint64_t now) {
  static uint64_t last_run = 0;
  static long distance = 0;
  static uint32_t filtered_distance = 100;
  static uint32_t sum = 0;
  static uint32_t avg_buffer[ULTRA_BUFFER_WINDOW] = {0};
  static uint8_t index = 0;
  static uint8_t count = 0;

  
  if (now < last_run + ULTRA_TIME_THRESHOLD) {
    return distance;
  }
  last_run = now;

  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;  // cm

  sum -= avg_buffer[index];

  avg_buffer[index] = distance;

  sum += distance;

  index = (index + 1) % ULTRA_BUFFER_WINDOW;

  if (count < ULTRA_BUFFER_WINDOW)
  {
    count++;
    filtered_distance =  sum / count;
  }
  else
  {
    filtered_distance = sum / ULTRA_TIME_THRESHOLD;
  }
    

  // filtered_distance += distance;
  // filtered_distance /= 10;

  // filtered_distance = (filtered_distance + distance)/10;

  Serial.printf("dist: %ld, f_dit: %ld\n", distance, filtered_distance);
  return distance;
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  direcao.attach(pinDirecao, 500, 2500);
  esc.attach(pinEsc, 1000, 2000);

  esc.writeMicroseconds(escStop);
  direcao.write(90 + offsetDirecao);

  BLEDevice::init("TARDIS_KART");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE iniciado! Conecta no 'TARDIS_KART'");
}

void loop() {
  long distancia = lerDistancia(millis());


  if (!deviceConnected) {
    esc.writeMicroseconds(escStop);
    direcao.write(90 + offsetDirecao);
    digitalWrite(ledPin, LOW);
    estadoObstaculo = 0;
    delay(100);
    return;
  }

  // long distancia = lerDistancia(millis());

  // DETECÇÃO DE OBSTÁCULO
  if (distancia > 0 && distancia < distanciaObstaculo && estadoObstaculo == 0) {
    Serial.print("OBSTÁCULO DETECTADO A ");
    Serial.print(distancia);
    Serial.println("cm! Parando e desviando...");
    estadoObstaculo = 1;
    tempoObstaculo = millis();
  }

  if (estadoObstaculo == 1) {  // Para + acende LED 1s
    esc.writeMicroseconds(escStop);
    digitalWrite(ledPin, HIGH);
    if (millis() - tempoObstaculo > 1000) {
      estadoObstaculo = 2;
      tempoObstaculo = millis();
    }
  } else if (estadoObstaculo == 2) {  // Ré 1s
    esc.writeMicroseconds(aceleracaoRe);
    digitalWrite(ledPin, LOW);
    if (millis() - tempoObstaculo > 1000) {
      estadoObstaculo = 3;
      tempoObstaculo = millis();
    }
  } else if (estadoObstaculo == 3) {  // Vira esquerda forte 2s
    esc.writeMicroseconds(aceleracaoFixa);
    direcao.write(angleMin + offsetDirecao);  // máxima esquerda
    if (millis() - tempoObstaculo > 2000) {
      estadoObstaculo = 0;  // volta ao normal
      Serial.println("Desvio concluído! Voltando ao modo autônomo.");
    }
  } else {
    // MODO AUTÔNOMO NORMAL
    int direcaoAngle = map(steerCommand, -100, 100, angleMin, angleMax);
    direcaoAngle += offsetDirecao;
    direcaoAngle = constrain(direcaoAngle, angleMin, angleMax);
    direcao.write(direcaoAngle);

    esc.writeMicroseconds(aceleracaoFixa);
    digitalWrite(ledPin, LOW);
  }
}
