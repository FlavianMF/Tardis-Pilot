# 1 "D:\\Windows.old\\Users\\flavi\\Documents\\Arduino\\Tardis-Pilot\\Tardis-Pilot.ino"
# 2 "D:\\Windows.old\\Users\\flavi\\Documents\\Arduino\\Tardis-Pilot\\Tardis-Pilot.ino" 2
# 3 "D:\\Windows.old\\Users\\flavi\\Documents\\Arduino\\Tardis-Pilot\\Tardis-Pilot.ino" 2
# 4 "D:\\Windows.old\\Users\\flavi\\Documents\\Arduino\\Tardis-Pilot\\Tardis-Pilot.ino" 2
# 5 "D:\\Windows.old\\Users\\flavi\\Documents\\Arduino\\Tardis-Pilot\\Tardis-Pilot.ino" 2
# 6 "D:\\Windows.old\\Users\\flavi\\Documents\\Arduino\\Tardis-Pilot\\Tardis-Pilot.ino" 2

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
int aceleracaoFixa = 1650; // ajusta pra velocidade boa

// Ré (ajusta pra teu ESC — geralmente < 1500)
// int aceleracaoRe = 1650;
int aceleracaoRe = 1350;

// Ultrassônico HC-SR04


const int ledPin = 2;




// Distância pra parar (em cm)
const int distanciaObstaculo = 5; // para quando < 30cm

// UUIDs BLE



BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int steerCommand = 0; // valor recebido do celular (-100 a +100)

unsigned long tempoObstaculo = 0;
int estadoObstaculo = 0; // 0 = normal, 1 = parar + LED, 2 = ré, 3 = virar esquerda

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
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0) {
      int received = (unsigned char)value[0];
      steerCommand = received - 100; // -100 a +100
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
  static uint32_t avg_buffer[10] = {0};
  static uint8_t index = 0;
  static uint8_t count = 0;


  if (now < last_run + 200) {
    return distance;
  }
  last_run = now;

  digitalWrite(5, 0x0);
  delayMicroseconds(2);
  digitalWrite(5, 0x1);
  delayMicroseconds(10);
  digitalWrite(5, 0x0);

  long duration = pulseIn(18, 0x1);
  distance = duration * 0.034 / 2; // cm

  sum -= avg_buffer[index];

  avg_buffer[index] = distance;

  sum += distance;

  index = (index + 1) % 10;

  if (count < 10)
  {
    count++;
    filtered_distance = sum / count;
  }
  else
  {
    filtered_distance = sum / 200;
  }


  // filtered_distance += distance;
  // filtered_distance /= 10;

  // filtered_distance = (filtered_distance + distance)/10;

  Serial.printf("dist: %ld, f_dit: %ld\n", distance, filtered_distance);
  return distance;
}

void setup() {
  Serial.begin(115200);

  pinMode(5, 0x03);
  pinMode(18, 0x01);
  pinMode(ledPin, 0x03);
  digitalWrite(ledPin, 0x0);

  direcao.attach(pinDirecao, 500, 2500);
  esc.attach(pinEsc, 1000, 2000);

  esc.writeMicroseconds(escStop);
  direcao.write(90 + offsetDirecao);

  BLEDevice::init("TARDIS_KART");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService("0000ffe0-0000-1000-8000-00805f9b34fb");

  pCharacteristic = pService->createCharacteristic(
    "0000ffe1-0000-1000-8000-00805f9b34fb",
    BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
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
    digitalWrite(ledPin, 0x0);
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

  if (estadoObstaculo == 1) { // Para + acende LED 1s
    esc.writeMicroseconds(escStop);
    digitalWrite(ledPin, 0x1);
    if (millis() - tempoObstaculo > 1000) {
      estadoObstaculo = 2;
      tempoObstaculo = millis();
    }
  } else if (estadoObstaculo == 2) { // Ré 1s
    esc.writeMicroseconds(aceleracaoRe);
    digitalWrite(ledPin, 0x0);
    if (millis() - tempoObstaculo > 1000) {
      estadoObstaculo = 3;
      tempoObstaculo = millis();
    }
  } else if (estadoObstaculo == 3) { // Vira esquerda forte 2s
    esc.writeMicroseconds(aceleracaoFixa);
    direcao.write(angleMin + offsetDirecao); // máxima esquerda
    if (millis() - tempoObstaculo > 2000) {
      estadoObstaculo = 0; // volta ao normal
      Serial.println("Desvio concluído! Voltando ao modo autônomo.");
    }
  } else {
    // MODO AUTÔNOMO NORMAL
    int direcaoAngle = map(steerCommand, -100, 100, angleMin, angleMax);
    direcaoAngle += offsetDirecao;
    direcaoAngle = ((direcaoAngle)<(angleMin)?(angleMin):((direcaoAngle)>(angleMax)?(angleMax):(direcaoAngle)));
    direcao.write(direcaoAngle);

    esc.writeMicroseconds(aceleracaoFixa);
    digitalWrite(ledPin, 0x0);
  }
}
