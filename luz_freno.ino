/*
 * Luz de freno para bici, ESP32 + BLE.
 * Recibe 1 byte desde la app (luz-freno.html):
 *   0 = rodando   -> luz tenue fija
 *   1 = frenando  -> 3 destellos rápidos y luego fija a tope
 *   2 = parado    -> fija a tope (semáforo)
 *   3 = emergencia-> destello rápido
 * Si pasan más de 4 s sin recibir nada (móvil lejos, app cerrada), vuelve a luz tenue fija.
 *
 * Montaje: GPIO LED_PIN -> puerta de un MOSFET de nivel lógico (p. ej. IRLZ44N o AO3400)
 * que maneja un LED rojo de potencia o una tira de LEDs rojos de 5-12 V.
 * Para probar sin montar nada, deja LED_PIN = 2 (LED azul de muchas placas ESP32).
 *
 * Placa: cualquier ESP32 (DevKit, C3, S3). Núcleo Arduino-ESP32 2.x o 3.x. Sin librerías extra.
 */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define LED_PIN 2
#define PWM_FREQ 2000
#define PWM_BITS 8
#define DIM_LEVEL 40            // 0-255 brillo de la luz tenue
#define FAILSAFE_MS 4000

#define SERVICE_UUID "b1c10001-0c4d-4b83-9a6d-5d1a1e000001"
#define CHAR_UUID    "b1c10002-0c4d-4b83-9a6d-5d1a1e000001"

volatile uint8_t state = 0;
volatile uint32_t lastCmdMs = 0;
volatile uint32_t stateSinceMs = 0;
bool connected = false;

void setLed(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(LED_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    auto v = c->getValue();
    if (v.length() < 1) return;
    uint8_t s = (uint8_t)v[0];
    if (s > 3) return;
    lastCmdMs = millis();
    if (s != state) { state = s; stateSinceMs = millis(); }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { connected = true; lastCmdMs = millis(); }
  void onDisconnect(BLEServer *server) override {
    connected = false;
    state = 0; stateSinceMs = millis();
    BLEDevice::startAdvertising();      // vuelve a ser visible para reconectar
  }
};

void setup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(LED_PIN, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(0, PWM_FREQ, PWM_BITS);
  ledcAttachPin(LED_PIN, 0);
#endif
  setLed(255);                           // prueba de arranque: encendida un instante
  delay(400);

  BLEDevice::init("LuzBici");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *service = server->createService(SERVICE_UUID);
  BLECharacteristic *chr = service->createCharacteristic(
      CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  chr->setCallbacks(new CmdCallbacks());
  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);     // la app filtra por este servicio
  adv->setScanResponse(true);
  adv->start();

  lastCmdMs = millis();
  stateSinceMs = millis();
}

void loop() {
  uint32_t now = millis();
  uint8_t s = state;
  if (now - lastCmdMs > FAILSAFE_MS) s = 0;      // sin latido: luz tenue fija

  uint32_t since = now - stateSinceMs;
  uint8_t duty;
  switch (s) {
    case 1:                                     // 3 destellos y fija
      duty = (since < 600) ? ((since / 100) % 2 == 0 ? 255 : 30) : 255;
      break;
    case 2:
      duty = 255;
      break;
    case 3:
      duty = ((now / 100) % 2 == 0) ? 255 : 20;
      break;
    default:
      duty = DIM_LEVEL;
  }
  setLed(duty);
  delay(10);
}
