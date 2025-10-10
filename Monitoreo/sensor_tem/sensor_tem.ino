#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// --- Configuración WiFi ---
const char* ssid = "Tec-IoT";
const char* password = "spotless.magnetic.bridge";

// --- Configuración MQTT ---
const char* mqtt_server = "10.25.14.132";
const char* mqttUser = "Cyphex";
const char* mqttPassword = "12345678";

WiFiClient espClient;
PubSubClient client(espClient);

// --- Configuración DHT22 ---
#define DHTTYPE DHT22
#define DHTPIN 4  // Pin del ESP32 conectado al sensor DHT22
DHT dht(DHTPIN, DHTTYPE);

long lastMsg = 0;
const int intervalo = 600000; // 10 minutos
bool sensorOnline = true;     // Estado inicial del sensor

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client_DHT22")) { // , mqttUser, mqttPassword
      Serial.println("Conectado a broker MQTT");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" -> Reintentando en 5s");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando...");
  dht.begin();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  long now = millis();
  if (now - lastMsg > intervalo) {
    lastMsg = now;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    // Si hay error en lectura
    if (isnan(h) || isnan(t)) {
      Serial.println("Error al leer el sensor DHT22");

      if (sensorOnline) { // Solo avisa una vez
        sensorOnline = false;
        client.publish("sensores", "DHT22 OFFLINE");
        Serial.println("DHT22 OFFLINE");
      }

      return;
    }

    // Si antes estaba offline, ahora se reconectó
    if (!sensorOnline) {
      sensorOnline = true;
      client.publish("sensores", "DHT22 ONLINE");
      Serial.println("DHT22 ONLINE");
    }

    // Publicar lecturas válidas
    char humStr[8];
    dtostrf(h, 6, 2, humStr);
    client.publish("humedad", humStr);

    char tempStr[8];
    dtostrf(t, 6, 2, tempStr);
    client.publish("temperatura", tempStr);

    Serial.print("Humedad: ");
    Serial.print(h);
    Serial.print(" %  |  Temp: ");
    Serial.print(t);
    Serial.println(" °C");
  }
}
