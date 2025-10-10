#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Tec-IoT";
const char* password = "spotless.magnetic.bridge";
const char* mqtt_server = "10.25.14.132";

WiFiClient espClient;
PubSubClient client(espClient);

int sensorPins[4] = {32, 33, 34, 35};
bool sensorOnline[4] = {true, true, true, true};
int prevValue[4] = {0, 0, 0, 0};
int stableCount[4] = {0, 0, 0, 0};

long lastMsg = 0;
const int intervalo = 600000; 
const int seco = 1400;
const int humedo = 1200;

void setup_wifi() {
  Serial.print("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado!");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client_FC28")) {
      Serial.println("Conectado a broker MQTT");
    } else {
      Serial.print("Falló, rc=");
      Serial.print(client.state());
      Serial.println(" -> reintento en 5s");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

// Convierte la lectura (1200–1400) a porcentaje (100–0)
float lecturaAHumedad(int lectura) {
  float porcentaje = map(lectura, seco, humedo, 0, 100);
  if (porcentaje < 0) porcentaje = 0;
  if (porcentaje > 100) porcentaje = 100;
  return porcentaje;
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  long now = millis();
  if (now - lastMsg > intervalo) {
    lastMsg = now;

    float suma = 0;

    for (int i = 0; i < 4; i++) {
      int lectura = analogRead(sensorPins[i]);
      float humedad = lecturaAHumedad(lectura);
      suma += humedad;

      bool dentroRango = (lectura >= 1000 && lectura <= 1600);

      // Detectar desconexión
      if (!dentroRango) {
        if (sensorOnline[i]) {
          sensorOnline[i] = false;
          String msg = "Sensor " + String(i + 1) + " OFFLINE";
          client.publish("status", msg.c_str());
          Serial.println(msg);
        }
      } else {
        // Si estaba offline y vuelve, marca reconectado
        if (!sensorOnline[i]) {
          sensorOnline[i] = true;
          String msg = "Sensor " + String(i + 1) + " ONLINE";
          client.publish("status", msg.c_str());
          Serial.println(msg);
        }
      }

      // Detección por estabilidad (sin cambios en 3 lecturas consecutivas)
      if (abs(lectura - prevValue[i]) < 5) {
        stableCount[i]++;
        if (stableCount[i] >= 3 && sensorOnline[i]) {
          sensorOnline[i] = false;
          String msg = "Sensor " + String(i + 1) + " SIN CAMBIO (posible fallo)";
          client.publish("status", msg.c_str());
          Serial.println(msg);
        }
      } else {
        stableCount[i] = 0;
      }

      prevValue[i] = lectura;

      // Publicar la humedad individual
      char msgHumedad[50];
      snprintf(msgHumedad, sizeof(msgHumedad), "Sensor %d: %.2f %%", i + 1, humedad);
    }

    float promedio = suma / 4.0;
    char msgPromedio[50];
    snprintf(msgPromedio, sizeof(msgPromedio), "Promedio: %.2f %%", promedio);
    client.publish("sustrato", msgPromedio);
    Serial.println(msgPromedio);
  }
}
