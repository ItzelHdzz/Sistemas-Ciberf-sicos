const int analogPin = A0;   // Sensor de presión
const int pwmPin = 9;       // Pin PWM de la bomba
const int valvePin = 7;     // Pin digital para la válvula electroneumática

// CONTROLADOR PI
float Kp = 5;
float Ki = 0.7;

float error = 0;
float errorAcumulado = 0;
float presionDeseada = 40.0; // kPa 
float presionMedida = 0;
float pwmOutput = 0;

float dt = 0.1;  // 100 ms

bool valvulaAbierta = false;
bool mensajeEnviado = false; // Para enviar "cerrado" solo una vez

void setup() {
  Serial.begin(9600);
  pinMode(pwmPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  digitalWrite(valvePin, LOW);  // válvula inicia apagada

  Serial.println("CONTROLADOR PI DE PRESIÓN + VÁLVULA ELECTRONEUMÁTICA");
  Serial.print("Setpoint: ");
  Serial.print(presionDeseada);
  Serial.println(" kPa");
  Serial.println("Comandos: ON -> abrir válvula | OFF -> cerrar válvula");
}

void loop() {
  // Lectura del sensor de presión 
  int sensorValue = analogRead(analogPin);
  float voltaje = sensorValue * (5.0 / 1023.0);
  presionMedida = (50 * voltaje - 125) + 3;

  // Control PI
  error = presionDeseada - presionMedida;
  errorAcumulado += error * dt;

  float pwmBase = 97;  
  float control = Kp * error + Ki * errorAcumulado;
  pwmOutput = pwmBase + control;

  if (pwmOutput > 255) pwmOutput = 255;
  if (pwmOutput < 0) pwmOutput = 0;

  analogWrite(pwmPin, (int)pwmOutput);

  // Control de la válvula por monitor serial
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.equalsIgnoreCase("ON")) {
      digitalWrite(valvePin, HIGH);
      valvulaAbierta = true;
      mensajeEnviado = false; // reiniciar bandera
      Serial.println("VÁLVULA ACTIVADA");
    } 
    else if (comando.equalsIgnoreCase("OFF")) {
      digitalWrite(valvePin, LOW);
      valvulaAbierta = false;
      mensajeEnviado = false;
      Serial.println("VÁLVULA DESACTIVADA");
    }
  }

  // Cuando la válvula está abierta, verificar si se alcanzó la presión deseada
  if (valvulaAbierta && !mensajeEnviado) {
    if (abs(presionDeseada - presionMedida) < 0.5) {  // tolerancia de ±1 kPa
      Serial.println("cerrado");
      mensajeEnviado = true;
    }
  }

  // Monitoreo del controlador
  Serial.print("Presion deseada: ");
  Serial.print(presionDeseada);
  Serial.print(" kPa | Medida: ");
  Serial.print(presionMedida);
  Serial.print(" kPa | Error: ");
  Serial.print(error);
  Serial.print(" | PWM: ");
  Serial.println(pwmOutput);

  delay(100);
}
