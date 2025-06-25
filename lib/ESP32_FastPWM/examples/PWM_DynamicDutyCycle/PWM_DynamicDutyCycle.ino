/********************************************************************
  PWM_DynamicDutyCycle_con_pot.ino
  Cambia el ciclo de trabajo del PWM en función del voltaje en GPIO36
  Usa ESP32_FAST_PWM para generar PWM de hardware preciso
********************************************************************/

#include <Arduino.h>
#include <ESP32_FastPWM.h>

#define PWM_PIN          26     // Pin de salida PWM
#define ANALOG_INPUT_PIN 36     // Pin analógico donde conectás el potenciómetro
#define PWM_FREQ         5000.0 // Frecuencia del PWM en Hz
#define RES_BITS         12     // Resolución del ADC (0–4095)

// Instancia dinámica de PWM
ESP32_FAST_PWM* PWM_Instance;

float dutyCycle = 0;
char dashLine[] = "========================================================";

void printPWMInfo(ESP32_FAST_PWM* pwm) {
  Serial.println(dashLine);
  Serial.print("Pin: ");
  Serial.print(pwm->getPin());
  Serial.print(" | DutyCycle: ");
  Serial.print(pwm->getActualDutyCycle());
  Serial.print("% | Freq: ");
  Serial.print(pwm->getActualFreq(), 2);
  Serial.println(" Hz");
  Serial.println(dashLine);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(RES_BITS); // Resolución de 12 bits para el ADC

  PWM_Instance = new ESP32_FAST_PWM(PWM_PIN, PWM_FREQ, 50.0f); // 50% inicial
  if (!PWM_Instance || !PWM_Instance->setPWM()) {
    Serial.println("Error al inicializar el PWM. Detenido.");
    while (true) delay(1000);
  }

  Serial.println("PWM inicializado correctamente.");
  printPWMInfo(PWM_Instance);
}

void loop() {
  int analogValue = analogRead(ANALOG_INPUT_PIN); // Valor de 0 a 4095
  dutyCycle = (analogValue / 4095.0f) * 100.0f;   // Convertido a % (0–100)

  PWM_Instance->setPWM(PWM_PIN, PWM_FREQ, dutyCycle); // Aplica el duty cycle

  Serial.print("ADC: ");
  Serial.print(analogValue);
  Serial.print(" | Nuevo DutyCycle: ");
  Serial.print(dutyCycle, 1);
  Serial.println("%");

  delay(200); // Refresca cada 200 ms
}
