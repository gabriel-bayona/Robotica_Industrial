#include <Arduino.h>
#include <ESP32_FastPWM.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>

// Pines
#define CHA_A 12 // A ENCODER
#define CHA_B 14 // B ENCODER
//se les suma alimentacion y masa.

#define PWM_PIN 17  // Salida PWM
#define ENABLE 5
#define SENTIDOHORARIO 15
#define SENTIDOANTIHORARIO 16

// PWM
#define RESOLUTION 10 // Resolución del PWM en bits (0-4095) // NO ES UN PIN
#define PWM_CHANNEL 0 // Canal PWM // NO ES UN PIN
#define PWM_FREQ 4000

ESP32_FAST_PWM* pwm;

// Encoder
ESP32Encoder encoder;

// variables del PID 
double input = 0;
double output = 0;
double setpoint = 1000;  // Posición objetivo (en pulsos)
double Kp = 1.0, Ki = 0.1, Kd = 0.05;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
//sobrecargar constructor para agergar setpoint como valor
//lo mismo para input


// Contador de muestras
const int MAX_DATOS = 100;
int contador = 0;
float encoder_lect[MAX_DATOS];
float pwm_lect[MAX_DATOS];

// Interrup timer
hw_timer_t *Timer0_Cfg = NULL;
volatile bool Int_State = true;  // Arrancamos activos

// servicio de interrupcion, se ejecuta cada vez que el timer llega a 0
void IRAM_ATTR Timer0_ISR()
{
  Int_State = false;
}

void setup() {
  Serial.begin(115200);

  // Pines de control
  pinMode(SENTIDOHORARIO, OUTPUT);
  pinMode(SENTIDOANTIHORARIO, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);

  // PWM
  pwm = new ESP32_FAST_PWM(PWM_PIN, PWM_FREQ, 0, PWM_CHANNEL, RESOLUTION);
  if (!pwm) {
    Serial.println("Error al crear PWM");
    while (1);
  }

  // Encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(CHA_A, CHA_B);
  encoder.clearCount();
  encoder.setFilter(1023);

  // PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100, 100);  // Duty cycle y sentido

  // Timer configuración
  Timer0_Cfg = timerBegin(0, 80, true); //Timer 0, prescaler en 80, count up
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true); // attach interrupt to timer 0
  timerAlarmWrite(Timer0_Cfg, 10000, true); // set alarm to 10000 us (10 ms)
  timerAlarmEnable(Timer0_Cfg); // enable alarm  
}

void loop() {
  // Leer entrada serial sin bloqueo
  //serial string también se puede usar para controlar el motor
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '0') {
      Int_State = false;  // Detener motor
      Serial.println("Motor detenido (Int_State = false)");
    } else if (c == '1') {
      Int_State = true;   // Motor activo
      Serial.println("Motor activo (Int_State = true)");
    }
  }

  if (Int_State) {
    input = encoder.getCount();
    myPID.Compute();

    Serial.print("Encoder Count: %i", input);
    //agregar el doxygen al compute

    int freq = PWM_FREQ;
    int duty = abs((int)output);

    // Sentido según el signo del output
    if (output >= 0) {
      digitalWrite(SENTIDOHORARIO, LOW);
      digitalWrite(SENTIDOANTIHORARIO, HIGH);
    } else {
      digitalWrite(SENTIDOHORARIO, HIGH);
      digitalWrite(SENTIDOANTIHORARIO, LOW);
    }

    pwm->setPWM(PWM_PIN, freq, duty);

    // Registro para análisis
    if (contador < MAX_DATOS) {
      encoder_lect[contador] = input;
      pwm_lect[contador] = output;
      contador++;

      Serial.print(">Encoder: ");
      Serial.print(encoder_lect[contador]);
      Serial.print(", PWM: ");
      Serial.print(pwm_lect[contador]);
    }
  } else {
    // Detener motor
    digitalWrite(ENABLE, LOW);
    digitalWrite(SENTIDOHORARIO, LOW);
    digitalWrite(SENTIDOANTIHORARIO, LOW);
    pwm->setPWM(PWM_PIN, PWM_FREQ, 0);
  }
  

}
