#include <Arduino.h>
#include <ESP32_FastPWM.h>


// Interrup timer
hw_timer_t *Timer0_Cfg = NULL;
            /*
                    - hw_timer_t es un tipo de dato que representa un timer de hardware en el ESP32.
                    - Timer0_Cfg es un puntero que se usará para configurar y controlar el timer.
                    - Se inicializa como NULL para indicar que inicialmente no está configurado.
            */
volatile bool Int_State = false; //es una bandera booleana que se usa para comunicar el estado de la interrupción entre la ISR y el código principal.
            /*
                    - volatile indica al compilador que esta variable puede cambiar en cualquier momento (por una interrupción), por lo que no debe optimizar su acceso.
                    - Int_State se usa para indicar si la interrupción del timer está activa o no.
            */


// servicio de interrupcion, se ejecuta cada vez que el timer llega a 0
void IRAM_ATTR Timer0_ISR()
{
  Int_State = false;
}

//
#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 
#define CHA_A 12 // A ENCODER
#define CHA_B 14 // B ENCODER
//se les suma alimentacion y masa.


#define PWM_PIN 17       // Salida PWM
#define POT_PIN 2      // Entrada del potenciómetro (ADC) - MEDIO DEL POTENCIOMETRO
#define RESOLUTION 10    // Resolución del PWM en bits (0-4095) // NO ES UN PIN
#define PWM_CHANNEL 0    // Canal PWM // NO ES UN PIN

#define ENABLE 5                 //ESTE PIN HABILITA EL DRIVER
#define SENTIDOHORARIO 15     // Pin para sentido horario 
#define SENTIDOANTIHORARIO 16     // Pin para sentido antihorario

#define MAXIMA_LONGITUD_DATOS 100

#define FRECUENCIA 4000 // Frecuencia del PWM en Hz
#define MAX_VALUE_ENCODER 1000000 // Valor máximo del encoder --- esto depende del encoder, VERIFICAR EN DATASHEET

ESP32_FAST_PWM* pwm;

//
ESP32Encoder encoder;
long QEI_Pos,QEI_Pos_1; //24300 pulsos por vuelta

int contador;
float encoder_lect[MAXIMA_LONGITUD_DATOS];
float pwm_lect[MAXIMA_LONGITUD_DATOS];


//Bandera de interrupcion
bool b_mostrar; // bandera que indica si se debe mostrar la salida por serial
bool b_iden;
//int MEF_iden1;
bool b_PID;
bool b_planta; // bandera que indica si se debe trabajar la planta indicada//cargada
int duty;


void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Resolución ADC de 12 bits (0-4095)
  pinMode(SENTIDOANTIHORARIO, OUTPUT);
  pinMode(SENTIDOHORARIO, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);

  pwm = new ESP32_FAST_PWM(PWM_PIN, 4000, 50, PWM_CHANNEL, RESOLUTION);
  if (!pwm) {
    Serial.println("Error al crear PWM");
    while (1);
  }

  


  encoder.attachFullQuad( CHA_A, CHA_B );
  //encoder.setCount(100); 
  encoder.clearCount();
  encoder.setFilter(1023);

  QEI_Pos = 0;
  QEI_Pos_1 = 0;



  //estas son variables de control del programa (me mueven entre las diferentes fuciones)
  b_mostrar = false;
  b_iden = false;
  b_PID = false;
  b_planta = false;
  contador = 0;
}


inline void Leer_serial(){
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Eliminar espacios en blanco al inicio y al final

    if (input == "start") {
      contador = 0;
      encoder.clearCount();
      Serial.println("Iniciando...");
    } else if (input == "stop") {
      Serial.println("Deteniendo...");
      contador = 100; // Para detener el bucle en loop()
    } else {
      Serial.printf("Comando no reconocido: %s\n", input.c_str());
    }
  }
}



void loop() {
  
  QEI_Pos_1 = QEI_Pos; // Save the previous position
  QEI_Pos = encoder.getCount();
  

//lo tengo que pasar a que el PID lo asigna
//  int duty = map(encoder.getCount(), 0, MAX_VALUE_ENCODER, 0, 100);

  // Aplicar los valores PWM -- Trabajar
  pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

  encoder_lect[contador]=encoder.getCount();
  
// Primer if para identificar la planta
if (b_iden) {
  switch (contador) {
    case 0:
      digitalWrite(SENTIDOHORARIO, LOW);
      digitalWrite(SENTIDOANTIHORARIO, HIGH);
      duty = 20;
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
      
      break;
      
    case 20:
      duty = 50;
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

      break;
      
    case 40:
      duty = 0;
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
      digitalWrite(SENTIDOANTIHORARIO, LOW);
      digitalWrite(SENTIDOHORARIO, LOW);
      break;
      
    case 60:
      digitalWrite(SENTIDOHORARIO, HIGH);
      duty = 40;
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
      break;
      
    case 80:
      duty = 20;
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
      break;
      
    case 100:
      duty = 0;
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
      digitalWrite(SENTIDOHORARIO, LOW);
      digitalWrite(SENTIDOANTIHORARIO, LOW);
      b_iden = false;
      break;

  }
  
  encoder_lect[contador] = encoder.getCount();
  pwm_lect[contador] = duty;
}  
  
  //Segundo if, asigna el PID
  if (b_PID){
    // Aquí se puede implementar la lógica del PID
    // Por ejemplo, se puede usar un PID para controlar la velocidad o posición del motor
    // duty = PID_calculate(encoder.getCount()); // Esta función debe ser implementada
    // Por ahora, solo un ejemplo simple:
    duty = map(encoder.getCount(), 0, MAX_VALUE_ENCODER, -100, 100);
    pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
  }

  // Trabaja la planta con los valores del PID y varía el setpoint
  if(b_planta){
    //1. ingreso con los valores que puse para MI PID
    
    //2. logíca pid

    //3. asignar el duty PERO, no tiene sentido si lo tenemos quieto.
    // si el encoder.getCount() es igual al setpoint, entonces no hay que hacer nada.
    

    //4. cambiamos el setpoint del bracito, entonces obligamos al pid a cambiar para alcanzar el setpoint.

    
  }

// Mostrar por Serial
  if (b_mostrar) {
    //implementar la logica de mostrar por serial y/o guardar en un archivo.
    Serial.printf("Encoder: %ld | Duty: %d\n", encoder.getCount(), duty);

      // // Mostrar por Serial
  // Serial.printf(">Pot: %d | Frecuencia: %d Hz | Duty: %d%%\n", potValue, freq, duty);
  // Serial.printf(">PWM_Duty:%ld:%d\n", millis(), duty);
  }

  Leer_serial();
  //while(Int_State){Leer_serial();} // Espera a que se complete la interrupcion del timer;
  //Int_State = true;
  
  contador++;
  delay(100);
}