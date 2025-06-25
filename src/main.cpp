#include <Arduino.h>
#include <ESP32_FastPWM.h>


//
#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 
#define CHA_A 12 // A ENCODER
#define CHA_B 14 // B ENCODER
//se les suma alimentacion y masa.


#define PWM_PIN 15       // Salida PWM
#define POT_PIN 2      // Entrada del potenciómetro (ADC) - MEDIO DEL POTENCIOMETRO
#define RESOLUTION 10    // Resolución del PWM en bits (0-4095) // NO ES UN PIN
#define PWM_CHANNEL 0    // Canal PWM // NO ES UN PIN

#define ENABLE 5                 //ESTE PIN HABILITA EL DRIVER
#define SENTIDOHORARIO 17     // Pin para sentido horario 
#define SENTIDOANTIHORARIO 16     // Pin para sentido antihorario

ESP32_FAST_PWM* pwm;

//
ESP32Encoder encoder;

int contador;

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

  digitalWrite(SENTIDOHORARIO, LOW);
  digitalWrite(SENTIDOANTIHORARIO, HIGH);
  encoder.attachFullQuad( CHA_A, CHA_B );
  //encoder.setCount(100); 
  encoder.clearCount();
  encoder.setFilter(1023);

}

void loop() {
  int potValue = analogRead(POT_PIN); // Leer potenciómetro (0-1023

  // Mapear valor del potenciómetro a frecuencia (500 Hz - 2000 Hz)
  //int freq = map(potValue, 0, 1023, 500, 2000);
  int freq = 4000;

  // Mapear valor del potenciómetro a duty cycle (0% - 100%)
  int duty = map(potValue, 0, 1023, 0, 100);

  // Aplicar los valores PWM
  pwm->setPWM(PWM_PIN, freq, duty);


  
  if (contador>=10){
    contador=0;
  }

  if (contador < 5)
  {delay(2000);
    delay(25);
    //digitalWrite(SENTIDOANTIHORARIO, LOW);
    //digitalWrite(SENTIDOHORARIO, HIGH);
    digitalWrite(ENABLE, LOW); //  HABILITA el driver
   // delay(2000);
//    digitalWrite(ENABLE, HIGH); //  DESHABILITA el driver
  }else{
    delay(25);
//    digitalWrite(SENTIDOHORARIO, LOW);

    //digitalWrite(SENTIDOANTIHORARIO, HIGH);
    
    digitalWrite(ENABLE, LOW); // HABILITA el driver
    delay(2000);
  //  digitalWrite(ENABLE, HIGH); //  DESHABILITA el driver
  }
  contador++;


  //
  long newPosition = encoder.getCount();
  Serial.print("Posicion: ");Serial.println(newPosition);

  // Mostrar por Serial
  Serial.printf(">Pot: %d | Frecuencia: %d Hz | Duty: %d%%\n", potValue, freq, duty);
  Serial.printf(">PWM_Duty:%ld:%d\n", millis(), duty);

  

  
}
