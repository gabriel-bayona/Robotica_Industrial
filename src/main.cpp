#include <Arduino.h>
#include <ESP32_FastPWM.h>
#include <PID_v1.h>

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
#define SENTIDOHORARIO 16     // Pin para sentido horario 
#define SENTIDOANTIHORARIO 15     // Pin para sentido antihorario

#define MAXIMA_LONGITUD_DATOS 100

#define FRECUENCIA 4000 // Frecuencia del PWM en Hz
#define MAX_VALUE_ENCODER 10000 // Valor máximo del encoder --- esto depende del encoder, VERIFICAR EN DATASHEET

ESP32_FAST_PWM* pwm;

//
ESP32Encoder encoder;
long QEI_Pos,QEI_Pos_1; //24300 pulsos por vuelta

int contador;
float encoder_lect[MAXIMA_LONGITUD_DATOS];
float pwm_lect[MAXIMA_LONGITUD_DATOS];



// Configuración del PID
double input, output, setpoint;
PID myPID(&input, &output, &setpoint, 2.0, 5.0, 1.0, DIRECT); // KP, KI, KD: valores a ajustar


//Bandera de interrupcion
bool b_mostrar; // bandera que indica si se debe mostrar la salida por serial
bool b_iden;
bool b_iden2; // bandera que indica si se debe trabajar la identificación 2
//int MEF_iden1;
bool b_PID;
bool b_planta; // bandera que indica si se debe trabajar la planta indicada//cargada
bool b_graficos; // bandera que indica si se deben mostrar los gráficos por serial
int duty;

bool encabezado = false;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Resolución ADC de 12 bits (0-4095)
  pinMode(SENTIDOANTIHORARIO, OUTPUT);
  pinMode(SENTIDOHORARIO, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);

  // Configuración del timer
  Timer0_Cfg = timerBegin(0, 80, true); // Timer 0, prescaler en 80
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true); // Asocia la ISR
  timerAlarmWrite(Timer0_Cfg, 100000, true); // 100000 us = 100 ms
  timerAlarmEnable(Timer0_Cfg); // Activa la alarma



  //Configuracion del PWM
  pwm = new ESP32_FAST_PWM(PWM_PIN, 4000, 50, PWM_CHANNEL, RESOLUTION);
  if (!pwm) {
    Serial.println("Error al crear PWM");
    while (1); //si no se crea, me clavo acá
  }

  //Configuracion del encoder
  encoder.attachFullQuad( CHA_A, CHA_B );
  //encoder.setCount(100); 
  encoder.clearCount();
  encoder.setFilter(1023);


  // Configuración del PID
  myPID.SetOutputLimits(-100, 100); // Salida PID entre -100 y 100
  myPID.SetMode(AUTOMATIC);


  QEI_Pos = 0;
  QEI_Pos_1 = 0;


  //estas son variables de control del programa (me mueven entre las diferentes fuciones)
  b_mostrar = false;
  b_iden = false;
  b_iden2 = false; 
  b_PID = false;
  b_planta = false;
  contador = 0;

}


inline void Leer_serial() {
  if (Serial.available()) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim(); // Elimina espacios en blanco

    if (inputStr == "strt") {
      contador = 0;
      b_iden = false;
      b_PID = false;
      encoder.clearCount();
      Serial.println("Modo predeterminado iniciado.");
      digitalWrite(SENTIDOHORARIO, LOW);
      digitalWrite(SENTIDOANTIHORARIO, HIGH);
      duty = 50;
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

    } else if (inputStr == "stop") {
      Serial.println("PWM y motor apagados.");
      b_iden = false;
      b_PID = false;
      digitalWrite(SENTIDOHORARIO, LOW);
      digitalWrite(SENTIDOANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, 0);
      duty = 0;

    } else if (inputStr == "iden") {
      b_iden = true;
      contador = 0;
      encoder.clearCount();
      Serial.println("Identificación 1 activada.");
    }else if (inputStr == "ide2")
    {
      b_iden2 = true;
      contador = 0;
      encoder.clearCount();
      Serial.println("Identificación 2 activada.");
    
    } else if (inputStr.startsWith("setp")) {
      inputStr.remove(0, 4);
      inputStr.trim();
      double valor = inputStr.toDouble();
      setpoint = valor;
      Serial.printf("Setpoint asignado: %.2f\n", setpoint);
      b_PID = true;

    } else if (inputStr.startsWith("spid")) {
      inputStr.remove(0, 4);
      inputStr.trim();

      int firstSpace = inputStr.indexOf(' ');
      int secondSpace = inputStr.indexOf(' ', firstSpace + 1);

      if (firstSpace > 0 && secondSpace > firstSpace) {
        double kp = inputStr.substring(0, firstSpace).toDouble();
        double kd = inputStr.substring(firstSpace + 1, secondSpace).toDouble();
        double ki = inputStr.substring(secondSpace + 1).toDouble();

        myPID.SetTunings(kp, ki, kd);
        Serial.printf("PID actualizado -> Kp: %.2f | Ki: %.2f | Kd: %.2f\n", kp, ki, kd);
      } else {
        Serial.println("Error: Formato incorrecto. Usa: spid <kp> <kd> <ki>");
      }

    }  else if (inputStr.startsWith("show")) {
      inputStr.remove(0, 4);
      inputStr.trim();

      if (inputStr == "pid") {
        double kp, ki, kd;
        kp = myPID.GetKp();
        ki = myPID.GetKi();
        kd = myPID.GetKd();
        Serial.printf("Valores PID actuales:\nKp: %.2f | Ki: %.2f | Kd: %.2f\n", kp, ki, kd);
        Serial.printf("\nSetpoint actual: %.2f\n", setpoint);
      } else if (inputStr == "data") {
        Serial.println("Mostrando arrays:");
        for (int i = 0; i < MAXIMA_LONGITUD_DATOS; i++) {
          Serial.printf("%d\t%.2f\t%.2f\n", i, encoder_lect[i], pwm_lect[i]);
        }
      
      
      } else if (inputStr == "pout") {
        Serial.println("Pines:");
        Serial.printf("- PWM_PIN: %d\n", PWM_PIN);
        Serial.printf("- POT_PIN: %d\n", POT_PIN);
        Serial.printf("- ENABLE: %d\n", ENABLE);
        Serial.printf("- SENTIDOHORARIO: %d\n", SENTIDOHORARIO);
        Serial.printf("- SENTIDOANTIHORARIO: %d\n", SENTIDOANTIHORARIO);
        
      }else {
        Serial.println("Uso de 'show':\n - show pid\n - show data");
      }


    } else if (inputStr == "rest") {
      Serial.println("Reiniciando sistema...");
      for (int i = 0; i < MAXIMA_LONGITUD_DATOS; i++) {
        encoder_lect[i] = 0;
        pwm_lect[i] = 0;
      }
      b_iden = false;
      b_PID = false;
      b_mostrar = false;
      duty = 0;
      encoder.clearCount();
      digitalWrite(SENTIDOHORARIO, LOW);
      digitalWrite(SENTIDOANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, 0);

    } else if (inputStr == "enab") {
      digitalWrite(ENABLE, HIGH);
      Serial.println("Habilitador encendido.");

    } else if (inputStr == "disa") {
      digitalWrite(ENABLE, LOW);
      Serial.println("Habilitador apagado.");

    } else if (inputStr == "graf") {
      b_graficos = !b_graficos; // Cambia el estado de la bandera
      if (b_graficos) {
        Serial.println("Gráficos activados.");
      } else {
        Serial.println("Gráficos desactivados.");
      }
    }else if (inputStr == "help")
    {
      Serial.println("Comandos disponibles:");
      Serial.println(" - strt: Iniciar modo predeterminado.");
      Serial.println(" - stop: Detener PWM y motor.");
      Serial.println(" - iden: Activar identificación (variacion discreta de pwm).");
      Serial.println(" - ide2: Activar identificación 2 (variacion continua de pwm).");
      Serial.println(" - pout: Mostrar pines utilizados.");
      Serial.println(" - setp <valor>: Asignar setpoint.");
      Serial.println(" - spid <kp> <kd> <ki>: Actualizar PID.");
      Serial.println(" - show pid: Mostrar valores PID actuales.");
      Serial.println(" - show data: Mostrar arrays de datos.");
      Serial.println(" - rest: Reiniciar sistema.");
      Serial.println(" - enab: Encender habilitador.");
      Serial.println(" - disa: Apagar habilitador.");
      Serial.println(" - graf: Activar/desactivar gráficos por serial.");
    }
    
    else {
      Serial.printf("Comando no reconocido: %s\n", inputStr.c_str());
    }
      
  }
}
// Fin de la funcion Leer_serial


void loop() {
  
  while(Int_State){Leer_serial();} // Espera a que la interrupción ocurra
  Int_State = true;

  // QEI_Pos_1 = QEI_Pos; // Save the previous position
  // QEI_Pos = encoder.getCount();
  
  // Aplicar los valores PWM -- Trabajar
  pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

  encoder_lect[contador]=encoder.getCount();
    
  // Primer if para identificar la planta
  if (b_iden) {
    switch (contador) {
      case 0:
        digitalWrite(SENTIDOHORARIO, LOW);
        digitalWrite(SENTIDOANTIHORARIO, HIGH);
        duty = 25;
        pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
        break;
        
      case 20:
        duty = 60;
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
        duty = 90;
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
  
  // Segundo if para identificar la planta con PID
  if(b_iden2){

    if(contador <30){
      digitalWrite(SENTIDOHORARIO, LOW);
      digitalWrite(SENTIDOANTIHORARIO, HIGH);
      duty = contador * 2.5; // Incrementa el duty de 0 a 75
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

    } else if (contador >= 30 && contador <= 50) {
      digitalWrite(SENTIDOHORARIO, HIGH);
      digitalWrite(SENTIDOANTIHORARIO, LOW);
      duty = 70 - contador; // Decrementa el duty de 75 a 0
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

    } else if (contador > 50 && contador < 70) {
      digitalWrite(SENTIDOHORARIO, LOW);
      digitalWrite(SENTIDOANTIHORARIO, HIGH);
      duty =  contador  - 50; // Incrementa el duty de 0 a 75
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

    } else {
      digitalWrite(SENTIDOHORARIO, LOW);
      digitalWrite(SENTIDOANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, 0); // Apaga el PWM
      b_iden2 = false; // Termina la identificación con PID

    }

    encoder_lect[contador] = encoder.getCount();
    pwm_lect[contador] = duty;
  }

  //Segundo if, asigna el PID
  if (b_PID){

    input = encoder.getCount();
    
    myPID.Compute();             // Calcula el nuevo valor de salida del PID
    duty = (int)output;          // Casteo
    
    duty = map(duty,MAX_VALUE_ENCODER*(-1) ,MAX_VALUE_ENCODER , -100, 100); // verificar mapeo con el profe


    if (output >= 0) {
      digitalWrite(SENTIDOHORARIO, HIGH);
      digitalWrite(SENTIDOANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, (int)output);
  } else {
      digitalWrite(SENTIDOHORARIO, LOW);
      digitalWrite(SENTIDOANTIHORARIO, HIGH);
      pwm->setPWM(PWM_PIN, FRECUENCIA, (int)(-output)); // valor positivo para PWM / evito tener dos variables duty, la ultima es el duty casteado.
  }

  }

  // Trabaja la planta con los valores del PID y varía el setpoint
  if(b_planta){

    //SIMILAR AL ANTERIOR, PERO VOY MODIFICANDO EL SETPOINT
    //1. ingreso con los valores que puse para MI PID
    //2. logíca pid
    //3. asignar el duty PERO, no tiene sentido si lo tenemos quieto.
    // si el encoder.getCount() es igual al setpoint, entonces no hay que hacer nada.
    //4. cambiamos el setpoint del bracito, entonces obligamos al pid a cambiar para alcanzar el setpoint.

    input = encoder.getCount();
    
    myPID.Compute();             // Calcula el nuevo valor de salida del PID
    duty = (int)output;          // Casteo
    
    duty = map(duty,MAX_VALUE_ENCODER*(-1) ,MAX_VALUE_ENCODER , -100, 100); // verificar mapeo con el profe


    if (output >= 0) {
      digitalWrite(SENTIDOHORARIO, HIGH);
      digitalWrite(SENTIDOANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, (int)output);
    } else {
      digitalWrite(SENTIDOHORARIO, LOW);
      digitalWrite(SENTIDOANTIHORARIO, HIGH);
      pwm->setPWM(PWM_PIN, FRECUENCIA, (int)(-output)); // valor positivo para PWM / evito tener dos variables duty, la ultima es el duty casteado.
    }
    
    switch (contador)
    {
    case 0:
      setpoint = 0.1 * MAX_VALUE_ENCODER; // Cambia el set  point

      break;

    case 20:
      setpoint = 0.25 * MAX_VALUE_ENCODER; // Cambia el set  
      break;

    case 40:
      setpoint = 0.5 * MAX_VALUE_ENCODER; // Cambia el set  
      break;

    case 60:
      setpoint = 0.75 * MAX_VALUE_ENCODER; // Cambia el set
      break;

    case 80:
      setpoint = 0.25 * MAX_VALUE_ENCODER; // Cambia el set 
      break;

    case 100:
      setpoint = 0; // Cambia el set
      b_planta = false; // Termina la planta
      digitalWrite(SENTIDOHORARIO, LOW);  
      digitalWrite(SENTIDOANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, 0); // Apaga
      break;

    default:
      break; 
    }
    encoder_lect[contador] = encoder.getCount();
    pwm_lect[contador] = duty;

  }

// Mostrar por Serial
  if (b_graficos){

  //implementar la logica de mostrar por serial y/o guardar en un archivo.

  
  //Serial.printf(">Encoder: %ld | Duty: %d\n", encoder.getCount(), duty);

  // Convertimos la posición del encoder a un ángulo para mostrarlo como un gráfico circular (torta) en Teleplot
  long count = encoder.getCount();
  long pos = count % MAX_VALUE_ENCODER;
  if (pos < 0) pos += MAX_VALUE_ENCODER;


  //Grafico la posición del encoder en formato polar
  float angle_deg = (float(pos) / MAX_VALUE_ENCODER) * 360.0;
  //Serial.printf(">@polar(%.2f, 1) PosicionEncoder\n", angle_deg);
  Serial.printf(">Contador: %d\n", contador);

  if(digitalRead(SENTIDOANTIHORARIO) == HIGH){
    //Serial.printf(">| Frecuencia: %d Hz | Duty: %d%%\n", FRECUENCIA, duty);
    Serial.printf(">PWM_Duty:%ld:%d\n", millis(), duty * (-1));  //Grafico el duty negativo para el sentido antihorario
  }

  if(digitalRead(SENTIDOHORARIO) == HIGH){
    //Serial.printf(">| Frecuencia: %d Hz | Duty: %d%%\n", FRECUENCIA, duty);
    Serial.printf(">PWM_Duty:%ld:%d\n", millis(), duty);
  }

  }


  if((b_iden || b_PID || b_planta || b_iden2) && contador < MAXIMA_LONGITUD_DATOS) {
    contador++;
  }else {
    contador = 0; // Reinicia el contador si no está en modo identificación o PID
  }
  
  
}