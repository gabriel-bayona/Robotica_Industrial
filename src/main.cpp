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
#define SENTIDO_HORARIO 16     // Pin para sentido horario 
#define SENTIDO_ANTIHORARIO 15     // Pin para sentido antihorario

#define MAXIMA_LONGITUD_DATOS 100

#define FRECUENCIA 8000 // Frecuencia del PWM en Hz
#define MAX_VALUE_ENCODER 62300 // Valor máximo del encoder --- esto depende del encoder, VERIFICAR EN DATASHEET

#define V_PID_MIN -12 // Valor mínimo del PID
#define V_PID_MAX 12 // Valor máximo del PID

ESP32_FAST_PWM* pwm;

//
ESP32Encoder encoder;


long QEI_Pos,QEI_Pos_1; //24300 pulsos por vuelta

int contador;
float encoder_lect[MAXIMA_LONGITUD_DATOS];
float pwm_lect[MAXIMA_LONGITUD_DATOS];



// Configuración del PID
double error_actual, output, setpoint;

float x0, x0_1;
float x0_p, x0_p1;
float a, b, c, Ts; //Constantes para parámetros de controlador PID
float rT, eT, iT, dT, yT, uT, iT0, eT0, u_1; //Variables de controlador PID

double kp;
double ki;
double kd;


long filtro[5]; //para calcular un promedio de valores

bool b_windup = true;

void apagarControl();

//funcion para apagar todo



//Bandera de interrupcion
bool b_mostrar; // bandera que indica si se debe mostrar la salida por serial
bool b_iden;
bool b_iden2; // bandera que indica si se debe trabajar la identificación 2
//int MEF_iden1;
bool b_PID;
bool b_planta; // bandera que indica si se debe trabajar la planta indicada//cargada
bool b_encoder = false; // bandera que indica si se debe mostrar el encoder
bool b_graficos; // bandera que indica si se deben mostrar los gráficos por serial
int duty;

bool encabezado = false;


void apagarControl() {
  b_PID = false;
  b_iden2 = false;
  b_iden = false;
  b_planta = false;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Resolución ADC de 12 bits (0-4095)
  pinMode(SENTIDO_ANTIHORARIO, OUTPUT);
  pinMode(SENTIDO_HORARIO, OUTPUT);
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


  //Configuracion del PID
  Ts = 0.01; //tiempo de muestreo

  b_windup = true;
	Ts = 0.01;
  x0_1 = 0.0;
	iT0 = 0.0;  // acumulador integrador
	eT0 = 0.0;  // valor previo deriv
	u_1 = 0.0;  // valor previo u

  a = 80;
	b = 0.5 * a * Ts; 
	c = 0.001 * a / Ts; 

  //estas son variables de control del programa (me mueven entre las diferentes fuciones)
  b_mostrar = false;
  b_iden = false;
  b_iden2 = false; 
  b_PID = false;
  b_planta = false;
  b_encoder = false; // bandera para mostrar el encoder
  contador = 0;

}


inline void Leer_serial() {
  if (Serial.available()) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim(); // Elimina espacios en blanco

    if (inputStr == "strt") {
      contador = 0;
      apagarControl();
      
      encoder.clearCount();
      Serial.println("Modo predeterminado iniciado.");
      digitalWrite(SENTIDO_HORARIO, LOW);
      digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
      duty = 50;
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

    } else if (inputStr == "stop") {

      apagarControl();
      b_graficos = false;
      b_encoder = false;
      b_mostrar = false;

      encoder.clearCount();
      digitalWrite(SENTIDO_HORARIO, LOW);
      digitalWrite(SENTIDO_ANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, 0);
      duty = 0;

      Serial.println("PWM y motor detenidos.");

    } else if (inputStr == "iden") {
      apagarControl();
      b_iden = true;
      contador = 0;
      encoder.clearCount();
      Serial.println("Identificación 1 activada.");
    }else if (inputStr == "ide2"){
      apagarControl();
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

    } else if (inputStr.startsWith("spid")) {
      inputStr.remove(0, 4);
      inputStr.trim();

      int firstSpace = inputStr.indexOf(' ');
      int secondSpace = inputStr.indexOf(' ', firstSpace + 1);

      if (firstSpace > 0 && secondSpace > firstSpace) {
        kp = inputStr.substring(0, firstSpace).toDouble();
        kd = inputStr.substring(firstSpace + 1, secondSpace).toDouble();
        ki = inputStr.substring(secondSpace + 1).toDouble();

        Serial.printf("PID actualizado -> Kp: %.2f | Ki: %.2f | Kd: %.2f\n", kp, ki, kd);
      } else {
        Serial.println("Error: Formato incorrecto. Usa: spid <kp> <kd> <ki>");
      }

    }
    else if (inputStr == "epid")
        {
          apagarControl();
          b_PID = true;

          contador = 0;
          encoder.clearCount();
          Serial.println("PID activado.");
        } 

    else if (inputStr.startsWith("spwm")) {
      apagarControl();
      inputStr.remove(0, 4);
      inputStr.trim();

      int duty_fijo = inputStr.toInt();

      //De acuerdo al valor de duty_fijo, se define el sentido del motor
      if (duty_fijo >0 && duty_fijo <= 100) {
        digitalWrite(SENTIDO_HORARIO, HIGH);
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);
      } else if (duty_fijo < 0 && duty_fijo >= -100) {
        digitalWrite(SENTIDO_HORARIO, LOW);
        digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
      } else 
      {
        digitalWrite(SENTIDO_HORARIO, LOW);
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);
      }

      duty_fijo = abs(duty_fijo); // Asegurarse de que el valor sea positivo (lo requiere el objeto pwm)
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty_fijo);
      Serial.printf("PWM actualizado -> PWM: %.2f\n", (float)duty_fijo);

      

    }else if (inputStr.startsWith("show")) {
      inputStr.remove(0, 4);
      inputStr.trim();

      if (inputStr == "pid") {
        Serial.printf("Valores PID actuales:\nKp: %.2f | Ki: %.2f | Kd: %.2f\n", kp, ki, kd);
        Serial.printf("\nSetpoint actual: %.2f\n", setpoint);
      } else if (inputStr == "data") {
        //Serial.println("Mostrando arrays:");
        for (int i = 0; i < MAXIMA_LONGITUD_DATOS; i++) {
          Serial.printf("%d\t%.2f\t%.2f\n", i, encoder_lect[i], pwm_lect[i]);
        }
      
      
      } else if (inputStr == "pwm"){

        if (SENTIDO_HORARIO) {
          Serial.printf("- PWM: %d\n", pwm->getActualDutyCycle());
        } else if (SENTIDO_ANTIHORARIO) {
          Serial.printf("- PWM: %d\n",(-1) * pwm->getActualDutyCycle());
        } else 
        {
          Serial.printf("- PWM: %d\n", pwm->getActualDutyCycle());
        }
        

      }else if (inputStr == "pout") {
        Serial.println("Pines:");
        Serial.printf("- PWM_PIN: %d\n", PWM_PIN);
        Serial.printf("- POT_PIN: %d\n", POT_PIN);
        Serial.printf("- ENABLE: %d\n", ENABLE);
        Serial.printf("- SENTIDO_HORARIO: %d\n", SENTIDO_HORARIO);
        Serial.printf("- SENTIDOANTIHORARIO: %d\n", SENTIDO_ANTIHORARIO);
        
      }else if (inputStr == "pose") {
        b_encoder = !b_encoder; // Cambia el estado de la bandera
        if (b_encoder) {
          Serial.println("Lectura del encoder activada.");
        } else {
          Serial.println("Lectura del encoder desactivada.");
        }
      }     
      else {
        Serial.println("Uso de 'show':\n - show pid\n - show data\n - show pose\n - show pout\n");
      }


    }else if (inputStr == "rest") {
      Serial.println("Reiniciando sistema...");
      for (int i = 0; i < MAXIMA_LONGITUD_DATOS; i++) {
        encoder_lect[i] = 0;
        pwm_lect[i] = 0;
      }
      apagarControl();
      b_mostrar = false;
      b_graficos = false;
      duty = 0;
      encoder.clearCount();
      digitalWrite(SENTIDO_HORARIO, LOW);
      digitalWrite(SENTIDO_ANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, 0);

    }else if (inputStr == "enab") {
      digitalWrite(ENABLE, HIGH);
      Serial.println("Habilitador encendido.");

    }else if (inputStr == "disa") {
      digitalWrite(ENABLE, LOW);
      Serial.println("Habilitador apagado.");

    }else if (inputStr == "graf") {
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
      Serial.println(" - setp <valor>: Asignar setpoint.");
      Serial.println(" - spwm <duty>: Actualizar PWM (0-100 o -100 a 100).");
      Serial.println(" - spid <kp> <kd> <ki>: Actualizar PID.");
      Serial.println(" - show pid: Mostrar valores PID actuales.");
      Serial.println(" - show data: Mostrar arrays de datos.");
      Serial.println(" - show pout: Mostrar pines utilizados.");
      Serial.println(" - show pose: Mostrar posicion actual encoder.");
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
  
  //pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

  encoder_lect[contador]=encoder.getCount();

  QEI_Pos_1 = QEI_Pos; // Save the previous position
  QEI_Pos = encoder.getCount();
    
  // Primer if para identificar la planta
  if (b_iden) {
    switch (contador) {
      case 0:
        digitalWrite(SENTIDO_HORARIO, LOW);
        digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
        duty = 10;
        pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
        break;
        
      case 20:
        digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
        digitalWrite(SENTIDO_HORARIO, LOW);
        duty = 20;
        pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

        break;
        
      case 40:
        duty = 0;
        pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);
        digitalWrite(SENTIDO_HORARIO, LOW);
        break;
        
      case 60:
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);
        digitalWrite(SENTIDO_HORARIO, HIGH);
        duty = 10; //-10
        pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
        break;
        
      case 80:
        duty = 20; //-20
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);
        digitalWrite(SENTIDO_HORARIO, HIGH);
        pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
        break;
        
      case 100:
        duty = 0;
        pwm->setPWM(PWM_PIN, FRECUENCIA, duty);
        digitalWrite(SENTIDO_HORARIO, LOW);
        digitalWrite(SENTIDO_ANTIHORARIO, LOW);
        b_iden = false;
        break;

    }
    

    // Guardar los valores de encoder y PWM en los arrays
    //EL PWM SE GUARDA SEGUN SENTIDO, YA QUE EL OBJETO PWM NO ME PERMITE HACERLO CON UN VALOR NEGATIVO
    if(digitalRead(SENTIDO_HORARIO) == HIGH){
      encoder_lect[contador] = encoder.getCount();
      pwm_lect[contador] = (-1)*duty;

    }else if(digitalRead(SENTIDO_ANTIHORARIO) == HIGH){
      encoder_lect[contador] = encoder.getCount();
      pwm_lect[contador] = duty;
    }else{
      encoder_lect[contador] = encoder.getCount();
      pwm_lect[contador] = 0; // Si no hay sentido, guardo 0
    }
    
  }  
  
  // Segundo if para identificar la planta con PID
  if(b_iden2){

    if(contador <30){
      digitalWrite(SENTIDO_HORARIO, LOW);
      digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
      duty = contador * 2.5; // Incrementa el duty de 0 a 75
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

    } else if (contador >= 30 && contador <= 50) {
      digitalWrite(SENTIDO_HORARIO, HIGH);
      digitalWrite(SENTIDO_ANTIHORARIO, LOW);
      duty = 70 - contador; // Decrementa el duty de 75 a 0
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

    } else if (contador > 50 && contador < 70) {
      digitalWrite(SENTIDO_HORARIO, LOW);
      digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
      duty =  contador  - 50; // Incrementa el duty de 0 a 75
      pwm->setPWM(PWM_PIN, FRECUENCIA, duty);

    } else {
      digitalWrite(SENTIDO_HORARIO, LOW);
      digitalWrite(SENTIDO_ANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, 0); // Apaga el PWM
      b_iden2 = false; // Termina la identificación con PID

    }

    // Guardar los valores de encoder y PWM en los arrays
    //EL PWM SE GUARDA SEGUN SENTIDO, YA QUE EL OBJETO PWM NO ME PERMITE HACERLO CON UN VALOR NEGATIVO
    if(digitalRead(SENTIDO_HORARIO) == HIGH){
      encoder_lect[contador] = encoder.getCount();
      pwm_lect[contador] = (-1)*duty;

    }else if(digitalRead(SENTIDO_ANTIHORARIO) == HIGH){
      encoder_lect[contador] = encoder.getCount();
      pwm_lect[contador] = duty;
    }else{
      encoder_lect[contador] = encoder.getCount();
      pwm_lect[contador] = 0; // Si no hay sentido, guardo 0
    }
  }

  if (b_PID){
    /*
      Fórmula discreta (digital) del controlador PID:
        u(k) = a . e(k) + i(k) + d(k)
        e(k) = r(k) - y(k) : Error actual (setpoint menos la salida)
        i(k) = b . e(k) + i(k-1) : termino integral acumulado (error por constante más el anterior)
        d(k) = c . [y(k) - y(k-1)] / Ts : Termino derivativo (tasa de cambio, tomo deltas).

        a-> ganancia proporcional
        b-> coeficiente integral (normalmente b = Kp . Ti)
        c-> coeficiente derivativo (normalmente c = Kd / Ts)
        u(k)-> señal de control, en este caso el PWM
        Ts-> tiempo de muestreo

    */
    
    //Calculo de constantes:

    a= kp +  ki * Ts;
    b = ki * Ts;
    c = kd / Ts;

    //e(k) = r(k) - y(k) : Error actual (setpoint menos la salida)
    eT= setpoint - (encoder.getCount())/MAX_VALUE_ENCODER;

/*
"Si el anti-windup está desactivado, o bien el control anterior no está saturado en el límite positivo
 mientras el error quiere seguir subiendo, y no está saturado en el límite negativo mientras el error
  quiere seguir bajando, entonces permite que el integrador acumule."

Solo acumula si:
No está saturado por arriba o el error quiere ir hacia abajo, y
No está saturado por abajo o el error quiere ir hacia arriba
*/
    if ((!b_windup)|| (((u_1 < 12) || (eT < 0)) && ((u_1 > -12) || (eT > 0)))){

      iT = b * eT + iT0;
    }
    
    //Se hace un filtro de media móvil de 5 valores para suavizar el ruido del encoder.
    filtro[0]=filtro[1];
    filtro[1]=filtro[2];
    filtro[2]=filtro[3];
    filtro[3]=filtro[4];
    filtro[4]=QEI_Pos - QEI_Pos_1;

    //d(k) = c . [y(k) - y(k-1)] / Ts : Termino derivativo (tasa de cambio promedio, tomo deltas).
    dT = c * ((filtro[0]+filtro[1]+filtro[2]+filtro[3]+filtro[4]) / 5) / (MAX_VALUE_ENCODER);


    //u(k) = a . e(k) + i(k) + d(k)
    //Cálculo de la señal de control total:
    uT = a * eT + iT + dT; //Calcular senal de control u(kT)

    //limitar la salida - SATURACION
    if (uT > V_PID_MAX){
      uT = V_PID_MAX;
    }else if (uT < V_PID_MIN){
      uT = V_PID_MIN;
    }

    duty = uT/(V_PID_MAX)*100;


    //Actualización de variables para la siguiente iteración:
    u_1 = uT;
    iT0 = iT;
		eT0 = eT;
    x0_1 = x0;
    
    if (duty >= 0) {
      digitalWrite(SENTIDO_HORARIO, HIGH);
      digitalWrite(SENTIDO_ANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, (int)duty);
  } else {
      digitalWrite(SENTIDO_HORARIO, LOW);
      digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
      pwm->setPWM(PWM_PIN, FRECUENCIA, (int)(-duty)); // valor positivo para PWM
  }

    // Guardar los valores de encoder y PWM en los arrays
    //EL PWM SE GUARDA SEGUN SENTIDO, YA QUE EL OBJETO PWM NO ME PERMITE HACERLO CON UN VALOR NEGATIVO
    if(digitalRead(SENTIDO_HORARIO) == HIGH){
      encoder_lect[contador] = encoder.getCount();
      pwm_lect[contador] = duty;

    }else if(digitalRead(SENTIDO_ANTIHORARIO) == HIGH){
      encoder_lect[contador] = encoder.getCount();
      pwm_lect[contador] = (-1) * duty;
    }else{
      encoder_lect[contador] = encoder.getCount();
      pwm_lect[contador] = 0; // Si no hay sentido, guardo 0
    }

  }

  // Trabaja la planta con los valores del PID y varía el setpoint
  //CORREGIR PARA EL NUEVO PID 
  if(b_planta){

    //SIMILAR AL ANTERIOR, PERO VOY MODIFICANDO EL SETPOINT
    //1. ingreso con los valores que puse para MI PID
    //2. logíca pid
    //3. asignar el duty PERO, no tiene sentido si lo tenemos quieto.
    // si el encoder.getCount() es igual al setpoint, entonces no hay que hacer nada.
    //4. cambiamos el setpoint del bracito, entonces obligamos al pid a cambiar para alcanzar el setpoint.
/* 
    input = encoder.getCount();
    
    duty = (int)output;          // Casteo
    
    duty = map(duty,MAX_VALUE_ENCODER*(-1) ,MAX_VALUE_ENCODER , -100, 100); // verificar mapeo con el profe


    if (output >= 0) {
      digitalWrite(SENTIDO_HORARIO, HIGH);
      digitalWrite(SENTIDO_ANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, (int)output);
    } else {
      digitalWrite(SENTIDO_HORARIO, LOW);
      digitalWrite(SENTIDO_ANTIHORARIO, HIGH);
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
      digitalWrite(SENTIDO_HORARIO, LOW);  
      digitalWrite(SENTIDO_ANTIHORARIO, LOW);
      pwm->setPWM(PWM_PIN, FRECUENCIA, 0); // Apaga
      break;

    default:
      break;  
    }
    // Guardar los valores de encoder y PWM en los arrays
    //EL PWM SE GUARDA SEGUN SENTIDO, YA QUE EL OBJETO PWM NO ME PERMITE HACERLO CON UN VALOR NEGATIVO
    if(SENTIDO_ANTIHORARIO){
      encoder_lect[contador] = encoder.getCount();
      pwm_lect[contador] = duty;

    }else if(SENTIDO_HORARIO){
      encoder_lect[contador] = encoder.getCount();
      pwm_lect[contador] = (-1)*duty;
    } */

  }

  if (b_encoder)
  {
    Serial.printf(">Encoder:%d\n", encoder.getCount());
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
  //Serial.printf(">PosicionEncoder%ld:%d\n", millis(),(int) encoder.getCount());


  Serial.printf(">Encoder:%d\n", encoder.getCount());
  Serial.printf(">Contador: %d\n", contador);

  if(digitalRead(SENTIDO_ANTIHORARIO) == HIGH){
    //Serial.printf(">| Frecuencia: %d Hz | Duty: %d%%\n", FRECUENCIA, duty);
    Serial.printf(">PWM_Duty:%ld:%d\n", millis(), duty * (-1));  //Grafico el duty negativo para el sentido antihorario
  }

  if(digitalRead(SENTIDO_HORARIO) == HIGH){
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
