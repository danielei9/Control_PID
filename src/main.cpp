#include <Arduino.h>
/*
PRACTICA 1: IMPLEMENTACION DE UN CONTROLADOR PARA UN MOTOR DE DC

*/

// DECLARACIONES //////////////////////////////////////////////////////////////////////////

#define NOMBRE_PRAC "P1-A"
#define VERSION_SW "1.0"

#define ACTIVA_P1A
// #define DEBUG_P1A
#define ACTIVA_P1B1
#define ACTIVA_P1B2
#define ACTIVA_P1B3
#define ACTIVA_P1C
#define DEBUG_P1C
// #define ACTIVA_P1C_MED_ANG
// #define ACTIVA_P1D2
// #define ACTIVA_P1D3

// Display OLED ///////////////////////////////////////////////////////////////////////////
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1	 // Reset pin # (or -1 if sharing Arduino reset pin)

// Parametros cola de la interrupcion del encoder ///////////////////////////////////////
#define TAM_COLA_I 1024 /*num mensajes*/
#define TAM_MSG_I 1		/*num caracteres por mensaje*/

// TIEMPOS
#define BLOQUEO_TAREA_LOOPCONTR_MS 10
#define BLOQUEO_TAREA_MEDIDA_MS 100

// Configuración PWM  ////////////////////////////////////////////////////////////////////
uint32_t pwmfreq = 1000; // 1KHz
const uint8_t pwmChannel = 0;
const uint8_t pwmresolution = 8;
const int PWM_Max = pow(2, pwmresolution) - 1; //

// Pines driver motor ////////////////////////////////////////////////////////////////////
const uint8_t PWM_Pin = 32; // Entrada EN
const uint8_t PWM_f = 16;	// Entrada PWM1
const uint8_t PWM_r = 17;	// Entrada PWM2

// Voltaje maximo motor ////////////////////////////////////////////////////////////////////
float SupplyVolt = 12;

// Pines encoder ////////////////////////////////////////////////////////////////////
const uint8_t A_enc_pin = 35;
const uint8_t B_enc_pin = 34;

// Conversión a angulo y velocidad del Pololu 3072
// const float conv_rad = ;
// const float conv_rev = ;
// const float conv_rad_grados = ;

// Declarar funciones ////////////////////////////////////////////////////////////////////
void config_sp();					  // Configuracion puerto serie
void config_oled();					  // Configuracion OLED
void config_enc();					  // Configuracion del encoder
void config_PWM();					  // Configuracion PWM
void excita_motor(float v_motor);	  // Excitacion motor con PWM
float interpola_vel_vol_lut(float x); // Interpolacion velocidad/voltios LUT

// TABLA VELOCIDAD-VOLTAJE P1D
#ifdef ACTIVA_P1D2
#define LONG_LUT 12
// Vector de tensiones
const float Vol_LUT[LONG_LUT] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
// Vector de velocidades
const float Vel_LUT[LONG_LUT] = {0, 1.15, 3, 4.1, 5.4, 5.9, 6.4, 7, 7.5, 7.5, 7.5, 7.5};
#endif
uint8_t byteVal = 0;

uint8_t aVal, bVal;

// Variables globales //////////////////////////////////////////////////////////////////// V12
int32_t ang_cnt = 0;
float pwm_volt = 0;
// int32_t pwm_motor = 0;
// int32_t sign_v_ant = 0;
float v_medida = 0;	   // Valor medido de angulo o velocidad -----------------
float ref_val = 0;	   // Valor de referencia de angulo o velocidad
int8_t start_stop = 0; // 1 -> en funcionamiento | 0 -> parado
// float K_p = ;
// VELOCIDAD
float kp = 5.0, kd = 0.1, ki = 5.0;
float Tm = 0.01;

// Declaracion objetos  ////////////////////////////////////////////////////////////////////

xQueueHandle cola_enc; // Cola encoder

float proporcional, integral, derivativo;
/*
 RUTINAS ATENCION INTERRUPCIONES ########################################################################
*/

/*
 Rutina de atención a interrupción ISC_enc --------------------------------------------
*/
bool angulo = true;
bool windup = true;

void IRAM_ATTR ISR_enc()
{
	// Lee las salidas del Encoder
	aVal = digitalRead(A_enc_pin);
	bVal = digitalRead(B_enc_pin);

	/*Serial.println("Lectura De pines ");
	Serial.print("Aval: ");
	Serial.println(aVal);
	Serial.print("Bval: ");
	Serial.print(bVal);*/
	// Procesa los datos
	// Serial.println("ISR_enc");
	byteVal = (bVal) + (aVal * 2);

	// Enviar los bytes a la cola
	if (xQueueSendFromISR(cola_enc, &byteVal, NULL) != pdTRUE)
	{
		printf("Error de escritura en la cola cola_enc \n");
	};
}

/*
 TAREAS #############################################################################
*/

/*
 Tarea task_enc #####################################################################
*/
uint8_t buff[7];
int lastByteVal = 0;
#ifdef ACTIVA_P1A
void task_enc(void *arg)
{
	// Declaracion de variables locales
	// Serial.println("Dentro de task_enc");
	while (1)
	{
		// Espera a leer los datos de la cola
		if (xQueueReceive(cola_enc, &buff, (TickType_t)portMAX_DELAY) == pdTRUE)
		{
			//	Serial.println("Recibiendo datos");

			// Codificar la fase del encoder
			// HIGH va a ser el a y Low B
			switch (buff[0])
			{
			case 0: //
				buff[0] = 1;
				break;
			case 1: //
				buff[0] = 2;
				break;
			case 2: //
				buff[0] = 4;
				break;
			case 3: //
				buff[0] = 3;
				break;
			}
			// Serial.println(byteVal);
			// Serial.println(lastByteVal);

			// Calcular incremento/decremento y actualizar valor

			if (buff[0] == 4)
			{
				if (lastByteVal == 3)
				{
					ang_cnt++;
				}
				else
				{
					ang_cnt--;
				}
			}
			else if (buff[0] == 1)
			{
				if (lastByteVal == 2)
				{
					ang_cnt--;
				}
				else
				{
					ang_cnt++;
				}
			}
			else
			{
				if (lastByteVal < buff[0])
				{
					ang_cnt++;
				}
				else
				{
					ang_cnt--;
				}
			}

			lastByteVal = buff[0];

#ifdef DEBUG_P1A
			// Enviar al monitor serie
			Serial.print("ang_cnt: ");
			Serial.println(ang_cnt);
#endif
		}
		else
		{
			printf("Error de lectura de la cola cola_enc \n");
		}
	}
}
#endif

/*
Tarea de configuración de parámetros  #####################################################################
*/
void task_config(void *pvParameter)
{
	char ini_char = '0';
	while (1)
	{
		// Detectar caracter enviado
		if (Serial.available())
		{
			ini_char = Serial.read();
			if (ini_char == 'V')
			{
				// Guardar valor recibido
				float recived = Serial.parseFloat();
				// Escribir el valor recibido en la consola
				pwm_volt = recived;
				printf("El Voltaje valor es %f.\n", pwm_volt);
			}
			else if (ini_char == 'R')
			{
				float recived = Serial.parseFloat();
				ref_val = recived;
				// En caso de ser Angulo se debe convertir a radianes
				if (angulo)
				{
					// USAR ESTOS VALORES EN ANGULO
					kp = 10.0, kd = 1.0, ki = 5.0;

					// kp = 10; //3.5
					// kd = 0.1;
					// ki = 0.02;
					ref_val = ref_val * PI / 180;
				}

				printf("El ref_val valor es %f rps.\n", ref_val);
			}
			else if (ini_char == 'S')
			{
				float recived = Serial.parseFloat();
				start_stop = recived;
				if (start_stop)
					printf("---START---\n");
				else
					printf("---STOP---\n");
			}
			else if (ini_char == 'p')
			{
				float recived = Serial.parseFloat();
				kp = recived;
				printf("El kp valor es %f.\n", kp);
			}
			else if (ini_char == 'd')
			{
				float recived = Serial.parseFloat();
				kd = recived;
				printf("El kd valor es %f.\n", kd);
			}
			else if (ini_char == 'i')
			{
				float recived = Serial.parseFloat();
				ki = recived;
				printf("El ki valor es %f.\n", ki);
			}
			else if (ini_char == 'A')
			{
				float recived = Serial.parseInt();
				angulo = recived;
				printf("Referencia en angulo activada?  %d.\n", angulo);
			}
			else if (ini_char == 'W')
			{
				float recived = Serial.parseInt();
				windup = recived;
				printf("Windup activado?  %d.\n", windup);
			}
		}
		// Activacion de la tarea cada 0.1s
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

/*
Tarea del lazo principal del controlador  #####################################################################
*/
float lastAngulo = 0;
float vel = 0, lastError = 0, lastIntegral = 0;
float error = 0;

#ifdef ACTIVA_P1B3
void task_loopcontr(void *arg)
{
	while (1)
	{
		lastAngulo = v_medida;
		v_medida = -(2 * PI * ang_cnt) / 1200; // ang radianes V10

		vel = (v_medida - lastAngulo) / Tm; // en segundos
		vel = vel / (2 * PI);

		if (angulo)
		{
			error = ref_val - v_medida;
		}
		else
			error = ref_val - vel;

		// if (pwm_volt < 0)
		// 	vel = -vel;

		if (start_stop)
		{
#ifdef ACTIVA_P1D2
			pwm_volt = interpola_vel_vol_lut(ref_val);
#endif
#ifdef ACTIVA_P1D3
			if (error < 0)
				pwm_volt -= 0.2;
			if (error > 0)
				pwm_volt += 0.2;

#endif
			proporcional = (error * kp);				  // implementación de kp se
			integral = (lastIntegral + error * ki * Tm);  // implementación de kp se
			derivativo = ((error - lastError) * kd / Tm); // (sum(lastError - error) * ki); // implementación de kp se

			pwm_volt = proporcional + integral + derivativo;
			// WindUP
			//  SI la salida esta saturada
			if (windup)
			{
				if (abs(pwm_volt) > 9)
				{
					// Quitamos integral
					integral = lastIntegral;
				}
			}
		}
		else
		{
			// se parará el motor y se pondrán a cero las variables globales ang_cnt, pwm_motor y v_medida
			ang_cnt = 0;
			pwm_volt = 0;
			v_medida = 0;
		}
		lastError = error;
		lastIntegral = integral;
		//  Excitacion del motor con PWM
		excita_motor(pwm_volt);
		// Activacion de la tarea cada 0.01s
		vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
	}
}
#endif

/*
Tarea del lazo principal del controlador  #####################################################################
*/
#ifdef DEBUG_P1C
void task_medidas(void *arg)
{
	while (1)
	{
		// Mostrar medidas de angulo y velocidad del motor
		if (angulo)
		{
			float ang = v_medida * (360 / (2 * PI));
			printf(" radianes: %f, error: %f, ref: %f, proporcional: %f, integral: %f, derivativo: %f, vmotor: %f\n", v_medida, error, ref_val, proporcional, integral, derivativo, pwm_volt);
			// printf("grados: %f, radianes: %f, error: %f, ref: %f \n", ang, v_medida, error, ref_val);
		}
		else
		{
			//  kp = 5.0, kd = 1.0, ki = 5.0;
			// printf("velocidad: %f, lastAng: %f, currentAng: %f, refVal: %f, kp: %f, kd: %f, ki: %f\n", vel, lastAngulo, v_medida, ref_val, kp, kd, ki);
			printf("velocidad: %f, refVal: %f\n", vel, ref_val);
		}

		vTaskDelay(BLOQUEO_TAREA_MEDIDA_MS / portTICK_PERIOD_MS);
		// Activacion de la tarea cada 1s
	}
}
#endif

/*
SET UP -----------------------------------------------------------------------------------
*/
void setup()
{
	// Configuracion puerto serie
	config_sp();
	config_enc();
	// Configuracion OLED
	config_oled();

	// Configuracion PWM
	config_PWM();

	// Crear cola_enc
	cola_enc = xQueueCreate(TAM_COLA_I, TAM_MSG_I);
	if (cola_enc == NULL)
	{
		Serial.println("Error en creacion de cola_enc");
		exit(-1);
	};
	// Serial.println("Creando la Taread task_enc...");
	//  Crear la tarea task_enc
	if (xTaskCreate(task_enc, "task_enc", 2048, NULL, 1, NULL) != pdPASS)
	{
		Serial.println("Error en creacion tarea task_enc");
		exit(-1);
	}

	// Crear la tarea task_config
	if (xTaskCreate(task_config, "task_config", 2048, NULL, 1, NULL) != pdPASS)
	{
		Serial.println("Error en creacion tarea task_config");
		exit(-1);
	}

	// Crear la tarea task_loopcontr
	if (xTaskCreate(task_loopcontr, "task_loopcontr", 2048, NULL, 1, NULL) != pdPASS)
	{
		Serial.println("Error en creacion tarea task_loopcontr");
		exit(-1);
	}

#ifdef DEBUG_P1C
	// Crear la tarea task_medidas
	if (xTaskCreate(task_medidas, "task_medidas", 2048, NULL, 1, NULL) != pdPASS)
	{
		Serial.println("Error en creacion tarea task_medidas");
		exit(-1);
	}
#endif

	// Configuracion del encoder
}

/*
LOOP ---- NO USAR -------------------------------------------------------------------
*/
void loop() {}
// FUNCIONES ////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del encoder
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1A
void config_enc()
{
	// Configuracion de pines del encoder
	pinMode(A_enc_pin, INPUT);
	pinMode(B_enc_pin, INPUT);
	// Configuracion interrupcion
	attachInterrupt(digitalPinToInterrupt(A_enc_pin), ISR_enc, CHANGE);
	attachInterrupt(digitalPinToInterrupt(B_enc_pin), ISR_enc, CHANGE);
	// printf("Interrupcion \n");
}
#endif
////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del PWM
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1B2
void config_PWM()
{
	// printf("config_PWM\n");
	//  Configuracion de pines de control PWM
	int freq = pwmfreq;
	int channel = pwmChannel;
	int resolution = pwmresolution;
	pinMode(PWM_f, OUTPUT);
	pinMode(PWM_r, OUTPUT);
	// Configuracion LED PWM
	ledcSetup(channel, freq, resolution);
	// Asignar el controlador PWM al GPIO
	ledcAttachPin(PWM_Pin, channel);
}
#endif

////////////////////////////////////////////////////////////////////////////////////
// Funcion excitacion del motor con PWM
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1B3
void excita_motor(float v_motor) // voltaje en motor
{
	// Calcula y limita el valor de configuración del PWM
	// sentido A
	// printf("vmotor AA %f", v_motor);
	if (v_motor > 0)
	{
		digitalWrite(PWM_r, HIGH);
		digitalWrite(PWM_f, LOW);
		// printf("GIRA 1\n");
	}
	if (v_motor < 0)
	{
		digitalWrite(PWM_r, LOW);
		digitalWrite(PWM_f, HIGH);
		// printf("GIRA 2\n");
	}

	if (abs(v_motor) > 12)
	{
		v_motor = 12;
	}

	// El valor de excitación debe estar entro 0 y PWM_Max
	v_motor = map(abs(v_motor), 0, 12, 0, PWM_Max);
	// printf("** vmotor BB %f\n", v_motor);
	// vmotor=
	//  Excitacion del motor con PWM
	ledcWrite(pwmChannel, v_motor);
}
#endif

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del puerto serie
////////////////////////////////////////////////////////////////////////////////////
void config_sp()
{
	Serial.begin(115200);
	Serial.println("  ");
	Serial.println("--------------------------------------------");
	Serial.println("PRACTICA CONTROLADOR MOTOR " NOMBRE_PRAC);
	Serial.println("--------------------------------------------");
}

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del OLED
////////////////////////////////////////////////////////////////////////////////////
void config_oled()
{
	Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	display.clearDisplay();
	display.setTextColor(WHITE); //
	display.setCursor(0, 0);	 // Start at top-left corner
	display.println(F("CONTR. MOTOR " NOMBRE_PRAC));
	display.display();
	delay(1000);
	display.setTextColor(BLACK, WHITE); //
	display.setCursor(0, 20);			// Start at top-left corner
	display.println(F(" SW v." VERSION_SW));
	display.display();
	delay(1000);
}

////////////////////////////////////////////////////////////////////////////////////
// Funcion de interpolacion LUT de Velocidad-Voltaje
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1D2
float interpola_vel_vol_lut(float x)
{
	// Buscar el valor superior más pequeño del array
	int8_t i = 0;
	if (x >= Vel_LUT[LONG_LUT - 2])
	{
		i = LONG_LUT - 2;
	}
	else
	{
		while (x > Vel_LUT[i + 1])
			i++;
	}

	// Guardar valor superior e inferior
	float xL = Vel_LUT[i];
	float yL = Vol_LUT[i];
	float xR = Vel_LUT[i + 1];
	float yR = Vol_LUT[i + 1];

	// Interpolar
	float dydx = (yR - yL) / (xR - xL);

	return yL + dydx * (x - xL);
}
#endif