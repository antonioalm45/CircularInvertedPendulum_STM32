/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  *
  * 			CONTROL DE UN PÉNDULO INVERTIDO CIRCULAR USANDO EL
  * 					MICROCONTROLADOR STM32F103C8
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIPO_CONTROL 1
#define LIMITE_ANGULO 700		// Angulo a partir del cual conmuta el control
#define LIMITE_GIRO 350
#define LIMITE_PWM 6000 		// Valor máximo de PWM sobre 7200 (12V).
#define LIMITE_VOLTAJE 1100		// Valor mínimo de voltaje del motor 1110 (11V)
#define Ts 0.005				// Periodo de muestreo
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* Parámetros del controlador formado por dos PD */
// Todas las K están multiplicadas por 7200 y divididas por 12
float KP_Pos = 61, //141, //25*1040/360
		KD_Pos = 50, //43, //600*1040/360 * 0.0250 = 15*1040/360;
		KP_Bal = 692, //592, //50*2134/180
		KD_Bal = 20.2; //14.4; //244*0.005 *2134/180 = 1.22*2134/180;
float Ki_Pos = 0;
float p_angulo = 0, p_posicion = 0.8;
/* Parámetros del controlador de realimentación de estados*/
// Todas las K están multiplicadas por 7200 y divididas por 12
float Ki = 100, K1 = 101, K3 = 46, K2 = 682, K4 = 18.2;
float param_filtro_pos = 0.956, param_filtro_ang =0;
float referencia = 0, referencia_local=0, referencia_externa=0;

/* Parámetros de configuración del controlador desde la pantalla
 	 - Menu: 		Indica la pantalla
 	 - Amplitude1: 	Incremento del valor de KP_Bal
 	 - Amplitude2: 	Incremento del valor de KD_Bal
 	 - Amplitude3:  Incremento del valor de KP_Pos
 	 - Amplitude4:  Incremento del valor de KD_Bal	*/
uint8_t Menu=1, Amplitude1=1, Amplitude2=1, Amplitude3=1, Amplitude4=1;

/* Parametros que indican el modo actual*/
uint8_t Tipo_ref;	// Indica si se aplica una rampa o un escalón
uint8_t Cuenta_diez_periodos =0;		// Cuenta si se ha repetido 10 veces Ts
uint16_t Cuenta_un_segundo;		// Cuenta si se ha repetido 1000 veces Ts
uint16_t Cuenta_cinco_segundos = 1000;// Cuenta si se ha repetido 5000 veces Ts
uint16_t VERTICAL = 3055;	// Angulo de la posición vertical
uint16_t INFERIOR;			// Angulo de la posición horizontal
uint8_t SWING_PRIMERO = 0, ORIGEN_REF = 1;
uint8_t Flag_control_motor, Flag_swing_up;	// Indican el modo de control

/* Señales medidas */
uint32_t valores_ADC[32];
float angulo_f, posicion_f, voltaje_f;

/* Señal de salida */
float PWM_motor;

/* Para la comunicación el ordenador */
float acc_integral;
float velocidad_1;
float velocidad_2;
uint8_t rxBuffer [8];
uint8_t txBuffer[29];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

float control_dos_PID (float, float, float);
float control_realim_estados (float, float, float);
float swing_up (float, float);

float filtro_posicion (float, float);
float filtro_angulo (float, float);
float filtro_swing (float, float);
float filtro_voltaje (float, float);
float filtro_apaga (float, float);
float filtro_media_10 (float);
float filtro_media_5 (float);
float filtro_butterworth (float, float);
float filtro_butterworth_2 (float, float);
float filtro_roc (float, float);

void poner_PWM_motor (float);
uint8_t comprueba_limites_control(void);
uint8_t comprueba_limites_swing(void);

void envia_datos_serie (float, float, float, float, float, float, float);
int click_N_Double (uint8_t);
void key(void);
void oled_show(void);
float obtener_angulo(void);
float obtener_referencia(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	/**************************************************************************
	 * Función de servicio a la interrupción del temporizador 1
	 * Ejecucion periódica cada 5ms, que el periodo de muestreo.
	**************************************************************************/
	if (htim->Instance == TIM1) {
		// Esto solo es para distinguir la interrupción de otros temporizadores,
		// si estos tienen interrupción activada.
		// Leer el valor del encoder con Timer 4 (la posicion)
		// Te devuelve un int (se debe pasar a float)
		posicion_f = (float)__HAL_TIM_GET_COUNTER(&htim4);
		// Leer el voltaje de la batería con ADC1 (CANAL 6)
		// Hay que ordenar la conversión y después esperar a que termine

		// Como hay un divisor resistivo que hay en la placa R2=10K R3=1K
		// Hay que realizar el siguiente cálculo:  VBAT= VA6·(R2+R3)/R3
		voltaje_f= ((float)valores_ADC[1])*3.3*11*100/1.0/4096;
		voltaje_f= filtro_voltaje(voltaje_f,0.8);
		// Leer el angulo con ADC1 (CANAL 3): 5 muestras
		angulo_f = obtener_angulo();
		// Obtener la referencia de posicion: local o externa
		referencia = obtener_referencia();
		// CONTROL
		if (Flag_control_motor) {
	    	if (Cuenta_un_segundo < 300) {
				Cuenta_un_segundo++;
				// Durante este tiempo está en el control agresivo
	    	}
			if (Cuenta_cinco_segundos > 0) {
				Cuenta_cinco_segundos--;
			}
	    	else if(Cuenta_un_segundo==300) {
				param_filtro_pos = 0.956;
				param_filtro_ang = 0.8;
				Ki = 163;
				K1 = 101;
				K2 = 683;
				K3 = 46;
				K4 = 21.4;		//21.4 11.4
				// Cuando pase 1 segundo, vuelve al control suave
	    		Cuenta_un_segundo ++;
	    	}
	    	switch (TIPO_CONTROL) {
			case 0: {
				PWM_motor = control_dos_PID(referencia, posicion_f, angulo_f);
				break;
			}
			case 1: {
				PWM_motor = control_realim_estados(referencia, posicion_f,
						angulo_f);
			}
	    	}
			// Apaga motor y termina el control si el péndulo cae, o baja el
			// voltaje por debajo del límite.
			if (comprueba_limites_control())
				PWM_motor = 0;
			poner_PWM_motor (PWM_motor);
	    }

	    if (Flag_swing_up) {
	    	PWM_motor =swing_up(posicion_f, angulo_f);
			if (comprueba_limites_swing())
				PWM_motor = 0;
	    	poner_PWM_motor(PWM_motor);
	    }

	    // Comprueba si se ha activado el pulsador
	    key();
		// Cuenta el numero de veces que se repite TIM1 hasta llegar a 10 (50ms)
		Cuenta_diez_periodos++;
		//if (Cuenta_diez_periodos%2==0)

		// Envía señales al ordenador
		envia_datos_serie((posicion_f - 10000) * 360 / 1040,	// ch1
		referencia,					// ch2
				(angulo_f - VERTICAL) * 180 / 2134,		// ch3
						   (PWM_motor)*100.0/7200.0,	// ch4
				acc_integral,							// ch5
				velocidad_1,					// ch6
				velocidad_2);				// ch7
	}
}

float obtener_referencia(void) {
	/**************************************************************************
	 * Obtiene el valor de la referencia
	 **************************************************************************/
	float ref;
	if (ORIGEN_REF==1)	// Local
		ref = referencia_local;
	else if (ORIGEN_REF==2)	// Externa (desde el ordenador)
		ref = referencia_externa;
	return(ref);
}
float obtener_angulo(void){
	/**************************************************************************
	 * Obtiene el valor del ángulo a partir de la media de varias conversiones
	 * seguidas del ADC.
	 **************************************************************************/
	float angulo_medio=0;
	angulo_medio = ((float) (valores_ADC[0] + valores_ADC[2] + valores_ADC[3]
			+ valores_ADC[4] + valores_ADC[5] + valores_ADC[6] + valores_ADC[7]
			+ valores_ADC[8] + valores_ADC[9] + valores_ADC[10]
			+ valores_ADC[11] + valores_ADC[12] + valores_ADC[13]
			+ valores_ADC[14] + valores_ADC[15] + valores_ADC[16]
			+ valores_ADC[18] + valores_ADC[19] + valores_ADC[20]
			+ valores_ADC[21] + valores_ADC[22] + valores_ADC[23]
			+ valores_ADC[24] + valores_ADC[25] + valores_ADC[26]
			+ valores_ADC[27] + valores_ADC[28] + valores_ADC[29]
			+ valores_ADC[30] + valores_ADC[31])) / 30.0;
	return (angulo_medio);
}

float control_dos_PID(float ref, float posicion, float angulo)
{
	/**************************************************************************
	 * Realiza el control usando dos controladores PD aditivos, uno para la
	 * posición y otro para el ángulo.
	 **************************************************************************/
	// CONTROL CON 2 CONTROLADORES PD
	float Pa_k, Da_k;
	static float Pp_k, Dp_k, Ip_k, Ip_k_1;
	static float angulo_ant, posicion_bias, posicion_bias_ant;
	float uk;

	/* Inicializacion variables */
	static uint8_t flag_inicio=0;
	if (flag_inicio==0) {
			angulo_ant=angulo;
			posicion_bias_ant=posicion_bias;
			flag_inicio=1;
	}
	static uint8_t Cuenta_cinco_periodos=0;
	uint8_t REP = 5;

	/* Escalado de las señales para convertir a grados */
	angulo = (angulo-VERTICAL)*180/2134;
	posicion = (posicion-10000)*360/1040;

	/* ECUACION EN DIFERENCIAS DEL CONTROL DEL ANGULO (cada 5ms) */
	Pa_k = KP_Bal*(0-angulo);
	Da_k = (KD_Bal/Ts)*((0-angulo)-(0-angulo_ant));
	Da_k = filtro_angulo(Da_k, p_angulo);
	angulo_ant = angulo;

	/* ECUACIÓN EN DIFERENCIAS DEL CONTROL DE POSICION (cada 25ms) */
	if (++Cuenta_cinco_periodos>(REP-1)) {
		// Filtro paso bajo de primer orden para posición
		posicion_bias = filtro_posicion(posicion, p_posicion);
		Pp_k = KP_Pos * (ref - posicion_bias);
		Dp_k = (KD_Pos / (5 * Ts)) * (-posicion_bias + posicion_bias_ant);
		Ip_k = Ip_k_1 + Ki_Pos * (5 * Ts) * (ref - posicion);
		posicion_bias_ant = posicion_bias;
		Cuenta_cinco_periodos=0;
	}
	uk =  Pa_k+Da_k+Pp_k+Dp_k+Ip_k;

	/* Saturación y antiwindup */
	if (uk>LIMITE_PWM) {
		uk=LIMITE_PWM;
		Ip_k = Ip_k_1;
	}
	else if (uk<-LIMITE_PWM){
		uk=-LIMITE_PWM;
		Ip_k = Ip_k_1;
	}

	/* Actualizo valores anteriores */
	if (Cuenta_cinco_periodos==0)
		Ip_k_1=Ip_k;

	/* Para visualizacion */
	acc_integral = Ip_k;
	velocidad_1 = Dp_k/KD_Pos;
	velocidad_2 = Da_k/KD_Bal;
	return (uk);
}

float control_realim_estados (float ref, float posicion, float angulo) {
	/**************************************************************************
	 * Función que calcula el PWM que hay que aplicar usando la metodología
	 * de control por realimentación en variables de estado con un lazo
	 * externo para seguimiento de referencias con integrador
	 **************************************************************************/
	float x1_k, x2_k, x3_k, x4_k, e1_k, xf2_k, xf1_k, uk, vk;
	static float int_k_1, xf1_k_1, xf2_k_1;
	float int_k;
	static uint8_t flag_inicio=0;
	// Inicializacion variables
	if (flag_inicio==0) {
			xf1_k_1= (posicion-10000)*360/1040;
			xf2_k_1=(angulo - VERTICAL)*180/2134;
			int_k_1=0;
			flag_inicio=1;
	}

	/* Escalado de las variables */
	posicion = (posicion - 10000) * 360 / 1040;	// Convierte a grados
	angulo = (angulo - VERTICAL) * 180 / 2134;	// Convierte a grados
	x1_k=posicion;
	x2_k=angulo;

	/* Filtro para la posición del brazo */
	xf1_k = filtro_posicion(x1_k, param_filtro_pos);
	//xf1_k = filtro_butterworth(posicion,8);

	/* filtro para el ángulo del pendulo */
	xf2_k = filtro_angulo(x2_k, param_filtro_ang);
	//xf2_k = filtro_butterworth_2(angulo,70);
	e1_k = posicion-ref;

	/* Estimación de las velocidades */
	x3_k = (xf1_k-xf1_k_1)/Ts;
	x4_k = (xf2_k-xf2_k_1)/Ts;

	/* Ley de control */
	int_k = int_k_1-(Ki*Ts)*e1_k;
	vk = int_k-K1*x1_k-K2*x2_k-K3*x3_k-K4*x4_k;

	/* Saturación y antiwindup */
	if (vk>LIMITE_PWM) {
		uk=LIMITE_PWM;
		int_k=int_k_1;
	}
	else if (vk<-LIMITE_PWM){
		uk=-LIMITE_PWM;
		int_k=int_k_1;
	}
	else
		uk=vk;

	/* Actualizamos valores anteriores en memoria */
	xf1_k_1 = xf1_k;
	xf2_k_1 = xf2_k;
	int_k_1=int_k;

	/* Variables para visualizacion por el ordenador */
	acc_integral = int_k;
	velocidad_1 = x3_k;
	velocidad_2 = x4_k;
	return (uk);
}


float filtro_roc (float yk, float roc_max) {
	/**************************************************************************
	 * Aplica un filtro de limitación de razón de cambio máxima para PWM
	 **************************************************************************/
	static float yfk_1;
	float yfk;
	static uint8_t flag_inicio;

	// Inicializa los valores anteriores la primera llamada a la función
	if (flag_inicio==0) {
		yfk_1 = 0;
		flag_inicio =1;
	}

	if ((yk-yfk_1)>roc_max*Ts)
		yfk=yfk_1+roc_max*Ts;
	else if ((yk-yfk_1)<-roc_max*Ts)
		yfk=yfk_1-roc_max*Ts;
	else yfk=yk;
	yfk_1=yfk;
	return (yfk);
}

float filtro_posicion (float yk, float a) {
	/**************************************************************************
	 * Aplica un filtro de de orden 1  para la posición
	 **************************************************************************/
	static float yfk_1;
	float yfk;
	static uint8_t flag_inicio=0;

	// Inicializa los valores anteriores la primera llamada a la función
	if (flag_inicio==0) {
		yfk_1 = yk;
		flag_inicio =1;
	}
	// a = exp(-wc*Ts)	Frecuencia de corte del filtro
	yfk = a*yfk_1 + (1-a)*yk;
	yfk_1 =yfk;
	return (yfk);
}
float filtro_angulo (float yk, float a) {
	/**************************************************************************
	 * Aplica un filtro de de orden 1 para el ángulo
	 **************************************************************************/
	static float yfk_1;
	float yfk;
	static uint8_t flag_inicio=0;
	// Inicializa los valores anteriores la primera llamada a la función
	if (flag_inicio==0) {
		yfk_1 = yk;
		flag_inicio =1;
	}
	yfk = a*yfk_1 + (1-a)*yk;
	yfk_1 =yfk;
	return (yfk);
}
float filtro_swing (float yk, float a) {
	/**************************************************************************
	 * Aplica un filtro de de orden 1 para ángulo durante swing up
	 **************************************************************************/
	static float yfk_1;
	float yfk;
	static uint8_t flag_inicio=0;
	// Inicializa los valores anteriores la primera llamada a la función
	if (flag_inicio==0) {
		yfk_1 = yk;
		flag_inicio =1;
	}


	yfk = a*yfk_1 + (1-a)*yk;
	yfk_1 =yfk;

	return (yfk);
}
float filtro_voltaje (float yk, float a) {
	/**************************************************************************
	 * Aplica un filtro de de orden 1 para el voltaje
	 **************************************************************************/
	static float yfk_1;
	float yfk;
	static uint8_t flag_inicio=0;
	// Inicializa los valores anteriores la primera llamada a la función
	if (flag_inicio==0) {
		yfk_1 = yk;
		flag_inicio =1;
	}
	yfk = a*yfk_1 + (1-a)*yk;
	yfk_1 =yfk;
	return (yfk);
}
float filtro_apaga (float yk, float a) {
	/**************************************************************************
	 * Aplica un filtro de de orden 1 para ángulo durante la comprobación de
	 * los límites del control
	 **************************************************************************/
	static float yfk_1;
	float yfk;
	static uint8_t flag_inicio=0;
	// Inicializa los valores anteriores la primera llamada a la función
	if (flag_inicio==0) {
		yfk_1 = VERTICAL;
		flag_inicio =1;
	}

	yfk = a*yfk_1 + (1-a)*yk;
	yfk_1 =yfk;
	return (yfk);
}
float filtro_media_10 (float yk) {
	/**************************************************************************
	 * Aplica un filtro de media móvil de longitud M=10
	 **************************************************************************/
	static uint8_t flag_inicio=0;
	static float yk_1, yk_2, yk_3, yk_4, yk_5, yk_6, yk_7, yk_8, yk_9;
	float yfk;
	// Inicializa los valores anteriores la primera llamada a la función
	if (flag_inicio==0) {
		yk_1 = yk;	yk_2 = yk; yk_3 = yk; yk_4 = yk; yk_5 = yk;
		yk_6 = yk; yk_7 = yk; yk_8 = yk; yk_9 = yk;
		flag_inicio =1;
	}
	yfk = (yk+yk_1+yk_2+yk_3+yk_4+yk_5+yk_6+yk_7+yk_8+yk_9)/10;
	// Actualizo valores anteriores
	yk_1=yk;
	yk_2=yk_1;
	yk_3=yk_2;
	yk_4=yk_3;
	yk_5=yk_4;
	yk_6=yk_5;
	yk_7=yk_6;
	yk_8=yk_7;
	yk_9=yk_8;
	return (yfk);
}
float filtro_media_5 (float yk) {
	/**************************************************************************
	 * Aplica un filtro de media móvil de longitud M=5
	 **************************************************************************/
	static uint8_t flag_inicio=0;
	static float yk_1, yk_2, yk_3, yk_4;
	float yfk;
	// Inicializa los valores anteriores la primera llamada a la función
	if (flag_inicio==0) {
		yk_1 = yk;	yk_2 = yk; yk_3 = yk; yk_4 = yk;
		flag_inicio =1;
	}
	yfk = (yk+yk_1+yk_2+yk_3+yk_4)/5;
	// Actualizo valores anteriores
	yk_1=yk;
	yk_2=yk_1;
	yk_3=yk_2;
	yk_4=yk_3;
	return(yfk);
}
float filtro_butterworth (float yk, float wc) {
	/**************************************************************************
	 * Aplica un filtro butterworth de orden 1 paso bajo
	 *  Yf(s) = a/(s+a) -> analogico
	 *  yf(k) (2/T+a) + (2/T+a) + a·yk()
	 **************************************************************************/
	static float yfk_1=0, yk_1=0;
	float yfk, a;
	static uint8_t flag_inicio=0;
		// Inicializa los valores anteriores la primera llamada a la función
		if (flag_inicio==0) {
			yfk_1 = yk;
			flag_inicio =1;
		}
	//a=(2/Ts)*tan(wc*Ts/2);
	a=wc;
	yfk =-((a*Ts-2)/(a*Ts+2))*yfk_1 + (a*Ts/(a*Ts+2))*(yk+yk_1);
	yfk_1 =yfk;
	yk_1 = yk;
	return (yfk);
}
float filtro_butterworth_2 (float yk, float wc) {
	/**************************************************************************
	 * Aplica un filtro butterworth de orden 1 paso bajo
	 *  Yf(s) = a/(s+a) -> analogico
	 *  yf(k) (2/T+a) + (2/T+a) + a·yk()
	 **************************************************************************/
	static float yfk_1=0, yk_1=0;
	float yfk, a;
	static uint8_t flag_inicio=0;
		// Inicializa los valores anteriores la primera llamada a la función
		if (flag_inicio==0) {
			yfk_1 = yk;
			flag_inicio =1;
		}
	//a=(2/Ts)*tan(wc*Ts/2);
	a=wc;
	yfk =-((a*Ts-2)/(a*Ts+2))*yfk_1 + (a*Ts/(a*Ts+2))*(yk+yk_1);
	yfk_1 =yfk;
	yk_1 = yk;
	return (yfk);
}
void poner_PWM_motor(float valor_pwm) {
	/**************************************************************************
	 * Cambia el valor del ciclo de trabajo de la señal PWM en función del
	 * valor de la entrada "valor_pwm"
	 **************************************************************************/

	// Señales del driver del motor:
	//	- PB12 = BIN2
	//  - PB13 = BIN1

	//static uint16_t valor_pwm_ant=0;
	uint16_t valor_pwm_abs;
	// Indica la dirección del motor
	if (valor_pwm>0) {
		valor_pwm_abs=valor_pwm;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,0);		// BIN2 = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,1);		// BIN1 = 1;
	}
	else if (valor_pwm==0) {

		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,0);		// BIN2 = 0;
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,0);		// BIN1 = 0;
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,1);		// BIN2 = ~BIN2;
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,1);		// BIN1 = ~BIN1;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		valor_pwm_abs=0;		// Es 0 y no 7199
	}
	else {
		valor_pwm_abs=-valor_pwm;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);		// BIN2 = 1;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,0);		// BIN1 = 0;
	}
	// Colocar valor de PWM en el temporizador 3
	//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,valor_pwm_abs);
	TIM3->CCR4=valor_pwm_abs;
	//valor_pwm_ant = valor_pwm;
}
uint8_t comprueba_limites_swing(void)
/**************************************************************************
 * Indica si se ha salido de los límites de la posición durante el
 * swing up. En ese caso, devuelve 1 y termina swing up
 **************************************************************************/
{
	if ((posicion_f < (10000 + LIMITE_GIRO))
			&& (posicion_f > (10000 - LIMITE_GIRO)))
		return 0;
	else  {
		Flag_swing_up=0;
		return 1;

	}
}
uint8_t comprueba_limites_control(void) {
	/**************************************************************************
	 * Indica si se ha salido de los límites de posición, angulo o voltaje.
	 * En ese caso devuelve 1 y termina el control.
	 **************************************************************************/
	uint8_t temp; //variable que indica que se ha apagado el motor
	float ang_fil;
	ang_fil = filtro_apaga(angulo_f,0.8);
	if ((Flag_control_motor == 0) || (ang_fil < (VERTICAL - LIMITE_ANGULO))
			|| (ang_fil > (VERTICAL + LIMITE_ANGULO))
			|| (voltaje_f < LIMITE_VOLTAJE))
	{
		Flag_control_motor = 0;
		// Termina el control si cualquiera de las causas de antes se produjo
		temp=1;
	}
	else
		temp=0;
	return temp;
}


int click_N_Double(uint8_t time) {
	/**************************************************************************
	 * Determina si la patilla PA2 se ha pulsado 1 vez o dos veces.
	 * - Si se ha pulsado 1 vez, devuelve 1.
	 * - Si se ha pulsado 2 veces, devuelve 2.
	 * - Si no se ha pulsado, devuevle 0.
	 **************************************************************************/
	static uint8_t flag_key, count_key, double_key;
	static uint16_t count_single, Forever_count;
	uint8_t KEY2;
	KEY2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);	// Lee el valor de  PA2
	if (KEY2==0)				// La activación del botón es a nivel bajo 0
		Forever_count++;		// Cuenta el tiempo (se incrementa cada 5ms)
	else
		Forever_count=0;		// Se ha soltado el botón

	if ((0==KEY2)&&(0==flag_key))	flag_key=1;
	if (0==count_key) {
		if (flag_key==1) {
			double_key++;
			count_key=1;
		}
		if (double_key==2) {	// Se ha pulsado 2 veces el botón
			double_key=0;
			count_single=0;
			return 2;
		}
	}
	if (1==KEY2)	flag_key=0, count_key=0;	// Desactivación
	if (1==double_key)	{
		count_single++;
		if ((count_single>time) && (Forever_count<time))
		{
			double_key=0;
			count_single=0;
			return 1;			// Se ha pulsado 1 vez el botón
		}
		if (Forever_count>time)
		{
			double_key=0;
			count_single=0;
		}
	}
	return 0;
}

void key(void) {
	/**************************************************************************
	 * Comprueba la activación de PA2 y en ese caso aplica un cambio en la
	 * referencia de posición del péndulo, que puede ser:
	 * - Rampa
	 * - Escalón
	 **************************************************************************/
	float Position=2*360;	// Se puede poner cte

	//Target position The original position of the motor is 10000.
	// One turn is 1040. It is related to the accuracy of the encoder.
	//The default is that the swing lever Z-turn is one turn and the output is
	// 1040 jump edges.
	static int tmp, flag;
	static float count;
	// Determina si se ha pulsado 1 o 2 veces el botón (tmp=1 o tmp=2)
	tmp = click_N_Double (100);

	if (tmp==1) flag=1; //++
	if (tmp==2) flag=2; //--

	//  Cambio referencia escalón
	if (Tipo_ref==0) {
		if (flag==1)
		{
			referencia_local = referencia_local+90;
			flag=0;
		}
		if (flag==2)
		{
			referencia_local = referencia_local-90;
			flag=0;
		}
	}
	//  Cambio referencia rampa
	else {
		if (flag == 1)
		// El pendulo se mueve en las agujas del reloj
		{
			referencia_local = referencia_local + 1 * 360.0 / 1040;
			// Incrementa 1/1040 grados cada 5ms
			count=count+1*360.0/1040;
			if(count>Position) 	flag=0,count=0;
		}
		if (flag == 2)
		// El pendulo se mueve en sentido en las agujas del reloj
		{
			referencia_local=referencia_local-1*360.0/1040;
			count=count+1*360.0/1040;
			if (count>Position)	flag=0, count=0;
		}
	}
}



void oled_show(void) {
	/**************************************************************************
	 * Actualiza o refresca el contenido de la pantalla, en función del menu
	 * que se haya seleccionado.
	 **************************************************************************/
	OLED_ShowString(00,00,(uint8_t*)("PENDULO CIRCULAR"));
	switch (Menu) {
	case 1: {
		OLED_ShowString(0,20,(uint8_t*)("MENU:  SWING UP"));
		OLED_ShowString(0, 40, (uint8_t*) ("Prim swing="));
		OLED_ShowNumber(80, 40, SWING_PRIMERO, 3, 12);
		OLED_ShowChar(100, 40, '-', 12, 1);
		break;
	}
	case 2: {
		OLED_ShowString(0,20,(uint8_t*)("MENU:  CONTROL "));
		OLED_ShowString(0, 40, (uint8_t*) ("K1= "));
		OLED_ShowNumber(30, 40, K1, 3, 12);
		OLED_ShowChar(50, 40, '-', 12, 1);
		OLED_ShowString(0, 50, (uint8_t*) ("K2= "));
		OLED_ShowNumber(30, 50, K2, 3, 12);
		OLED_ShowString(70, 40, (uint8_t*) ("K3= "));
		OLED_ShowNumber(100, 40, K3, 3, 12);
		OLED_ShowString(70, 50, (uint8_t*) ("K4= "));
		OLED_ShowNumber(100, 50, K4, 3, 12);
		break;
	}
	case 3: {
		OLED_ShowString(0,20,(uint8_t*)("MENU:  CONTROL "));
		OLED_ShowString(0, 40, (uint8_t*) ("K1= "));
		OLED_ShowNumber(30, 40, K1, 3, 12);
		OLED_ShowString(0, 50, (uint8_t*) ("K2= "));
		OLED_ShowNumber(30, 50, K2, 3, 12);
		OLED_ShowChar(50, 50, '-', 12, 1);
		OLED_ShowString(70, 40, (uint8_t*) ("K3= "));
		OLED_ShowNumber(100, 40, K3, 3, 12);
		OLED_ShowString(70, 50, (uint8_t*) ("K4= "));
		OLED_ShowNumber(100, 50, K4, 3, 12);
		break;
	}
	case 4: {
		OLED_ShowString(0,20,(uint8_t*)("MENU:  CONTROL "));
		OLED_ShowString(0, 40, (uint8_t*) ("K1= "));
		OLED_ShowNumber(30, 40, K1, 3, 12);
		OLED_ShowString(0, 50, (uint8_t*) ("K2= "));
		OLED_ShowNumber(30, 50, K2, 3, 12);
		OLED_ShowString(70, 40, (uint8_t*) ("K3= "));
		OLED_ShowNumber(100, 40, K3, 3, 12);
		OLED_ShowChar(120, 40, '-', 12, 1);
		OLED_ShowString(70, 50, (uint8_t*) ("K4= "));
		OLED_ShowNumber(100, 50, K4, 3, 12);
		break;
	}
	case 5: {
		OLED_ShowString(0,20,(uint8_t*)("MENU:  CONTROL "));
		OLED_ShowString(0, 40, (uint8_t*) ("K1= "));
		OLED_ShowNumber(30, 40, K1, 3, 12);
		OLED_ShowString(0, 50, (uint8_t*) ("K2= "));
		OLED_ShowNumber(30, 50, K2, 3, 12);
		OLED_ShowString(70, 40, (uint8_t*) ("K3= "));
		OLED_ShowNumber(100, 40, K3, 3, 12);
		OLED_ShowString(70, 50, (uint8_t*) ("K4= "));
		OLED_ShowNumber(100, 50, K4, 3, 12);
		OLED_ShowChar(120, 50, '-', 12, 1);
		break;
	}
	case 6: {
			OLED_ShowString(0,20,(uint8_t*)("MENU:  CONTROL "));
		OLED_ShowString(0, 40, (uint8_t*) ("Ki= "));
		OLED_ShowNumber(30, 40, Ki, 3, 12);
		OLED_ShowChar(50, 40, '-', 12, 1);
			break;
		}
	case 7: {
		OLED_ShowString(0,20,(uint8_t*)("MENU: MEDIDAS 1"));
		OLED_ShowString(0, 40, (uint8_t*) ("Ang= "));
		OLED_ShowNumber(30, 40, (int32_t) angulo_f, 4, 12);
		OLED_ShowString(0, 50, (uint8_t*) ("Pos= "));
		OLED_ShowNumber(30, 50, (int32_t) posicion_f, 5, 12);
		OLED_ShowString(70, 40, (uint8_t*) ("Vol= "));
		OLED_ShowNumber(70, 50, (int32_t) voltaje_f / 100, 2, 12);
		OLED_ShowString(83,50,(uint8_t*)".");
		OLED_ShowNumber(93,50,((int)voltaje_f)%100,2,12);	// Parte decimal
		if(((int)voltaje_f)%100<10) 	OLED_ShowNumber(87,40,0,2,12);
		break;
	}
	case 8: {
		OLED_ShowString(0,20,(uint8_t*)("MENU: MEDIDAS 2"));
		if (velocidad_2>=0) {
			OLED_ShowString(0, 40, (uint8_t*) ("dA= "));
			OLED_ShowNumber(30, 40, (int32_t) (+velocidad_2), 5, 12);
			OLED_ShowString(25,40,(uint8_t*)"+");
		}
		else {
			OLED_ShowString(0, 40, (uint8_t*) ("dA= "));
			OLED_ShowNumber(30, 40, (int32_t) (-velocidad_2), 5, 12);
			OLED_ShowString(25,40,(uint8_t*)"-");
		}
		if (acc_integral>=0) {
			OLED_ShowString(70, 40, (uint8_t*) ("in= "));
			OLED_ShowNumber(95, 40, (int32_t) (+acc_integral), 5, 12);
			OLED_ShowString(95,40,(uint8_t*)"+");
		}
		else {
			OLED_ShowString(70, 40, (uint8_t*) ("in= "));
			OLED_ShowNumber(95, 40, (int32_t) (-acc_integral), 5, 12);
			OLED_ShowString(95,40,(uint8_t*)"-");
		}
		if (velocidad_1>=0) {
			OLED_ShowString(0, 50, (uint8_t*) ("dP= "));
			OLED_ShowNumber(30, 50, (int32_t) (+velocidad_1), 5, 12);
			OLED_ShowString(25,50,(uint8_t*)"+");
		}
		else {
			OLED_ShowString(0, 50, (uint8_t*) ("dP= "));
			OLED_ShowNumber(30, 50, (int32_t) (-velocidad_1), 5, 12);
			OLED_ShowString(25,50,(uint8_t*)"-");
		}
		break;
	}
	case 9: {
		OLED_ShowString(0,20,(uint8_t*)("MODO: REFERENCIA"));
		if (Tipo_ref==0) OLED_ShowString(0,30,(uint8_t*)("Tipo: Escalon"));
		else OLED_ShowString(0,30,(uint8_t*)("Tipo:   Rampa"));
		OLED_ShowChar(120,30,'-',12,1);
		if (ORIGEN_REF==1) OLED_ShowString(0,40,(uint8_t*)("Fuente: Local"));
		else OLED_ShowString(0,40,(uint8_t*)("Fuente: Exter"));
		OLED_ShowString(0, 50, (uint8_t*) ("P= "));
		OLED_ShowNumber(25, 50, (int32_t) posicion_f, 5, 12);
		OLED_ShowString(65, 50, (uint8_t*) ("R= "));
		OLED_ShowNumber(90, 50, (int32_t) referencia, 5, 12);
		break;
	}
	case 10: {
		OLED_ShowString(0,20,(uint8_t*)("MODO: REFERENCIA"));
		if (Tipo_ref==0) OLED_ShowString(0,30,(uint8_t*)("Tipo: Escalon"));
		else OLED_ShowString(0,30,(uint8_t*)("Tipo:   Rampa"));
		if (ORIGEN_REF==1) OLED_ShowString(0,40,(uint8_t*)("Fuente: Local"));
		else  OLED_ShowString(0,40,(uint8_t*)("Fuente: Exter"));
		OLED_ShowChar(120,40,'-',12,1);
		OLED_ShowString(0, 50, (uint8_t*) ("P= "));
		OLED_ShowNumber(25, 50, (int32_t) posicion_f, 5, 12);
		OLED_ShowString(65, 50, (uint8_t*) ("R= "));
		OLED_ShowNumber(90, 50, (int32_t) referencia, 5, 12);
		break;
	}
	}
	OLED_Refresh_Gram();
}


// INTERRUPCION POR PATILLA EXTERNA (FLANCO CAIDA)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	/**************************************************************************
	 * Interrupción por activación de una patilla externa.
	 * Es común para todas las entradas, por lo que hay que distinguir cuál
	 * es la que se ha pulsado.
	 **************************************************************************/
	HAL_Delay(5);	// Espera 5ms a que acaben rebotes mecánicos.
	if ((GPIO_Pin==GPIO_PIN_5)||(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)==0))
	// Interrupcion por flanco bajada PA5 (También entran aqui por PB5 y PC5)
	// Activación/desactivación del swing-up o control automático
	{
		if (Flag_control_motor==0 && Flag_swing_up==0) {	// Si está apagado
			if (SWING_PRIMERO) {
				Flag_swing_up=1;
				referencia_local = 0;
				__HAL_TIM_SET_COUNTER (&htim4,10000);
			}
			else {
				Flag_control_motor=1;
				referencia_local = 0;
				__HAL_TIM_SET_COUNTER(&htim4, 10000);
				// Reinica a 10000 cuenta encoder
			}
		}
	}
	if ((GPIO_Pin==GPIO_PIN_7)||(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==0))
	// Interrupcion por flanco bajada PA7
	{
		if (Menu==10) {	// Cambia el menu de la pantalla
			Menu=1;
		}
		else Menu++;
	}
	if ((GPIO_Pin==GPIO_PIN_11)||(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)==0))
	// Interrupcion por flanco bajada PA1
	{
		if (Menu==1)		SWING_PRIMERO =!SWING_PRIMERO;
		else if (Menu==2)		K1-=Amplitude1;	// Reducir valores PID
		else if (Menu==3)	K2-=Amplitude2;
		else if (Menu==4)	K3-=Amplitude3;
		else if (Menu==5)	K4-=Amplitude4;
		else if (Menu==6)	Ki-=1;
		else if (Menu==9)	Tipo_ref = !Tipo_ref;
		else if (Menu==10)	ORIGEN_REF = !ORIGEN_REF;
	}
	if ((GPIO_Pin==GPIO_PIN_12)||(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)==0))
	// Interrupcion por flanco bajada PA12
	{
		if (Menu==1)		SWING_PRIMERO =!SWING_PRIMERO;
		else if (Menu==2)	K1+=Amplitude1;	// Aumentar valores PID
		else if (Menu==3)	K2+=Amplitude2;
		else if (Menu==4)	K3+=Amplitude3;
		else if (Menu==5)	K4+=Amplitude4;
		else if (Menu==6)	Ki+=1;
		else if (Menu==9)	Tipo_ref = !Tipo_ref;
		else if (Menu==10)	ORIGEN_REF = !ORIGEN_REF;
	}
	if (K1<=0) KP_Bal=0;
	if (K2<=0) KD_Bal=0;
	if (K3<=0) KP_Pos=0;
	if (K4<=0) KD_Pos=0;
}

void envia_datos_serie(float dato1, float dato2, float dato3, float dato4,
		float dato5, float dato6, float dato7) {
	/**************************************************************************
	 * Envia señales de los sensores, referencia y PWM por el puerto serie
	 * para visualizarlos en el ordenador
	 **************************************************************************/
	// Envia datos al ordenador
	// Convierte los datos tipo float en 4 byes, que envía por
	// el puerto serie.
		  txBuffer [0] = '$';
		  txBuffer [1] = *((uint8_t*)(&dato1));
		  txBuffer [2] = *((uint8_t*)(&dato1)+1);
		  txBuffer [3] = *((uint8_t*)(&dato1)+2);
		  txBuffer [4] = *((uint8_t*)(&dato1)+3);
		  txBuffer [5] = *((uint8_t*)(&dato2));
		  txBuffer [6] = *((uint8_t*)(&dato2)+1);
		  txBuffer [7] = *((uint8_t*)(&dato2)+2);
		  txBuffer [8] = *((uint8_t*)(&dato2)+3);
		  txBuffer [9] =  *((uint8_t*)(&dato3));
		  txBuffer [10] = *((uint8_t*)(&dato3)+1);
		  txBuffer [11] = *((uint8_t*)(&dato3)+2);
		  txBuffer [12] = *((uint8_t*)(&dato3)+3);
		  txBuffer [13] = *((uint8_t*)(&dato4));
		  txBuffer [14] = *((uint8_t*)(&dato4)+1);
		  txBuffer [15] = *((uint8_t*)(&dato4)+2);
		  txBuffer [16] = *((uint8_t*)(&dato4)+3);
		  txBuffer [17] = *((uint8_t*)(&dato5));
		  txBuffer [18] = *((uint8_t*)(&dato5)+1);
		  txBuffer [19] = *((uint8_t*)(&dato5)+2);
		  txBuffer [20] = *((uint8_t*)(&dato5)+3);
		  txBuffer [21] = *((uint8_t*)(&dato6));
		  txBuffer [22] = *((uint8_t*)(&dato6)+1);
		  txBuffer [23] = *((uint8_t*)(&dato6)+2);
		  txBuffer [24] = *((uint8_t*)(&dato6)+3);
		  txBuffer [25] = *((uint8_t*)(&dato7));
		  txBuffer [26] = *((uint8_t*)(&dato7)+1);
		  txBuffer [27] = *((uint8_t*)(&dato7)+2);
		  txBuffer [28] = *((uint8_t*)(&dato7)+3);
	//HAL_UART_Transmit(&huart1, txBuffer, 29,1);	// Sin interrupciones
	HAL_UART_Transmit_DMA(&huart1, txBuffer, 29);	// Con interrupciones
}


float swing_up(float posicion, float angulo) {
	/**************************************************************************
	 * Función que realiza el procedimiento de levantamiento o swing up
	 **************************************************************************/
#define dist_1  -200
	static uint16_t etapa=0;
	static int PWM;


	switch (etapa) {
	case 0: {
		PWM = 5300;	// DERECHA
		if (((10000 + dist_1 - 100) < posicion)
				&& (posicion < (10000 + dist_1))) { 
			etapa=1;
			PWM = 0;
		}
		break;
	}
	case 1:
	{
		PWM = -4800; // IZQUIERDA
		if(((VERTICAL-1000)<angulo)&&(angulo<(VERTICAL))){
		    PWM=3800; // DERECHA
		    etapa=2;
	    }
		break;
	}
	case 2:
	{
		if (((VERTICAL-400)<angulo)&&(angulo<VERTICAL))
			PWM=3000;
		if (((VERTICAL-100)<angulo)&&(angulo<(VERTICAL+100))){
			Flag_control_motor=1;
			Flag_swing_up=0;
			PWM=0;
			// CAMBIO LA REFERENCIA AL LUGAR DONDE SE HA CONSEGUIDO EL SWING UP
			referencia_local=(10000-10000)*360/1040;
			//referencia_local=(posicion_f-10000)*360/1040;
			referencia_externa=(10000-10000)*360/1040;
			//referencia_externa=(posicion_f-10000)*360/1040;
			// Reinicia  10000 cuenta encoder
			__HAL_TIM_SET_COUNTER(&htim4, 10000); 
			
			etapa =0;
		}
		break;
	}
	}

	return (filtro_roc(PWM,120000));
}

// INTERRUPCIONES POR RECEPCIÓN O TRANSMISIÓN POR EL PUERTO SERIE
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart) {
	/**************************************************************************
	 * Función asociada a una recepción de una trama de datos completa
	 **************************************************************************/
	uint8_t inicio=4;
	for (uint8_t i=0; i<4; i++) {
		if(rxBuffer[i]=='A')
			inicio = i;
	}
	if(inicio<3)
		referencia_externa =  *((int16_t*)(rxBuffer+inicio+1));
	else if (inicio==3)
		referencia_externa =  *((int16_t*)(rxBuffer));
	else
		referencia_externa = referencia; // No altera valor
}
void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart) {
	/**************************************************************************
	 * Función asociada al envío de una trama de datos completa
	 **************************************************************************/
	
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	static uint8_t Menu_ant=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Primero lee el valor del ángulo en la posición inferior.
  // A partir de este, determina el ángulo de la vertical.
  // Hay que activar el ADC:
  HAL_ADC_Start_DMA (&hadc1,valores_ADC,32);
  HAL_Delay(10);
  INFERIOR = valores_ADC[0];
  VERTICAL =INFERIOR+2134;

  // Configura el temporizador que lee el valor del encoder
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
	// Comienza contaje encoder por  TEMP4
	__HAL_TIM_SET_COUNTER(&htim4, 10000);
	// Empieza en 10000 cuenta encoder
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_ALL,TIM_INPUTCHANNELPOLARITY_FALLING);
  HAL_TIM_Base_Start_IT(&htim1);

  // Comienza generación señal PWM por TEMP3
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
  __HAL_TIM_SET_PRESCALER(&htim3, 0);


  // Inicializacion pantalla OLED
  OLED_Init();

  HAL_UART_Receive_DMA(&huart1, rxBuffer, 4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Comprueba si ha cambiado el menu
	  if (Menu!=Menu_ant) {
		  // Limpia la pantalla para el siguiente menu
		  OLED_Clear();
		  Menu_ant = Menu;
	  }
	  // Actualiza pantalla
	  oled_show();

      // Cuenta el numero de veces que se repite TIM1 hasta llegar a 10 (50ms)
      while (Cuenta_diez_periodos < 10);
      Cuenta_diez_periodos = 0;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 16;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_16;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 128000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BIN2_Pin|BIN1_Pin|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA5 PA7 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN2_Pin BIN1_Pin PB3 PB4
                           PB5 */
  GPIO_InitStruct.Pin = BIN2_Pin|BIN1_Pin|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
