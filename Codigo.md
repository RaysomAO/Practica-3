#include "main.h"

volatile uint8_t SSD_CODE[4];
volatile uint32_t Transistores [4];
ETH_HandleTypeDef heth;
UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_OTG_FS;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  IO_Setup();
  Tim6_Setup();
  int time = 0000, PushTime = 0;
  int mode =0, count=0, delay = 1750;

while (1){

		if(GPIOE->IDR == (0<<0) && mode==2){// conteo en pantalla antes de stop
			LedsUp(0,(HAL_GetTick() - time)%10,'-',0);
			LedsUp(0,((HAL_GetTick() - time)/10)%10,'-',1);
			LedsUp(0,((HAL_GetTick() - time)/100)%10,'-',2);
			LedsUp(0,((HAL_GetTick() - time)/1000)%10,'-',3);
		}
		if(GPIOE->IDR != (1<<0) && mode == 0){// waiting for start y esta en HI MODE
			LedsUp(1,time%10,'-',0);
			LedsUp(1,(time/10)%10,'I',1);
			LedsUp(1,(time/100)%10,'H',2);
			LedsUp(1,(time/1000)%10,'-',3);
		}
		if(GPIOE->IDR == (1<<0) && mode == 0){// press start y esta en READY MODE
			//start
			mode = 1;
			count = HAL_GetTick();
			LedsUp(1,time%10,'-',0);
			LedsUp(1,(time/10)%10,'-',1);
			LedsUp(1,(time/100)%10,'-',2);
			LedsUp(1,(time/1000)%10,'-',3);

		}else if(GPIOE->IDR == (1<<4) && mode == 2){// press stop
			time = HAL_GetTick() - time;
			GPIOC->ODR &= ~(1<<0);
			mode = 4;
			count = 0;
			LedsUp(0,time%10,'A',0);
			LedsUp(0,(time/10)%10,'A',1);
			LedsUp(0,(time/100)%10,'A',2);
			LedsUp(0,(time/1000)%10,'A',3);

		}else if(GPIOE->IDR == (1<<2)){//press reset no hay condicion
			//reset
			mode = 0;
			time = 0;
			count = 0;
			GPIOC->ODR &= ~(1<<0);
		}
		if(mode == 1){
			if(GPIOE->IDR == (1<<4)){//mecanismo para tramposos que presionen stop antes de tiempo
				mode = 3;
				LedsUp(1,time%10,'T',0);
				LedsUp(1,(time/10)%10,'E',1);
				LedsUp(1,(time/100)%10,'H',2);
				LedsUp(1,(time/1000)%10,'C',3);
				GPIOC->ODR &= ~(1<<0);
			}//si no se activa el mecanismo de tramposos, empieza a contar y esperar un stop
			if(count + delay < HAL_GetTick()){
				GPIOC->ODR |= (1<<0);
				mode = 2;
				time = HAL_GetTick();
			}
		}
		if(mode == 2 && HAL_GetTick() - time > 2000){// si pasamos de 2 segundos con la pantalla contando
			//pedimos reset
			mode = 3;
			LedsUp(1,time%10,'T',0);
			LedsUp(1,(time/10)%10,'S',1);
			LedsUp(1,(time/100)%10,'E',2);
			LedsUp(1,(time/1000)%10,'R',3);
			GPIOC->ODR &= ~(1<<0);
		}
	 }
 }

void TIM6_DAC_IRQHandler(){
		static int SSD_DIS_COUNT = 0;//variable estatica para que no se pierda el conteo de transistores
		GPIOC->ODR &= ~(1<<3);//apagamos todos los transistores
		GPIOE->ODR &= ~(255<<8);//apagamos todos los transistores
		GPIOF->ODR &= ~(1064<<0);//apagamos todos los transistores
		GPIOE->ODR |= (SSD_CODE[SSD_DIS_COUNT]<<8);// pasamos valores de variable global al registro de los pines
		//que se conectan a los LEDS
		if(SSD_DIS_COUNT == 0){//prendemos transistores dependiendo la variable estatica
			GPIOC->ODR |= (1<<3);//transitor 0
		}else if(SSD_DIS_COUNT == 1){
			GPIOF->ODR |= (1<<3);//transitor 1
		}else if(SSD_DIS_COUNT == 2){
			GPIOF->ODR |= (1<<5);//transitor 2
		}else if(SSD_DIS_COUNT == 3){
			GPIOF->ODR |= (1<<10);//transitor3
		}

		if(SSD_DIS_COUNT > 3){
			SSD_DIS_COUNT = 0;//aumentamos conteo para que pasemos al siguiente transistor
		}else{
			SSD_DIS_COUNT++;//si estamos en el transistor 3 debemos reiniciar
	}
	TIM6->SR =0;
}
//PIN PE0 PUSB STRT
//PIN PE2 PUSB RSET
//PIN PE4 PUSB STP
//PIN PC0 LED DE REACCION
//PIN PC3 TRANS 0
//PIN PF3 TRANS 1
//PIN PF5 TRANS 2
//PIN PF10 TRANS 3
//PIN PE8 DIS A
//PIN PE9 DIS B
//PIN PE10 DIS C
//PIN PE11 DIS D
//PIN PE12 DIS E
//PIN PE13 DIS F
//PIN PE14 DIS G
//PIN PE15 DIS H

void IO_Setup(){
	RCC->AHB1ENR |= (1<<1);//PORTS B
	RCC->AHB1ENR |= (1<<2);//PORTS C
	RCC->AHB1ENR |= (1<<3);//PORTS D
	RCC->AHB1ENR |= (1<<4);//PORTS E
	RCC->AHB1ENR |= (1<<5);//PORTS F
	//PIN PD7 PE0 start
	GPIOE->MODER &= ~(3<<0);
	GPIOE->OSPEEDR |= (3<<0);
	GPIOE->PUPDR &= ~(3<<0);
	//PIN PD6 PE2 reset
	GPIOE->MODER &= ~(3<<4);
	GPIOE->OSPEEDR |= (3<<4);
	GPIOE->PUPDR &= ~(3<<4);
	//PIN PD5 PE4 stop
	GPIOE->MODER &= ~(3<<8);
	GPIOE->OSPEEDR |= (3<<8);
	GPIOE->PUPDR &= ~(3<<8);
	//PIN PC0 LED DE REACCION
	GPIOC->MODER |=(1<<0);
	GPIOC->MODER &= (~(1<<1));
	GPIOC->OTYPER &= (~(1<<0));
	GPIOC->OSPEEDR &= (3<<0);
	//PIN PC3 TRANS 0
	GPIOC->MODER |=(1<<6);
	GPIOC->MODER &= (~(1<<7));
	GPIOC->OTYPER &= (~(1<<3));
	GPIOC->OSPEEDR &= (3<<6);
	//PIN PF3 TRANS 1
	GPIOF->MODER |=(1<<6);
	GPIOF->MODER &= (~(1<<7));
	GPIOF->OTYPER &= (~(1<<3));
	GPIOF->OSPEEDR &= (3<<6);
	//PIN PF5 TRANS 2
	GPIOF->MODER |=(1<<10);
	GPIOF->MODER &= (~(1<<11));
	GPIOF->OTYPER &= (~(1<<5));
	GPIOF->OSPEEDR &= (3<<10);
	//PIN PF10 TRANS 3
	GPIOF->MODER |=(1<<20);
	GPIOF->MODER &= (~(1<<21));
	GPIOF->OTYPER &= (~(1<<10));
	GPIOF->OSPEEDR &= (3<<20);
	//PIN PE8 DIS A
	GPIOE->MODER |=(1<<16);
	GPIOE->MODER &= (~(1<<17));
	GPIOE->OTYPER &= (~(1<<8));
	GPIOE->OSPEEDR &= (3<<16);
	//PIN PE9 DIS B
	GPIOE->MODER |=(1<<18);
	GPIOE->MODER &= (~(1<<19));
	GPIOE->OTYPER &= (~(1<<8));
	GPIOE->OSPEEDR &= (3<<18);
	//PIN PE10 DIS C
	GPIOE->MODER |=(1<<20);
	GPIOE->MODER &= (~(1<<21));
	GPIOE->OTYPER &= (~(1<<10));
	GPIOE->OSPEEDR &= (3<<20);
	//PIN PE11 DIS D
	GPIOE->MODER |=(1<<22);
	GPIOE->MODER &= (~(1<<23));
	GPIOE->OTYPER &= (~(1<<11));
	GPIOE->OSPEEDR &= (3<<22);
	//PIN PE12 DIS E
	GPIOE->MODER |=(1<<24);
	GPIOE->MODER &= (~(1<<25));
	GPIOE->OTYPER &= (~(1<<12));
	GPIOE->OSPEEDR &= (3<<24);
	//PIN PE13 DIS F
	GPIOE->MODER |=(1<<26);
	GPIOE->MODER &= (~(1<<27));
	GPIOE->OTYPER &= (~(1<<13));
	GPIOE->OSPEEDR &= (3<<26);
	//PIN PE14 DIS G
	GPIOE->MODER |=(1<<28);
	GPIOE->MODER &= (~(1<<29));
	GPIOE->OTYPER &= (~(1<<14));
	GPIOE->OSPEEDR &= (3<<28);
	//PIN PE15 DIS H
	GPIOE->MODER |=(1<<30);
	GPIOE->MODER &= (~(1<<31));
	GPIOE->OTYPER &= (~(1<<15));
	GPIOE->OSPEEDR &= (3<<30);
}


void Tim6_Setup(){
	RCC->APB1ENR |= (1<<4);//activamos TIM6
	TIM6->PSC = 600-1;//indicamos valor de prescalador
	TIM6->ARR = 60-1;//indicamos valor autorecargar
	TIM6->DIER |= 1; // activamos que tenga interrupciones 
	TIM6->CR1 |= 1; //
	NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 1));//creamos prioridad
	//de interrupcion
	NVIC_EnableIRQ(TIM6_DAC_IRQn);//assigamos la prioridad creada
}


void LedsUp(int mode, int time, char letra, int posicion){//funcion para actualizar valores en variable global
	//recibe mode 0 es para presentar numeros 1 es letras, time es digito a presentar en display
	//letra es la letra a presentar en display, posicion es para saber si lleva un punto decimal
	if(mode == 0){// LEDS MUESTRAN NUMEROS
		if(time == 0){
			//ports must be = 0 0111111
			SSD_CODE[posicion] = 63;
		}else if(time == 1){
			//ports must be = 1 0000110 6
				SSD_CODE[posicion] = 6;
		}else if(time == 2){
			//ports must be = 2 1011011
			SSD_CODE[posicion] = 91;
		}else if(time == 3){
			//ports must be = 3 1001111
			SSD_CODE[posicion] = 79;
		}else if(time == 4){
			//ports must be = 4 1100110
			SSD_CODE[posicion] = 102;
		}else if(time == 5){
			//ports must be = 5 1101101
			SSD_CODE[posicion] = 109;
		}else if(time == 6){
			//ports must be = 6 1111101
			SSD_CODE[posicion] = 125;
		}else if(time == 7){
			//ports must be = 7 0000111
			SSD_CODE[posicion] = 7;
		}else if(time == 8){
			//ports must be = 8 1111111
			SSD_CODE[posicion] = 127;
		}else if(time == 9){
			//ports must be = 9 1101111
			SSD_CODE[posicion] = 111;
		}
		if(posicion == 3){//si es el primer digito encendemos el valor del punto
			SSD_CODE[posicion] |= 128;
		}
	}else if(mode == 1){// LEDS MUESTRAN LETRAS
		if(letra == 'H')
			SSD_CODE[posicion] = 118; //ports must be = H 1110110

		else if (letra == 'I')
			SSD_CODE[posicion] = 48; //ports must be = I 0110000

		else if (letra == 'C')
			SSD_CODE[posicion]= 57; //ports must be = C 0111001

		else if (letra == 'E')
			SSD_CODE[posicion] = 121; //ports must be = E 1111001

		else if (letra == 'T')
			SSD_CODE[posicion] = 120; //ports must be = t 1111000

		else if (letra == 'R')
			SSD_CODE[posicion] = 80; //ports must be = r 1010000

		else if (letra == 'D')
			SSD_CODE[posicion] = 94; //ports must be = d 1011110

		else if (letra == 'Y')
			SSD_CODE[posicion] = 110; //ports must be = y 1101110

		else if (letra == 'S')
			SSD_CODE[posicion] = 109; //ports must be = S 1101101 = 5

		else if (letra == 'T')
			SSD_CODE[posicion] = 120; //ports must be = t 1111000

		else if (letra == 'G')
			SSD_CODE[posicion] = 111; //ports must be = G 0111101

		else if (letra == 'O')
			SSD_CODE[posicion] = 63; //ports must be = O 0111111

		else if (letra == '-')
			SSD_CODE[posicion] = 0; //ports must be off 0 = ' ' = 0000000
	}
}
