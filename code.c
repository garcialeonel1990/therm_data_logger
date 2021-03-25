//---------------------------------------------------------------------------------------//
//----------- Declaracion de includes 

#include <hidef.h> /* for EnableInterrupts macro */
#include "derivative.h" /* include peripheral declarations */

#include "lcd_char.h"
#include "stdio.h"
#include "string.h"

//---------------------------------------------------------------------------------------//
//----------- Declaracion de definiciones 
		
#define LED_Y		PTCD_PTCD2			// pin= 42

#define LED_ON				0
#define LED_OFF				1

#define BOTON_S1        	PTGD_PTGD0			// pin=	21	Min			KBI0
#define BOTON_S2       		PTGD_PTGD1			// pin= 22  MAX			KBI1

#define BOTON_MENU       	PTDD_PTDD2			// pin= 33 Si/Up		KBI2
#define BOTON_UP       		PTGD_PTGD2			// pin= 34 Registro		KBI6
#define BOTON_DOWN        	PTGD_PTGD3			// pin= 35 No/Down		KBI7

#define BOTON_T        		PTFD_PTGF0			// pin= 4 para saber si hay termocupla

#define ADC_LM35        	0x45				// pin= 28

#define ADC_CANAL8        	0x48				// pin= 29



//---------------------------------------------------------------------------------------//
//--------------------------------- Declaracion de variable------------------------------//

unsigned char TRIM_32KHz @0xFFAF;   // oscilador interno.

//----- Medicion termopares ------------

float temp_act[8]={0,0,0,0,0,0,0,0};	// vector temperatura actual

float temp_max[8]={0,0,0,0,0,0,0,0};	// vector temperatura maxima

float temp_min[8]={350,350,350,350,350,350,350,350};	// vector temperatura minima

float adc_ch8;			// variable donde guardo el valor del adc en .flotante

float adc_lm35;			// variable donde guardo el valor del adc en .flotante

float temp_ambiente;

signed int termo=0;		// variable que identifica cada termopar

unsigned int mseg=0; 	// variable delay

 unsigned char f_enviar_tempmax= 0;
 
 unsigned char f_enviar_tempmin= 0;
 
 unsigned char f_enviar_tempambiente= 0;
 
//---------------------------------------


//------------ variables Menu -----------

		
unsigned int flag=0, f_menu=0, flag_set_point=0, f=0, serie=0; // variables banderas menu de opciones

int contador=0,t_registro=0, disp_intervalo=0, disp_t_registro=0;		// variables seleccion menu.

unsigned char f_boton_up, f_boton_menu, f_boton_down;	// botones seleccion menu.

long intervalo=0,timer_intervalo=0;

//---------------------------------------

unsigned char f_boton_s1=0, f_boton_s2=0, buffer[30];	// botones temp Max y Min, buffer display

unsigned char f_mostrar_temp_ambiente =0;

//---------------- RS-232------------------

unsigned char  timeout_rx2, f_rx_sci2, f_sci2,b_sci2[30],output[80];

int v=1;

//---------------------------------------------------------------------------------------//


//------------------------------ Prototipos funciones -----------------------------------//


void inicio_puertos(void);
void inicio_cop_lvi_irq(void);
void inicio_rtc(void);
void inicio_kbi(void);
void delay_mseg(unsigned int ms);
void inicio_adc(void);

void inicio_osc(void);
void inicio_sci2(unsigned int br);
void sci2_byte(unsigned char sci_tx);
void sci2_string(unsigned char *dato);

void registro_temp (void);

float medir_temp (void);
void mostrar_temp(void);
void mostrar_temp_max(void);
void mostrar_temp_min(void);
void mostrar_temp_ambiente(void);

float medir_temp_max (void);
float medir_temp_min (void);
float medir_temp_ambiente(void);

unsigned char enviar_datos (void);
unsigned char enviar_tempmax (void);
unsigned char enviar_tempmin (void);
unsigned char enviar_tempambiente (void);

void menu (void);

void confi_rs232(void);

//---------------------------------------------------------------------------------------//


//---------------------------------------- Main -----------------------------------------//

void main(void)
{
	inicio_cop_lvi_irq();
	inicio_puertos();
	inicio_rtc();
	inicio_kbi();
	inicio_adc();
	inicio_osc();
	inicio_sci2(9600);
	lcd_char_init4();			//inicializacion del display
	
	EnableInterrupts;// habilita todas las interrupciones
	
	
	PTBD = 0x00;			// cargo toddo el puerto B con cero. 
	LED_Y = LED_OFF;				// apago el led amarillo porque
	
	for(;;)
	{
		registro_temp ();	// funcion recorre las 8 termopares 
		
		mostrar_temp ();	// muestra las temperaturas en display
		
		medir_temp_max ();	// obtiene la temperatura maxima de cada termopar
		
		medir_temp_min ();	// obtiene la temperatura minima de cada termopar
	
		
		if (serie)
		{
			if(t_registro>=0)
			{	
				LED_Y = LED_ON;	// prendo el led que indica envio rs232
				if(!timer_intervalo)
				{
					enviar_datos ();		// envia los datos obtenidos por los termopares por RS-232
					timer_intervalo = intervalo;
					t_registro--;
					
				}
			}	
		}
			
		if (t_registro==0 && serie==1)
			{
			serie=0;// fin envio RS-232
			LED_Y = LED_OFF;// apago el led que indica envio rs232
			f_enviar_tempmax= 1;
			f_enviar_tempmin= 1;
			f_enviar_tempambiente= 1;
			}
		
		if (f_enviar_tempmax)	// envio al final del la transmision temp max
			{
				enviar_tempmax ();
				f_enviar_tempmax= 0;
			}
		
		if (f_enviar_tempmin)	// envio al final del la transmision temp min
			{
				enviar_tempmin ();
				f_enviar_tempmin= 0;
			}
		
		if (f_enviar_tempambiente)	// envio al final del la transmision temp ambiente
			{
				temp_ambiente = medir_temp_ambiente();
				enviar_tempambiente ();
				f_enviar_tempambiente= 0;
			}	
		
		if (f_menu==1) menu ();		// entrada a la funcion del menu de RS-232
		
		if (f_boton_s1) mostrar_temp_max();		// entrada a la funcion de muestra de temp maximas
	
		if (f_boton_s2) mostrar_temp_min();		// entrada a la funcion de muestra de temp minimas
		
		if(f_mostrar_temp_ambiente) mostrar_temp_ambiente(); // entrada a la funcion de muestra de temp ambiente
			

	} //------ FOR(ever)

}//----- main end

//---------------------------------------------------------------------------------------//

//----------------------------- Declaracion de interrupciones-----------------------------//
interrupt 29 void RTC_ISR(void)
{
	RTCSC_RTIF = 1;	// limpio la bandea

	if(mseg != 0) mseg--;
	if (timer_intervalo !=0) timer_intervalo--;
}

interrupt 26 void ADC_ISR(void)
{
	if(ADCSC1 == (ADC_CANAL8 | 0x80))
		{
			adc_ch8 = ADCR;										// 0x48
			ADCSC1 = ADC_LM35; // se pasa al canal del lm35		//COCO	0 =  conversion no terminada 
		}														//AEIN	1 = Conversion complete interrupt enabled
																//ADCO	0 =  conversion no continua
																//ADCH	0100 = seleccionado canal 8
		if(ADCSC1 == (ADC_LM35 | 0x80))
		{
			adc_lm35 = ADCR;
			ADCSC1 = ADC_CANAL8;
		}	
}

interrupt 25 void KBI_ISR(void)
{
	int a;
			
	if(BOTON_MENU==0)
	{
		for(a=0;a<5000;a++);
		if (BOTON_MENU==0) f_menu = 1;
	}
		
	if(BOTON_S1==0)
	{
		for(a=0;a<3000;a++);
		if(BOTON_S1==0 && BOTON_S2==1 && serie==0)f_boton_s1 =~f_boton_s1;
		if(BOTON_S1==0 && BOTON_S2==0 && serie==0) f_mostrar_temp_ambiente=~f_mostrar_temp_ambiente;
		while(BOTON_S1==0);
		for(a=0;a<3000;a++);	
	} 
	
	if(BOTON_S2==0)
	{
		for(a=0;a<3000;a++);
		if(BOTON_S1==1 && BOTON_S2==0  && serie==0)f_boton_s2 =~f_boton_s2;
		if(BOTON_S1==0 && BOTON_S2==0 && serie==0) f_mostrar_temp_ambiente=~f_mostrar_temp_ambiente;
		while(BOTON_S2==0);
		for(a=0;a<3000;a++);	
	}
	
	KBISC_KBACK = 1;								// limpio la bandera
}


interrupt 23 void int_sci2(void)			// interrumpe rx sci2
{									
	if(timeout_rx2 != 0) f_sci2++;
	SCI2S1=SCI2S1;         			
	b_sci2[f_sci2]=SCI2D;
	f_rx_sci2 = 1;						// limpia el flag
	timeout_rx2=50;
}

//---------------------------------------------------------------------------------------//


//---------------------------------- Declaracion de funciones-----------------------------//

void inicio_puertos(void)
{
			//PUERTO A	
	PTAD = 0;					// configuro las salidas=0
	PTADD = 0xFF;				// configuro todos el puerto como salidas
	PTAPE = 0xFF;				// habilito pullup
	PTASE = 0xFF;

			//PUERTO B
	PTBD = 0;					// salidas=0
	PTBDD = 0x1F;				// configuro salidas
	PTBPE = 0xFF;				// habilito pullup
	PTBSE = 0xFF;

			//PUERTO C
	PTCD = 0;					// salidas=0
	PTCDD = 0xFF;				// configuro salidas
	PTCPE = 0xFF;				// habilito pullup
	PTCSE = 0xFF;

			//PUERTO D
	PTDD = 0;					// salidas=0
	PTDDD = 0xFB;				// configuro salidas, PTD2 como entrada
	PTDPE = 0xFF;				// habilito pullup
	PTDSE = 0xFF;

			//PUERTO E
	PTED = 0;					// salidas=0
	PTEDD = 0xFF;				// configuro salidas
	PTEPE = 0xFF;				// habilito pullup
	PTESE = 0xFF;
	
			//PUERTO F
	PTFD = 0;					// salidas=0
	PTFDD = 0xFC;				// configuro salidas/ f0 como entrada
	PTFPE = 0xFF;				// habilito pullup
	PTFSE = 0xFF;
	
			//PUERTO G
	PTGD = 0;					// salidas=0
	PTGDD = 0xF0;				// configuro salidas, PTG0/PTG1/PTG2/PTG3 entrada
	PTGPE = 0xFF;				// habilito pullup
	PTGSE = 0xFF;
}		
	
void inicio_cop_lvi_irq(void)
{
	IRQSC = 0;   			   	            		// IRQ disable
	SOPT1 = 0;  		    			       		// cop disable
	SOPT2 = 0;  		    	    
	SPMSC1 = 0;         	        				// LVD disable
	SPMSC2 = 0;
}

void inicio_rtc(void)
{
	RTCMOD = 0;
	RTCSC = 0x18;									// RTCLKS=00->1Khz,RTIE=1,RTCPS=0x8->1mseg
}

void inicio_kbi(void)
{
	KBISC = 0x04;									// KBACK=1
	KBIPE = 0xC7;									// KBP0=1,KBP1=1,KBP2=1,KBP6=1,KBP7=1
	KBIES = 0x00;									// KBP0 Y KBP1  Falling edge/low level
	KBISC = 0x06;									// KBACK=1,KBIE=1,KMOD=1
}

void delay_mseg(unsigned int ms)
{
	mseg = ms;
	while(mseg!=0);										
}

void inicio_osc(void)
{								
	MCGTRM = TRIM_32KHz;
	MCGC2 = 0x40;								// Fref = 31.25KHz; BDIV=1; VDIV=4; R=1;
	MCGC1 = 0x04;								// M = VDIV * 256 ;
	MCGC3 = 0x01;								// FLL = M * fref / R ; Fbus = (FLL/BDIV) / 2
	while(MCGSC_LOCK == 0) ; 					// espera enganche FLL
}

void inicio_adc(void)
{
	APCTL1 = 0x20;
	APCTL2 = 0x01;									
	ADCCFG = 0xF5;								// 12bit,clock/8,bus_clock/2,low speed,long sample								 
	ADCSC2 = 0;
	ADCSC1 = 0x48;								// habilito Int, conversion simple
}


//---------------------------------- Registro termopares ----------------------------------//
void registro_temp (void)
{
//----------------------- Termopar 1 -----------------------------------	
	if (PTBD_PTBD0 == 0 || PTBD_PTBD1 == 0 )
		{
			termo=0;
			delay_mseg (20);
			temp_act[0]= medir_temp();
			PTBD = 0x01;
		}		
							
//----------------------- Termopar 2 -----------------------------------
	
		if (PTBD_PTBD0 == 1 || PTBD_PTBD1 == 0 )
			{
				termo = 1;
				delay_mseg (20);
				temp_act[1]= medir_temp();
				PTBD = 0x02;		
			}	
//----------------------- Termopar 3 -----------------------------------
				
		if (PTBD_PTBD0 == 0 || PTBD_PTBD1 == 1 )
			{
				termo = 2;
				delay_mseg (20);
				temp_act[2]= medir_temp();
				PTBD = 0x03;	
			}	
				
//----------------------- Termopar 4 -----------------------------------
		
		if (PTBD_PTBD0 == 1 || PTBD_PTBD1 == 1 )
			{
				termo = 3;
				delay_mseg (20);
				temp_act[3]= medir_temp();
				PTBD = 0x04;
			}	
			
//----------------------- Termopar 5 -----------------------------------
										
		if (PTBD_PTBD0 == 0 || PTBD_PTBD1 == 0 || PTBD_PTBD2 == 1 )
			{
				termo = 4;
				delay_mseg (20);
				temp_act[4]= medir_temp();
				PTBD = 0x05;
			}	
		
//----------------------- Termopar 6 -----------------------------------
									
		if (PTBD_PTBD0 == 1 || PTBD_PTBD1 == 0 || PTBD_PTBD2 == 1 )
			{
				termo = 5;
				delay_mseg (20);
				temp_act[5]= medir_temp();
				PTBD = 0x06;
			}			
					
//----------------------- Termopar 7 -----------------------------------
																
		if (PTBD_PTBD0 == 0 || PTBD_PTBD1 == 1 || PTBD_PTBD2 == 1 )
			{
				termo = 6;
				delay_mseg (20);
				temp_act[6]= medir_temp();
				PTBD = 0x07;
			}	
					
//----------------------- Termopar 8 -----------------------------------
		
		if (PTBD_PTBD0 == 1 || PTBD_PTBD1 == 1 || PTBD_PTBD2 == 1)
			{
				termo = 7;
				delay_mseg (20);
				temp_act[7]= medir_temp();
				PTBD = 0x00;
			}				

}

//------------------------- Medicion de Temperatura (Act, Max, Min )------------------------//

float medir_temp (void)
{	
	float b=0;
	int a;
	
	for (a=0;a<100;a++) // tomo n cantidad de muestras.
		{
			if (PTFD_PTFD0 != 0)
			
				b+= (adc_ch8*1.221*0.1);
		
			else // si no hay termocupla conectada en esa salida hago lo siguente
				
				b=0; // mando al valor actual cero.
			
		}
	b=b/100; // saco el promedio de las muestras
	return b;
	}

float medir_temp_max (void)
{
int r=0;
	
for (r=0;r<8;r++)
	{
		if (temp_act[r]!=0)
		{
			if (temp_act[r]>temp_max[r])
				temp_max[r]=temp_act[r];
		}
		else
			temp_max[r]=0;
	}
}

float medir_temp_min (void)
{
int r=0;
		
for (r=0;r<8;r++)
	{
		if (temp_min[r]!=0)
		{
			if (temp_act[r]<temp_min[r])
				temp_min[r]=temp_act[r];
		}				
		else		
			temp_min[r]=350;	
	}
}

float medir_temp_ambiente(void)
{
	float b=0;
	int a;
	for (a=0;a<50;a++) // tomo n cantidad de muestras.
	{
		b+= (adc_lm35*1.221*0.1);			
	}	
	b=b/50; // saco el promedio de las muestras
	return b;
}	

//---------------------------- Mostrar en Display --------------------------------------------//

void mostrar_temp (void)
{                  
	sprintf(&buffer[0] ,"T1:%.1f        ",temp_act[0]); // minimo escribir 10 caracteres
	sprintf(&buffer[10],"T5:%.1f        ",temp_act[4]); 
	line_lcd4(buffer,1);

	sprintf(&buffer[0] ,"T2:%.1f        ",temp_act[1]); 
	sprintf(&buffer[10],"T6:%.1f        ",temp_act[5]);
	line_lcd4(buffer,2);	
	
	sprintf(&buffer[0] ,"T3:%.1f        ",temp_act[2]); 
	sprintf(&buffer[10],"T7:%.1f        ",temp_act[6]); 
	line_lcd4(buffer,3);
	
	sprintf(&buffer[0] ,"T4:%.1f        ",temp_act[3]); 
	sprintf(&buffer[10],"T8:%.1f        ",temp_act[7]); 
    line_lcd4(buffer,4);	
}

void mostrar_temp_max (void)
{
	KBIPE = 0x01; // desabilito los otros botones
	delay_mseg (100);
	while (f_boton_s1 )		// si se presiono el boton S2
	{
		sprintf(&buffer[0] ,"T1M:%.1f        ",temp_max[0]); // minimo escribir 10 caracteres
		sprintf(&buffer[10],"T5M:%.1f        ",temp_max[4]); 
		line_lcd4(buffer,1);
	
		sprintf(&buffer[0] ,"T2M:%.1f        ",temp_max[1]); 
		sprintf(&buffer[10],"T6M:%.1f        ",temp_max[5]);
		line_lcd4(buffer,2);	
			
		sprintf(&buffer[0] ,"T3M:%.1f        ",temp_max[2]); 
		sprintf(&buffer[10],"T7M:%.1f        ",temp_max[6]); 
		line_lcd4(buffer,3);
			
		sprintf(&buffer[0] ,"T4M:%.1f        ",temp_max[3]); 
		sprintf(&buffer[10],"T8M:%.1f        ",temp_max[7]); 
		line_lcd4(buffer,4);
	}	
	KBIPE = 0xC7;  // habilito los botones otra vez
}

void mostrar_temp_min (void)
{
	KBIPE = 0x02;		// desabilito los otros botones
	delay_mseg (100);
	while (f_boton_s2)
	{	
		int r=0;
	
		for (r=0;r<8;r++)
		{
			if (temp_min[r]== 350)temp_min[r]=0;
		}
		sprintf(&buffer[0] ,"T1m:%.1f        ",temp_min[0]); // minimo escribir 10 caracteres
		sprintf(&buffer[10],"T5m:%.1f        ",temp_min[4]); 
		line_lcd4(buffer,1);

		sprintf(&buffer[0] ,"T2m:%.1f        ",temp_min[1]); 
		sprintf(&buffer[10],"T6m:%.1f        ",temp_min[5]);
		line_lcd4(buffer,2);
	
		sprintf(&buffer[0] ,"T3m:%.1f        ",temp_min[2]); 
		sprintf(&buffer[10],"T7m:%.1f        ",temp_min[6]); 
		line_lcd4(buffer,3);
		
		sprintf(&buffer[0] ,"T4m:%.1f        ",temp_min[3]); 
		sprintf(&buffer[10],"T8m:%.1f        ",temp_min[7]); 
		line_lcd4(buffer,4);
	}
	KBIPE = 0xC7;		// habilito los botones otra vez
}		

void mostrar_temp_ambiente(void)
{	
	int a=5;
	KBIPE = 0x00;
	while (a!=0)
	{
		temp_ambiente = medir_temp_ambiente();
		delay_mseg (100);
		line_lcd4("                    ",1);
		line_lcd4("  Temp. Ambiente:   ",2);	
		line_lcd4("                    ",4);
		sprintf(&buffer[0] ,"       %.1f         ",temp_ambiente); 
		line_lcd4(buffer,3);
		a--;
	}
	delay_mseg (100);
	f_mostrar_temp_ambiente=~f_mostrar_temp_ambiente;
	KBIPE = 0xC7;
}


//------------------------------ Config comunicacion Serie ------------------------------------//

void inicio_sci2(unsigned int br)
{
	unsigned char a;
	SCI2S1 = SCI2S1;
	a = SCI2D;
	SCI2BD = (500000/br); 	// Baud Rate = BUSCLK / ([SBR12:SBR0] x 16)=(8M/16)/br
	SCI2C1 = 0x00; 			// Loop mode disabled, disable SCI, Tx output not inverted,
	SCI2C2 = 0x2C; 			// Enable SCI receive interrupts, Enable transmitter and
	SCI2C3 = 0x00; 			// Disable all error interrupts 
}
void sci2_byte(unsigned char sci_tx)
{
  SCI2D = sci_tx;             						// cargo el buffer del transmisor
  while(SCI2S1_TC==0);        						// espero completar la transmision   
}
void sci2_string(unsigned char *dato)
{
  while(*dato != '\0') sci2_byte(*(dato++)); 		// envia datos por SCI
}


//---------------------------------------------------------------------------------------------//

//---------------------------------- Envio datos RS-232 ---------------------------------------//

unsigned char enviar_datos (void)
{
	while (v)
	{ 
		sci2_byte(0x0D); //	fin y enter por si hay basura
		sci2_byte(0x0A);
		sprintf(&output[0],"T1,T2,T3,T4,T5,T6,T7,T8 ");	// envia encabezado 
		sci2_string(output);
		sci2_byte(0x0D);
		sci2_byte(0x0A);
		
		v=0;
	}		
				
	sprintf(&output[0],"%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",temp_act[0],temp_act[1],temp_act[2],temp_act[3],temp_act[4],temp_act[5],temp_act[6],temp_act[7]);
                                                           
	sci2_string(output);					// envia cadena 
	sci2_byte(0x0D);
	sci2_byte(0x0A);
	delay_mseg(10);										
							
}				
				
unsigned char enviar_tempmax (void)
{
	delay_mseg(2000);
	sprintf(&output[0],"T1M,T2M,T3M,T4M,T5M,T6M,T7M,T8M ");	// envia encabezado 
	sci2_string(output);
	sci2_byte(0x0D);
	sci2_byte(0x0A);	
	sprintf(&output[0],"%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",temp_max[0],temp_max[1],temp_max[2],temp_max[3],temp_max[4],temp_max[5],temp_max[6],temp_max[7]);
    sci2_string(output);					// envia cadena 
	sci2_byte(0x0D);
	sci2_byte(0x0A);
}

unsigned char enviar_tempmin (void)
{
	int r=0;
	for (r=0;r<8;r++)
		{
			if (temp_min[r]== 350)temp_min[r]=0;
		}
	delay_mseg(2000);
	sprintf(&output[0],"T1m,T2m,T3m,T4m,T5m,T6m,T7m,T8m ");	// envia encabezado 
	sci2_string(output);
	sci2_byte(0x0D);
	sci2_byte(0x0A);	
	sprintf(&output[0],"%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",temp_min[0],temp_min[1],temp_min[2],temp_min[3],temp_min[4],temp_min[5],temp_min[6],temp_min[7]);
	sci2_string(output);					// envia cadena 
	sci2_byte(0x0D);
	sci2_byte(0x0A);
	delay_mseg(10);							
}	

unsigned char enviar_tempambiente (void)
{
	delay_mseg(2000);
	sprintf(&output[0],"Temperatura ambiente: ");	// envia encabezado 
	sci2_string(output);
	sci2_byte(0x0D);
	sci2_byte(0x0A);	
	sprintf(&output[0],"%.1f",temp_ambiente);
    sci2_string(output);					// envia cadena 
	sci2_byte(0x0D);
	sci2_byte(0x0A);
}

//---------------------------------------------------------------------------------------------//	

void menu (void)
{
	
	KBIPE = 0xC4;		// desabilito los botones de temperaturas max y min

	while (f_menu==1 && flag_set_point==0)	// entra al presionar 
	{
		line_lcd4("                    ",1);	//muestra este menu en display
		line_lcd4("       RS-232       ",2);	// hasta presionar SI o NO
		line_lcd4("                    ",3);
		line_lcd4("  SI            NO  ",4);

		if(BOTON_UP==0)		// opcion de SI
		{
			delay_mseg(50);
			if(BOTON_UP==0)
			{
				flag=1;
				confi_rs232();
			}
		}  
		   
		if(BOTON_DOWN==0)		// opcion de NO
		{
			delay_mseg(50);
			if(BOTON_DOWN==0) 
			{
				flag_set_point=0;
				f_menu=0;
		   		serie=0;
		   		LED_Y = LED_OFF;
		   		KBIPE = 0xC7;
			}
		}
	}	
}

//--------------------------------------------------------------------------------------------//	   
		
void confi_rs232(void)
{
	int a;
		while (flag==1 && flag_set_point==0)
		{
			f=1;
			contador=5;
			line_lcd4("                    ",1);
			line_lcd4("    Congifuracion   ",2);
			line_lcd4("       RS-232       ",3);
			line_lcd4("                    ",4);
			delay_mseg(1500);
	
//---------------------------- INTERVALO -----------------------------------------------
	
			while(BOTON_MENU!=0 && f==1)
			{
				line_lcd4("Intervalo medicion: ",1);
				line_lcd4("                    ",3);
				line_lcd4("                    ",4);
											
				if(BOTON_UP==0)
				{
					delay_mseg(20);
					contador= contador +5;
					if(contador == 25)contador = contador+5;
				}
																		
				if(BOTON_DOWN==0)
				{
					delay_mseg(20);
					contador=contador - 5;
					if(contador == 25)contador =contador-5;
				}
		    		    	   			
				if(contador<=0) contador=30;
				if(contador>30) contador=5;							
				sprintf(&buffer[0],"       %i min     ",contador);
				line_lcd4(buffer,2);
										
			}    
		
		    	    	
			if(BOTON_MENU==0 && f==1)
			{
				delay_mseg(100);
				line_lcd4("                    ",3);
				line_lcd4("                    ",4);
				line_lcd4("   ud. selecciono:  ",1);
				sprintf(&buffer[0],"        %i min      ",contador);
				line_lcd4(buffer,2);
				disp_intervalo=contador;
				intervalo=contador*60000;
				timer_intervalo = intervalo;
				delay_mseg(1500);
				flag_set_point=1;
			}
		}
		
//------------------------------------------------------------------------------------	
	
//--------------------------------- TIEMPO TOTAL --------------------------------------
	while(flag==1 && flag_set_point==1)
	{
		f=1;
		contador=1;
		   
		while(BOTON_MENU!=0 && f==1)
		{
			line_lcd4("    Tiempo total    ",1);
			line_lcd4("                    ",3);
			line_lcd4("                    ",4);
		   		    
			if(BOTON_UP==0)
			{
				delay_mseg(20);
		   		contador++;
		   	}
		   		    
			if(BOTON_DOWN==0)
			{
		   		delay_mseg(20);
		   		contador--;
			}
		   		    
		   	if(contador>5) contador=1;
		   	if(contador<=0) contador=5;
		   	sprintf(&buffer[0],"       %i HS        ",contador);
		   	line_lcd4(buffer,2);
		}
		   
		if(BOTON_MENU==0 && f==1)
		{
			delay_mseg(100);
			line_lcd4("  ud. selecciono:   ",1);
			line_lcd4("                    ",3);
			line_lcd4("                    ",4);
		   	sprintf(&buffer[0],"       %i HS        ",contador);
		   	line_lcd4(buffer,2);
		   	disp_t_registro=contador;
			t_registro=contador;
			delay_mseg(1500);
		}
		   
//---------------------------------------------------------------------------------------
	
//----------------------------- Display config terminada---------------------------------
	
		line_lcd4("                    ",1);	
		line_lcd4("    Configuracion   ",2);	
		line_lcd4("         OK         ",3);			
			
		sprintf(&buffer[0],"%i min      ",disp_intervalo);
		line_lcd4(buffer,4);
		sprintf(&buffer[10],"      %i HS",disp_t_registro);
		line_lcd4(buffer,4);
		delay_mseg(2000);
		t_registro=( disp_t_registro*60/disp_intervalo);
		flag_set_point=0;
		f_menu=0;
		flag=0;
		serie=1;
		v=1;
	}
	
  KBIPE = 0xC7;		// habilito los botones otra vez
}			

//--------------------------------------------------------------------------------------------//				
					


