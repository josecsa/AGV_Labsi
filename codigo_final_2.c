//Labsi 2020
//José Sá 1180505
//Tiago Cunha 1180922

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL				
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#define t_500ms  122 //122
#define USART_BAUDRATE 9600 // Desired Baud Rate
#define BAUD_PRESCALER (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define EIGHT_BIT (3<<UCSZ00)
#define DISABLED    (0<<UPM00)
#define ONE_BIT (0<<USBS0)
#define on 1
#define off 0
#define thresholddistance 20
#define	MOV_L_FWD() 	PORTD=  (PORTD & 0b11100111) | 0b00010000 ;
#define	MOV_R_FWD() 	PORTC=  (PORTC & 0b11100111) | 0b00010000 ;
#define	MOV_L_BCK() 	PORTD=  (PORTD & 0b11100111) | 0b00001000 ;
#define	MOV_R_BCK() 	PORTC=  (PORTC & 0b11100111) | 0b00001000 ;
#define	MOV_L_STOP() 	PORTD=  (PORTD & 0b11100111) | 0b00011000 ;
#define	MOV_R_STOP() 	PORTC=  (PORTC & 0b11100111) | 0b00011000 ;
#define	MOV_L_OFF() 	PORTD=  (PORTD & 0b11100111);
#define	MOV_R_OFF() 	PORTC=  (PORTC & 0b11100111);
volatile unsigned int hc_sr04_cnt ,a=0 ;
uint16_t Teco=0 , pulse = 0;
volatile int f = 0 , flag , car = off , distance , direction, modo=3, welcome=3;
volatile unsigned int Counter=122,Counterled = 61 ;	//contador utilizado de forma a apenas utilizar 1 timer para PWM e timer inves de 2
char msg[20];
volatile uint8_t USART_ReceiveBuffer;

void init (void){       //portas D C B

	DDRC =	0b00111100;             //a alterar
	PORTC =	0b00000000;

	DDRD = 0b01111000;		//PD0 e PD1 RX e TX - Bluetooth //ainda n mexi aqui ***********************************************
	PORTD =	0b00000000;

	DDRB = 0b00101111;
	PORTB = 0b00000000;


	TCCR0A |= (1 << COM0A1) | (1 << COM0B1)  | (1 << WGM00);		// Modo fast PWM ativo, com OC0A ligado	foi escolhido o modo fast PWM dado que com o uso do modo ctc nao seria possivel gerar um PWM.
	TCCR0B |= (1 << CS02) | (0 << CS01) | (0 << CS00) ; // 		 
	//TIMSK0 |= (1 << OCIE0A) | (1 << OCIE0B) ;
	OCR0B =0;	//PWM a 0;
	OCR0A =0;	//PWM a 0;
	
  	EIMSK |= (1<<INT0); //enable external interrupt
  	EICRA |= (1<<ISC00); // Any logical change on INT0 generates an interrupt request.
	  
	TIMSK0 |= (1 << TOIE0);	  //enable interrupção
	  
	  
  	TCCR1A|=(1<<COM1A1) | (1<<WGM11);				// Modo Fast PWM (14) ativo, com OC1A ligado (clear OC0x on Compare Match - non-inverting mode)
  	TCCR1B|=(1<<WGM12) | (1<<WGM13) | (1<<CS11);	// PSC 1
  	ICR1=39999;
  	OCR1A=3475;
	  
  	TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20) ;

	sei();		//ativa interrupções
}
uint16_t read_sonar();
uint16_t read();
uint8_t USART_ReceivePolling();
float mode(uint16_t a[],int n);
void USART_TransmitPolling(uint8_t DataByte);
void USART_Init();
void send_msg(char *buffer);
void send_valor(int valor);
void servo_motor(void);
void brake();
void slight_backwards();
void forward(int speed);
void turn_left();
void turn_slight_left();
void turn_right();
void turn_slight_right();
void backwards();
void turn(int direction);
int scan();


int main(void){
	init();
	USART_Init();
	char LocalData;
	sprintf(msg, "Ready ...... \n");
	send_msg(msg);
	


while (1){	
	while(car!=on){

		MOV_R_STOP();
		MOV_L_STOP();
		OCR0A=0;
		OCR0B=0;
		

	}//espera receber sinal para ligar
	if(welcome==1){
		sprintf(msg, "Carro On escolha o modo");
		send_msg(msg);
		welcome==0;
		
	}
		while(modo==1 && car == on){
			int tempor=0;
			if(USART_ReceiveBuffer=='w'){
				USART_ReceiveBuffer= ' ';		
				MOV_R_FWD();
				MOV_L_FWD();
				OCR0A= 80;
				OCR0B= 80;		
				_delay_ms(100);	
				brake();
			
			}
			if(USART_ReceiveBuffer=='a'){
				USART_ReceiveBuffer= ' ';
				MOV_R_FWD();
				MOV_L_OFF();
				OCR0A= 80;
				OCR0B= 0;
				_delay_ms(100);
				brake();
				
				
			}
			if(USART_ReceiveBuffer=='d'){
				USART_ReceiveBuffer= ' ';
				MOV_L_FWD();
				MOV_R_OFF();
				OCR0A= 0;
				OCR0B= 80;
				_delay_ms(100);
				brake();
				
			}
			if(USART_ReceiveBuffer=='s'){
				USART_ReceiveBuffer= ' ';
				MOV_R_BCK();
				MOV_L_BCK();
				OCR0A= 80;
				OCR0B= 80;
				_delay_ms(100);
				brake();
				}
				
			}
				
		

		while( car == on && modo==0){	
			    
				sprintf(msg, "loop start:");
				send_msg(msg);
				

				Teco=read();
				send_valor(Teco);
			
				if ((Teco > thresholddistance) && (Teco < 1000)){
					forward(0);

				}
				
				else{
					
					brake();
					_delay_ms(500);
					direction=scan();
					turn(direction);
					
					if(direction==-1){
						while(direction==-1){
							backwards();
							brake();
							direction=scan();
							_delay_ms(150);
						}
						turn(direction);
					}
				}
	
		_delay_ms(15);
		}				
		}
	}

		




void turn(int direction){
	
	if(direction==1){
		turn_left();
		brake();
		_delay_ms(250);
	}
	if(direction==0){
		turn_right();
		brake();
		_delay_ms(250);
	}
}

void brake(){
	MOV_R_STOP();
	MOV_L_STOP();
	OCR0A=0;
	OCR0B=0;

}

void forward(int speed){
	MOV_R_FWD();
	MOV_L_FWD();
	OCR0A= 80 + speed*20;
	
	OCR0B= 80 + speed*20;
}
void turn_left(){
	slight_backwards();
	MOV_R_FWD();
	MOV_L_OFF();
	OCR0A= 50;
	OCR0B= 0;
	_delay_ms(500);
}
void turn_right(){
	slight_backwards();
	MOV_L_FWD();
	MOV_R_OFF();
	OCR0A= 0;
	OCR0B= 50;
	_delay_ms(500);
}

void slight_backwards(){
	brake();
	_delay_ms(250);
	MOV_R_BCK();
	MOV_L_BCK();
	OCR0A= 60;
	OCR0B= 60;
	_delay_ms(200);
}
void backwards(){
	MOV_R_BCK();
	MOV_L_BCK();
	OCR0A= 80;
	OCR0B= 80;
	_delay_ms(300);
}
int scan() {
	int i , distance[4], maior=0 , escolhido;
	OCR1A=1475;
	_delay_ms(550);
	distance[0]=read_sonar();
	send_valor(distance[0]);
	OCR1A=5375;
	_delay_ms(550);
	distance[1]=read_sonar();
	send_valor(distance[1]);
	for(i=0;i<2;i++){
		if(distance[i]>maior){
			maior=distance[i];
			escolhido=i;
		}
	}
	if(distance[escolhido]<30){
		escolhido=-1;
	}
	OCR1A=3500;
	_delay_ms(300);
	send_valor(escolhido);
	return escolhido;
}
uint16_t read_sonar(){
	int i=0;
	uint16_t dist , media;
	read();
	_delay_ms(20);
	for (i=0;i<5;i++)
	{
		dist=read();
		send_valor(dist);
		media=media+dist;
		_delay_ms(15);
	}
	media=media/5;
	return media;
}

uint16_t read(){
	uint16_t tc;
	PORTB|=(1<<PINB2);
	_delay_us(10);
	PORTB &=~(1<<PINB2)	;
	while(flag==0){};
	flag=0;
	tc=(pulse*62)/58;
	if(tc > 0){
		return tc;
	}	
}


void send_valor(int valor){
	itoa(valor,msg, 10);
	sprintf(msg, "%s , ",msg);
	send_msg(msg);
	USART_ReceiveBuffer== ' ';
}
void send_msg(char *buffer){
	unsigned char i=0;
	while(buffer[i]!='\0')						// Verifica o fim da string
	{
		while (( UCSR0A & (1<<UDRE0)) == 0){}			// Verifica se o buffer de transmissão está vazio (Espera UDRE1 ir a 1)
		UDR0=buffer[i];							// Coloca 1 byte no registo de transmissão
		i++;
	}
	
}

uint8_t USART_ReceivePolling()
{
	uint8_t DataByte;
	while (( UCSR0A & (1<<RXC0)) == 0) {}; // Do nothing until data have been received
	DataByte = UDR0 ;
	return DataByte;
}

ISR(INT0_vect)
{
	if (f==1)
	{
		TCCR2B=0;
		pulse=TCNT2; 
		flag=1;
		TCNT2=0;
		f=0;
	}
	if (f==0)
	{	
		TCNT2=0;
		TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20) ;
		f=1;
	}
}

void USART_TransmitPolling(uint8_t DataByte)
{
	while (( UCSR0A & (1<<UDRE0)) == 0) {}; // Do nothing until UDR is ready
	UDR0 = DataByte;
}
void USART_Init()
{
	// Set Baud Rate
	UBRR0H = BAUD_PRESCALER >> 8;
	UBRR0L = BAUD_PRESCALER;
	
	//UCSR0A = (1<<U2X0);											
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1 << RXCIE0);				// Enable receiver and transmitter
	UCSR0C = (0<<USBS0)|(0<<UPM01)|(1<<UCSZ01)|(1<<UCSZ00);		// Set frame format: 8data, 1stop bit
}
ISR(TIMER0_OVF_vect){	//a cada 1ms (1,024ms) 
	Counterled -- ;			//decrementa o contador até que se chega a 0, assim que é atingido o valor de 0 através do uso de um XOR ligamos ou desligamos a led e reinicia o contador
	if(Counterled==0){
		PORTC ^= 0b00000100;
		Counterled = 61;
	}
	
}
ISR(USART_RX_vect)
{	
	USART_ReceiveBuffer = UDR0;
	
	if(USART_ReceiveBuffer=='1'){
		car=on;
		sprintf(msg, "A ligar ...");	
		send_msg(msg);
		welcome==1;
	}
	if(USART_ReceiveBuffer=='0'){
		car=off;
		sprintf(msg, "A desligar ...");	
		send_msg(msg);
	}
	if(USART_ReceiveBuffer=='6'){
		modo=0;
		sprintf(msg, "Modo Automatico");
		send_msg(msg);
	}
	if(USART_ReceiveBuffer=='5'){
		modo=1;
		sprintf(msg, "Modo Manual");	
		send_msg(msg);
	}
	
}