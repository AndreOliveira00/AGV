/*
 * VEÍCULO AUTÓNOMO
 *
 * Created: 16/01/2021 21:23:32
 * Author : André
 */ 

// BIBLIOTECAS
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
//#include <string.h> 

// CONSTANTES
#define F_CPU 1000000UL
#define _500ms		244								//	Valor para contar 500ms 244
#define pwm(val)	255*val/100						//	% Duty Cicle - fazer pwm(dutycicle)		
//#define pc2_on  0b00000100						//  Sonar sem interrupt (direito)
#define dist_minf 50								//  Distancia minima sonar da frente
#define dist_mint 30								//  Distancia minima sonar da frente
#define ON 1										//  Modo automático ativo
#define OFF 0										//  Modo controlado ativo

// VARIAVEIS
volatile unsigned int cont_timer = _500ms, tempo=0, dist_m=0, dist=0, dist_e=0, dist_d=0, COUNTA[10],pulse;		// 2 bytes (65536)
volatile unsigned char dir=0, velesq=pwm(1),vel_aux=0, reset_velo=0, aut=ON, cnt_10us=0,f_t=0,k=0, flag=0, flag_t=0, f_avirar=0, flag_servo=0, cont=0, cont_10us=0, tempinit=0, tempfim=0, direcao=0, flag_tras=0;	// vel = velocidade controlada
char  data= ' ', vel=pwm(20), velo[6]="000", msg[20]; 
/*
* cont_timer	-> Contador auxiliar para ciclo de 500 ms
* tempo			-> Tempo da leitura do sonar frontal
* dist			-> Distância registada pelo sonar frontal
* dist_e		-> Distância registada pelo sonar frontal quando servo está virado à esquerda
* dist_d		-> Distância registada pelo sonar frontal quando servo está virado à direita
* pulse			-> Tempo da leitura do sonar traseiro direito
* dir			-> Direção do AGV
* velesq		-> Velocidade extra para compensação do deifeito do motor esquerdo
* vel_aux		-> Velocidade auxiliar na ocorrencia de falha de conversão da velocidade emitida pelo utilizador
* reset_velo	-> Flag que verifica se o vetor velo[6] e as flgas associadas já foram resetadas
* aut			-> Flag que indica o estado automático ou controlado do AGV
* f_t			-> Flag que controla o acesso à interrupção 1
* k				-> Contador para conversão da velocidade emitida pelo utilizador
* flag			-> Flag que controla a correta determinação da distância do sonar frontal
* flag_t		-> Flag que controla a correta determinação da distância dos sonar traseiro esquerdo
* flag_tras		-> Flag que indica que os sonares traseiros já detetaram um obtáculo e não pode recuar mais
* f_avirar		-> Flag que indica que o AGV está numa fase de rotação,ou seja, a determinar a melhor direção para seguir em frente
* flag_servo	-> Flag que indica que o servomotor está numa fase de rotação de 20 graus
* cont			-> Contador auxiliar responsável por indicar o numero de overflows do timer 0 para efeitos de determinação do tempo gerado pelo sinal do sonar frontal
* cont_10us		-> Flag para efeito gasto de cicos do CPU de forma a passaram pelo menos 10 us (operação usada nos sonares)
* tempinit		-> Tempo inicial do timer 0 no incio da determinação do tempo gerado pelo sinal do sonar frontal
* tempfim		-> Tempo final do timer 0 no fim da determinação do tempo gerado pelo sinal do sonar frontal
* direcao		-> Establece o estado de operação do AGV
* data			-> Char retornada da comunicação Bluetooth
* vel			-> Velocidade controlada do AGV
* velo[6]		-> Vetor usado para auxilio da conversão da string recebida pela comunicação Bluetooth num inteiro lógico
* msg[20]		-> String a condicionar ao longo do programa para enviar para o terminal na aplicação
*/
// PROTOTIPOS DE FUNÇÕES
void init(void);
void motores(void);
void frente(void);
void tras(void);
void direita(void);
void esquerda(void);
void frentedireita(void);
void frenteesquerda(void);
void trasdireita(void);
void trasesquerda(void); 
void stop(void);
float mode(int a[],int n);
void send_msg(char *buffer);
unsigned char USART_ReceivePolling();
void teste_traseiro(void);
void send_dist();
void scan_e(void);
void scan_d(void);

void init(void)
{
	DDRD  = 0b11110010;			// Definir PD6 e PD5 como saída	(OC0A=ENA(motorA) e OC0B=ENB(motorB) respetivamente) PD4(Trig saída frente) PD2(Eco entrada) PD7 (saída IN3) PD2 (INT0) PD3 (INT1) PD1(RX0)
 	PORTD = 0b00000000;			// Inicializar OC0A e OC0B
	
	DDRB  = 0b00110111;			// Definir pino 0,1,2,4 e 5 do PORTB como saída	(LED 1Hz, PWM, IN2, IN4 e IN1 respetivamente)
	PORTB = 0b11001000;			// Inicializar OC0A e ligar pull-ups nas entradas não utilizadas
	
	DDRC  = 0b00000011;			// 2 Sonares atrás PC0 e PC1(Trig saída) PC2(Eco entrada)
	PORTC = 0b11111100;
	
	// TIMER0 para os Motores e Led
	// Modo Fast PWM
	TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);		// Modo fast PWM ativo, com OC0A, OC0B ligado (clear OC0x on Compare Match - non-inverting mode)
	TCCR0B |= (0 << WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00);			// PSC 8
	// T = (PSC * 256) / 1X10E6
	// T = (8 * 256) / 1X10E6 = 2.048 ms
	// 244 * 2.048e-3 = 499.712 ms
	TIMSK0 |= (1 << TOIE0);		// Ativar a interrupt para  Timer/Counter0, Overflow Interrupt Enable
	
	// TIMER1
	TCCR1A|=(1<<COM1A1) | (1<<WGM11);				// Modo Fast PWM (14) ativo, com OC1A ligado (clear OC0x on Compare Match - non-inverting mode)
	TCCR1B|=(1<<WGM12) | (1<<WGM13) | (1<<CS10);	// PSC 1
	ICR1=19999;										// f = 50 Hz
	OCR1A=1850;										// Servo posição 0 graus
	
	//TIMER2
	//TCCR2A|= (1<<COM2A1) | (1<<WGM21) | (1<<WGM20); //Fast PWM com TOP=0xFF
	TCCR2A|= (1<<WGM21) | (1<<WGM20);			//Fast PWM com TOP=0xFF
	TCCR2B|= (1<<CS22) ; //prescaler 64		prescaler 32 = (1<<CS21) | (1<<CS20)
	// T = (PSC * 256) / 1X10E6
	// T = (64 * 256) / 1X10E6 = 16.384 ms		//Suficiente para fazer a contagem do tempo do sonar 4 metros / 343 metros/s = 11.66 ms
	
	// USART
	UBRR0 = 12;													// Set baud rate
	UCSR0A = (1<<U2X0);											// Dobro da velocidade
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1 << RXCIE0);				// Enable receiver and transmitter
	UCSR0C = (0<<USBS0)|(0<<UPM01)|(1<<UCSZ01)|(1<<UCSZ00);		// Set frame format: 8data, 1stop bit
	
	// EXTERNAL INTERRUPTS
	EIMSK |= (1<<INT0) | (1<<INT1); // Ativar as interrupções externas 0 e 1
	EICRA |= (1<<ISC00) | (1<<ISC10); // Any logical change on INT0 and INT1 generates an interrupt request.
	EICRA &= (~(1<<ISC01));
	EICRA &= (~(1<<ISC11));
	//EICRA |= (1<<ISC00) | (1<<ISC10) | (1<<ISC11); 				// Subida do sinal gera interrupção (INT0)

	sei();						// Ativar interrupções globais
}

ISR (TIMER0_OVF_vect)
{
	cont++;
	cont_timer --;
	if(cont_timer == 0)
	{
		PORTB ^= 0b00000001;	//XOR (Se igual 0, se diferente 1), ou seja, se desligado (0) passa a estar ligado
		cont_timer = _500ms;
	}
}

ISR(INT0_vect){				// Interrupção externa
	if(flag == 1){			// flag = 1, primeira vez na interrupção
		tempinit = TCNT0;	// 1 tcnt0 = 8 us
		cont = 0;			// Reset nº de overflows
		flag = 2;
		//EICRA &= (~(1 << ISC00));
	}
	//if(flag == 2){		//flag = 2, segunda vez na interrupção
	else{
		tempfim = TCNT0;
		tempo=(8*(((cont-1)*256)+(255-tempinit)+tempfim));	//us
		//dist_m = (tempo/58);  // Em cm
		dist = (tempo/58);		// Em cm
		flag = 0;				// Obtive valor novo de dist
	}
}

ISR(INT1_vect)
{
	if (f_t==1)					// Segunda vez
	{
		TCCR2B=0;				// Desliga o timer2
		pulse=TCNT2;			// 1 tcnt2 = 0.064ms
		flag_t=1;
		TCNT2=0;
		f_t=0;
	}
	if (f_t==0)					// Primeira vez
	{
		//EICRA &= (~(1 << ISC10));
		TCNT2=0;
		TCCR2B|= (1<<CS22) ;	// Liga o timer2
		f_t=1;
	}
}

void USARTWriteChar(char buffer)
{
	//Wait until the transmitter is ready
	while((UCSR0A & (1<<UDRE0))==0);			// Bloqueado até o transmissor (UDR0) estar preparado
	UDR0=buffer;
}

void send_msg(char *buffer){
	unsigned char j=0;
	while(buffer[j]!='\0')						// Verifica o fim da string
	{
		while (( UCSR0A & (1<<UDRE0)) == 0){}	// Verifica se o buffer de transmissão está vazio (Espera UDRE0 ir a 1)
		UDR0=buffer[j];							// Coloca 1 byte no registo de transmissão
		j++;
	}
}

unsigned char USART_ReceivePolling(){
	char DataByte;
	while (( UCSR0A & (1<<RXC0)) == 0) {}; // Do nothing until data have been received
	DataByte = UDR0 ;
	return DataByte;
}

ISR(USART_RX_vect){			// Enviar a informação apenas quando alterada
	data=UDR0;
	//USARTWriteChar(data);
	if (data=='A')
	aut=OFF;				// Controlado
	else if (data=='B')
	aut=ON;			// Automático
	if (aut==OFF)
	{
		switch (data)
		{
			case 'F':
				dir=1;
				break;
			case 'T':
				dir=2;
				break;
			case 'D':
				dir=3;
				break;
			case 'E':
				dir=4;
				break;
			case 'C':	// Frente-direita
				dir=5;
				break;
			case 'G':	// Frente-esquerda
				dir=6;
				break;
			case 'H':	// Trás-direita
				dir=7;
				break;
			case 'I':	// Trás-esquerda
				dir=8;
				break;
			case 'S':	// STOP
				dir=0;
				break;
		}
	}
	if (data=='W'){		// Reset array
		reset_velo=1;
		velo[0]='0';
		velo[1]='0';
		velo[2]=' ';
		velo[3]=' ';
		k=0;
		}else if (data=='Y'){					// Converte para velocidade
		reset_velo=0;
		//k=0;
		velo[0]='0';							// Eliminar o 'W'
		vel_aux=atoi(velo);						// Converter vetor (string que pode conter numeros) para um inteiro
		if (vel_aux==0) USARTWriteChar('R');	// ERRO
		if (vel_aux>0 && vel_aux<128){			// 128 = 6V (Para alimetação de 12 V) 104 = 6 V (Para alimetação de 14.8 V)
			vel=vel_aux;
		}else vel=127;							// Ligeiramente abaixo de 6 V
		for (int t=1; t<=k; t++){
			USARTWriteChar(velo[t]);			// Imprimir velocidade
		}
		//USARTWriteChar(velo[1]);
		//USARTWriteChar(velo[2]);
		//USARTWriteChar(velo[3]);
	}
	if (reset_velo==1){	//Já está resetado o array da velocidade e vai escrever na posição correspondente o recebido em data
		velo[k]=data;	//velo[0]='W'
		k++;
	}
}

void motores()  // PB1 PB2 PB3 PB4
{
	switch (dir)
	{
		case 1:
			frente();
			break;
		case 2:
			tras();
			break;
		case 3:
			direita();
			break;
		case 4:
			esquerda();
			break;
		case 5:
			frentedireita();
			break;
		case 6:
			frenteesquerda();
			break;
		case 7:
			trasdireita();
			break;
		case 8:
			trasesquerda();
			break;
		case 0:
			stop();
			break;
	}
}

void frente(){
	PORTB &= ~(1<<PINB5) & ~(1<<PINB2) & ~(1<<PINB4);
	PORTD &= ~(1<<PIND7);
	//PORTB &= ~(1<<PINB5); // FRENTE
	//PORTD &= ~(1<<PIND7);
	PORTB |= (1<<PINB2) | (1<<PINB4);
	OCR0A = vel;
	OCR0B = vel;
}
void tras(){
	PORTB &= ~(1<<PINB5) & ~(1<<PINB2) & ~(1<<PINB4);
	PORTD &= ~(1<<PIND7);
	//PORTB &= ~(1<<PINB2) & ~(1<<PINB4);	// TRÁS
	PORTB |=  (1<<PINB5);
	PORTD |= (1<<PIND7);
	OCR0A = vel;
	OCR0B = vel;
}
void direita(){
	PORTB &= ~(1<<PINB5) & ~(1<<PINB2) & ~(1<<PINB4);
	PORTD &= ~(1<<PIND7);
	//PORTB &= ~(1<<PINB5) & ~(1<<PINB4); // DIREITA
	PORTD |= (1<<PIND7);
	PORTB |= (1<<PINB2);
	OCR0A = vel+velesq;
	OCR0B = vel;
}
void esquerda(){
	PORTB &= ~(1<<PINB5) & ~(1<<PINB2) & ~(1<<PINB4);
	PORTD &= ~(1<<PIND7);
	//PORTB &= ~(1<<PINB2) // ESQUERDA
	//PORTD &= ~(1<<PIND7);
	PORTB |= (1<<PINB5) | (1<<PINB4);
	OCR0A = vel+velesq;
	OCR0B = vel;
}
void frentedireita(){
	PORTB &= ~(1<<PINB5) & ~(1<<PINB2) & ~(1<<PINB4);
	PORTD &= ~(1<<PIND7);
	//PORTB &= ~(1<<PINB5); // FRENTE DIREITA
	//PORTD &= ~(1<<PIND7);
	PORTB |= (1<<PINB2) | (1<<PINB4);
	OCR0A = vel+pwm(3);
	OCR0B = vel;
}	
void frenteesquerda(){
	PORTB &= ~(1<<PINB5) & ~(1<<PINB2) & ~(1<<PINB4);
	PORTD &= ~(1<<PIND7);
	//PORTB &= ~(1<<PINB5); // FRENTE ESQUERDA
	//PORTD &= ~(1<<PIND7);
	PORTB |= (1<<PINB2) | (1<<PINB4);
	OCR0A = vel;
	OCR0B = vel+pwm(3);
}
void trasdireita(){
	PORTB &= ~(1<<PINB5) & ~(1<<PINB2) & ~(1<<PINB4);
	PORTD &= ~(1<<PIND7);
	//PORTB &= ~(1<<PINB2) & ~(1<<PINB4);	// TRÁS DIREITA
	PORTB |=  (1<<PINB5);
	PORTD |= (1<<PIND7);
	OCR0A = vel+pwm(3)+velesq;
	OCR0B = vel;
}
void trasesquerda(){
	PORTB &= ~(1<<PINB5) & ~(1<<PINB2) & ~(1<<PINB4);
	PORTD &= ~(1<<PIND7);
	//PORTB &= ~(1<<PINB2) & ~(1<<PINB4);	// TRÁS ESQUERDA
	PORTB |=  (1<<PINB5);
	PORTD |= (1<<PIND7);
	OCR0A = vel;
	OCR0B = vel+pwm(3);

}
void stop(){
	PORTB &= ~(1<<PINB5) & ~(1<<PINB2) & ~(1<<PINB3) & ~(1<<PINB4); // PARADO
	OCR0A = 0;
	OCR0B = 0;
}

void triggersonar(){
	//for (int d = 0; d < 5; ++d)
	//{
	PORTD |= (1<<PD4);					// Ativa trigger
	cont_10us=0;						// Cada definição a zero gasta 2 ciclos de clock = 2us
	cont_10us=0;
	cont_10us=0;
	tempo=0;							// Inicializar variaveis temporais
	tempfim=0;
	tempinit=0;
	PORTD &= (~(1<<PD4));				// Desativa trigger
	flag=1;
	while(flag!=0){};					// Obtive valor novo de dist
	if ((dist<1) || (dist>400)) dist = 400;	// Limite lógico definido para o sonar
	//EICRA|= (1<<ISC01) | (1<<ISC00);	// Subida do sinal gera interrupção (INT0)
	//COUNTA[d]=dist_m;
	//}
	//dist = moda(COUNTA,5);
}

void scan_e(){				// Ativar trigger sonar traseiro esquerda
	PORTC |= (1<<PC0);
	cont_10us=0;						// Cada definição a zero gasta 2 ciclos de clock = 2us
	cont_10us=0;
	cont_10us=0;
	cont_10us=0;						// Inicializar variaveis temporais para garantir pelo menos 10 us
	cont_10us=0;
	cont_10us=0;
	PORTC &= (~(1<<PC0));				// Desativa trigger
	while(flag_t==0){};
	//EICRA|= (1<<ISC11) | (1<<ISC10);	// Subida do sinal gera interrupção (INT1)
	flag_t=0;
	f_t=0;
	dist_e=(pulse*64)/58;
}
void scan_d(){
	int pulsa=0;
	PORTC &= ~(1<<PORTC1);				// Desligar PORTC1
	cont_10us=0;
	TCNT2=0;
	PORTC |= (1<<PORTC1);				// Ligar PORTC1
	cont_10us=0;						// Cada definição a zero gasta 2 ciclos de clock = 2us
	cont_10us=0;
	cont_10us=0;
	cont_10us=0;						// Inicializar variaveis temporais para garantir pelo menos 10 us
	cont_10us=0;
	cont_10us=0;
	PORTC &= ~(1<<PORTC1);				// Desligar PORTC1
	while( !(PINC & (1 << PINC2)) );	// Espera que PINC2 tome o valor 1
	TCCR2B|= (1<<CS22);                 //LIGAR TIMER
	while( PINC & (1 << PINC2) );		// Espera que PINC2 tome o valor 0
	TCCR2B = 0;			                //PARAR TIMER
	pulsa=TCNT2;						//COPIAR REGISTO DO TIMER
	dist_d=((pulsa*70)/58)-46;           //Conversão tempo em distância
	if (dist_d<=0) dist_d=1;
	TCNT2=0;                            // Reiniciar TIMER2 a 0
}

/*
int moda(int a[],int n) {
	int  maxCount = 0;
	float maxValue = 0;
	for (int k = 0; k < n; ++k) {
		int count = 0; 
		for (int j = 0; j < n; ++j) {
			if ( ( a[j] > (0.95*a[k]) ) && ( a[j] < (1.05*a[k]) ) )
			++count;
		}
		if (count > maxCount) {
			maxCount = count;
			maxValue = a[k];
		}
	}
	return maxValue;
}		
*/
void teste_traseiro()
{
	char buff[20];
	if (flag_tras==0){				//flag_tras = 1 já detetou um obstáculo atrás
		scan_e();
		//itoa(dist_e,msg, 10);			// Converte de inteiro para string, 10 significa decimal (2 - binario e 16 - hexadecimal)
		//sprintf(buff, "E=%s, ",msg);
		//send_msg(buff);
		if( dist_e < dist_mint)
		{
			//USARTWriteChar('E');
			//USARTWriteChar('E');
			//_delay_ms(1000);
			stop();
			flag_tras=1;
		}else{
			tras();
			_delay_ms(250);
			//_delay_ms(500);
		}	
	}
	if (flag_tras==0){				//flag_tras = 1 já detetou um obstáculo atrás
		scan_d();
		//itoa(dist_d,msg, 10);			// Converte de inteiro para string, 10 significa decimal (2 - binario e 16 - hexadecimal)
		//sprintf(buff, "D=%s, ",msg);
		//send_msg(buff);
		if( dist_d < dist_mint)
		{
			stop();
			//USARTWriteChar('D');
			//USARTWriteChar('D');
			//_delay_ms(1000);
			flag_tras=1;
			}else{
			tras();
			_delay_ms(250);
			//_delay_ms(500);
		}
	}
}

void send_dist(){
	itoa(dist,msg, 10);				// Converte de inteiro para string 10 significa decimal (2 - binario e 16 - hexadecimal)
	sprintf(msg, "%s , ",msg);
	send_msg(msg);
}

int main(void)
{
	char buff[20],i=0, f_s=0;
	int dist_dir = 0, dist_esq = 0, servo=1850;
	init();
	sprintf(msg, "\nAGV READY...... \n");
	send_msg(msg);
	/*sprintf(msg, "*RR252G111B3");
	send_msg(msg);
	sprintf(msg, "*LR252G111B3");
	send_msg(msg);*/
	OCR1A=1850;
	//while(1){
		/*scan_e();
		itoa(dist_e,msg, 10);			// Converte de inteiro para string, 10 significa decimal (2 - binario e 16 - hexadecimal)
		sprintf(buff, "E=%s, ",msg);
		send_msg(buff);
		_delay_ms(200);
		scan_d();
		itoa(dist_d,msg, 10);			// Converte de inteiro para string, 10 significa decimal (2 - binario e 16 - hexadecimal)
		sprintf(buff, "D=%s, ",msg);
		send_msg(buff);
		_delay_ms(400);*/
		/*teste_traseiro();
		flag_tras=0;
		triggersonar();
		_delay_ms(250);
		itoa(dist,msg, 10);			// Converte de inteiro para string, 10 significa decimal (2 - binario e 16 - hexadecimal)
		sprintf(buff, "F=%s, ",msg);
		send_msg(buff);
	}*/
	while (1) {	
		if (aut==ON){					// AUTOMÁTICO
			if ((flag==0)&&(f_avirar==0)){				// Acabou uma leitura do sonar e o AGV não está a virar
				if (direcao == 0){		// Em frente	
					i++;
					if(i==4){			// Roda o servo constantemente (ao fim de 50 ciclos) para a seguinte posição
						i=0;
						f_s=1;			// Flag que indica que servo vai rodar
						if(OCR1A==2100){
							OCR1A=1850;
						}else if(OCR1A==1850){
							if (flag_servo==0){
								flag_servo=1;
								OCR1A=1600;
							}else{
								flag_servo=0;
								OCR1A=2100;
							} 
						}else{
							OCR1A=1850;
						}
					}
					frente();
					if (f_s==1){		// Delay mecanico da rotação do servo necessário
						_delay_ms(250);
						f_s=0;
					}
					//triggersonar();
					triggersonar();
					send_dist();
					//if ((dist>=2) && (dist<40)){
						//triggersonar();
						if ((dist>=2) && (dist<=dist_minf)){
							stop();
							sprintf(buff, "STOP\n",msg);
							send_msg(buff);
							//USARTWriteChar('S');
							direcao=1;
							flag_tras=0;		// Inicializa a flag para permitir leitura atrás
							teste_traseiro();
							teste_traseiro();	
							flag_tras=0;		// Inicializa a flag para permitir leitura atrás
							//USARTWriteChar('B');
							//tras();		
							//_delay_ms(1000);	// Tempo de deslocamento para trás
							servo=OCR1A;		// Guarda posição do servo para ser processada
							stop();
						}
					//}
				}else if (direcao == 1){
					OCR1A = 2800;				// Sonar esquerda
					_delay_ms(600);				// Delay mecânico
					//triggersonar();
					triggersonar();
					itoa(dist,msg, 10);			// Converte de inteiro para string, 10 significa decimal (2 - binario e 16 - hexadecimal)
					sprintf(buff, "ESQ=%s, ",msg);
					send_msg(buff);
					dist_esq = dist;
					/*if ((dist<1) || (dist>400)){
						dist_esq = 400;
					}else{ 
						dist_esq = dist;
					}*/
					direcao=2;					
				}else if (direcao == 2){	
					OCR1A = 800;				// Sonar direita
					_delay_ms(600);				// Delay mecânico
					//triggersonar();
					triggersonar();
					itoa(dist,msg, 10);			// Converte de inteiro para string, 10 significa decimal (2 - binario e 16 - hexadecimal)
					sprintf(buff, "DIR=%s, ",msg);
					send_msg(buff);
					dist_dir = dist;
					/*if ((dist<1) || (dist>400)){	
						dist_dir = 400;
						}else{
						dist_dir = dist;
					}*/
					f_avirar=1;					// O AGV vai começar a virar
					direcao = 0;
					OCR1A = servo;				// Servo continua na posição antes do sonar detetar um objeto
					_delay_ms(400);				// Delay mecanico
					if (dist_dir >= dist_esq){
						direita();
						//USARTWriteChar('D');
					}else {
						esquerda();
						//USARTWriteChar('E');
					} 
					//_delay_ms(500);
				}
			}else if ((flag==0)&&(f_avirar==1)){	// Acabou uma leitura do sonar e o AGV está a virar. Este if é responsável por detetar a posição que o AGV vai tomar quando detetar um obstáculo
				if (servo!=1850){					// Se posição do servo quando detetou o obstáculo for diferente do centro
						servo=1850;					
						OCR1A=1850;					// Coloca lo no centro para ler valores "frontais" do AGV
						//_delay_ms(2000);					
						//USARTWriteChar('A');
				}else{
					triggersonar();					// Se posição do servo quando detetou o obstáculo for a do centro faz as leituras necessárias até encontrar a posição que respeite a próxima condição
					if (dist>=120){
						f_avirar=0;					// O AGV detetou melhor caminho
						stop();						// O AGV para de virar
						//USARTWriteChar('B');
						//_delay_ms(2000);
						//_delay_ms(100);			 			
					}
				}
			}
			
		}else{			// CONTROLO REMOTO
			motores();	
		}
	}
}		