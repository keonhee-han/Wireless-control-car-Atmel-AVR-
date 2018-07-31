//Port Settings
/*
-PORT A (Geared DC Motor enable)
0(Out) : L_motor rotatation bit
1(Out) : R_motor rotatation bit

-PORT B (PWM generator to each DC Motor)
5(Out) : OCR1A

-PORT E (UART Transmission of 8 bit for Bluetooth) 
0(In)  : RXD
1(Out) : TXD
*/

#include <mega128.h>
#include <delay.h>   
#include <stdio.h>   

//Motor Control
#define LM_CW  PORTA.4 = 1 
#define LM_CCW PORTA.4 = 0  
#define RM_CW  PORTA.5 = 1 
#define RM_CCW PORTA.5 = 0  

unsigned char XBK_REF_C = 0;
unsigned char XFD_REF_C = 0;
unsigned char M_Speed = 0;  

unsigned char C_MCR= 0b00000000; 
//Custom SW Register for Bluetooth communication
unsigned char TMP= 0b00000000;
//Temporary Register

//Functions
//UART RXD
char Getch(void)
{
   while(!(UCSR0A & 0x80)); //수신완료flag 비트가1되면정지
   return UDR0; //UART0번사용
} 

//UART
void usart_init()
{
    UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
    //U2X0 = 0이므로Asynchronous Normal Mode를의미한다

    UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
    //UCSR0B의UCSZ02, UCSR0C의UCSZ01,UCSZ00 bit : 011 = Data bit : 8bit

    UCSR0C = 0x86; //1000 0110
    //UCSR0C = (Reserved ) | (0<<UMSEL0) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
    //UCSR0C의UPM01,UPM00은'00' Parity bit : X
    //UMSEL0은0 : 비동기Asynchronous)
    
    UBRR0H = 0;
    UBRR0L = 0x67;    //0110 0111
    //UBRRnH를먼저설정하고UBRRnL을설정UBRRnH가비어있으면Baud Rate 설정이안됨
    //UBRR0H = (---- 0000)
    //UBRR0L = (0110 0111)이므로이는UBRR0 11~0 bit까지USART0 Baud Rate Register를의미한다
    //계산해보면67(Hexa) = 103(Decimal) => UBRR = 103 이므로Baud Rate = 9600이나오게된다 (공식참조  

    //비동기전송모드는수신데이터의정확한검출을위하여x16 / x8 의Frequency를사용하여Sampling한다 
    //이렇게하면1bit를Sampling할때16개/ 8개의Clock Period를사용하게되어어느정도오차가있는경우에도
    //검출정확도를높일수있다 또한전압스파이크잡음에의하여데이터수신의오류를막기위하여1bit를3차례검출하는방법을사용한다
} 

// Robot motor speed change
void M_PWM(unsigned char LM, unsigned char RM)
{
      OCR1A = LM;          
      delay_ms(100);  
}

//Robot Movement functions
void R_SP(void)
{
      OCR1A = 0; 
      OCR1B = 0;           
      delay_ms(100);  
}

void R_FD(void)
{
      LM_CW;  
      RM_CCW;           
      delay_ms(100);  
}

void R_BK(void)
{
      LM_CCW;  
      RM_CW;              
      delay_ms(100);  
}

void R_RT(void)
{
      LM_CW;  
      RM_CW;              
      delay_ms(100);  
}

void R_LT(void)
{
      LM_CCW;  
      RM_CCW;              
      delay_ms(100);  
}

void main(void)
{

//Custom Register for Motor Control
//C_MCR = (0<<LMD) | (0<<RMD) | (0<<LMSP2) | (0<<LMSP1) | (0<<LMSP0) | (0<<RMSP2) | (0<<RMSP1) | (0<<RMSP0);

// Input/Output Ports initialization
// Port A initialization (DC motor rotating direction)
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRA=(1<<DDA7) | (1<<DDA6) | (1<<DDA5) | (1<<DDA4) | (1<<DDA3) | (1<<DDA2) | (1<<DDA1) | (1<<DDA0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization (PWM Generation)
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRB=(0<<DDB7) | (1<<DDB6) | (1<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTB=(0<<PORTB7) | (1<<PORTB6) | (1<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC7) | (0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization (LCD)
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (1<<DDD3) | (1<<DDD2) | (1<<DDD1) | (1<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Port E initialization (UART)
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=Out Bit0=In 
DDRE=(0<<DDE7) | (0<<DDE6) | (0<<DDE5) | (0<<DDE4) | (0<<DDE3) | (0<<DDE2) | (1<<DDE1) | (0<<DDE0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);

// Port F initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRF=(0<<DDF7) | (0<<DDF6) | (0<<DDF5) | (0<<DDF4) | (0<<DDF3) | (0<<DDF2) | (0<<DDF1) | (0<<DDF0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (0<<PORTF0);

// Port G initialization
// Function: Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRG=(0<<DDG4) | (0<<DDG3) | (0<<DDG2) | (0<<DDG1) | (0<<DDG0);
// State: Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTG=(0<<PORTG4) | (0<<PORTG3) | (0<<PORTG2) | (0<<PORTG1) | (0<<PORTG0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=0xFF
// OC0 output: Disconnected
ASSR=0<<AS0;
TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;
OCR0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 16000.000 kHz
// Mode: Fast PWM top=0x00FF
// OC1A output: Non-Inverted PWM
// OC1B output: Disconnected
// OC1C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.016 ms
// Output Pulse(s):
// OC1A Period: 0.016 ms Width: 0 us
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (1<<WGM10);
//Timer/Counter1을정의 WGM13,12,11,10을101로설정하여FastPWM(8bit)로설정
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
//Timer/Counter1을정의 COM1A1,1A0을0으로설정하여FastPWM(8bit)모드에서의비교출력모드를on-invertingmode로설정한다
//또한CS12,CS11,CS10을01으로설정하여lkI/O/1으로분주비를설정한다
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;
OCR1CH=0x00;
OCR1CL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer2 Stopped
// Mode: Normal top=0xFF
// OC2 output: Disconnected
TCCR2=(0<<WGM20) | (0<<COM21) | (0<<COM20) | (0<<WGM21) | (0<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2=0x00;

// Timer/Counter 3 initialization
// Clock source: System Clock
// Clock value: Timer3 Stopped
// Mode: Normal top=0xFFFF
// OC3A output: Disconnected
// OC3B output: Disconnected
// OC3C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer3 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR3A=(0<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (0<<WGM31) | (0<<WGM30);
TCCR3B=(0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (0<<WGM32) | (0<<CS32) | (0<<CS31) | (0<<CS30);
TCNT3H=0x00;
TCNT3L=0x00;
ICR3H=0x00;
ICR3L=0x00;
OCR3AH=0x00;
OCR3AL=0x00;
OCR3BH=0x00;
OCR3BL=0x00;
OCR3CH=0x00;
OCR3CL=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (0<<TOIE0);
ETIMSK=(0<<TICIE3) | (0<<OCIE3A) | (0<<OCIE3B) | (0<<TOIE3) | (0<<OCIE3C) | (0<<OCIE1C);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: Off
// INT3: Off
// INT4: Off
// INT5: Off
// INT6: Off
// INT7: Off
EICRA=(0<<ISC31) | (0<<ISC30) | (0<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (0<<ISC51) | (0<<ISC50) | (0<<ISC41) | (0<<ISC40);
EIMSK=(0<<INT7) | (0<<INT6) | (0<<INT5) | (0<<INT4) | (0<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);
                       
// USART0 initialization
   UCSR0A=0x00; //flag 레지스터를사용하지않음
   UCSR0B=0x18; //수신enable, 송신enable, 전송비트8bit
   UCSR0C=0x06; //비동기식통신
   UBRR0H=0;                      
   UBRR0L=103; //9600bps baud rate  

// USART1 initialization
// USART1 disabled
UCSR1B=(0<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (0<<RXEN1) | (0<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);



// ADC initialization
// ADC Clock frequency: 156.250 kHz
// ADC Voltage Reference: AREF pin
// ADC High Speed Mode: Off
// Only the 8 most significant bits of
// the AD conversion result are used
ADMUX=0x20; //기준전압을AREF 내부기준전압.56V으로사용(1<<ADLAR) 좌측정렬
ADCSRA=0xCF;//1100 1111 (ADEN,ADSC,ADIE,ADPS2,ADPS1,ADPS0), ADC활성화 ADC바로시작 Interrupt 활성화 20,000*1/128 = 156.25KHz



// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

// USART initialization
usart_init(); 

// Global enable interrupts 
//#asm("sei") // this disable will prevent ISR automatically

while (1)
    {  
            //Get UART TXD
        C_MCR = Getch();
        delay_ms(100);     
        
        //Motor Direction detector
        //LMD, RMD, LMSP2, LMSP1, LMSP0, RMSP2, RMSP1, RMSP0    
        //Changing Specific Bits :
        //(0<<LMD) | (1<<RMD)
        //0b 01 xxx xxx       
        TMP     = C_MCR & (0b11000000);              
        //Turn left
        if      (TMP == 0b00000000) PORTA = (0<<PORTA1) | (0<<PORTA0); //Left Motor CCW, Right Motor CCW   
        //Turn right
        else if (TMP == 0b11000000) PORTA = (1<<PORTA1) | (1<<PORTA0); //Left Motor CW, Right Motor CW
        //Forward
        else if (TMP == 0b10000000) PORTA = (1<<PORTA1) | (0<<PORTA0); //Left Motor CW, Right Motor CCW
        //Backward
        else if (TMP == 0b01000000) PORTA = (0<<PORTA1) | (1<<PORTA0); //Left Motor CCW, Right Motor CW
                                       
        //DC Motor Speed detector LSB로부터 6개는 출력 PWM을 결정한다.
        TMP = C_MCR & (0b00111111);    
        if      (TMP == 0b00000000) OCR1A = 0;
        else if      (TMP == 0b00001001) OCR1A = 70;
        else if (TMP == 0b00010010) OCR1A = 100;
        else if (TMP == 0b00011011) OCR1A = 130; 
        else if (TMP == 0b00100100) OCR1A = 160; 
        else if (TMP == 0b00101101) OCR1A = 190; 
        else if (TMP == 0b00110110) OCR1A = 220;
        else if (TMP == 0b00111111) OCR1A = 250;     
    }   //즉, 값이 높을수록 PWM이 높아지며 모터의 회전속도가 증가하게 된다.
}
