//Port Settings
/*
-PORT D (LCD Display for Accelerometer value & Command)
0xff(out)
-PORT E (UART Transmission of 8 bit for Bluetooth)
0(In)  : RXD
1(Out) : TXD
-PORT F (ADC from X, Y axis Accelerometer)
0(In) : X axis
1(In) : Y axis
*/

#include <mega128.h>
#include <delay.h>   
#include <stdio.h>   
#include <lcd.h>   
                  
//About LCD
unsigned char lcd_data[40]; 
// LCD
#asm
   .equ __lcd_port=0x12 //PORTD 8
#endasm

//ADC
unsigned char xval = 0;
unsigned char yval = 0;
//ADC Reference values                 
unsigned char XBK_REF = 72;  
unsigned char XFD_REF = 78;   
unsigned char YBK_REF = 72;
unsigned char YFD_REF = 78;

unsigned char C_MCR = 0b00000000;
//Custom SW Register for Bluetooth communication
unsigned char TMP = 0b00000000;
//Temporary Register
unsigned char COM = 'N';   
//LCD_movement_display

//About ADC
unsigned char adc_data[2] = {0, 0}; //8bit ADC 0 : X, 1 : Y
unsigned char mux = 0; //X,Y 축ADC MUX 중X축먼저ADC 사용
// Voltage Reference: AREF pin
#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (1<<ADLAR))
// ADC interrupt service routine 
interrupt [ADC_INT] void adc_isr(void)
{    
    adc_data[mux]=ADCH;  //ADC된데이터를adc_data행렬에저장 ADC 결과가unsigned 8bit ADCH Register로저장
    mux++;                 //mux순서올림
    if(mux >= 2) mux = 0;  //mux초기화   3H = 0011B 부터Opamp Gain값나옴이걸안한다는뜻
    ADMUX = mux | 0x60;  //60H => (0<<REFS1, 1<<REFS0, 0<<ADLAR) AVCC 기준전압설정후우측정렬로초기설정
                         //mux와OR 연산자를통해해당(n = mux) PFn pin을통해ADC함
    ADCSRA |= 0x40;       //40H => (1<<ADSC) 윗행에서설정된해당ADMUX에대한AD컨버젼시작
}          

//Functions
//UART TXD
void Putch(char data)
{
   while(!(UCSR0A & 0x20)); //송신완료flag 비트가SET되면ADC정지
   UDR0=data;               //data라는변수를UDR0 송신Register buffer에저장
}

void usart_init()           //UART 초기화
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

void ADC(void)  //ADC from Accelerometer
{
      xval = adc_data[1];
      yval = adc_data[0];
}
          
//LCD Display    
void LCD_DP(void)
{                   
                    //LCD 내용을clear하여새로운문자를받도록함
					lcd_clear();   
                    // LCD 첫행에아래와같은문자출력
                    lcd_gotoxy(0, 0);
                    lcd_putsf("X, Y, COM");
                    delay_ms(100);
                    
					// 두번째행에Accelerometer로부터받은X, Y축두개의전압을ADC하고이값을출력
                    // 그리고ADC에따른로봇의움직임을Display함           
                    lcd_gotoxy(0, 1);    
					sprintf(lcd_data, "%d, %d, %c", xval, yval, COM);
					lcd_puts(lcd_data); 
}


void main(void)
{

//Custom Register for Motor Control
//C_MCR = (0<<LMD) | (0<<RMD) | (0<<LMSP2) | (0<<LMSP1) | (0<<LMSP0) | (0<<RMSP2) | (0<<RMSP1) | (0<<RMSP0);
                
// LCD module initialization
lcd_init(16);

// Input/Output Ports initialization
// Port A initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC7) | (0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization (LCD)
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Port E initialization (USART)
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRE=(0<<DDE7) | (0<<DDE6) | (0<<DDE5) | (0<<DDE4) | (0<<DDE3) | (0<<DDE2) | (1<<DDE1) | (0<<DDE0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);

// Port F initialization (ADC)
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
// Clock value: Timer1 Stopped
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// OC1C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
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
   UCSR0A=0x00; //flag 레지스터를 사용하지 않음
   UCSR0B=0x18; //수신enable, 송신enable, 전송비트8bit
   UCSR0C=0x06; //비동기식통신
   UBRR0H=0;                      
   UBRR0L=103; //9600bps baud rate 사용 
   

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
SFIOR=(0<<ACME);


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
#asm("sei")

while (1)
      {
      ADC();    //ADC from Accelerometer  
      LCD_DP(); //Display X, Y Accelerometer & Command to LCD
      
//실질적인ADC X,Y축에따른Motor Control
if ((XBK_REF < xval && xval < XFD_REF) && (71 < yval && yval<79)) //stop
  {       
          //LMD, RMD, LMSP2, LMSP1, LMSP0, RMSP2, RMSP1, RMSP0    
          //Changing Specific Bits :
          //(0<<LMSP2) | (0<<LMSP1) | (0<<LMSP0) | (0<<RMSP2) | (0<<RMSP1) | (0<<RMSP0)
          //0b xx 000 000
    TMP = C_MCR & (0b11000000);
    C_MCR = TMP | (0b00000000);
    COM = 'S'; //LCD_Stop    
  } 
  else 
  {  
     if ((XFD_REF < xval && xval<=108) && (YBK_REF < yval && yval < YFD_REF)) //Forward
     {      
        //LM_CW  PORTA.4 = 1  
        //LM_CCW PORTA.4 = 0 
        //RM_CW  PORTA.5 = 1 
        //RM_CCW PORTA.5 = 0 
          
        //LMD, RMD, LMSP2, LMSP1, LMSP0, RMSP2, RMSP1, RMSP0    
        //Changing Specific Bits :
        //(1<<LMD) | (0<<RMD)
        //0b 10 xxx xxx      
        TMP = C_MCR & (0b00111111);
        C_MCR = TMP | (0b10000000);
        //LM_CW;  
        //RM_CCW;        
        
        
        //LMD, RMD, LMSP2, LMSP1, LMSP0, RMSP2, RMSP1, RMSP0    
        //Changing Specific Bits :
        //(0<<LMSP2) | (0<<LMSP1) | (0<<LMSP0) | (0<<RMSP2) | (0<<RMSP1) | (0<<RMSP0)
        //0b xx 000 000      
        if(xval < XFD_REF + 2) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00000000);
        //C_MCR = (0<<LMSP2) | (0<<LMSP1) | (0<<LMSP0) | (0<<RMSP2) | (0<<RMSP1) | (0<<RMSP0);
        }//XFD_REF_C = 0
        
        //0b xx 001 001          
        else if(xval < XFD_REF + 4) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00001001);
        //C_MCR = (0<<LMSP2) | (0<<LMSP1) | (1<<LMSP0) | (0<<RMSP2) | (0<<RMSP1) | (1<<RMSP0);
        }//XFD_REF_C = 1
        
        //0b xx 010 010 
        else if(xval < XFD_REF + 6) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00010010);
        //C_MCR = (0<<LMSP2) | (1<<LMSP1) | (0<<LMSP0) | (0<<RMSP2) | (1<<RMSP1) | (0<<RMSP0);
        }//XFD_REF_C = 2
         
        //0b xx 011 011
        else if(xval < XFD_REF + 8) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00011011);
        //C_MCR = (0<<LMSP2) | (1<<LMSP1) | (1<<LMSP0) | (0<<RMSP2) | (1<<RMSP1) | (1<<RMSP0); 
        }//XFD_REF_C = 3
               
        //0b xx 100 100
        else if(xval < XFD_REF + 10) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00100100);
        //C_MCR = (1<<LMSP2) | (0<<LMSP1) | (0<<LMSP0) | (1<<RMSP2) | (0<<RMSP1) | (0<<RMSP0);
        }//XFD_REF_C = 4
        
        //0b xx 101 101
        else if(xval < XFD_REF + 12) 
        {           
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00101101);
        //C_MCR = (1<<LMSP2) | (0<<LMSP1) | (1<<LMSP0) | (1<<RMSP2) | (0<<RMSP1) | (1<<RMSP0);
        }//XFD_REF_C = 5 
        
        //0b xx 110 110
        else if(xval < XFD_REF + 14) 
        {                             
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00110110);
        //C_MCR = (1<<LMSP2) | (1<<LMSP1) | (0<<LMSP0) | (1<<RMSP2) | (1<<RMSP1) | (0<<RMSP0);
        }//XFD_REF_C = 6
                        
        //0b xx 111 111
        else if(xval < XFD_REF + 16) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00111111);
        //C_MCR = (1<<LMSP2) | (1<<LMSP1) | (1<<LMSP0) | (1<<RMSP2) | (1<<RMSP1) | (1<<RMSP0); 
        }//XFD_REF_C = 7
       
     COM = 'F'; //LCD_Forward    
     }      
     
     else if ((0 < xval && xval<=XBK_REF) && (YBK_REF < yval && yval < YFD_REF)) //Backward
     {          
        
        //LMD, RMD, LMSP2, LMSP1, LMSP0, RMSP2, RMSP1, RMSP0    
        //Changing Specific Bits :
        //(0<<LMD) | (1<<RMD)
        //0b 01 xxx xxx      
        TMP = C_MCR & (0b00111111);
        C_MCR = TMP | (0b01000000);
        
        
        //LMD, RMD, LMSP2, LMSP1, LMSP0, RMSP2, RMSP1, RMSP0    
        //Changing Specific Bits :
        //(0<<LMSP2) | (0<<LMSP1) | (0<<LMSP0) | (0<<RMSP2) | (0<<RMSP1) | (0<<RMSP0)
        //0b xx 000 000      
        if(xval > XBK_REF - 2) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00000000);
        //C_MCR = (0<<LMSP2) | (0<<LMSP1) | (0<<LMSP0) | (0<<RMSP2) | (0<<RMSP1) | (0<<RMSP0);
        }//XFD_REF_C = 0
        
        //0b xx 001 001          
        else if(xval > XBK_REF - 4) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00001001);
        //C_MCR = (0<<LMSP2) | (0<<LMSP1) | (1<<LMSP0) | (0<<RMSP2) | (0<<RMSP1) | (1<<RMSP0);
        }//XFD_REF_C = 1
        
        //0b xx 010 010 
        else if(xval > XBK_REF - 6) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00010010);
        //C_MCR = (0<<LMSP2) | (1<<LMSP1) | (0<<LMSP0) | (0<<RMSP2) | (1<<RMSP1) | (0<<RMSP0);
        }//XFD_REF_C = 2
         
        //0b xx 011 011
        else if(xval > XBK_REF - 8) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00011011);
        //C_MCR = (0<<LMSP2) | (1<<LMSP1) | (1<<LMSP0) | (0<<RMSP2) | (1<<RMSP1) | (1<<RMSP0); 
        }//XFD_REF_C = 3
               
        //0b xx 100 100
        else if(xval > XBK_REF - 10) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00100100);
        //C_MCR = (1<<LMSP2) | (0<<LMSP1) | (0<<LMSP0) | (1<<RMSP2) | (0<<RMSP1) | (0<<RMSP0);
        }//XFD_REF_C = 4
        
        //0b xx 101 101
        else if(xval > XBK_REF - 12) 
        {           
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00101101);
        //C_MCR = (1<<LMSP2) | (0<<LMSP1) | (1<<LMSP0) | (1<<RMSP2) | (0<<RMSP1) | (1<<RMSP0);
        }//XFD_REF_C = 5 
        
        //0b xx 110 110
        else if(xval > XBK_REF - 14) 
        {                             
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00110110);
        //C_MCR = (1<<LMSP2) | (1<<LMSP1) | (0<<LMSP0) | (1<<RMSP2) | (1<<RMSP1) | (0<<RMSP0);
        }//XFD_REF_C = 6
                        
        //0b xx 111 111
        else if(xval > XBK_REF - 16) 
        {
        TMP = C_MCR & (0b11000000);
        C_MCR = TMP | (0b00111111);
        //C_MCR = (1<<LMSP2) | (1<<LMSP1) | (1<<LMSP0) | (1<<RMSP2) | (1<<RMSP1) | (1<<RMSP0); 
        }//XFD_REF_C = 7 
        
      //M_Speed = ((XBK_REF_C + 1) * 32) - 1; 
      //8 * 32 - 1= 256 - 1 = 255                    
      //To achieve 0 ~ 255 8bit  OCR1n PWM maximum speed range, negative value(= -1) required
      //M_PWM(M_Speed, M_Speed);
      //R_BK();      
      //PORTA = 0x02;
     COM = 'B'; //LCD_Backward  
     }   
     
     else if ((XBK_REF < xval && xval < XFD_REF) && (108 > yval && yval >= YFD_REF)) //Right
     {     
        //LM_CCW;  
        //RM_CCW;   
                                 
        //LMD, RMD, LMSP2, LMSP1, LMSP0, RMSP2, RMSP1, RMSP0    
        //Changing Specific Bits :
        //C_MCR = (0<<LMD) | (0<<RMD) | (0<<LMSP2) | (x<<LMSP1) | (x<<LMSP0) | (x<<RMSP2) | (x<<RMSP1) | (x<<RMSP0);
        //0b 00 011 011 
        C_MCR = 0b00011011;
        //LM_CW  PORTA.4 = 1  
        //LM_CCW PORTA.4 = 0 
        //RM_CW  PORTA.5 = 1 
        //RM_CCW PORTA.5 = 0  
        if(yval > YFD_REF + 2) C_MCR = 0b00100100;
        else if(yval > YFD_REF + 4) C_MCR = 0b00101101;  

      //M_PWM(150, 150);
      //R_LT();     
      //PORTA = 0x04;   
     COM = 'R'; //LCD_Turn_Right    
     }          
    
     else if ((XBK_REF < xval && xval < XFD_REF) && (0 < yval && yval <= YBK_REF)) //Left   
     {
    //LM_CW;  
    //RM_CW;          
    
        //LMD, RMD, LMSP2, LMSP1, LMSP0, RMSP2, RMSP1, RMSP0    
        //Changing Specific Bits :
        //C_MCR = (1<<LMD) | (1<<RMD) | (x<<LMSP2) | (x<<LMSP1) | (x<<LMSP0) | (x<<RMSP2) | (x<<RMSP1) | (x<<RMSP0);
        //0b 11 011 011 
        C_MCR = 0b11011011;  
    //LM_CW  PORTA.4 = 1  
    //LM_CCW PORTA.4 = 0 
    //RM_CW  PORTA.5 = 1 
    //RM_CCW PORTA.5 = 0
        if(yval < YBK_REF - 2) C_MCR = 0b11100100;
        else if(yval < YBK_REF - 4) C_MCR = 0b11101101;  
    
      //M_PWM(150, 150);
      //R_RT();        
      //PORTA = 0x08; 
     COM = 'L'; //LCD_Turn_Left    
     }   
  }
         
      //C_MCR = (0<<LMD) | (0<<RMD) | (0<<LMSP2) | (0<<LMSP1) | (1<<LMSP0) | (0<<RMSP2) | (1<<RMSP1) | (0<<RMSP0);
              
      Putch(C_MCR); //Bluetooth 8bit Transmit            
      delay_ms(100);     
      }
}
