/*
Software for performing acousto-optic imaging using triggerable pulsed laser and an ultrasound scanner. Determines trigger to emission delay and adjusts delay of US pulses accordingly.
For details, please refer to the following article:
https://doi.org/10.1364/BOE.444270
*/

//***************************************************************************************************************************************
// Defined values
//***************************************************************************************************************************************
#define usTriggerIn 2
#define pdTriggerIn 3
#define camTriggerOut 4   //camera/US trigger
#define usSyncOut 5   //synchronization signal for the US scanner
#define LASDEL_BUFLEN 100 //size of buffer for delay times

#define F_CPU 16000000UL // the CPU Frequency
#define USART_BAUDRATE 9600 // Desired Baud Rate
#define BAUD_PRESCALER (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define ASYNCHRONOUS (0<<UMSEL00) // USART Mode Selection

#define DISABLED    (0<<UPM00)
#define EVEN_PARITY (2<<UPM00)
#define ODD_PARITY  (3<<UPM00)
#define PARITY_MODE  DISABLED // USART Parity Bit Selection

#define ONE_BIT (0<<USBS0)
#define TWO_BIT (1<<USBS0)
#define STOP_BIT ONE_BIT      // USART Stop Bit Selection

#define FIVE_BIT  (0<<UCSZ00)
#define SIX_BIT   (1<<UCSZ00)
#define SEVEN_BIT (2<<UCSZ00)
#define EIGHT_BIT (3<<UCSZ00)
#define DATA_BIT   EIGHT_BIT  // USART Data Bit Selection

#define FIXED_US_DELAY  205 *2  //assumed fixed delay for US scanner in us - the microcontroller will determine current laser delay, and adjust it to have 200 us head start by the US scanner, times 2 for 2 MHz timer 0 freq




//***************************************************************************************************************************************
// Constants and variables
//***************************************************************************************************************************************
//Constants - transmission codes:
const uint8_t startCode = 96;   //sequence to receive, command to start the triggering and measurements
const uint8_t stopCode = 69;   //sequence to receive, indicating that the DAQ procedure has finished
const uint16_t endOfDataCode = 21845; //code to send via USART, indicating the end of the transmitted data



//Global variables:
//FLAGS MUST BE DECLARED AS VOLATILE!!!!!!
volatile bool usflag = false;
volatile bool newDataFlag = false;   //flag indicating that new delay data were stored in the delayBuff
volatile bool enableUS = false;  //enable US triggering
byte timLowByte = 0;
byte timHighByte = 0;
int countSavedBytes = 0; //count how many bytes were saved (time stamps data)
uint16_t delayTimes[LASDEL_BUFLEN];  //array with recorded delay times (2 bytes each = uint16)
volatile uint16_t delayBuff = 0;   //buffer for storing the most recent results of measurements
uint16_t delayCorr = 0;   //calculated delay correction value
uint8_t ppt = 0;    //pulses per trigger - number of laser trigger pulses illuminating the frame; to be sent from the main control unit before every single frame
int licz = 0; //universal loop counter







//***************************************************************************************************************************************
// FUNCTIONS AND INTERRUPT ROUTINES
//***************************************************************************************************************************************
//Receive byte from USART:
uint8_t USART_Receive()
{
  uint8_t DataByte;
  while(!(UCSR0A & (1<<RXC0))) {}; //wait for data, do nothing
  DataByte = UDR0;
  return DataByte;
}


//Transmit byte via USART:
void USART_Transmit(uint8_t DataByte)
{
  while(!(UCSR0A & (1 << UDRE0)));  //wait for the transmit buffer to be empty
  UDR0 = DataByte;
}



//When trigger from US scanner was detected, start counting:
void usTriggered() {
  TCNT1H  = 0;//clear timer counter - first high byte, goes to temp reg
  TCNT1L  = 0;//clear timer counter - next write the low byte
  //Enable timer, no prescaling:
  TCCR1B |= (1 << CS10);
  if(enableUS){
    TCNT0  = 0;//initialize counter value to 0
    TCCR0B |= (1<<CS01);  // enable timer 0 with prescaler 8 -> 2 clock cycles per us 
  }
  usflag = true;
  PORTD &= ~(1 << usSyncOut);  //set US sync output to low state - it will go HIGH again by the timer 0 interrupt
}




//When trigger from photodiode was detected, stop counting and transmit data to serial port:
void pdTriggered() {
    if(usflag){ //data are transmitted only if laser trigger was detected first
    // Disable timers:
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); //disable timer 1
    timLowByte = TCNT1L;
    timHighByte = TCNT1H;
    delayBuff = ( timHighByte << 8 |  timLowByte);  //store the determined delay value
    EIFR = EIFR  & B11111100; //clear external interrupt flags
    usflag = false;
    newDataFlag = true;
  
  }
}

//Timer 0 CTC A interrupt - trigger US after the specified compensation time:
ISR(TIMER0_COMPA_vect){  
   
   PORTD |= (1 << usSyncOut);
   TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); //disable timer 0
}







//***************************************************************************************************************************************
// SETUP
//***************************************************************************************************************************************
void setup() {
//*******************************************************************************
//                     Configure inputs
//*******************************************************************************
//Set outputs:
DDRD |= (1 << camTriggerOut) | (1 << usSyncOut);
//initialize with LOW outputs:
PORTD &= ~((1 << camTriggerOut) | (1 << usSyncOut));
//*******************************************************************************




//*******************************************************************************
//           Initialize timer 0 - compensate variable laser delay component
//*******************************************************************************
cli();
  TCCR0A = 0; // clear timer 0 settings
  TCCR0B = 0; // disable timer 0 at start: &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); 
  TCNT0 = 0;  //clear timer 0
  
  TCCR0A |= (1<<WGM01); //set timer 0 to CTC mode
  
  TIMSK0 |= (1 << OCIE0A); //enable timer 0 CTC A interrupt
sei();
//*******************************************************************************


    
//*******************************************************************************
//                     Initialize timer 1 - measure laser pulse to trigger delay
//*******************************************************************************
cli();  //disable interrupts
//use 16. bit Timer 1
  TCCR1A = 0;// clear timer settings
  TCCR1B = 0;// 
  TCNT1  = 0;//clear timer counter
  // Disable timer:
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
sei();//enable interrupts
//*******************************************************************************


//*******************************************************************************
//                     Configure interrupts
//*******************************************************************************
cli();
attachInterrupt(digitalPinToInterrupt(usTriggerIn), usTriggered, RISING);     //rising slope for the 300 us laser trigger in
attachInterrupt(digitalPinToInterrupt(pdTriggerIn), pdTriggered, FALLING);    //falling slope from the photodiode signal conditioner output
EIFR = EIFR  & B11111100; //clear external interrupt flags
sei();
//*******************************************************************************

//*******************************************************************************
//       Configure UART for serial communication with Matlab
//*******************************************************************************
  cli();
  // Set Baud Rate
  UBRR0H = BAUD_PRESCALER >> 8;
  UBRR0L = BAUD_PRESCALER;
  // Set Frame Format
  UCSR0C = ASYNCHRONOUS | PARITY_MODE | STOP_BIT | DATA_BIT;
  // Enable Receiver and Transmitter
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);
  sei();

}









//***************************************************************************************************************************************
// MAIN LOOP
//***************************************************************************************************************************************
void loop() {
  cli();
  EIFR = EIFR  & B11111100; //clear external interrupt flags
  //clear buffers and counters
  for(licz=0 ; licz < LASDEL_BUFLEN ; licz++) delayTimes[licz]=0; //clear delay buffer
  countSavedBytes = 0;  //clear data counter


  //wait for transmission to start:
  
  while(USART_Receive() != startCode) {};  //wait for the start command from the main control unit
  sei();
  ppt = USART_Receive();  //read number of laser pulses illuminating the camera frame to be captured
  newDataFlag = false;
  while(!newDataFlag){};  //wait for the first delay value
  delayCorr = (uint8_t)(delayBuff >> 3); //divide by 8 (bitshift 3) to adjust for different prescaller settings in tim0 and tim1
  delayCorr -= FIXED_US_DELAY;
  OCR0A = delayCorr;    //set the delay adjustment for US trigger
  enableUS = true; //next time the laser is triggered - the US will be triggered as well, with the adjusted delay value
  PORTD |= (1 << camTriggerOut); //trigger the camera - ACQUISITION STARTS HERE

  
    
  for(licz=0 ; licz < ppt ; licz++) {
    newDataFlag = false;
    while(!newDataFlag){};  //wait for the new delay value
    delayCorr = delayBuff >> 3; //divide by 8 (bitshift 3) to adjust for different prescaller settings in tim0 and tim1
    delayCorr -= FIXED_US_DELAY;
    OCR0A = (uint8_t)delayCorr;
    enableUS = true; 
    delayTimes[licz] = delayBuff; //store the measured laser pulse to trigger delay
    countSavedBytes++;
  }
  enableUS = false; //stop triggering the US scanner, as it counts subsequent imaging points; laser may run, camera is off by now anyway
  



  
  while(USART_Receive() != stopCode) {};  //wait for the confirmation that data acquisition has finished
  PORTD &= ~(1 << camTriggerOut);  //release the camera trigger, prepare for subsequent acquisition
  for(licz = 0 ; licz < countSavedBytes ; licz++)  //transmit the captured delay times
  {
      USART_Transmit((uint8_t)(delayTimes[licz] >> 8)); //transmit high byte first
      USART_Transmit((uint8_t)(delayTimes[licz] & 0xFF)); //...followed by the low byte
  }
  //send the transmission end code:
  USART_Transmit((uint8_t)(endOfDataCode >> 8)); //transmit high byte first
  USART_Transmit((uint8_t)(endOfDataCode & 0xFF)); //...followed by the low byte
  //back to beginning, wait for the new transmission start signal
}
