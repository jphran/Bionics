/*
Auth: Justin Francis
Owner: BE Lab
Created: 8/14/19
Edited:

Sum: Attempt at SPI comms to rpi and bit banging 3-wire-SPI on the dsPIC33FJ64MC802 using pins RB0, RB1, RB2
*/


//***************************PREPROCESSOR****************************
#include <p33FJ64MC802.h>


//***************************INIT****************************
// Configuration Bit Settings
_FOSCSEL(FNOSC_FRC) // use internal FRC
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON) // Clock switching enabled and OSC2 pin is IO
_FWDT(FWDTEN_OFF) // watchdog timer controlled in software
_FPOR(FPWRT_PWR128) // power on reset timer value
_FICD(ICS_PGD3 & JTAGEN_OFF) // using PGD3 programming pins and no jtag

void init_SPI(void);
void write_SPI(short);

//from /Downloads/dsPIC33_BitBang.c
bool sdio_set(bool);
void clock_pulse(void);
uint8_t read(void);
void write(uint8_t);
void jDelay(int);

const int FSYS = 120e6; //system frequency


//***************************MAIN****************************
int main() {

  // Configure PLL prescaler, PLL postscaler, PLL divisor
  PLLFBD = 43;
  CLKDIVbits.PLLPOST = 0; // N2 = 2
  CLKDIVbits.PLLPRE = 0; // N1 = 2

  // Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
  __builtin_write_OSCCONH(0x01);
  __builtin_write_OSCCONL(0x01);

  // Wait for Clock switch to occur
  while (OSCCONbits.COSC != 0b001);

  // Wait for PLL to lock
  while (OSCCONbits.LOCK != 1) {
  };

  // unlock to set the pins
  __builtin_write_OSCCONL(OSCCON & (~(1 << 6))); // clear bit 6

  // pin select for SPI1
  RPINR20bits.SCK1R = 0b01100; // SCK1 Pin Select (RP12)
  RPINR20bits.SDI1R = 0b01101; // SDI1 Pin Select (RP13)
  RPOR7bits.RP14R = 0b00111; // SDO1 Pin Select (RP14)
  RPINR21bits.SS1R = 0b01111; // SS1 Pin Select (RP15)

  // pin select for OC1
  RPOR1bits.RP3R = 0b10010; // OC1 on B3

  // lock pins
  __builtin_write_OSCCONL(OSCCON | (1 << 6)); // Set bit 6

  // turn off ADC pins
  AD1PCFGL = 0xFFFF;

  init_SPI();

  // Put a 50% 1kHz PWM on B3 to use as a heartbeat and know the board is working
  // Initialize Output Compare Module
  OC1CONbits.OCM = 0b000; // Disable Output Compare Module
  OC1R = 2500; // Write the duty cycle for the first PWM pulse
  OC1RS = 2500; // Write the duty cycle for the second PWM pulse
  OC1CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
  OC1CONbits.OCM = 0b110; // Select the Output Compare mode

  // Initialize and enable Timer2
  T2CONbits.TON = 0; // Disable Timer
  T2CONbits.TCS = 0; // Select internal instruction cycle clock
  T2CONbits.TGATE = 0; // Disable Gated Timer mode
  T2CONbits.TCKPS = 0b01; // Select 1:8 Prescaler
  TMR2 = 0x00; // Clear timer register
  PR2 = 5000; // Load the period value, so 40000000/8/5000 = 1000hz
  T2CONbits.TON = 1; // turn on timer2

  short inData;
  uint8_t pos;


  //from /Downloads/dsPIC33_BitBang.c
  TRISBbits.TRISB0 = 0; //set up RB0 as output for sck
  TRISBbits.TRISB1 = 0; //set up RB1 as output for cs

  LATBbits.LATB1 = 1; //set cs high 

  //init vars
  uint8_t read_data = 0x00;
  int enc_pos;
  int enc_pos1;
  int enc_pos2;

  while (1) {
    pos = read();
    SPI1BUF = pos;
    while (!SPI1STATbits.SPITBF); // wait for the data to be sent out
  }

  return 0; //simply unexplainable by science
}



//******************************FN*********************************

bool sdio_set(bool isINPUT)
{
  if(isINPUT)
  {
    TRISBbits.TRISB2 = 1; //set RB2 as input for sdi
    NOP; //delay one cycle
  }
  else
  {
    TRISBbits.TRISB2 = 0; //set RB2 as output for sdo
    NOP; //delay one cycle
  }
  return true;
}

//custom microsecond timer
void jDelay(int microseconds)
{
  static float delay = (microseconds * 10e-6) * (FSYS/2);

  WriteCoreTimer(0);
  while(ReadCoreTimer() < delay)
  {
    //wait in empty block
  }
}

//pulses the RB0 high then low
void clock_pulse(void)
{ //float delayTime = 1/freq;
  LATBbits.LATB0 = 1;
  jDelay(5);
  LATBbits.LATB0 = 0;
  jDelay(5);
}


uint8_t read(void)
{
  sdio_set(true); //set rb2 to input
  int idx = 0; 
  static uint8_t in_data = 0x00;

  //Loop through all 16 bits or 2 bytes at once
  for (idx = 0; idx < 8; idx++) {
    in_data <<= 1; //Bit shift the data left
    in_data |= PORTBbits.PORTB2; //Store the incoming data from the sensor
    clock_pulse();
  }

  jDelay(15);

  return in_data;
}

void write(uint8_t outData)
{
  uint8_t i;
  bool sensor_val;
  LATBbits.LATB1 = 0; //Set Chip select low to indicate data transmission
  sdio_set(false); //Configure DIO to be an output

  //Loop through the byte, bit by bit
  for (i = 0; i < 8; i++) {
    sensor_val = ((out_data & 0x80) ? HIGH : LOW); //Bitwise "and" to
    //generate an output, if the "and" results in > 0, sensor value is set high
    LATBbits.LATB2 = sensor_val; //Output sensor value to sensor
    out_data <<= 1; //Bit shift the data to write to the sensor
  clock_pulse();
  }
  jDelay(15);
}

void init_SPI(void) {
  // CN pins are shared, turn them off
  CNEN1 = 0;
  CNEN2 = 0;

  SPI1BUF = 0;
  IFS0bits.SPI1IF = 0; // Clear the Interrupt Flag
  IEC0bits.SPI1IE = 0; // Disable the Interrupt
  SPI1CON1bits.DISSCK = 0; // Internal Serial Clock is Enabled
  SPI1CON1bits.DISSDO = 0; // SDOx pin is controlled by the module
  SPI1CON1bits.SSEN = 1; // use the slave select pin
  SPI1CON1bits.MODE16 = 1; // Communication is 16 bit
  SPI1CON1bits.SMP = 0; // Input data is sampled at the middle of data
  SPI1CON1bits.CKE = 0; // Serial output data changes on transition
  SPI1CON1bits.CKP = 0; // Idle state for clock is a low level; active
  SPI1CON1bits.MSTEN = 0; // Master mode Disabled
  SPI1STATbits.SPIROV = 0; // No Receive Overflow has occurred
  SPI1STATbits.SPIEN = 1; // Enable SPI module
  // Interrupt Controller Settings
  //IFS0bits.SPI1IF = 0; // Clear the Interrupt Flag
  //IEC0bits.SPI1IE = 1; // Enable the Interrupt
}






