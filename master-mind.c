/*
 * MasterMind implementation: template; see comments below on which parts need to be completed
 * CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
 * This repo: https://gitlab-student.macs.hw.ac.uk/f28hs-2021-22/f28hs-2021-22-staff/f28hs-2021-22-cwk2-sys

 * Compile: 
 gcc -c -o lcdBinary.o lcdBinary.c
 gcc -c -o master-mind.o master-mind.c
 gcc -o master-mind master-mind.o lcdBinary.o
 * Run:     
 sudo ./master-mind

 OR use the Makefile to build
 > make all
 and run
 > make run
 and test
 > make test

 ***********************************************************************
 * The Low-level interface to LED, button, and LCD is based on:
 * wiringPi libraries by
 * Copyright (c) 2012-2013 Gordon Henderson.
 ***********************************************************************
 * See:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
*/

/* ======================================================= */
/* SECTION: includes                                       */
/* ------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <unistd.h>
#include <string.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
//TD
#define _GNU_SOURCE
/* --------------------------------------------------------------------------- */
/* Config settings */
/* you can use CPP flags to e.g. print extra debugging messages */
/* or switch between different versions of the code e.g. digitalWrite() in Assembler */
#define DEBUG
#undef ASM_CODE

// =======================================================
// Tunables
// PINs (based on BCM numbering)
// For wiring see CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
// GPIO pin for green LED
#define LED 13
// GPIO pin for red LED
#define LED2 5
// GPIO pin for button
#define BUTTON 19
// =======================================================
// delay for loop iterations (mainly), in ms
// in mili-seconds: 0.2s
#define DELAY   200
// in micro-seconds: 3s
#define TIMEOUT 3000000
// =======================================================
// APP constants   ---------------------------------
// number of colours and length of the sequence
#define COLS 3
#define SEQL 3
// =======================================================
//====================== GENERAL CONSTANTS ======================


#define BUFFER 100 //Buffer value for any arbitrary length
// generic constants

#ifndef	TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(1==2)
#endif

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define	INPUT			 0
#define	OUTPUT			 1

#define	LOW			 0
#define	HIGH			 1


static unsigned int gpiobase;
static uint32_t *gpio;
// =======================================================
// Wiring (see inlined initialisation routine)

#define STRB_PIN 24
#define RS_PIN   25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

/* ======================================================= */
/* SECTION: constants and prototypes                       */
/* ------------------------------------------------------- */

// =======================================================
// char data for the CGRAM, i.e. defining new characters for the display

static unsigned char newChar [8] = 
{
  0b11111,
  0b10001,
  0b10001,
  0b10101,
  0b11111,
  0b10001,
  0b10001,
  0b11111,
} ;

/* Constants */

static const int colors = COLS;
static const int seqlen = SEQL;

static char* color_names[] = { "red", "green", "blue" };

static int* theSequence = NULL;

static int *seq1, *seq2, *cpy1, *cpy2;
//TD
static int timeOut = 0;
static int valueCount = 0;
static int presses = 0;
static int waitingStat = 0;

static int lcdControl;

/* --------------------------------------------------------------------------- */

// data structure holding data on the representation of the LCD
struct lcdDataStruct
{
    int bits;
    int rows;
    int cols;
    int rsPin, strbPin;
    int dataPins[8];
    int cx, cy;
} ;

static int lcdControl ;

/* ***************************************************************************** */
/* INLINED fcts from wiringPi/devLib/lcd.c: */
// HD44780U Commands (see Fig 11, p28 of the Hitachi HD44780U datasheet)

#define	LCD_CLEAR	0x01
#define	LCD_HOME	0x02
#define	LCD_ENTRY	0x04
#define	LCD_CTRL	0x08
#define	LCD_CDSHIFT	0x10
#define	LCD_FUNC	0x20
#define	LCD_CGRAM	0x40
#define	LCD_DGRAM	0x80

// Bits in the entry register

#define	LCD_ENTRY_SH		0x01
#define	LCD_ENTRY_ID		0x02

// Bits in the control register

#define	LCD_BLINK_CTRL		0x01
#define	LCD_CURSOR_CTRL		0x02
#define	LCD_DISPLAY_CTRL	0x04

// Bits in the function register

#define	LCD_FUNC_F	0x04
#define	LCD_FUNC_N	0x08
#define	LCD_FUNC_DL	0x10

#define	LCD_CDSHIFT_RL	0x04
//TD
// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define	PI_GPIO_MASK	(0xFFFFFFC0)
static unsigned int gpiobase ;
static uint32_t *gpio ;
//helper function
int *splitDigits(int code);

/* ------------------------------------------------------- */
// misc prototypes

int failure (int fatal, const char *message, ...);
//void waitForEnter (void);
void waitForButton (uint32_t *gpio, int button);

/* ======================================================= */
/* SECTION: hardware interface (LED, button, LCD display)  */
/* ------------------------------------------------------- */
/* low-level interface to the hardware */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Either put them in a separate file, lcdBinary.c, and use   */
/* inline Assembler there, or use a standalone Assembler file */
/* You can also directly implement them here (inline Asm).    */
/* ********************************************************** */

/* These are just prototypes; you need to complete the code for each function */

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
void digitalWrite (uint32_t *gpio, int pin, int value){
  int offset;
  int output;
  // register number for GPCLR/GPSET
  offset = (value == LOW) ? 10 : 7;

    asm volatile(
      //loading values into registers 
        "\tLDR R1, %[gpio]\n"
        "\tADD R0, R1, %[offset]\n"
        "\tMOV R2, #1\n"
        "\tMOV R1, %[pin]\n"

      //(pin & 31)
        "\tAND R1, #31\n"
      // 1 << (pin & 31)
        "\tLSL R2, R1\n"

        "\tSTR R2, [R0, #0]\n"
        "\tMOV %[output], R2\n"

        : [output] "=r"(output)
        : [pin] "r"(pin) , [gpio] "m"(gpio) , [offset] "r"(offset * 4)
        : "r0", "r1", "r2", "cc"
    );
}

/* set the @mode@ of a GPIO @pin@ to INPUT or OUTPUT; @gpio@ is the mmaped GPIO base address */
void pinMode(uint32_t *gpio, int pin, int mode){
    int fSel = pin / 10;
    int shift = (pin % 10) * 3;
    int output;

    if(mode == OUTPUT) {
        asm(
        //loading values into registers
        "\tLDR R1, %[gpio]\n"
        //*(gpio + fSel)
        "\tADD R0, R1, %[fSel]\n"
        "\tLDR R1, [R0, #0]\n"
        //moving binary value of 7 into register 
        "\tMOV R2, #0b111\n"

        //(7 << shift)
        "\tLSL R2, %[shift]\n"
        "\tBIC R1, R1, R2\n"
        //(mode << shift)
        "\tMOV R2, #1\n"
        "\tLSL R2, %[shift]\n"
        //(*(gpio + fSel) & ~(7 << shift)) || (mode << shift)
        "\tORR R1, R2\n"
        "\tSTR R1, [R0, #0]\n"
        "\tMOV %[output], R1\n"
        : [output] "=r"(output)
        : [pin] "r"(pin) , [gpio] "m"(gpio) , [fSel] "r"(fSel * 4), [shift] "r"(shift)
        : "r0", "r1", "r2", "cc"
        );
    }

    else if(mode == INPUT) {
        asm(
        //loading values into registers
        "\tLDR R1, %[gpio]\n"
        //*(gpio + fSel)
        "\tADD R0, R1, %[fSel]\n"
        "\tLDR R1, [R0, #0]\n"
        //moving binary value of 7 into register 
        "\tMOV R2, #0b111\n"


        //(7 << shift)
        "\tLSL R2, %[shift]\n"
        "\tBIC R1, R1, R2\n"
        "\tSTR R1, [R0, #0]\n"
        "\tMOV %[output], R1\n"
        : [output] "=r"(output)
        : [pin] "r"(pin) , [gpio] "m"(gpio) , [fSel] "r"(fSel * 4), [shift] "r"(shift)
        : "r0", "r1", "r2", "cc"
        );
    }

    else {
        fprintf(stderr, "Invalid PIN Mode");
    }
}

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
/* can use digitalWrite(), depending on your implementation */

void writeLED(uint32_t *gpio, int led, int value){
  int offset, output;
    
    if(value == LOW) {
        offset = 10;
        asm volatile(
          //loading values into registers 
            "\tLDR R1, %[gpio]\n"
          //*(gpio + 7)
            "\tADD R0, R1, %[offset]\n"
          //loading values into registers    
            "\tMOV R2, #1\n"
            "\tMOV R1, %[pin]\n"
          //(led & 31)  
            "\tAND R1, #31\n"
          //1 << (led & 31)  
            "\tLSL R2, R1\n"
            "\tSTR R2, [R0, #0]\n"
            "\tMOV %[output], R2\n"
            : [output] "=r"(output)
            : [pin] "r"(led) , [gpio] "m"(gpio) , [offset] "r"(offset * 4)
            : "r0", "r1", "r2", "cc"
        );
    } else {
        offset = 7;
        asm volatile(
           //loading values into registers
            "\tLDR R1, %[gpio]\n"
            //*(gpio + 10)
            "\tADD R0, R1, %[offset]\n"
            //loading values into registers
            "\tMOV R2, #1\n"
            "\tMOV R1, %[pin]\n"
            //(led & 31)
            "\tAND R1, #31\n"
            //1 << (led & 31)
            "\tLSL R2, R1\n"
            "\tSTR R2, [R0,#0]\n"
            "\tMOV %[output], R2\n"
            : [output] "=r"(output)
            : [pin] "r"(led), [gpio] "m"(gpio), [offset] "r"(offset * 4)
            : "r0", "r1", "r2", "cc"
        );
    }
}

/* read a @value@ (LOW or HIGH) from pin number @pin@ (a button device); @gpio@ is the mmaped GPIO base address */
int readButton(uint32_t *gpio, int button){
   int status = 0;
    asm volatile(
      //loading values into registers 
        "MOV R1, %[gpio]\n"
        "LDR R2, [R1, #0x34]\n"
        "MOV R3, %[pin]\n"
      //(1 << button)  
        "MOV R4, #1\n"
        "LSL R4, R3\n"
        "AND %[state], R2, R4\n"
        : [state] "=r"(status)
        : [pin] "r"(button) , [gpio] "r"(gpio)
        : "r0", "r1", "r2", "r3", "r4", "cc"
    );
    
    return status > 0;
}

/* wait for a button input on pin number @button@; @gpio@ is the mmaped GPIO base address */
/* can use readButton(), depending on your implementation */
void waitForButton (uint32_t *gpio, int button){
  for(int counter=0 ; counter<13 ; counter++) {
        if(readButton(gpio, button)) {
            break;
        }
        delay(DELAY);
    }
}

/* ======================================================= */
/* SECTION: game logic                                     */
/* ------------------------------------------------------- */
/* AUX fcts of the game logic */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* initialise the secret sequence; by default it should be a random sequence */
void initSeq() {

  srand(time(NULL));
  theSequence = (int *)malloc(seqlen * sizeof(int));
    if (theSequence == NULL) {
        printf("Array is null!");
        exit(0);
    } else {
        for (int i = 0; i < seqlen; ++i) {
            theSequence[i] = rand() % 3 + 1;
        }
    }
}

/* display the sequence on the terminal window, using the format from the sample run in the spec */
void showSeq(int *seq) {
  printf("The sequence is: ");
    for (int i = 0; i < seqlen; ++i) {
        switch (theSequence[i]) {
        case 1:
            printf("R ");
            break;
        case 2:
            printf("G ");
            break;
        case 3:
            printf("B ");
            break;
        }
    }
    printf("\n");
}

#define NAN1 8
#define NAN2 9

/* counts how many entries in seq2 match entries in seq1 */
/* returns exact and approximate matches, either both encoded in one value, */
/* or as a pointer to a pair of values */
int /* or int* */ countMatches(int *seq1, int *seq2) {
  int *check = (int *)malloc(seqlen * sizeof(int));
    int exact = 0;
    int approx = 0;
    
    for (int checkCount=0 ; checkCount<seqlen ; checkCount++) {
        check[checkCount] = 0;
    }

    //for caluculating exact matches    
    for (int exactCount=0 ; exactCount<seqlen ; exactCount++) {
        if (seq2[exactCount] == seq1[exactCount]) {
            check[exactCount] = 1;
            exact++;
        }
    }

    for (int outerACount=0 ; outerACount<seqlen ; outerACount++) {
        if (seq2[outerACount] == seq1[outerACount]) {
            continue;
        } else {
            for (int innerACount=0 ; innerACount<seqlen ; innerACount++) {
                if (!check[innerACount] && innerACount!=outerACount && seq2[outerACount]==seq1[innerACount]) {
                    approx++;
                    check[innerACount] = 1;
                    break;
                }
            }
        }
    }
    //deallocates memory assigned 
    free(check);

    int ret = concat(exact, approx);
    return ret;
}

/* show the results from calling countMatches on seq1 and seq1 */
void showMatches(int /* or int* */ code, /* only for debugging */ int *seq1, int *seq2, /* optional, to control layout */ int lcd_format) {
  int index = 0;
    int *temp = splitDigits(code);
    int approx = temp[0];
    int correct = temp[1];
    printf("%d exact\n", correct);
    printf("%d approximate\n", approx);
    //deallocates memory assigned
    free(temp);
}

/* parse an integer value as a list of digits, and put them into @seq@ */
/* needed for processing command-line with options -s or -u            */
void readSeq(int *seq, int val) {
  int index = 0; 
  // Extracts digits from a number and stores them in an array, up to a maximum length

    for (; val != 0 && index < seqlen; ++index, val /= 10) {
    seq[index] = val % 10;
}


    reverse(seq, 0, seqlen - 1);
}

/* read a guess sequence fron stdin and store the values in arr */
/* only needed for testing the game logic, without button input */
int *readNum(int max) {
  int index = 0;
    
    int *arr = (int *)malloc(seqlen * sizeof(int));
    if (!arr) {
        return NULL;
    }

    while (max != 0 && index < SEQL) {
        arr[index] = max % 10;
        ++index;
        max /= 10;
    }

    reverse(arr, 0, SEQL - 1);

    return arr;
}

/* ======================================================= */
/* SECTION: TIMER code                                     */
/* ------------------------------------------------------- */
/* TIMER code */

/* timestamps needed to implement a time-out mechanism */
static uint64_t startT, stopT;

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* you may need this function in timer_handler() below  */
/* use the libc fct gettimeofday() to implement it      */
uint64_t timeInMicroseconds(){
  struct timeval tv, tNow, tLong, tEnd ;
  uint64_t now ;
  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec ; // in us
}

/* this should be the callback, triggered via an interval timer, */
/* that is set-up through a call to sigaction() in the main fct. */
void timer_handler (int signum) {
  
   static int counter = 0;
    stopT = timeInMicroseconds();
    counter++;
    startT = timeInMicroseconds();
    timeOut = 1;
}


/* initialise time-stamps, setup an interval timer, and install the timer_handler callback */
void initITimer(uint64_t timeout){
  //TD1
   struct sigaction sigAct;
    struct itimerval iTimer;

    /* Install timer_handler as the signal handler for SIGALRM. */
    memset (&sigAct, 0, sizeof(sigAct));
    sigAct.sa_handler = &timer_handler;

    sigaction (SIGALRM, &sigAct, NULL);

    /* Configure the timer to expire after 250 msec... */
    iTimer.it_value.tv_sec = timeout;
    iTimer.it_value.tv_usec = 0;
    /* ... and every 250 msec after that. */
    iTimer.it_interval.tv_sec = 0;
    iTimer.it_interval.tv_usec = 0;
    setitimer (ITIMER_REAL, &iTimer, NULL);
    
    startT = timeInMicroseconds();}

/* ======================================================= */
/* SECTION: Aux function                                   */
/* ------------------------------------------------------- */
/* misc aux functions */

int failure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal) //  && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
  vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}

/*
 * waitForEnter:
 *********************************************************************************
 */

void waitForEnter (void)
{
  printf ("Press ENTER to continue: ") ;
  (void)fgetc (stdin) ;
}

/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay (unsigned int howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}

/* From wiringPi code; comment by Gordon Henderson
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicroseconds (unsigned int howLong)
{
  struct timespec sleeper ;
  unsigned int uSecs = howLong % 1000000 ;
  unsigned int wSecs = howLong / 1000000 ;

  /**/ if (howLong ==   0)
    return ;
#if 0
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
#endif
  else
  {
    sleeper.tv_sec  = wSecs ;
    sleeper.tv_nsec = (long)(uSecs * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }
}

/* ======================================================= */
/* SECTION: LCD functions                                  */
/* ------------------------------------------------------- */
/* medium-level interface functions (all in C) */

/* from wiringPi:
 * strobe:
 *	Toggle the strobe (Really the "E") pin to the device.
 *	According to the docs, data is latched on the falling edge.
 *********************************************************************************
 */

void strobe (const struct lcdDataStruct *lcd)
{

  // Note timing changes for new version of delayMicroseconds ()
  digitalWrite (gpio, lcd->strbPin, 1) ;
  delayMicroseconds (50) ;
  digitalWrite (gpio, lcd->strbPin, 0) ;
  delayMicroseconds (50) ;
}

/*
 * sentDataCmd:
 *	Send an data or command byte to the display.
 *********************************************************************************
 */

void sendDataCmd (const struct lcdDataStruct *lcd, unsigned char data)
{
  register unsigned char myData = data ;
  unsigned char          i, d4 ;

  if (lcd->bits == 4)
  {
    d4 = (myData >> 4) & 0x0F;
    for (i = 0 ; i < 4 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
    strobe (lcd) ;

    d4 = myData & 0x0F ;
    for (i = 0 ; i < 4 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
  }
  else
  {
    for (i = 0 ; i < 8 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (myData & 1)) ;
      myData >>= 1 ;
    }
  }
  strobe (lcd) ;
}

/*
 * lcdPutCommand:
 *	Send a command byte to the display
 *********************************************************************************
 */

void lcdPutCommand (const struct lcdDataStruct *lcd, unsigned char command)
{
  digitalWrite (gpio, lcd->rsPin,   0) ;
  sendDataCmd  (lcd, command) ;
  delay (2) ;
}

void lcdPut4Command (const struct lcdDataStruct *lcd, unsigned char command)
{
  register unsigned char myCommand = command ;
  register unsigned char i ;

  digitalWrite (gpio, lcd->rsPin,   0) ;

  for (i = 0 ; i < 4 ; ++i)
  {
    digitalWrite (gpio, lcd->dataPins [i], (myCommand & 1)) ;
    myCommand >>= 1 ;
  }
  strobe (lcd) ;
}

/*
 * lcdHome: lcdClear:
 *	Home the cursor or clear the screen.
 *********************************************************************************
 */

void lcdHome (struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdHome: lcdPutCommand(%d,%d)\n", lcd, LCD_HOME);
#endif
  lcdPutCommand (lcd, LCD_HOME) ;
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
}

void lcdClear (struct lcdDataStruct *lcd)
{
  lcdPutCommand (lcd, LCD_CLEAR) ;
  lcdPutCommand (lcd, LCD_HOME) ;
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
}

/*
 * lcdPosition:
 *	Update the position of the cursor on the display.
 *	Ignore invalid locations.
 *********************************************************************************
 */

void lcdPosition (struct lcdDataStruct *lcd, int x, int y)
{
  // struct lcdDataStruct *lcd = lcds [fd] ;

  if ((x > lcd->cols) || (x < 0))
    return ;
  if ((y > lcd->rows) || (y < 0))
    return ;

  lcdPutCommand (lcd, x + (LCD_DGRAM | (y>0 ? 0x40 : 0x00)  /* rowOff [y] */  )) ;

  lcd->cx = x ;
  lcd->cy = y ;
}



/*
 * lcdDisplay: lcdCursor: lcdCursorBlink:
 *	Turn the display, cursor, cursor blinking on/offset
 *********************************************************************************
 */

void lcdDisplay (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_DISPLAY_CTRL ;
  else
    lcdControl &= ~LCD_DISPLAY_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

void lcdCursor (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_CURSOR_CTRL ;
  else
    lcdControl &= ~LCD_CURSOR_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

void lcdCursorBlink (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_BLINK_CTRL ;
  else
    lcdControl &= ~LCD_BLINK_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

/*
 * lcdPutchar:
 *	Send a data byte to be displayed on the display. We implement a very
 *	simple terminal here - with line wrapping, but no scrolling. Yet.
 *********************************************************************************
 */

void lcdPutchar (struct lcdDataStruct *lcd, unsigned char data)
{
  digitalWrite (gpio, lcd->rsPin, 1) ;
  sendDataCmd  (lcd, data) ;

  if (++lcd->cx == lcd->cols)
  {
    lcd->cx = 0 ;
    if (++lcd->cy == lcd->rows)
      lcd->cy = 0 ;
    
    // TODO: inline computation of address and eliminate rowOff
    lcdPutCommand (lcd, lcd->cx + (LCD_DGRAM | (lcd->cy>0 ? 0x40 : 0x00)   /* rowOff [lcd->cy] */  )) ;
  }
}


/*
 * lcdPuts:
 *	Send a string to be displayed on the display
 *********************************************************************************
 */

void lcdPuts (struct lcdDataStruct *lcd, const char *string)
{
  while (*string)
    lcdPutchar (lcd, *string++) ;
}

/* ======================================================= */
/* SECTION: aux functions for game logic                   */
/* ------------------------------------------------------- */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* --------------------------------------------------------------------------- */
/* interface on top of the low-level pin I/O code */

/* blink the led on pin @led@, @c@ times */
void blinkN(uint32_t *gpio, int led, int c) { 
     for(int counter=0 ; counter<c ; counter++) {
        // Turn the LED on
        writeLED(gpio, led, HIGH);
        delay(700);
        // Turn the LED off
        writeLED(gpio, led, LOW);
        delay(700);
    }
}

//helper function defined 
int* splitDigits(int code) {
    int index = 0;
    int *temp = malloc(2 * sizeof(int));
    while(code != 0 && index < seqlen) {
        temp[index] = code % 10;
        ++index;
        code /= 10;
    }
    
    return temp;
}

//function defined to format and show results of each round played on LCD
void showMatchesLCD(int matches, struct lcdStruct *lcd) {
    int* code = splitDigits(matches);
    int exact = code[1];
    int approx = code[0];
    char *temp = malloc(3);

    lcdPosition(lcd, 0, 0);
        lcdPuts(lcd, "Exact : ");
    lcdPosition(lcd, 9, 0);
        sprintf(temp, "%d", exact);
        lcdPuts(lcd, temp);
    
    blinkN(gpio, LED, exact);
    
    blinkN(gpio, LED2, 1);
    
    lcdPosition(lcd, 0, 1);
        lcdPuts(lcd, "Approx: ");
    lcdPosition(lcd, 9, 1);
        sprintf(temp, "%d", approx);
        lcdPuts(lcd, temp);
    
    blinkN(gpio, LED, approx);
    //deallocates memory assigned
    free(temp);
}

void showGuess(int colorNum, struct lcdStruct *lcd) {
    switch (colorNum)
    {
    case 1:
        lcdPuts(lcd, " R");
        fprintf(stderr, " R");
        break;
    case 2:
        lcdPuts(lcd, " G");
        fprintf(stderr, " G");
        break;
    case 3:
        lcdPuts(lcd, " B");
        fprintf(stderr, " B");
        break;
    }
}

//helper function to concatenate
int concat(int x, int y) {
    int tempVar = y;
    do
    {
        x *= 10;
        y /= 10;
    } while (y != 0);
    
    return x + tempVar;
}

//helper function to reverse an array 
void reverse(int arr[], int first, int last) {
    int tempArr;
    while (first < last)
    {
        tempArr = arr[first];
        arr[first] = arr[last];
        arr[last] = tempArr;
        first++;
        last--;
    }
}


void showSecretLCD(struct lcdStruct *lcd) {
    int pos = 10;
    for (int i = 0; i < seqlen; ++i) {
        lcdPosition(lcd, pos, 1);
        switch (theSequence[i]) {
        case 1:
            lcdPuts(lcd, "R ");
            pos += 2;
            break;
        case 2:
            lcdPuts(lcd, "G ");
            pos += 2;
            break;
        case 3:
            lcdPuts(lcd, "B ");
            pos += 2;
            break;
        }
    }
}





/* ======================================================= */
/* SECTION: main fct                                       */
/* ------------------------------------------------------- */

int main (int argc, char *argv[])
{ // this is just a suggestion of some variable that you may want to use
  struct lcdDataStruct *lcd ;
  int bits, rows, cols ;
  unsigned char func ;

  int found = 0, attempts = 0, i, j, code;
  int c, d, buttonPressed, rel, foo;
  int *attSeq;
  char *temp = malloc(17);


  int pinLED = LED, pin2LED2 = LED2, pinButton = BUTTON;
  int fSel, shift, pin,  clrOff, setOff, offset, res;
  int fd ;

  int  exact, contained;
  char str1[32];
  char str2[32];
  
  struct timeval t1, t2 ;
  int t ;

  char buf [32] ;

  // variables for command-line processing
  char str_in[20], str[20] = "some text";
  int verbose = 0, debug = 0, help = 0, opt_m = 0, opt_n = 0, opt_s = 0, unit_test = 0, res_matches = 0;
  

  char *guessSeq;
  guessSeq = (char *)malloc(seqlen * sizeof(char));
  // -------------------------------------------------------
  // process command-line arguments

  // see: man 3 getopt for docu and an example of command line parsing
  { // see the CW spec for the intended meaning of these options
    int opt;
    while ((opt = getopt(argc, argv, "hvdus:")) != -1) {
      switch (opt) {
      case 'v':
	verbose = 1;
	break;
      case 'h':
	help = 1;
	break;
      case 'd':
	debug = 1;
	break;
      case 'u':
	unit_test = 1;
	break;
      case 's':
	opt_s = atoi(optarg); 
	break;
      default: /* '?' */
	fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
	exit(EXIT_FAILURE);
      }
    }
  }

  if (help) {
    fprintf(stderr, "MasterMind program, running on a Raspberry Pi, with connected LED, button and LCD display\n"); 
    fprintf(stderr, "Use the button for input of numbers. The LCD display will show the matches with the secret sequence.\n"); 
    fprintf(stderr, "For full specification of the program see: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf\n"); 
    fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
    exit(EXIT_SUCCESS);
  }
  
  if (unit_test && optind >= argc-1) {
    fprintf(stderr, "Expected 2 arguments after option -u\n");
    exit(EXIT_FAILURE);
  }

  if (verbose && unit_test) {
    printf("1st argument = %s\n", argv[optind]);
    printf("2nd argument = %s\n", argv[optind+1]);
  }

  if (verbose) {
    fprintf(stdout, "Settings for running the program\n");
    fprintf(stdout, "Verbose is %s\n", (verbose ? "ON" : "OFF"));
    fprintf(stdout, "Debug is %s\n", (debug ? "ON" : "OFF"));
    fprintf(stdout, "Unittest is %s\n", (unit_test ? "ON" : "OFF"));
    if (opt_s)  fprintf(stdout, "Secret sequence set to %d\n", opt_s);
  }
   //INITIALISE SECRET SEQUENCE
    if(!opt_s) {
        initSeq();
    }

    if(debug) {
        showSeq(theSequence);
    } 

  
  seq1 = (int*)malloc(seqlen*sizeof(int));
  seq2 = (int*)malloc(seqlen*sizeof(int));
  cpy1 = (int*)malloc(seqlen*sizeof(int));
  cpy2 = (int*)malloc(seqlen*sizeof(int));

  // check for -u option, and if so run a unit test on the matching function
  if (unit_test && argc > optind+1) { // more arguments to process; only needed with -u 
    strcpy(str_in, argv[optind]);
    opt_m = atoi(str_in);
    strcpy(str_in, argv[optind+1]);
    opt_n = atoi(str_in);
    // CALL a test-matches function; see testm.c for an example implementation
    readSeq(seq1, opt_m); // turn the integer number into a sequence of numbers
    readSeq(seq2, opt_n); // turn the integer number into a sequence of numbers
    if (verbose)
      fprintf(stdout, "Testing matches function with sequences %d and %d\n", opt_m, opt_n);
    res_matches = countMatches(seq1, seq2);
    showMatches(res_matches, seq1, seq2, 1);
    exit(EXIT_SUCCESS);
  } else {
    /* nothing to do here; just continue with the rest of the main fct */
  }

  if (opt_s) { // if -s option is given, use the sequence as secret sequence
    if (theSequence==NULL)
      theSequence = (int*)malloc(seqlen*sizeof(int));
    readSeq(theSequence, opt_s);
    if (verbose) {
      fprintf(stderr, "Running program with secret sequence:\n");
      showSeq(theSequence);
    }
  }
  
  // -------------------------------------------------------
  // LCD constants, hard-coded: 16x2 display, using a 4-bit connection
  bits = 4; 
  cols = 16; 
  rows = 2; 
  // -------------------------------------------------------

  printf ("Raspberry Pi LCD driver, for a %dx%d display (%d-bit wiring) \n", cols, rows, bits) ;

  if (geteuid () != 0)
    fprintf (stderr, "setup: Must be root. (Did you forget sudo?)\n") ;

  // init of guess sequence, and copies (for use in countMatches)
  attSeq = (int*) malloc(seqlen*sizeof(int));
  cpy1 = (int*)malloc(seqlen*sizeof(int));
  cpy2 = (int*)malloc(seqlen*sizeof(int));

  // -----------------------------------------------------------------------------
  // constants for RPi2
  gpiobase = 0x3F200000 ;

  // -----------------------------------------------------------------------------
  // memory mapping 
  // Open the master /dev/memory device

  if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
    return failure (FALSE, "setup: Unable to open /dev/mem: %s\n", strerror (errno)) ;

  // GPIO:
  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpiobase) ;
  if ((int32_t)gpio == -1)
    return failure (FALSE, "setup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

  // -------------------------------------------------------
  // Configuration of LED and BUTTON

  
    pinMode(gpio, BUTTON, INPUT);
    pinMode(gpio, LED2, OUTPUT);
    pinMode(gpio, LED, OUTPUT);
    writeLED(gpio, LED2, LOW);
    writeLED(gpio, LED, LOW);
  // -------------------------------------------------------
  // INLINED version of lcdInit (can only deal with one LCD attached to the RPi):
  // you can use this code as-is, but you need to implement digitalWrite() and
  // pinMode() which are called from this code
  // Create a new LCD:
  lcd = (struct lcdDataStruct *)malloc (sizeof (struct lcdDataStruct)) ;
  if (lcd == NULL)
    return -1 ;

  // hard-wired GPIO pins
  lcd->rsPin   = RS_PIN ;
  lcd->strbPin = STRB_PIN ;
  lcd->bits    = 4 ;
  lcd->rows    = rows ;  // # of rows on the display
  lcd->cols    = cols ;  // # of cols on the display
  lcd->cx      = 0 ;     // x-pos of cursor
  lcd->cy      = 0 ;     // y-pos of curosr

  lcd->dataPins [0] = DATA0_PIN ;
  lcd->dataPins [1] = DATA1_PIN ;
  lcd->dataPins [2] = DATA2_PIN ;
  lcd->dataPins [3] = DATA3_PIN ;
  // lcd->dataPins [4] = d4 ;
  // lcd->dataPins [5] = d5 ;
  // lcd->dataPins [6] = d6 ;
  // lcd->dataPins [7] = d7 ;


  // lcds [lcdFd] = lcd ;


//LCD SETUP
    digitalWrite(gpio, lcd->rsPin, 0); pinMode(gpio, lcd->rsPin, OUTPUT);       //RS SETUP
    digitalWrite(gpio, lcd->strbPin, 0); pinMode(gpio, lcd->strbPin, OUTPUT);   //STRB SETUP
    
    digitalWrite(gpio, lcd->dataPins[0], 0); pinMode(gpio, lcd->dataPins[0], OUTPUT); //DATA PIN 4 SETUP
    digitalWrite(gpio, lcd->dataPins[1], 0); pinMode(gpio, lcd->dataPins[1], OUTPUT); //DATA PIN 5 SETUP
    digitalWrite(gpio, lcd->dataPins[2], 0); pinMode(gpio, lcd->dataPins[2], OUTPUT); //DATA PIN 6 SETUP
    digitalWrite(gpio, lcd->dataPins[3], 0); pinMode(gpio, lcd->dataPins[3], OUTPUT); //DATA PIN 7 SETUP
    delay (35) ; // mS

// Gordon Henderson's explanation of this part of the init code (from wiringPi):
// 4-bit mode?
//	OK. This is a PIG and it's not at all obvious from the documentation I had,
//	so I guess some others have worked through either with better documentation
//	or more trial and error... Anyway here goes:
//
//	It seems that the controller needs to see the FUNC command at least 3 times
//	consecutively - in 8-bit mode. If you're only using 8-bit mode, then it appears
//	that you can get away with one func-set, however I'd not rely on it...
//
//	So to set 4-bit mode, you need to send the commands one nibble at a time,
//	the same three times, but send the command to set it into 8-bit mode those
//	three times, then send a final 4th command to set it into 4-bit mode, and only
//	then can you flip the switch for the rest of the library to work in 4-bit
//	mode which sends the commands as 2 x 4-bit values.

  if (bits == 4)
  {
    func = LCD_FUNC | LCD_FUNC_DL ;			// Set 8-bit mode 3 times
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    func = LCD_FUNC ;					// 4th set: 4-bit mode
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcd->bits = 4 ;
  }
  else
  {
    failure(TRUE, "setup: only 4-bit connection supported\n");
    func = LCD_FUNC | LCD_FUNC_DL ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
  }

  if (lcd->rows > 1)
  {
    func |= LCD_FUNC_N ;
    lcdPutCommand (lcd, func) ; delay (35) ;
    delay(35);
  }

  // Rest of the initialisation sequence
  lcdDisplay     (lcd, TRUE) ;
  lcdCursor      (lcd, FALSE) ;
  lcdCursorBlink (lcd, FALSE) ;
  lcdClear       (lcd) ;

  lcdPutCommand (lcd, LCD_ENTRY   | LCD_ENTRY_ID) ;    // set entry mode to increment address counter after write
  lcdPutCommand (lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL) ;  // set display shift to right-to-left




/* Debug Part */

    if (debug) {
        fprintf(stdout, "===============================\n\n");
        fprintf(stdout, "            Debug Mode           \n");
        fprintf(stdout, "===============================\n\n");
        
        fprintf(stderr, "Only uses command-line interface\n");
        fprintf(stderr, "for inputting the guess sequence\n");
        
        fprintf(stderr, "\n Primarily to check Game Logic \n");


        while (attempts < 5) {
            attempts++;

            //input is stored in a array called guessSeq
            printf("\nAttempts #%d ", attempts);
            scanf("%s", guessSeq);

            //input converted to numbers 
            for (int m = 0; m < seqlen; m++) {
                switch (guessSeq[m]) {
                case 'R':
                    attSeq[m] = 1;
                    break;
                case 'G':
                    attSeq[m] = 2;
                    break;
                case 'B':
                    attSeq[m] = 3;
                    break;
                default:
                    attSeq[m] = 0;
                }
            }

            //compare and calculate exact and approx matches between @theSeq and @attSeq. 
            int matches = countMatches(theSequence, attSeq);

            //Challenge completed if 3 are the same 
            if (matches == 30) {
                found = 1; //The found flag is set to 1
                showMatches(matches, theSequence, attSeq, 1); //prints the matches to the terminal
                break;
            }
            showMatches(matches, theSequence, attSeq, 1);
            delay(1000);
        }
        
        //when the secret sequence is finally cracked
        if (found) {
            if (attempts != 1) {
                printf("\nGood job bro! You figured out the code in %d attempts!\n\n", attempts);
            } else {
                printf("\nCongratulations! You cracked the code in %d attempt!\n\n", attempts);
            }

            //deallocates memory assigned 
            free(guessSeq);
            free(attSeq);
            free(theSequence);
            
            return 0;
        } else {
            printf("\nUnfortunately, your time is up! You took %d attempts\n", attempts);
            printf("Aw too bad! It's okay. Try again!\n");
            showSeq(theSequence);
            
            //deallocates memory assigned 
            free(guessSeq);
            free(attSeq);
            free(theSequence);
            
            //QUIT
            return 0;
        }
    }
    
  // END lcdInit ------
  // -----------------------------------------------------------------------------
  // Start of game
  fprintf(stderr,"Printing welcome message on the LCD display ...\n");
  lcdPosition(lcd, 1, 0);
  lcdPuts(lcd, "Get Ready For");
  lcdPosition(lcd, 2, 1);
  lcdPuts(lcd, "Mastermind!");

  /* initialise the secret sequence */
  if (!opt_s)
    initSeq();
  if (debug)
    showSeq(theSequence);

  // optionally one of these 2 calls: 
  waitForButton (gpio, pinButton) ;

  // -----------------------------------------------------------------------------
  // +++++ main loop


    while(attempts < 5 ) {
        ++attempts;
        sprintf(temp, "Round %d!", attempts);
        lcdClear(lcd);
        delay(800);
        
        if(attempts > 1) {
            blinkN(gpio, LED2, 3);
        }
        
        lcdPosition(lcd, 4, 0); 
        lcdPuts(lcd, temp);
        delay(1000); 
        lcdClear(lcd); 
        fprintf(stderr, "\n%s", temp);
        
        
        lcdPosition(lcd, 4, 0); 
        lcdPuts(lcd, "Attempt:");
        fprintf(stderr, "\nAttempt: ");
        
        int digit = 0;
        int place = 4;
        
        for(int pos=0 ; pos<seqlen ; pos++) {
            timeOut = 0;
            initITimer(5);
          
            
            while(!timeOut) {
                //waitForButton(gpio, BUTTON);
                //buttonPressed = readButton(gpio, BUTTON);
                
                if(readButton(gpio, BUTTON) == HIGH) {
                  while (readButton(gpio, BUTTON)) {
                    delay(50);
                  }
                  
                    digit++;
                }
                if( digit > 2){
                  break;
                }
            }
            
            
            attSeq[pos] = digit;
            
            
            blinkN(gpio, LED2, 1);
            blinkN(gpio, LED, digit);
           
            lcdPosition(lcd, place, 1);
            showGuess(digit, lcd);
            place += 2;
            digit = 0;
    
        }
        fprintf(stdout, "\n");
        
        blinkN(gpio, LED2, 2); 
        delay(1000);
        lcdClear(lcd);
        
        int matches = countMatches(theSequence, attSeq);
        int exact = matches / 10;
        int approx = matches % 10;
        fprintf(stdout, "Exact : %d\n", exact);
        fprintf(stdout, "Approx: %d\n", approx);
        
        if(matches == 30) {
            found = 1;
            showMatchesLCD(matches, lcd);
            break;
        }
        
        showMatchesLCD(matches, lcd);
        delay(800);
        
    }
  if (found) {

        lcdClear(lcd); delay(35);
        if(attempts == 1) {
            sprintf(temp, "In %d Round", attempts);
            fprintf(stdout, "\nSUCCESS! You broke the code in %d attempt!\n", attempts);
        } else {
            sprintf(temp, "In %d Rounds", attempts);
            fprintf(stdout, "\nSUCCESS! You broke the code in %d attempts!\n", attempts);
        }
        
        lcdPosition(lcd, 4, 0); 
        lcdPuts(lcd, "SUCCESS!");
        lcdPosition(lcd, 3, 1); 
        lcdPuts(lcd, temp);
        
        writeLED(gpio, LED2, HIGH);
        blinkN(gpio, LED, 3);
        writeLED(gpio, LED2, LOW);
        delay(5000);
        lcdClear(lcd);
  } else {
   lcdClear(lcd); delay(35);
        fprintf(stdout, "The code could not be broken. It's okay. Better luck next time\n");
        
        showSeq(theSequence);
        
        lcdPosition(lcd, 3, 0); 
        lcdPuts(lcd, "GAME OVER!");
                
        lcdPosition(lcd, 1, 1); 
        lcdPuts(lcd, "Code is: ");
        showSecretLCD(lcd);
        delay(5000);
        lcdClear(lcd);
  }
    writeLED(gpio, LED, LOW);
    writeLED(gpio, LED2, LOW);
}
