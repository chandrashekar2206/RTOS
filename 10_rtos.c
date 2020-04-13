// RTOS Framework - Spring 2020
// J Losh

// Student Name:
// CHANDRASHEKAR MOHAN.

// Add xx_ prefix to all files in your project
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// xx_other files (except uart0.x and wait.x)
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PA2
// Orange: PA3
// Yellow: PA4
// Green:  PE0
// PBs on these pins
// PB0:    PC4
// PB1:    PC5
// PB2:    PC6
// PB3:    PC7
// PB4:    PD6
// PB5:    PD7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"

//#include <stdio.h>
//#include <stdlib.h>
//#include <ctype.h>
// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // off-board orange LED

#define PB0   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4))) // off-board push button 0
#define PB1   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) // off-board push button 1
#define PB2   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) // off-board push button 2
#define PB3   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4))) // off-board push button 3
#define PB4   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4))) // off-board push button 4
#define PB5   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 7*4))) // off-board push button 5

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();
extern void setPSP(uint32_t c);
extern uint32_t get_PSP();
extern uint32_t PUSH_r4_11();
extern uint32_t POP_r4_11(uint32_t p2);
extern uint32_t PUSH_xPSR_R0(uint32_t r0);
extern uint32_t get_SV_val();
extern uint32_t getR0();
extern uint32_t getR1();
extern void send_LR(void);
void yield();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint8_t LastUser;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char semaphorename[16];
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

uint32_t stack [MAX_TASKS][512];    //size of stack is 512
bool processflag=false;
bool priority=true;
bool firstcall=true;
bool prempt=true;
bool piflag = true;
uint16_t sumcount=0;
struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    uint32_t sum0;
    uint32_t sum1;
} tcb[MAX_TASKS];

#define MAX_CHARS 80
#define MAX_FIELDS 5

uint32_t sum[MAX_TASKS];
struct __userdata
{
    char uartStr[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t offset[MAX_FIELDS];
    uint8_t type[MAX_FIELDS];
    char* stri;
}UARTDATA;



//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
    if(priority)
    {
     uint8_t prioritynum,count,task;
     bool okp,pflag;
     okp=false;
     pflag=false;
     count=taskCurrent;
     task=taskCurrent;
     prioritynum=0;
     while(!okp)
     {
         if(!firstcall)
         {
         count++;
         }
         firstcall=false;
         if(count==task+1)
         {
             if(pflag)
             {
                 prioritynum++;
                 if(prioritynum>15)
                 {
                     priority=0;
                 }

             }
             pflag=true;
         }

         if(count>=MAX_TASKS)
         {
                 count=0;
         }
         if(tcb[count].currentPriority==prioritynum)
         {
                 if(tcb[count].state == STATE_READY || tcb[count].state == STATE_UNRUN)
                 {
                     okp=true;
                 }
         }
         }



         return count;
    }
    else
    {
        bool ok;
        static uint8_t task = 0xFF;
        ok = false;
        while (!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
        return task;
    }
}

void copy_str(char str1[], const char str2[])  // strcpy
{
    uint8_t i;
    for (i = 0; str2[i] != '\0'; ++i)
        {
            str1[i] = str2[i];
        }

        str1[i] = '\0';
}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name
    // add task if room in task list
    // allocate stack space for a thread and assign to sp below
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (((tcb[i].state != STATE_INVALID) && (tcb[i].pid != 0)) ) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            copy_str(tcb[i].name, name);
            tcb[i].sp = &stack[i+1][0];
            tcb[i].spInit=&stack[i+1][0];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            taskCount++;  // increment task count
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
    __asm(" SVC  #72 ");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm(" SVC  #69 ");



}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{

    __asm(" SVC  #74 ");

}

struct semaphore* createSemaphore(uint8_t count,const char semaname[])
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
        copy_str(pSemaphore->semaphorename, semaname);
    }
    return pSemaphore;
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{

    taskCurrent = rtosScheduler();
    tcb[taskCurrent].state= STATE_READY;


    setPSP((uint32_t)tcb[taskCurrent].sp);
    _fn fn = (_fn)(tcb[taskCurrent].pid);

    //timer configurration
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = 17;                             // one shot and count up
        //TIMER1_TAILR_R = 0;

    // systick configuration
    NVIC_ST_RELOAD_R  = 39999;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R |=NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;



    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on time


    (*fn)();
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
     __asm(" SVC  #80 ");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm(" SVC  #30 ");

}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm(" SVC  #90 ");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm(" SVC  #70 ");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void rev(char str[],uint16_t i)
{
    int s=0,e;
    char temp;

    for(e=i-1;s<e;e--)
    {
        temp=*(str+s);
        *(str+s) = *(str+e);
        *(str+e) = temp;
        s++;
    }
}


void systickIsr()
{
    uint8_t i;
    sumcount++;
    if (sumcount==1000)
    {
        sumcount=0;
        processflag=  ! processflag;
        if (processflag)
        {
            for(i=0;i < MAX_TASKS; i++ )
            {
                tcb[i].sum1=0;
            }
        }
        else
        {
             for(i=0;i < MAX_TASKS; i++ )
             {
                tcb[i].sum0=0;
             }
        }

    }
    for(i=0;i<MAX_TASKS;i++)
    {
        if(tcb[i].state == STATE_DELAYED)
        {
            tcb[i].ticks--;
            if(tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
            }
        }
    }
    if(prempt)
    {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{

    uint32_t p1 = PUSH_r4_11();
    tcb[taskCurrent].sp = (void*) p1;
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off
    if (processflag)
    {
        tcb[taskCurrent].sum1= tcb[taskCurrent].sum1 + TIMER1_TAV_R;
    }
    else
    {
        tcb[taskCurrent].sum0= tcb[taskCurrent].sum0 + TIMER1_TAV_R;
    }
    taskCurrent = rtosScheduler();
    TIMER1_TAV_R=0;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer    TIMER1_TAV_R=0;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    //POP_r4_11((uint32_t) tcb[taskCurrent].sp);
    if(tcb[taskCurrent].state == STATE_READY)
    {
        POP_r4_11((uint32_t) tcb[taskCurrent].sp);
    }else
    {
        setPSP((uint32_t)tcb[taskCurrent].sp);
        PUSH_xPSR_R0((uint32_t)tcb[taskCurrent].pid);
        tcb[taskCurrent].state = STATE_READY;
        send_LR();
    }



}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    struct semaphore *s;

        uint8_t i=0;
        uint8_t N =  (uint8_t)get_SV_val();

    switch (N)
        {
        case 80://yield
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;

        case 30://sleep
            tcb[taskCurrent].state = STATE_DELAYED ;
            tcb[taskCurrent].ticks = getR0();
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;

        case 90://wait
            //sem=getR0() ;
            s = (struct semaphore*)getR0();
            if(s->count > 0)
            {
                s->count--;
            }
            else
            {
 /*             while(s->processQueue[i] !=0 )
              {
                  i++;
                  if(i==s->queueSize)
                  {
                      break;
                  }
              }
              s->processQueue[i]=taskCurrent;*/

              s->processQueue[s->queueSize]=taskCurrent;
              s->queueSize++;
              if(piflag)
              {
                  if(tcb[s->LastUser].currentPriority > tcb[taskCurrent].currentPriority)
                  {
                      if(s->LastUser !=0 && taskCurrent !=0)
                      {
                          tcb[s->LastUser].currentPriority = tcb[taskCurrent].currentPriority;
                      }
                  }
              }
              tcb[taskCurrent].state = STATE_BLOCKED;
              tcb[taskCurrent].semaphore = (void*)s;
            }
            s->LastUser = taskCurrent;
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;

        case 70://post
            s = (struct semaphore*)getR0();
            s->count++;
            if(s->processQueue[0]!=0)
            {
                tcb[s->processQueue[0]].state= STATE_READY;
                s->queueSize--;
          //      tcb[s->processQueue[0]].semaphore = 0;
                s->processQueue[0]= s->processQueue[1];
                s->processQueue[1]= s->processQueue[2];
                s->processQueue[2]= s->processQueue[3];
                s->processQueue[3]= s->processQueue[4];
                s->processQueue[4]= 0;
                s->count--;
            }
            if(piflag)
            {
                tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;

            }

            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;
        case 99://get data
            if(processflag)
            {
                for(i = 0; i < MAX_TASKS; i++)
                {
                    sum[i] = tcb[i].sum0;
                }
            }else
            {
                for(i = 0; i < MAX_TASKS; i++)
                 {
                    sum[i] = tcb[i].sum1;
                 }

            }
            break;
        case 69:   //kill
        {
            for(i=0;i<MAX_TASKS;i++)
            {
                if(tcb[i].pid==(void *)getR0())
                {
                    break;
                }
            }
            if(i==1||i==6)
            {
            if(tcb[i].state==STATE_BLOCKED)
            {
                s=tcb[i].semaphore;
                s->queueSize--;

                s->processQueue[0]= s->processQueue[1];
                s->processQueue[1]= s->processQueue[2];
                s->processQueue[2]= s->processQueue[3];
                s->processQueue[3]= s->processQueue[4];
                s->processQueue[4]= 0;
            }
            if (tcb[i].state==STATE_READY || tcb[i].state==STATE_DELAYED)
            {
                if(tcb[i].semaphore!=0)
                {
                    s=tcb[i].semaphore;
                    tcb[s->processQueue[0]].state= STATE_READY;
                    s->queueSize--;
                    s->processQueue[0]= s->processQueue[1];
                    s->processQueue[1]= s->processQueue[2];
                    s->processQueue[2]= s->processQueue[3];
                    s->processQueue[3]= s->processQueue[4];
                    s->processQueue[4]= 0;
                }


            }
            }
            tcb[i].state=STATE_INVALID;
            break;
        }
        case 72:  //restart
            for(i=0;i<MAX_TASKS;i++)
            {
                if(tcb[i].pid==(void *)getR0())
                {
                    break;
                }
            }
            if(tcb[i].state==STATE_INVALID)
            {
                if(i==4)
                {
                    s=tcb[i+1].semaphore;
                    tcb[s->processQueue[0]].state= STATE_READY;
                    s->queueSize--;
                    s->processQueue[0]= s->processQueue[1];
                    s->processQueue[1]= s->processQueue[2];
                    s->processQueue[2]= s->processQueue[3];
                    s->processQueue[3]= s->processQueue[4];
                    s->processQueue[4]= 0;
                }
                tcb[i].sp=tcb[i].spInit;
                tcb[i].state=STATE_UNRUN;
            }
            break;
        case 74:  // set priority
            for(i=0 ; i< MAX_TASKS; i++)
                {
                    if(tcb[i].pid == (void *)getR0())
                    {
                        break;
                    }
                }

            tcb[i].currentPriority = getR1();
            break;

}
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons, and uart
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
            SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

            // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
            SYSCTL_GPIOHBCTL_R = 0;

            // Enable GPIO port F peripherals
            SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA|SYSCTL_RCGC2_GPIOC|SYSCTL_RCGC2_GPIOD|SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOF;

            // Enable Timer1

            SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer

            // Configure LED and pushbutton pins
            GPIO_PORTF_DIR_R = 0x04;
            GPIO_PORTF_DR2R_R = 0x04;
            GPIO_PORTF_DEN_R = 0x04;

            GPIO_PORTA_DIR_R = 0x1C;
            GPIO_PORTA_DR2R_R = 0x1C;
            GPIO_PORTA_DEN_R = 0x1C;

            GPIO_PORTE_DIR_R = 0x01;
            GPIO_PORTE_DR2R_R = 0x01;
            GPIO_PORTE_DEN_R = 0x01;

            GPIO_PORTC_DEN_R = 0xF0;
            GPIO_PORTC_PUR_R = 0xF0;

            GPIO_PORTD_LOCK_R = 0x4C4F434B;
            GPIO_PORTD_CR_R = 0x80;
            GPIO_PORTD_DEN_R = 0xC0;
            GPIO_PORTD_PUR_R = 0xC0;
            GPIO_PORTD_CR_R = 0;


}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t num=0;
    if(!(GPIO_PORTC_DATA_R & 0x10))
        {
            num = num + 1;
        }

    if(!(GPIO_PORTC_DATA_R & 0x20))
        {
            num = num + 2;
        }

    if(!(GPIO_PORTC_DATA_R & 0x40))
        {
            num = num + 4;
        }

    if(!(GPIO_PORTC_DATA_R & 0x80))
        {
            num = num + 8;
        }

    if(!(GPIO_PORTD_DATA_R & 0x40))
        {
            num = num + 16;
        }

    if(!(GPIO_PORTD_DATA_R & 0x80))
        {
            num = num + 32;
        }

    return num;

}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
char tolower(char a)
{

    if(a >= 65 && a <= 90)
    {
        a = a + 32; // convert into lower case
    }else if(a >= 97 && a <= 122)
    {
        return a;
    }

    return a;

}
char* tolowerstr(char *Str1)
{
    uint8_t i;
    for (i = 0; Str1[i]!='\0'; i++)
        {
            if(Str1[i] >= 'A' && Str1[i] <= 'Z')
            {
                Str1[i] = Str1[i] + 32;
            }
        }
    return Str1;
}

//itoa borrowed and modified from internet
//void swap(char *x, char *y) {
//    char t = *x; *x = *y; *y = t;
//}
//
//
//char* reverse(char *buffer, int i, int j)
//{
//    while (i < j)
//        swap(&buffer[i++], &buffer[j--]);
//
//    return buffer;
//}
//
//char* i2a(int value,char *buffer)
//{
////     char buffer[20];
//
//    uint32_t n = abs(value);
//
//    uint8_t i = 0;
//    while (n)
//    {
//        uint8_t r = n % 10;
//
//        if (r >= 10)
//            buffer[i++] = 65 + (r - 10);
//        else
//            buffer[i++] = 48 + r;
//
//        n = n / 10;
//    }
//
//    if (i == 0)
//    {
//        buffer[i] = '0';
//        buffer[i++] = '\0';
//    }
//
////    if (value < 0 )
////        buffer[i++] = '-';
//
//    buffer[i] = '\0';
//
//
//    return reverse(buffer, 0, i - 1);
//}

bool strcmp(char str1[], char str2[])
{
   uint8_t c = 0;

   while (str1[c] == str2[c]) {
      if (str1[c] == '\0' || str2[c] == '\0')
         break;
      c++;
   }

   if (str1[c] == '\0' && str2[c] == '\0')
      return true;
   else
      return false;
}


char *i22a(uint16_t num,char *str)
{
    uint8_t i = 0;
//    uint8_t rem = 0;

    if(num == 0)
    {
        str[i] = '0';
        str[++i] = '\0';
        return str;
    }

    while(num)
    {
        uint8_t rem = num % 10;

        if (rem >= 10)
            str[i++] = 65 + (rem - 10);
        else
            str[i++] = 48 + rem;
        num = num / 10;
    }
    str[i] = '\0';
    rev(str,i);
    return str;
}

void getUart0string()
{
    uint8_t count=0;

      while(count<MAX_CHARS+1)
      {
      char c=getcUart0();
                if (c==8)
                {
                    if (count>0)
                    {
                        count--;
                    }
                    continue;
                }
                if (c==13)
                {
                    UARTDATA.uartStr[count] = 0;
                    break;
                }
                if (c>=32)
                {
                    UARTDATA.uartStr[count++] = tolower(c);
                    if (count==MAX_CHARS)
                    {
                        UARTDATA.uartStr[count]=0;
                        break;
                    }
                }
      }
}

void parseUart0String()
{
    uint8_t i=0;
    UARTDATA.fieldCount=0;
    //uartStr[0]=0;
    if(i==0)
    {
        if ((UARTDATA.uartStr[i]>='a' && UARTDATA.uartStr[i]<='z'))
             {
                 UARTDATA.offset[UARTDATA.fieldCount]=i;
                 UARTDATA.type[UARTDATA.fieldCount]='a';

             }
             else if ((UARTDATA.uartStr[i]>=48 && UARTDATA.uartStr[i]<=57)||(UARTDATA.uartStr[i]>=45 && UARTDATA.uartStr[i]<=46))
             {
                 UARTDATA.offset[UARTDATA.fieldCount]=i;
                 UARTDATA.type[UARTDATA.fieldCount]='n';
             }
        UARTDATA.fieldCount++;

    }
 for (i=0;UARTDATA.uartStr[i]!=0;i++)
 {

     if((UARTDATA.uartStr[i]>=32 && UARTDATA.uartStr[i]<=37) ||(UARTDATA.uartStr[i]>=39 && UARTDATA.uartStr[i]<=44) || (UARTDATA.uartStr[i]==47) || (UARTDATA.uartStr[i]>=58 && UARTDATA.uartStr[i]<=64) || (UARTDATA.uartStr[i]>=91 && UARTDATA.uartStr[i]<=96) ||(UARTDATA.uartStr[i]>=123 && UARTDATA.uartStr[i]<=127))
     {
      continue;
     }
     else if ((UARTDATA.uartStr[i]>='a' && UARTDATA.uartStr[i]<='z'))
     {
         UARTDATA.offset[UARTDATA.fieldCount]=i;
         UARTDATA.type[UARTDATA.fieldCount]='a';

     }
     else if ((UARTDATA.uartStr[i]>=48 && UARTDATA.uartStr[i]<=57)||(UARTDATA.uartStr[i]>=45 && UARTDATA.uartStr[i]<=46))
     {
         UARTDATA.offset[UARTDATA.fieldCount]=i;
         UARTDATA.type[UARTDATA.fieldCount]='n';
     }
     if(((UARTDATA.uartStr[i-1]>=32 && UARTDATA.uartStr[i-1]<=37) || (UARTDATA.uartStr[i]>=39 && UARTDATA.uartStr[i]<=44) || (UARTDATA.uartStr[i-1]==47) || (UARTDATA.uartStr[i-1]>=58 && UARTDATA.uartStr[i-1]<=64) || (UARTDATA.uartStr[i-1]>=91 && UARTDATA.uartStr[i-1]<=96) ||(UARTDATA.uartStr[i-1]>=123 && UARTDATA.uartStr[i-1]<=127)) && ((UARTDATA.uartStr[i]>='a' && UARTDATA.uartStr[i]<='z')||(UARTDATA.uartStr[i]>=48 && UARTDATA.uartStr[i]<=57)||(UARTDATA.uartStr[i]>=45 && UARTDATA.uartStr[i]<=46)))
     {
         UARTDATA.fieldCount++;

     }
 }
 for (i=0;i<UARTDATA.uartStr[i]!=0;i++)
  {
          if((UARTDATA.uartStr[i]>=32 && UARTDATA.uartStr[i]<=37) || (UARTDATA.uartStr[i]>=39 && UARTDATA.uartStr[i]<=44) || (UARTDATA.uartStr[i]==47) || (UARTDATA.uartStr[i]>=58 && UARTDATA.uartStr[i]<=64) || (UARTDATA.uartStr[i]>=91 && UARTDATA.uartStr[i]<=96) ||(UARTDATA.uartStr[i]>=123 && UARTDATA.uartStr[i]<=127))
          {
              UARTDATA.uartStr[i]=0;
          }
  }
}

bool isCommand( char commandName[20] , int min_Arg)
{
       char *c= &UARTDATA.uartStr[UARTDATA.offset[0]];
    if(strcmp(commandName,c))
    {
       if( UARTDATA.fieldCount>min_Arg)
       {
           return(true);
       }
    }
    return (false);
}


int getValue(int arg_Number)
{
    int value=(atoi(&UARTDATA.uartStr[UARTDATA.offset[arg_Number+1]]));
    return value;
}


char* getString(int arg_Number)
{
    return &UARTDATA.uartStr[UARTDATA.offset[arg_Number+1]];
}


void get_stats(uint32_t sumi[])
{
    uint8_t i;


        for(i = 0; i < MAX_TASKS; i++)
        {
              sumi[i] = sum[i];
        }
}


// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

/*void idle2()
{
    while(true)
    {
        YELLOW_LED = 1;
        waitMicrosecond(1000);
        YELLOW_LED = 0;
        yield();
    }
}*/

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}



void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);

    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 32)
        {
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
//           your solution should not use C library calls for strings, as the stack will be too large
void shell()
{
    uint8_t i;
    uint32_t sumi[12], sumn[12];
    uint8_t sump[12];
    uint32_t totalsum;
    char *str1,strr[20];
    uint32_t summ[12];
    struct semaphore* s;
    while (true)
    {
                    getUart0string();
                    parseUart0String();

                    if(isCommand("ps",0))
                    {
//                        char strr[20];
                        __asm(" SVC  #99 ");
                        get_stats(sumi);
                        totalsum=0;
                        for(i=0;i<MAX_TASKS;i++)
                        {
                            totalsum=totalsum+sumi[i];

                        }
                        for (i=0;i<MAX_TASKS;i++)
                        {
                            summ[i]= ((sumi[i]*100) /totalsum);
                           // sumi[i]= (sumi[i] *100)/totalsum;
                            sumn[i] = sumi[i]*100;
                            sump[i] = sumn[i] % totalsum;

                        }
                        putsUart0("\r\nPID\t\t processname\t\t priority\t\t state\t\t\t     %CPU time\r\n\r\n");
                        for(i=0;i<MAX_TASKS;i++)
                        {
                            if(tcb[i].state == STATE_READY || tcb[i].state == STATE_UNRUN||tcb[i].state == STATE_DELAYED || tcb[i].state == STATE_BLOCKED )
                            {

                                UARTDATA.stri=i22a((uint32_t)tcb[i].pid,strr);
                                    putsUart0(UARTDATA.stri);
                                    putsUart0("\t\t");

                                    putsUart0(tcb[i].name);
                                    putsUart0("    \t\t  ");

                                    i22a(tcb[i].currentPriority,strr);
                                    putsUart0(strr);
                                    putsUart0("\t\t\t");
                                    if (tcb[i].state == STATE_READY)
                                    {
                                        putsUart0("READY     \t\t\t");
                                    }
                                    if (tcb[i].state == STATE_UNRUN)
                                    {
                                        putsUart0("UNRUN      \t\t\t");
                                    }
                                    if (tcb[i].state == STATE_DELAYED)
                                    {
                                        putsUart0("SLEEP by ");
                                        i22a(tcb[i].ticks,strr);
                                        putsUart0(strr);
                                        putsUart0("\t\t\t");
                                    }
                                    if (tcb[i].state==STATE_BLOCKED)
                                    {
                                        putsUart0("BLOCKED by ");
                                        s=(struct semaphore *) tcb[i].semaphore;
                                        putsUart0(s->semaphorename);
                                        putsUart0("\t\t");
                                    }

                                    i22a(summ[i],strr);
                                    putsUart0(strr);
                                    putsUart0(".");
                                    i22a(sump[i],strr);
                                    putsUart0(strr);
                                    putsUart0("\r\n\r\n");
                                }


                            }
                        putsUart0("\r\n");
                    }
                    if(isCommand("ipcs",0))
                    {

                        putsUart0("\r\nsemaphore\t\t waitcount\t    count\r\n\r\n");

                        s=keyPressed;
                        putsUart0(s->semaphorename);
                        putsUart0("\t\t   ");
                        i22a( s->queueSize,strr);
                        putsUart0(strr);
                        putsUart0("\t\t      ");
                        i22a(s->count,strr);
                        putsUart0(strr);
                        putsUart0("\r\n\r\n");

                        s=keyReleased;
                        putsUart0(s->semaphorename);
                        putsUart0("\t\t   ");
                        i22a(s->queueSize,strr);
                        putsUart0(strr);
                        putsUart0("\t\t      ");
                        i22a(s->count,strr);
                        putsUart0(strr);
                        putsUart0("\r\n\r\n");

                        s=flashReq;
                        putsUart0(s->semaphorename);
                        putsUart0("\t\t   ");
                        i22a(s->queueSize,strr);
                        putsUart0(strr);
                        putsUart0("\t\t      ");
                        i22a(s->count,strr);
                        putsUart0(strr);
                        putsUart0("\r\n\r\n");

                        s=resource;
                        putsUart0(s->semaphorename);
                        putsUart0("\t\t   ");
                        i22a(s->queueSize,strr);
                        putsUart0(strr);
                        putsUart0("\t\t      ");
                        i22a(s->count,strr);
                        putsUart0(strr);
                        putsUart0("\r\n\r\n\r\n");
                    }



                    if(isCommand("reboot",0))
                    {
                        putsUart0("\r\n");
                        NVIC_APINT_R = 0x05FA0000|0x04|0x01;
                    }


                    if(isCommand("sched",0))
                    {
                        if(strcmp("rr",getString(0)))
                        {
                            priority=false;
                        }
                        else if (strcmp("priority",getString(0)))
                        {
                            priority=true;
                        }
                        putsUart0("\r\n");
                    }

                    if(isCommand("pi",0))
                    {
                        if(strcmp("on",getString(0)))
                        {
                            piflag = true;

                        }
                        else if (strcmp("off",getString(0)))
                        {
                            piflag = false;

                        }
                        putsUart0("\r\n");
                    }

                    if(isCommand("preemption",0))
                    {
                        if(strcmp("on",getString(0)))
                        {
                            prempt=true;
                        }
                        else if (strcmp("off",getString(0)))
                        {
                            prempt=false;
                        }
                        putsUart0("\r\n");
                    }



                    if(isCommand("pidof",1))
                    {

                        for (i=0;i<MAX_TASKS;i++)
                        {
                            char strr[20];
                            str1 = tolowerstr(tcb[i].name);
                            if(strcmp(str1,getString(0)))
                            {
                                putsUart0("\r\n");
                                i22a(i,strr);
                                putsUart0(strr);
                                putsUart0("\t\t");

                                i22a((uint32_t)tcb[i].pid,strr);
                                putsUart0(strr);
                                putsUart0("\t\t");

                            }
                        }
                        putsUart0("\r\n");
                    }

                    if(isCommand("kill",1))
                    {
                        destroyThread((_fn)getValue(0));
                        putsUart0("\r\n");
                    }


                    for(i = 0; i < MAX_TASKS; i++)
                    {

                        str1 = tolowerstr(tcb[i].name);
                        if(strcmp(str1, &UARTDATA.uartStr[UARTDATA.offset[0]]))
                        {
                            if (0 == getValue(0))
                            {
                            restartThread((_fn)tcb[i].pid);
                            putsUart0("\r\n");
                            }
                        }
                    }


            }

 }






//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    initRtos();

    initUart0();
    setUart0BaudRate(115200,40000000);

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1,"key pressed");
    keyReleased = createSemaphore(0,"key released");
    flashReq = createSemaphore(5,"flash req");
    resource = createSemaphore(1,"resource");

//    keyPressed = createSemaphore(1);
//    keyReleased = createSemaphore(0);
//    flashReq = createSemaphore(5);
//    resource = createSemaphore(1);


    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 15, 1024);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
    ok &= createThread(oneshot, "OneShot", 4, 1024);
    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
    ok &= createThread(debounce, "Debounce", 12, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 12, 1024);
    ok &= createThread(shell, "Shell", 12, 1024);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}


