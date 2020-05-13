#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <asm/div64.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/delay.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/workqueue.h>    /* for work queue */
#include <linux/stat.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#define DEVICE_NAME                 "HCSR"  // device name to be created and registered
#define MAX 15

#define WR_DATA _IOW('a','a',struct Cmd*)  //command for writing data in to kernel space



static short int num_devices = 1;
int ultrasonic_speed= 346;    // 346 m/s
module_param(num_devices, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);

//struct workqueue_struct *wq[MAX];	// work queue
//#define SPIN_LOCK_UNLOCKED      (spinlock_t) {0,0};
 
#if defined(__i386__)

static __inline__ unsigned long long rdtsc(void)   // This has been obtained from an outside source
{                         // this is assembly code to obtain the Time stamp counter value
                          // based on the platform.
  unsigned long long int x;
     __asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
     return x;
}
#elif defined(__x86_64__)

static __inline__ unsigned long long rdtsc(void)
{
  unsigned hi, lo;
  __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
  return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
}

#elif defined(__powerpc__)

static __inline__ unsigned long long rdtsc(void)
{
  unsigned long long int result=0;
  unsigned long int upper, lower,tmp;
  __asm__ volatile(
                "0:                  \n"
                "\tmftbu   %0           \n"
                "\tmftb    %1           \n"
                "\tmftbu   %2           \n"
                "\tcmpw    %2,%0        \n"
                "\tbne     0b         \n"
                : "=r"(upper),"=r"(lower),"=r"(tmp)
                );
  result = upper;
  result = result<<32;
  result = result|lower;

  return(result);
}

#else

#error "No tick counter is available!"

#endif


static int gpio_num[80];



void cmd_table(int pin_1, int pin_2,void *ptr);
struct Cmd{
  char cmd_name[20];
  int para_1;
  int para_2;
};

// configuration 
struct configure{         // structure to store the user input parameters
    int trigPin; // trigPin=10 if trig is connected to IO 10
    int echoPin; 
    int m;      //sample Per Measurement
    int delta;  //sample Period
};

struct fifo_buffer{                      // the fifo buffer
  unsigned long long int time_stamp;
  unsigned long long int value;
};

struct mutex write_my_mutex;
struct mutex open_my_mutex;

struct mutex dev_my_mutex;
//spinlock_t mLock = SPIN_LOCK_UNLOCKED;


/* per device structure */
struct HCSR_dev {
	struct cdev cdev;               /* The cdev structure */
    char name[20];                  /* Name of device*/
    struct fifo_buffer buff[6];            /* buffer for the input string */
    int head;
    int curr;                        // indices for the fifo buffer
    int count;
    struct configure conf; 
    int measurement_flag;
  	struct hrtimer hrt;
    long long unsigned int distance;
  	int trigger_flag;    
  	int expire_time;
  	int data_available;
  	int delay_flag;
  	ktime_t ref1;
  	ktime_t ref2;
  	int irq_num;
  	struct miscdevice my_misc_dev;             							/* The  miscdevice  structure */
  	struct task_struct *dev_kthreads[10];
  	struct mutex perdev_my_mutex;
  	int threads_counter;
  	struct task_struct *thread;

};
struct HCSR_dev *HCSR_devp[MAX];


struct write_thread{

	struct HCSR_dev *HCSR_devp;
	int write_parameter;


};

void table_for_commands(int pin_1, int pin_2,void *ptr);