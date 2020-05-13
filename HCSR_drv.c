
//including header file
#include "headers.h"

//writing in to buffer
void write_buffer(unsigned long long int tsc,unsigned long long int val,void *ptr) // This function is used to write into the fifo buffer
{
	int next;
	struct HCSR_dev *HCSR_devp;
	HCSR_devp=(struct HCSR_dev *)ptr;
	 next = HCSR_devp->curr + 1;
    if (next >= 5)
        next = 0;

    if (next == HCSR_devp->head) 
    	{ HCSR_devp->head+=1;
          	if(HCSR_devp->head>=5)  
          		HCSR_devp->head=0;    
    	}
    
    HCSR_devp->buff[HCSR_devp->curr].time_stamp = tsc;

    HCSR_devp->buff[HCSR_devp->curr].value= val; 
//printk("inserted in %s \n",HCSR_devp->name);
   // printk("inserted time_stamp %llu \n",HCSR_devp->buff[HCSR_devp->curr].time_stamp);

 // printk("inserted value %llu \n",HCSR_devp->buff[HCSR_devp->curr].value); 
    HCSR_devp->curr = next; 
     //msleep_interruptible(1000);          
	}


//reading from buffer
int read_buffer(struct fifo_buffer *fb,void *ptr) // This function reads from the fifo buffer. this reads the first of the latest 5
{
	int next;
	struct HCSR_dev *HCSR_devp;
	HCSR_devp=(struct HCSR_dev *)ptr;
	if (HCSR_devp->head == HCSR_devp->curr)
		return -1;
	
    next = HCSR_devp->head + 1;
    if(next >= 5)
        next = 0;

    fb->time_stamp=HCSR_devp->buff[HCSR_devp->head].time_stamp;
	fb->value = HCSR_devp->buff[HCSR_devp->head].value;
    HCSR_devp->head = next;             
    return 0;  

}

//function to trigger 10 micro seconds trigger pulse
void Triggering_fun (void *ptr)  // this function triggers the sensor thus causing ultra sound to be transmitted.
{	struct HCSR_dev *HCSR_devp;
	HCSR_devp=(struct HCSR_dev *)ptr;
	HCSR_devp->trigger_flag=1;
	gpio_set_value_cansleep(HCSR_devp->conf.trigPin,0);
	udelay(2);
	gpio_set_value_cansleep(HCSR_devp->conf.trigPin,1);
	udelay(10);
	//mdelay(2000);
	gpio_set_value_cansleep(HCSR_devp->conf.trigPin,0);
	HCSR_devp->trigger_flag=0;
	//printk("Ultrasonic has been triggerd\n");
	
}

//echo trigger handler
static irq_handler_t handling_irq(unsigned int irq, void *dev_id) // interrupt handler
{
	long long int t;
	int val;
	struct HCSR_dev *HCSR_devp;
	HCSR_devp=(struct HCSR_dev *)dev_id;
	
	val=gpio_get_value(HCSR_devp->conf.echoPin);
	if(val==1)												// rising edge
	{
	HCSR_devp->ref1=ktime_set(0,0);
	HCSR_devp->ref1= ktime_get();
	irq_set_irq_type(HCSR_devp->irq_num,IRQ_TYPE_EDGE_FALLING);
	}

	else					// falling edge

	{
		
		HCSR_devp->ref2=ktime_set(0,0);				// performing the calculations and fiding the distance.
		HCSR_devp->ref2=ktime_get();
		//ktime_t time_lapsed = ktime_sub(ref2, HCSR_dev->ref1);
		t = ktime_to_us(ktime_sub(HCSR_devp->ref2,HCSR_devp->ref1));
		if(t>=38000){
			printk("no object is detected");
			HCSR_devp->distance=0;
			HCSR_devp->expire_time=1;
		} 
		else{
			HCSR_devp->distance = ((t*ultrasonic_speed)/2);
			do_div(HCSR_devp->distance,10000);
	       // printk("distance measured %lld \n",HCSR_devp->distance);
		} 

		HCSR_devp->data_available=1;
		irq_set_irq_type(HCSR_devp->irq_num,IRQ_TYPE_EDGE_RISING);  //changing edge type tp rising
		
	}

	return (irq_handler_t) IRQ_HANDLED;
	}



// it is used to configuring parametes
void configure_parameters(int m , int d,void *ptr)
{
	struct HCSR_dev *HCSR_devp;
	HCSR_devp=(struct HCSR_dev *)ptr;
	HCSR_devp->conf.m =m;//3;
	HCSR_devp->conf.delta =d;//20;
}


/*
* Open HCSR driver
*/
int HCSR_driver_open(struct inode *inode, struct file *file)
{
	int dev_minor;

	struct HCSR_dev *HCSR_devp=NULL;
	mutex_lock(&open_my_mutex);	
	
	dev_minor=iminor(inode);
	//extracting per device structure pointer 
	HCSR_devp = container_of(file->private_data, struct HCSR_dev, my_misc_dev);

	printk("minor number is %d\n", dev_minor);
	/* Easy access to cmos_devp from rest of the entry points */
	file->private_data = HCSR_devp;

	mutex_unlock(&open_my_mutex);
	return 0;
}

/*
 * Release HCSR driver
 */
int HCSR_driver_release(struct inode *inode, struct file *file)
{
	int i;

	struct HCSR_dev *HCSR_devp = file->private_data;

	gpio_set_value_cansleep(HCSR_devp->conf.trigPin, 0);
	gpio_set_value_cansleep(HCSR_devp->conf.echoPin, 0);
	free_irq(HCSR_devp->irq_num,HCSR_devp);
	HCSR_devp->threads_counter=0;
		// Free resources.
	for (i= 0; i <80; i++)
	{
		if(gpio_num[i]!=-1)
		gpio_free(gpio_num[i]);
	}


	return 0;
}









void measuring_dis(void *ptr)			// makes m+2 measurement ever delta milliseconds
{
	static int i;
	static unsigned long long sum=0;
	static unsigned long long max=0;
	static unsigned long long min= 100000000;

	struct HCSR_dev *HCSR_devp;
		HCSR_devp=(struct HCSR_dev *)ptr;
	mutex_lock_interruptible(&HCSR_devp->perdev_my_mutex);

	HCSR_devp->measurement_flag=1;

	
	for (i=0;i< HCSR_devp->conf.m +2 ;i++)
	 {  
	 	Triggering_fun((void *)HCSR_devp);
	 	//while(data_available==0)
	 	mdelay(1);
	 	//data_available=0;
	 	sum+=HCSR_devp->distance;
	 	//printk("sum : %llu \n",sum);
	 	if(HCSR_devp->distance > max)
	 		max=HCSR_devp->distance;
	 	if(HCSR_devp->distance<min)
	 		min=HCSR_devp->distance;

	 	HCSR_devp->delay_flag=1;
	 	mdelay(HCSR_devp->conf.delta);

		HCSR_devp->delay_flag=0;
	 }
	 
	 sum = sum - max;
	 sum = sum - min;
	 do_div(sum,HCSR_devp->conf.m);

	
	 write_buffer(rdtsc(),sum,(void *)HCSR_devp);
	 HCSR_devp->measurement_flag=0;
     sum=0;
     mutex_unlock(&HCSR_devp->perdev_my_mutex);

	}




 int work_handler(void *ptr_thread){

 	struct write_thread *write_tf =(struct write_thread *)ptr_thread;
	int write_parameter;
	struct HCSR_dev *HCSR_devp;

	mutex_lock_interruptible(&dev_my_mutex);

  	
	//printk("------test 4-------------");

	write_parameter=write_tf->write_parameter;
	HCSR_devp=write_tf->HCSR_devp;

	//printk("--------------inside work handler call of %s-----------\n",HCSR_devp->name);

	if(HCSR_devp->measurement_flag==1) // if there is a measurement going return EINVAL
		return -EINVAL;
	else 
	{
		if(write_parameter!=0) // if the parameter is non-zero clear the buffer else make the measurement.
		{
			printk("Clearing buffer\n");
			HCSR_devp->curr=0;
			HCSR_devp->head=0;
		}

		else{
				if(HCSR_devp->measurement_flag==1) // if there is a measurement going return EINVAL
					return -EINVAL;
				else{
					//printk("--------------inside write call of %s-----------",HCSR_devp->name);
					measuring_dis((void *)HCSR_devp);

				   // HCSR_devp->dev_kthreads[HCSR_devp->threads_counter] = kthread_run(work_handler, (void *)HCSR_devp,"running in background");
				 //    printk("thread number %d ",HCSR_devp->threads_counter);
				 //    	HCSR_devp->threads_counter++;
				 //   // mutex_unlock(&dev_my_mutex);

						//mutex_lock_interruptible(&my_mutex);
					  // queue_work(wq[HCSR_devp->workqueue_count], &HCSR_devp->my_work[HCSR_devp->same_works]);
					  // mutex_unlock(&my_mutex)
	
				}
			}

	}
mutex_unlock(&dev_my_mutex);

do_exit(0);

}






/*
 * Write to HCSR driver
 */
ssize_t HCSR_driver_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	//printk("--------------inside writecall of ----------\n");

	//mutex_lock(&dev_my_mutex);
	
	struct HCSR_dev *HCSR_devp = file->private_data;
	struct write_thread *write_tf;
	int write_parameter;

	mutex_lock(&write_my_mutex);
	copy_from_user(&write_parameter, (int *)buf, sizeof(int));


	
	write_tf = (struct write_thread *)kmalloc(sizeof(struct write_thread), GFP_KERNEL);
	//printk("------test b-------------");

	write_tf->write_parameter=write_parameter;
	//printk("------test c-------------");

	write_tf->HCSR_devp=HCSR_devp;

	//printk("------test 1-------------");

	if(HCSR_devp->measurement_flag==1) // if there is a measurement going return EINVAL
	{
		mutex_unlock(&write_my_mutex);
		return -EINVAL;
	}
	else 
	{
		//printk("------write parameter %d-------------",write_tf->write_parameter);

		HCSR_devp->thread=kthread_create(work_handler,(void *)write_tf,"running in background");

		//printk("------test 2-------------");

		if(HCSR_devp->thread) {
		//printk("------test 3-------------");

			wake_up_process(HCSR_devp->thread);
			} 	
		else {
			//printk(KERN_ERR "Cannot create kthread\n");
		}
	mutex_unlock(&write_my_mutex);
	return 0;
	}


}
/*
 * Read to HCSR driver
 */
ssize_t HCSR_driver_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	int bytes_read = 0,rf_flag;
	struct HCSR_dev *HCSR_devp = file->private_data;

	struct fifo_buffer *tmp = (struct fifo_buffer * )kmalloc(sizeof(struct fifo_buffer), GFP_KERNEL);
	while(1)
	{
		rf_flag=read_buffer(tmp,(void *)HCSR_devp);
		if(rf_flag==-1){
			msleep_interruptible(30);
			//printk("-----iam sleeping-----");
		}
		else{
			//spin_lock_irq(&mLock); 
			//printk("-----inside break else-----");
			copy_to_user(buf,tmp,sizeof(struct fifo_buffer)); // copy to user space.
			//mdelay(1000);
			//printk("-----done copying data-----");
			//spin_unlock_irq(&mLock);
			break;
		}
	

	}
	//msleep_interruptible(30);
	kfree(tmp);
	bytes_read=sizeof(struct fifo_buffer);
	return bytes_read;

}

long int HCSR_ioctl(struct file *file, unsigned int ioctl_num, unsigned long arg)   // pin and parameter configuration
{
    struct HCSR_dev *HCSR_devp = file->private_data;
    struct Cmd *cmd_buf;
    cmd_buf = ( struct Cmd* )kmalloc(sizeof( struct Cmd), GFP_KERNEL); 



    switch(ioctl_num){
	case WR_DATA:
		copy_from_user(cmd_buf,(struct Cmd *)arg,sizeof(struct Cmd));  //copying data from user
		break;
	default:
		return -EINVAL;
		
	}
	//printk("cmd name is %s",cmd_buf->cmd_name);
 
    if(strcmp(cmd_buf->cmd_name, "CONFIG_PINS") == 0){  // pin configuration
 
        
        if(cmd_buf->para_1 < 0 || cmd_buf->para_1 > 19){  // checking  if its valid
           return -EINVAL;
        }
 
        if(cmd_buf->para_2 < 0 || cmd_buf->para_2 > 19 || cmd_buf->para_2 ==7 || cmd_buf->para_2 == 8) { // checking if echo is valid
           return -EINVAL;
        }
 
      table_for_commands( cmd_buf->para_1, cmd_buf->para_2,(void *)HCSR_devp);  // pin mux table
               
    }
 
    else if(strcmp(cmd_buf->cmd_name, "SET_PARAMETERS") == 0){   // configuring parameters
        configure_parameters(cmd_buf->para_1,cmd_buf->para_2,(void *)HCSR_devp);        

        
        return 0;
    }
    else{
        return -EINVAL;
    }
    
    return 0;
}



/* File operations structure. Defined in linux/fs.h */
static struct file_operations HCSR_fops = {
    .owner		= THIS_MODULE,           /* Owner */
    .open		= HCSR_driver_open,        /* Open method */
    .release	= HCSR_driver_release,     /* Release method */
    .write		= HCSR_driver_write,       /* Write method */
    .read		= HCSR_driver_read,        /* Read method */
    .unlocked_ioctl        = HCSR_ioctl,
};





int __init HCSR_driver_init(void)   //initilize HCSR driver
{
	char name_dev[20],*temp;
	int i=0;
	int retval;
	printk("Number of devices : %d\n",num_devices);
	
	/* Registering the Miscellaneous drivers */
	//struct HCSR_dev *HCSR_devp[num_devices];
	for(i=0;i<num_devices;i++)   // registering misc devices
	{

	HCSR_devp[i] = (struct HCSR_dev *)kmalloc(sizeof(struct HCSR_dev), GFP_KERNEL);
	temp = (char *)kmalloc(sizeof(name_dev), GFP_KERNEL);
    HCSR_devp[i]->my_misc_dev.minor = i;
	sprintf(name_dev,"HCSR_%d",i); 
	sprintf(temp,name_dev); 
	HCSR_devp[i]->my_misc_dev.name=temp;
	strcpy(HCSR_devp[i]->name,name_dev);
	memset(name_dev, '\0', sizeof name_dev);

    HCSR_devp[i]->my_misc_dev.fops = &HCSR_fops;
	printk("Registering %s\n", HCSR_devp[i]->my_misc_dev.name);

    retval = misc_register(&HCSR_devp[i]->my_misc_dev);

	HCSR_devp[i]->head=0;
	HCSR_devp[i]->curr=0;
	HCSR_devp[i]->count=0;
	HCSR_devp[i]->distance = 0;
	HCSR_devp[i]->trigger_flag=0;    
	HCSR_devp[i]->expire_time=5;
	HCSR_devp[i]->data_available=0;
	HCSR_devp[i]->irq_num=0;
	HCSR_devp[i]->delay_flag=1;
	hrtimer_init(&HCSR_devp[i]->hrt,CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	
	if (!HCSR_devp[i]) {
		printk("Bad Kmalloc HCSR-DEVP\n"); return -ENOMEM;
	}

	HCSR_devp[i]->threads_counter=0;
	mutex_init(&HCSR_devp[i]->perdev_my_mutex);
	}
	mutex_init(&dev_my_mutex);
	mutex_init(&write_my_mutex);
	mutex_init(&open_my_mutex);

   // printk("------------------ test -3---------------------");
		for (i = 0; i <80; i++)
	{
		gpio_num[i]=-1;
	}


	return 0;
}
/* Driver Exit */
void __exit HCSR_driver_exit(void)
{
	int i;
	for(i=0;i<num_devices;i++) 
	{

	misc_deregister(&HCSR_devp[i]->my_misc_dev);
	kfree(HCSR_devp[i]);
	}
	printk("HCSR driver removed.\n");

}

module_init(HCSR_driver_init);
module_exit(HCSR_driver_exit);
MODULE_LICENSE("GPL v2");




//comands set table

void table_for_commands(int pin_1, int pin_2,void *ptr){  // 1: output, 2:input

	struct HCSR_dev *HCSR_devp;
	HCSR_devp=(struct HCSR_dev *)ptr;
    switch(pin_1){  //trig, output
        case 0:    
            //printk("trig pin is 0\n");
            gpio_num[11] = 11;
            gpio_num[32] = 32;
            if( gpio_request(11, "gpio_out_11") != 0 )  printk("gpio_out_11 error!\n");
            if( gpio_request(32, "dir_out_32") != 0 )  printk("dir_out_32 error!\n");
            HCSR_devp->conf.trigPin = 11;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value(32, 0);
            break;
        case 1:
           // printk("trig pin is 1\n");
            gpio_num[12] = 12;
            gpio_num[28] = 28;
            gpio_num[45] = 45;
            if( gpio_request(12, "gpio_out_12") != 0 )  printk("gpio_out_12 error!\n");
            if( gpio_request(28, "dir_out_28") != 0 )  printk("dir_out_28 error!\n");
            if( gpio_request(45, "pin_mux_45") != 0 )  printk("pin_mux_45 error!\n");
            HCSR_devp->conf.trigPin = 12;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(28, 0);
            gpio_set_value_cansleep(45, 0);
            break;
        case 2:
            //printk("trig pin is 2\n");
            gpio_num[13] = 13;
            gpio_num[34] = 34;
            gpio_num[77] = 77;
            if( gpio_request(13, "gpio_out_13") != 0 )  printk("gpio_out_13 error!\n");
            if( gpio_request(34, "dir_out_34") != 0 )  printk("dir_out_34 error!\n");
            if( gpio_request(77, "pin_mux_77") != 0 )  printk("pin_mux_77 error!\n");
            HCSR_devp->conf.trigPin = 13;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(34, 0);
            gpio_set_value_cansleep(77, 0);
            break;
        case 3:
            //printk("trig pin is 3\n");
            gpio_num[14] = 14;
            gpio_num[16]= 16;
            gpio_num[76]= 76;
            gpio_num[64]= 64;
            if( gpio_request(14, "gpio_out_14") != 0 )  printk("gpio_out_14 error!\n");
            if( gpio_request(16, "dir_out_16") != 0 )  printk("dir_out_16 error!\n");
            if( gpio_request(76, "pin_mux_76") != 0 )  printk("pin_mux_76 error!\n");
            if( gpio_request(64, "pin_mux_64") != 0 )  printk("pin_mux_64 error!\n");
            HCSR_devp->conf.trigPin = 14;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(16, 0);
            gpio_set_value_cansleep(76, 0);
            gpio_set_value_cansleep(64, 0);
            break;
        case 4:
            //printk("trig pin is 4\n");
            gpio_num[6] = 6;
            gpio_num[36]= 36;
            if( gpio_request(6, "gpio_out_6") != 0 )  printk("gpio_out_6 error!\n");
            if( gpio_request(36, "dir_out_36") != 0 )  printk("dir_out_36 error!\n");
            HCSR_devp->conf.trigPin = 6;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(36, 0);
            break;
        case 5:
            //printk("trig pin is 5\n");
            gpio_num[0] = 0;
            gpio_num[18] = 18;
            gpio_num[66] = 66;
            if( gpio_request(0, "gpio_out_0") != 0 )  printk("gpio_out_0 error!\n");
            if( gpio_request(18, "dir_out_18") != 0 )  printk("dir_out_18 error!\n");
            if( gpio_request(66, "pin_mux_66") != 0 )  printk("pin_mux_66 error!\n");
            HCSR_devp->conf.trigPin = 0;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(18, 0);
            gpio_set_value_cansleep(66, 0);
            break;
        case 6:
            //printk("trig pin is 6\n");
            gpio_num[1] = 1;
            gpio_num[20] = 20;
            gpio_num[68] = 68;
            if( gpio_request(1, "gpio_out_1") != 0 )  printk("gpio_out_1 error!\n");
            if( gpio_request(20, "dir_out_20") != 0 )  printk("dir_out_20 error!\n");
            if( gpio_request(68, "pin_mux_68") != 0 )  printk("pin_mux_68 error!\n");
            HCSR_devp->conf.trigPin = 1;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(20, 0);
            gpio_set_value_cansleep(68, 0);
            break;
        case 7:
            //printk("trig pin is 7\n");
            gpio_num[38] = 38;
            if( gpio_request(38, "gpio_out_38") != 0 )  printk("gpio_out_38 error!\n");
            HCSR_devp->conf.trigPin = 38;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 8:
            //printk("trig pin is 8\n");
            gpio_num[40] = 40;
            if( gpio_request(40, "gpio_out_40") != 0 )  printk("gpio_out_40 error!\n");
            HCSR_devp->conf.trigPin = 40;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
 
        case 9:
            //printk("trig pin is 9\n");
            gpio_num[4] = 4;
            gpio_num[22] = 22;
            gpio_num[70] = 70;
            if( gpio_request(4, "gpio_out_4") != 0 )  printk("gpio_out_4 error!\n");
            if( gpio_request(22, "dir_out_22") != 0 )  printk("dir_out_22 error!\n");
            if( gpio_request(70, "pin_mux_70") != 0 )  printk("pin_mux_70 error!\n");
            HCSR_devp->conf.trigPin = 4;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(22, 0);
            gpio_set_value_cansleep(70, 0);
            break;
        case 10:
            //printk("trig pin is 10\n");
            gpio_num[10] = 10;
            gpio_num[26] = 26;
            gpio_num[74] = 74;
            if( gpio_request(10, "gpio_out_10") != 0 )  printk("gpio_out_10 error!\n");
            if( gpio_request(26, "dir_out_26") != 0 )  printk("dir_out_26 error!\n");
            if( gpio_request(74, "pin_mux_74") != 0 )  printk("pin_mux_74 error!\n");
            HCSR_devp->conf.trigPin = 10;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(26, 0);
            gpio_set_value_cansleep(74, 0);
            break;
        case 11:
            //printk("trig pin is 11\n");
            gpio_num[5] = 5;
            gpio_num[24] = 24;
            gpio_num[44] = 44;
            gpio_num[72] = 72;
            if( gpio_request(5, "gpio_out_5") != 0 )  printk("gpio_out_5 error!\n");
            if( gpio_request(24, "dir_out_24") != 0 )  printk("dir_out_24 error!\n");
            if( gpio_request(44, "pin_mux_44") != 0 )  printk("pin_mux_44 error!\n");
            if( gpio_request(72, "pin_mux_72") != 0 )  printk("pin_mux_72 error!\n");
            HCSR_devp->conf.trigPin = 5;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(24, 0);
            gpio_set_value_cansleep(44, 0);
            gpio_set_value_cansleep(72, 0);
            break;
        case 12:
            //printk("trig pin is 12\n");
            gpio_num[15] = 15;
            gpio_num[42] = 42;
            if( gpio_request(15, "gpio_out_15") != 0 )  printk("gpio_out_15 error!\n");
            if( gpio_request(42, "dir_out_42") != 0 )  printk("dir_out_42 error!\n");
            HCSR_devp->conf.trigPin = 15;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(42, 0);
            break;
        case 13:
            //printk("trig pin is 13\n");
            gpio_num[7] = 7;
            gpio_num[30] = 30;
            gpio_num[46] = 46;
            if( gpio_request(7, "gpio_out_7") != 0 )  printk("gpio_out_7 error!\n");
            if( gpio_request(30, "dir_out_30") != 0 )  printk("dir_out_30 error!\n");
            if( gpio_request(46, "pin_mux_46") != 0 )  printk("pin_mux_46 error!\n");
            HCSR_devp->conf.trigPin = 7;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(30, 0);
            gpio_set_value_cansleep(46, 0);
            break;
        case 14:
            //printk("trig pin is 14\n");
            gpio_num[48] = 48;
            if( gpio_request(48, "gpio_out_48") != 0 )  printk("gpio_out_48 error!\n");
            HCSR_devp->conf.trigPin = 48;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 15:
            //printk("trig pin is 15\n");
            gpio_num[50] = 50;
            if( gpio_request(50, "gpio_out_50") != 0 )  printk("gpio_out_50 error!\n");
            HCSR_devp->conf.trigPin = 50;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 16:
            //printk("trig pin is 16\n");
            gpio_num[52] = 52;
            if( gpio_request(52, "gpio_out_52") != 0 )  printk("gpio_out_52 error!\n");
            HCSR_devp->conf.trigPin = 52;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 17:
            //printk("trig pin is 17\n");
            gpio_num[54] = 54;
            if( gpio_request(54, "gpio_out_54") != 0 )  printk("gpio_out_54 error!\n");
            HCSR_devp->conf.trigPin = 54;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            break;
        case 18:
            //printk("trig pin is 18\n");
            gpio_num[56] = 56;
            gpio_num[60] = 60;
            gpio_num[78] = 78;
            if( gpio_request(56, "gpio_out_56") != 0 )  printk("gpio_out_56 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(78, "pin_mux_78") != 0 )  printk("pin_mux_78 error!\n");
            HCSR_devp->conf.trigPin = 56;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(60, 1);
            gpio_set_value_cansleep(78, 1);
            break;
        case 19:
            //printk("trig pin is 19\n");
            gpio_num[58] = 58;
            gpio_num[60] = 60;
            gpio_num[79] = 79;
            if( gpio_request(58, "gpio_out_58") != 0 )  printk("gpio_out_58 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(79, "pin_mux_79") != 0 )  printk("pin_mux_79 error!\n");
            HCSR_devp->conf.trigPin = 58;
            //printk("HCSR_devp->conf.trigPin is:%d \n", HCSR_devp->conf.trigPin);
            gpio_direction_output(HCSR_devp->conf.trigPin, 0);
            gpio_set_value_cansleep(60, 1);
            gpio_set_value_cansleep(79, 1);
            break;
    }
 
    switch(pin_2){  //echo, input
    	int ret;
        case 0:    
            gpio_num[11] = 11;
            gpio_num[32] = 32;
            if( gpio_request(11, "gpio_in_11") != 0 )  printk("gpio_in_11 error!\n");
            if( gpio_request(32, "dir_in_32") != 0 )  printk("dir_in_32 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[11]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise",(void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 11;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(32,1);
            break;
        case 1:
            gpio_num[12] = 12;
            gpio_num[28] = 28;
            gpio_num[45] = 45;
            if( gpio_request(12, "gpio_in_12") != 0 )  printk("gpio_in_12 error!\n");
            if( gpio_request(28, "dir_in_28") != 0 )  printk("dir_in_28 error!\n");
            if( gpio_request(45, "pin_mux_45") != 0 )  printk("pin_mux_45 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[12]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 12;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(28,1);
            gpio_set_value_cansleep(45, 0);
            break;
        case 2:
            gpio_num[13] = 13;
            gpio_num[34] = 34;
            gpio_num[77] = 77;
            if( gpio_request(13, "gpio_in_13") != 0 )  printk("gpio_in_13 error!\n");
            if( gpio_request(34, "dir_in_34") != 0 )  printk("dir_in_34 error!\n");
            if( gpio_request(77, "pin_mux_77") != 0 )  printk("pin_mux_77 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[13]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 13;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(34,1);
            gpio_set_value_cansleep(77, 0);
            break;
        case 3:
            gpio_num[14] = 14;
            gpio_num[16] = 16;
            gpio_num[76] = 76;
            gpio_num[64] = 64;
            if( gpio_request(14, "gpio_in_14") != 0 )  printk("gpio_in_14 error!\n");
            if( gpio_request(16, "dir_in_16") != 0 )  printk("dir_in_16 error!\n");
            if( gpio_request(76, "pin_mux_76") != 0 )  printk("pin_mux_76 error!\n");
            if( gpio_request(64, "pin_mux_64") != 0 )  printk("pin_mux_64 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[14]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 14;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(16,1);
            gpio_set_value_cansleep(76, 0);
            gpio_set_value_cansleep(64, 0);
            break;
        case 4:
        	gpio_num[6] = 6;
            if( gpio_request(6, "gpio_in_6") != 0 )  printk("gpio_in_6 error!\n");
            HCSR_devp->irq_num= gpio_to_irq(gpio_num[6]);
			if(HCSR_devp->irq_num<0)
			{printk("IRQ NUMBER ERROR\n");}
			ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
			if (ret < 0) 
			{printk("Error in request_irq\n");}
            HCSR_devp->conf.echoPin = 6;
            gpio_direction_input(HCSR_devp->conf.echoPin);

            break;
        case 5:
            gpio_num[0] = 0;
            gpio_num[18] = 18;
            gpio_num[66] = 66;
            if( gpio_request(0, "gpio_in_0") != 0 )  printk("gpio_in_0 error!\n");
            if( gpio_request(18, "dir_in_18") != 0 )  printk("dir_in_18 error!\n");
            if( gpio_request(66, "pin_mux_66") != 0 )  printk("pin_mux_66 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[0]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 0;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(18, 1);
            gpio_set_value_cansleep(66, 0);
            break;
        case 6:
            gpio_num[1] = 1;
            gpio_num[20] = 20;
            gpio_num[68] = 68;
            if( gpio_request(1, "gpio_in_1") != 0 )  printk("gpio_in_1 error!\n");
            if( gpio_request(20, "dir_in_20") != 0 )  printk("dir_in_20 error!\n");
            if( gpio_request(68, "pin_mux_68") != 0 )  printk("pin_mux_68 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[1]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 1;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(20,1);
            gpio_set_value_cansleep(68, 0);
            break;
        case 7:
            gpio_num[38] = 38;
            if( gpio_request(38, "gpio_in_38") != 0 )  printk("gpio_in_38 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[38]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 38;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 8:
            gpio_num[40] = 40;
            if( gpio_request(40, "gpio_in_40") != 0 )  printk("gpio_in_40 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[40]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            gpio_direction_input(HCSR_devp->conf.echoPin);
            HCSR_devp->conf.echoPin = 40;
            break;
        case 9:
            gpio_num[4] = 4;
            gpio_num[22] = 22;
            gpio_num[70] = 70;
            if( gpio_request(4, "gpio_in_4") != 0 )  printk("gpio_in_4 error!\n");
            if( gpio_request(22, "dir_in_22") != 0 )  printk("dir_in_22 error!\n");
            if( gpio_request(70, "pin_mux_70") != 0 )  printk("pin_mux_70 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[4]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 4;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(22,1);
            gpio_set_value_cansleep(70, 0);
            break;
        case 10:
            gpio_num[10] = 10;
            gpio_num[26] = 26;
            gpio_num[74] = 74;
            if( gpio_request(10, "gpio_in_10") != 0 )  printk("gpio_in_10 error!\n");
            if( gpio_request(26, "dir_in_26") != 0 )  printk("dir_in_26 error!\n");
            if( gpio_request(74, "pin_mux_74") != 0 )  printk("pin_mux_74 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[10]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 10;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(26,1);
            gpio_set_value_cansleep(74, 0);
            break;
        case 11:
            //printk("echo pin is 11\n");
            gpio_num[5] = 5;
            gpio_num[24] = 24;
            gpio_num[44] = 44;
            gpio_num[72] = 72;
            if( gpio_request(5, "gpio_in_5") != 0 )  printk("gpio_in_5 error!\n");
            if( gpio_request(24, "dir_in_24") != 0 )  printk("dir_in_24 error!\n");
            if( gpio_request(44, "pin_mux_44") != 0 )  printk("pin_mux_44 error!\n");
            if( gpio_request(72, "pin_mux_72") != 0 )  printk("pin_mux_72 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[5]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 5;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(24,1);
            gpio_set_value_cansleep(44, 0);
            gpio_set_value_cansleep(72, 0);
            break;
        case 12:
            gpio_num[15] = 15;
            gpio_num[42] = 42;
            if( gpio_request(15, "gpio_in_15") != 0 )  printk("gpio_in_15 error!\n");
            if( gpio_request(42, "dir_in_42") != 0 )  printk("dir_in_42 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[15]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 15;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(42,1);
            break;
        case 13:
            gpio_num[7] = 7;
            gpio_num[30] = 30;
            gpio_num[46] = 46;
            if( gpio_request(7, "gpio_in_7") != 0 )  printk("gpio_in_7 error!\n");
            if( gpio_request(30, "dir_in_30") != 0 )  printk("dir_in_30 error!\n");
            if( gpio_request(46, "pin_mux_46") != 0 )  printk("pin_mux_46 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[7]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 7;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(30,1 );
            gpio_set_value_cansleep(46, 0);
            break;
        case 14:
            gpio_num[48] = 48;
            if( gpio_request(48, "gpio_in_48") != 0 )  printk("gpio_in_48 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[48]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num, (irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 48;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 15:
            gpio_num[50] = 50;
            if( gpio_request(50, "gpio_in_50") != 0 )  printk("gpio_in_50 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[50]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 50;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 16:
            gpio_num[52] = 52;
            if( gpio_request(52, "gpio_in_52") != 0 )  printk("gpio_in_52 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[52]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 52;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 17:
            gpio_num[54] = 54;
            if( gpio_request(54, "gpio_in_54") != 0 )  printk("gpio_in_54 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[54]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 54;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            break;
        case 18:
            gpio_num[56] = 56;
            gpio_num[60] = 60;
            gpio_num[78] = 78;
            if( gpio_request(56, "gpio_in_56") != 0 )  printk("gpio_in_56 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(78, "pin_mux_78") != 0 )  printk("pin_mux_78 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[56]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 56;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(60, 1);
            gpio_set_value_cansleep(78, 1);
            break;
        case 19:
            gpio_num[58] = 58;
            gpio_num[60] = 60;
            gpio_num[79] = 79;
            if( gpio_request(58, "gpio_in_58") != 0 )  printk("gpio_in_58 error!\n");
            if( gpio_request(60, "pin_mux_60") != 0 )  printk("pin_mux_60 error!\n");
            if( gpio_request(79, "pin_mux_79") != 0 )  printk("pin_mux_79 error!\n");
            HCSR_devp->irq_num = gpio_to_irq(gpio_num[58]);
            if(HCSR_devp->irq_num<0)  printk("IRQ NUMBER ERROR\n");
            ret = request_irq(HCSR_devp->irq_num,(irq_handler_t) handling_irq, IRQF_TRIGGER_RISING, "rise", (void*)HCSR_devp);
            if (ret < 0)   printk("Error in request_irq\n");
            HCSR_devp->conf.echoPin = 58;
            gpio_direction_input(HCSR_devp->conf.echoPin);
            gpio_set_value_cansleep(60, 1);
            gpio_set_value_cansleep(79, 1);
            break;
    }
}
