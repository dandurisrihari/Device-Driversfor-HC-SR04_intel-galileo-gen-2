ASU ID : 1217423272
NAME : DANDURI DATTA MANIKANTA SRIHARI


To use the program run Make file by typing "make" in terminal transfer the files in to board using scp command "scp main root@192.168.1.5:
scp HCSR_drv.ko root@192.168.1.5:" Replace the ip address with the static IP you have assigned to your board. 

after transferring use "insmod HCSR_drv.ko num_devices=2" to create two devices. Here num_devices is the parameter to be passed during run time
run the program by "./main" you can remove module by "rmmod HCSR_drv"


This document goes into detail about the driver program HCSR_drv.c which is the kernel driver for the HCSR04 ultrasonic sensor. The repository additionally contains a test program main.c and the corresponding make file for the program.


main.c - The user test program to test various scenarios.

HCSR_drv.c - The driver program for the HCSR sensor.

Makefile - Make file to compile the program.

Readme - readme file



hardware connections:

I connected the vcc pins of both sensors to 5v, GND pins of both are connected to GND. 

Device 1
trig pin 7
echo pin 3

Device 2
trig pin 8
echo pin 10



<i> the test program (main.c) </i>

In the test program, 

It opens the device. Then it calls the ioctl function to configure the trig pin, echo pin, the number of samples (m), and the sampling period (delta). Then multiple (2) threads are created. Each thread calls the read and write functions for each of the device. The write and read functions happen a number of times and after reading the values are printed to the console.

SAMPLE OUTPUT

root@quark:~# insmod HCSR_drv.ko num_devices=2
[ 8829.595901] Number of devices : 2
[ 8829.599266] Registering HCSR_0
[ 8829.616841] Registering HCSR_1
root@quark:~# ./main
This is a user t[ 8833.769749] minor number is 0
est program to d[ 8833.773516] minor number is 1
emonstrated the functionality of the HSCR driver
Time stamp : 3534883334628
Distance   : 6 cm
Time stamp : 3534937203732
Distance   : 3 cm
Time stamp : 3535282848342
Distance   : 6 cm
Time stamp : 3535336690138
Distance   : 3 cm
Time stamp : 3535682151032
Distance   : 6 cm
Time stamp : 3535735989522
Distance   : 3 cm
Time stamp : 3536081523122
Distance   : 5 cm
Time stamp : 3536135377284
Distance   : 3 cm
Time stamp : 3536480908726
Distance   : 6 cm
Time stamp : 3536534762606
Distance   : 3 cm
Time stamp : 3536880349162
Distance   : 6 cm
Time stamp : 3536934194160
Distance   : 3 cm
Time stamp : 3537279625760
Distance   : 6 cm
Time stamp : 3537333457606
Distance   : 3 cm
Time stamp : 3537679092002
Distance   : 6 cm
Time stamp : 3537732937584
Distance   : 3 cm
Time stamp : 3538078430406
Distance   : 6 cm
Time stamp : 3538132268868
Distance   : 3 cm
Time stamp : 3538477869778
Distance   : 6 cm
Time stamp : 3538531690142
Distance   : 3 cm
[ 8845.793209] Clearing buffer
[ 8845.821383] Clearing buffer















































