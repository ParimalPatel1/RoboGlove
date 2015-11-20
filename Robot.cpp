/*
	Robot Control Program 1
*/
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <sstream>
#include <ctype.h>
#include <math.h>
#include <sys/time.h>

struct timeval startTimeVal;
#define KEY_UP 8//72
#define KEY_DOWN 2//80
#define KEY_LEFT 4//75
#define KEY_RIGHT 6//77

using namespace std;

#ifndef RPIPWM1_H
#define PRIPWM1_H
#include <limits.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
/***********************************************************************
 * Author: Hussam al-Hertani (Hertaville.com)
 * Others are free to modify and use this code as they see fit so long as they
 * give credit to the author.
 *
 * The author is not liable for any kind of damage caused by this software.
 *
 * Acknowledgements: This 'C++' class is based on 'C' code available from :
 * - code from http://elinux.org/RPi_Low-level_peripherals
 * - http://www.raspberrypi.org/phpBB3/viewtopic.php?t=8467&p=124620 for PWM initialization
 * - frank's code...http://www.frank-buss.de/raspberrypi/pwm.c
 * - Gertboard's C source code
 *
 * The rpiPWM1 class provides a direct memory mapped (register-based)
 * interface to the PWM1 hardware on the Raspberry Pi's BCM2835 SOC.
 * The BCM2835 SOC was two PWM subsystems, PWM1 & PWM2. This code will
 * enable access to the PWM1 subsystem which outputs the PWM signal on
 * GPIO18 (ALT5).
 *
 * The class enables setting the Frequency (Max 19.2MHz), PWM resolution(4095),
 * DutyCycle and PWM Mode to be used. The Duty Cycle can be set as a
 * percentage (setDutyCycle() or setDutyCycleForce()) or as a  function
 * of the PWM Resoultion (setDutyCycleCount())
 *
 * Two PWM modes exist:
 *  - MSMODE - This is the traditional PWM Mode i.e. if PWM Resolution
 *             (counts) is 1024 and the dutyCycle (Pulse on time) is 512 (in counts or 50%)
 *             then the waveform would look like:
 *             |||||||||||||||||_________________
 *             MSMODE is ideal for servos and other applications that
 *             require classical PWM waveforms
 * - PWMMODE - Is a slightly modified version of the traditional PWM Mode
 *             described above. The duty cycle or ON time is still unchanged
 *             within the period but is distributed across the entire period instead
 *             on being concentrated in the first part of the period..i.e.if PWM Resolution
 *             (counts) is 1024 and the dutyCycle (Pulse on time) is 512 (in counts or 50%)
 *             then the waveform would look like:
 *             |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_
 *             This mode is ideal if you want to pass the signal through
 *             a low pass filter to obtain an analog signal equivalent to the . The even
 *             distibution of the ON time through the entire period
 *             significantly reduces ripple caused by using a simple RC
 *             low pass filter
 *
 * When setting the frequency via the constructor or 'setFrequency' method, one is strictly
 * requesting a particular frequency. The code will do its best to get as close as possible to
 * the requested frequency but it is likely to not create a PWM at the exact requested Frequency.
 * As an example say that we want to create a PWM waveform with a resolution of 256 counts (8-bit)
 * and a frequency of 8KHz....We get the divisor using the following algorithm
 *
 * Waveform  period = 1 / 8KHz = 0.125ms
 * Duration of a single count = period/256 = 0.125ms / 256 = 0.488us
 * Frequency of a single count = 1 / Duration of a single count = 1 / 0.488us = 2.048MHz
 * Divisor value = floor (PWM Clock Frequency / Frequency of a single count) = floor (19.2MHz / 2.048MHz) = floor(9.375) = 9
 *
 * With a divisor of 9, the actual Waveform Frequency = 1/((1/(19.2MHz/9))*256) = 8.333 KHz
 *
 * The actual Frequency will generally deviate further from the desired frequency as the count value (PWM resolution)
 * increases. For example, let's create a PWM waveform with a resolution of 1024 counts (10-bit)
 * and the same frequency as the above example:
 *
 * Waveform  period = 1 / 8KHz = 0.125ms
 * Duration of a single count = period/1024 = 0.125ms / 1024 = 122.070ns
 * Frequency of a single count = 1 / Duration of a single count = 1 / 122.070ns = 8.192MHz
 * Divisor value = floor (PWM Clock Frequency / Frequency of a single count)
 *               = floor (19.2MHz / 8.192MHz) = floor(2.34) = 2
 *
 * With a Divisor of 2, the actual Waveform Frequency = 1/((1/(19.2MHz/2))*1024) = 9.375KHz
 *
 * DIVISOR MUST BE AT LEAST 2....SO PICK YOUR COUNT AND DESIRED FREQUENCY VALUES CAREFULLY!!!!!
 * i.e MAXIMUM FREQUENCY FOR 10-BIT RESOLUTION (COUNT=1024) IS 9.375KHz
 *  &  MAXIMUM FREQUENCY FOR 8-BIT RESOLUTION (COUNT=256) IS 37.5KHz
 *
 * WARNING:    The RPI uses the PWM1 subsystem to produce audio. As such
 *             please refrain from playing audio on the RPI while this code
 *             is running.
 * *********************************************************************/
class rpiPWM1 {

public:
	rpiPWM1(){
		this->clk = mapRegAddr(CLOCK_BASE);// map PWM clock registers into memory
		this->pwm = mapRegAddr(PWM_BASE); //map PWM registers into memory
		this->gpio = mapRegAddr(GPIO_BASE);// map GPIO registers into memory 
		this->frequency = 1000.0; // set frequency
		this->counts = 256; //set PWM resolution
		this->dutyCycle = 50.0; //set duty cycle
		this->mode = PWMMODE; // set pwm mode
		configPWM1Pin(); //configure GPIO18 to ALT15 (PWM output)
		configPWM1();   // configure PWM1
	}
	// default constructor configures GPIO18 for PWM and Frequency 1000Hz,
	// PWM resolution (counts) 256, Duty Cycle 50% and PWM mode is 'PWMMODE'
	rpiPWM1(double Hz, unsigned int cnts, double duty,  int m){
		this->clk = mapRegAddr(CLOCK_BASE);
		this->gpio = mapRegAddr(GPIO_BASE);
		this->pwm = mapRegAddr(PWM_BASE);
		if( (cnts < 0) || (cnts > UINT_MAX) ) {
			printf("counts value must be between 0-%d\n",UINT_MAX);
			exit(1);
		}
		if ((Hz < 1e-5) || (Hz > 19200000.0f)){
			printf("frequency value must be between 0-19200000\n");
			exit(1);
		}

		if( (duty < 1e-5) || (duty> 99.99999) ) {
			printf("dutyCycle value must be between 0-99.99999\n");
			exit(1);
		}
		if( (m != PWMMODE) && (m != MSMODE) ) {
			printf("mode must be either PWMMODE(1) or MSMODE(2)\n");
			exit(1);
		}
		this->frequency = Hz;
		this->counts = cnts;
		this->dutyCycle = duty;
		this->mode = m;
		configPWM1Pin();
		configPWM1();  
	}
	//overloaded constructor..allows user to set initial values for Frequency,
	//PWM resolution, Duty Cycle and PWM mode.
	~rpiPWM1(){
		//lets put the PWM peripheral registers in their original state   
		*(pwm + PWM_CTL) = 0;
		*(pwm + PWM_RNG1) = 0x20;
		*(pwm + PWM_DAT1) = 0;
		// unmap the memory block containing PWM registers
		if(munmap((void*)pwm, BLOCK_SIZE) < 0){
			perror("munmap (pwm) failed");
			exit(1);
		}
		//lets put the PWM Clock peripheral registers in their original state  
		//kill PWM clock
		*(clk + PWMCLK_CNTL) = 0x5A000000 | (1 << 5);
		usleep(10);

		// wait until busy flag is set
		while ( (*(clk + PWMCLK_CNTL)) & 0x00000080){}

		//reset divisor
		*(clk + PWMCLK_DIV) = 0x5A000000;
		usleep(10);

		// source=osc and enable clock
		*(clk + PWMCLK_CNTL) = 0x5A000011;

		// unmap the memory block containing PWM Clock registers
		if(munmap((void*)clk, BLOCK_SIZE) < 0){
			perror("munmap (clk) failed");
			exit(1);
		}
	  

	   //lets put the GPIO peripheral registers in their original state
	   //first put it in input mode (default)
	   //taken from #define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
	   *(gpio+1) &= ~(7 << 24);
	   //then munmap
		if(munmap((void*)gpio, BLOCK_SIZE) < 0){
			perror("munmap (gpio) failed");
			exit(1);
		}
		//lets put the GPIO peripheral registers in their original state
		//first put it in input mode (default)
		//taken from #define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
		*(gpio+1) &= ~(7 << 24);
		//then munmap
		if(munmap((void*)gpio, BLOCK_SIZE) < 0){
			perror("munmap (gpio) failed");
			exit(1);
		}
	}
	// Destructor....safely releases all mapped memory and puts all used peripherals
	// (PWM clock, PWM peripheral and GPIO peripheral in their initial states
	unsigned int setFrequency(const double &hz){
		unsigned int retVal = 0;
		if (hz < 1e-5 || hz > 19200000.0f){ // make sure that Frequency is valid 
			retVal = ERRFREQ; //if not return error code 
		}
		else{
			this->frequency = hz;
			configPWM1();
		}
		return retVal; // return 0 for success.....
	}
	// Sets Frequency and reinitializes PWM peripheral
	unsigned int setCounts(const unsigned int &cnts){
		unsigned int retVal = 0;
		if( (cnts < 0) || (cnts > UINT_MAX) ) {
			retVal = ERRCOUNT;
		}
		else{
			this->counts = cnts;
			configPWM1();    
		}
		return retVal;
	}
	// Sets PWM resolution (counts) and reinitializes PWM peripheral
	unsigned int setDutyCycle(const double &duty, const int &m){
		unsigned int bitCount = 0;
		unsigned int retVal = 0;
		if( (duty < 1e-5) || (duty > 99.99999) ) {
			retVal = ERRDUTY;
		}
		else {
			this->dutyCycle = duty;
			bitCount = (int) ((this->dutyCycle/100.0) * this->counts);
			*(pwm + PWM_DAT1) = bitCount;  
		}
		return retVal;
	}
	// Sets Duty Cycle as a Percentage (Fast)
	unsigned int setDutyCycleForce(const double &duty, const int &m){
		int retVal = 0;
		if( (m != PWMMODE) && (m != MSMODE) ) {
			retVal = ERRMODE;
		}
		else if( (duty < 1e-5) || (duty > 99.99999) ) {
			retVal = ERRDUTY;
		}
		else{
			this->mode = m;
			this->dutyCycle = duty;
			// disable PWM & start from a clean slate
			*(pwm + PWM_CTL) = 0;
			// needs some time until the PWM module gets disabled, without the delay the PWM module crashs
			usleep(10); 
			// set the number of counts that constitute a period 
			*(pwm + PWM_RNG1) = this->counts;
			//set  duty cycle
			*(pwm + PWM_DAT1) = (int) ((this->dutyCycle/100.0) * this->counts);
			// start PWM1 in
			if(this->mode == PWMMODE) //PWM mode
				*(pwm + PWM_CTL) |= (1 << 0);
			else // M/S Mode
				*(pwm + PWM_CTL) |= ( (1 << 7) | (1 << 0) );  
			}
		return retVal;
	}
	// Sets Duty Cycle as a count value (Fast) i.e. if counts is 1024
	// and 'duty' is set to 512, a 50% duty cycle is achieved
	unsigned int setDutyCycleCount(const unsigned int &dutyCycleCnts ){
	   unsigned int retVal = 0;
	   if( (dutyCycleCnts < 0) || ( dutyCycleCnts > this->counts )) {
		retVal = ERRDUTY;
		}
		else{
			this->dutyCycle = ((dutyCycleCnts * 1.0)/ this->counts) * 100.0;
			*(pwm + PWM_DAT1) = dutyCycleCnts;
		}
		return retVal;
	}
	// disables PWM peripheral first,
	//Sets Duty Cycle as a Percentage and PWM mode...
	// then enables PWM peripheral
	unsigned int setMode(const  int &m){
		unsigned int retVal = 0;
		if( (m != PWMMODE) && (m != MSMODE) ) {
			retVal = ERRMODE;
		}
		else{
			this->mode = m;
			setDutyCycleForce(this->dutyCycle, this->mode);
		 }
		 
		return retVal;
	}
	// sets PWM mode...calls 'setDutyCycleForce()'
	double getFrequency() const{ return this->frequency;}
	int getCounts() const { return this->counts;}
	int getDivisor() const {return this->divisor;}
	double  getDutyCycle() const {return this->dutyCycle;}
	int getMode() const{return this->mode;}

	//Public constants
	static const int PWMMODE = 1;
	static const int MSMODE = 2;
	//Two PWM modes
	static const int ERRFREQ = 1;
	static const int ERRCOUNT = 2;
	static const int ERRDUTY = 3;
	static const int ERRMODE = 4;
	//Error Codes

private:
	//Private constants
	static const int BCM2708_PERI_BASE = 0x20000000;
	static const int PWM_BASE = (BCM2708_PERI_BASE + 0x20C000); /* PWM controller */
	static const int CLOCK_BASE = (BCM2708_PERI_BASE + 0x101000); /* Clock controller */
	static const int GPIO_BASE = (BCM2708_PERI_BASE + 0x200000); /* GPIO controller */
	//Base register addresses
	static const int PWM_CTL = 0;
	static const int PWM_RNG1 = 4;
	static const int PWM_DAT1 = 5;
	static const int PWMCLK_CNTL= 40;
	static const int PWMCLK_DIV = 41;
	// Register addresses offsets divided by 4 (register addresses are word (32-bit) aligned
	static const int BLOCK_SIZE = 4096;
	// Block size.....every time mmap() is called a 4KB
	//section of real physical memory is mapped into the memory of
	//the process
	//of the the PWM1 peripheral
	double frequency; // PWM frequency
	double dutyCycle; //PWM duty Cycle (%)
	unsigned int counts; // PWM resolution
	unsigned int divisor; // divisor value
	int mode;  // PWM mode
	volatile unsigned *clk, *pwm, *gpio; // pointers to the memory mapped sections
	//of our process memory
	  volatile unsigned *mapRegAddr(unsigned long baseAddr){
		    int mem_fd = 0;
			void *regAddrMap = MAP_FAILED;
			/* open /dev/mem.....need to run program as root i.e. use sudo or su */
			if (!mem_fd) {
				if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
				 perror("can't open /dev/mem");
				  exit (1);
				}
			}
			/* mmap IO */
			regAddrMap = mmap(
			  NULL,             //Any adddress in our space will do
			  BLOCK_SIZE,       //Map length
			  PROT_READ|PROT_WRITE|PROT_EXEC,// Enable reading & writting to mapped memory
			  MAP_SHARED|MAP_LOCKED,       //Shared with other processes
			  mem_fd,           //File to map
			  baseAddr         //Offset to base address
			);
			if (regAddrMap == MAP_FAILED) {
			  perror("mmap error");
			  close(mem_fd);
			  exit (1);
			}
			if(close(mem_fd) < 0){ //No need to keep mem_fd open after mmap
				//i.e. we can close /dev/mem
				perror("couldn't close /dev/mem file descriptor");
				exit(1);
			}	
			return (volatile unsigned *)regAddrMap;
	  }
	  // this function is used to map physical memory
	  void configPWM1Pin(){
		/*GPIO 18 in ALT5 mode for PWM0 */
		// Let's first set pin 18 to input
		//taken from #define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
		*(gpio+1) &= ~(7 << 24);
		//then set it to ALT5 function PWM0
		//taken from #define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
		*(gpio+1) |= (2<<24);
	  }
	  //this function sets GPIO18 to the alternat function 5 (ALT5)
	  // to enable the pin to output the PWM waveforms generated by PWM1
	  void configPWM1(){
		double period;
		double countDuration;
		// stop clock and waiting for busy flag doesn't work, so kill clock
		*(clk + PWMCLK_CNTL) = 0x5A000000 | (1 << 5);
		usleep(10);  
		// wait until busy flag is set 
		while ( (*(clk + PWMCLK_CNTL)) & 0x00000080){}   
		//calculate divisor value for PWM1 clock...base frequency is 19.2MHz
		period = 1.0/this->frequency;
		countDuration = period/(counts*1.0f);
		this->divisor = (int)(19200000.0f / (1.0/countDuration));
		if( this->divisor < 0 || this->divisor > 4095 ) {
		printf("divisor value must be between 0-4095\n");
		exit(-1);
		}
		//set divisor
		*(clk + PWMCLK_DIV) = 0x5A000000 | (this->divisor << 12);
		// source=osc and enable clock
		*(clk + PWMCLK_CNTL) = 0x5A000011;
		// disable PWM & start from a clean slate
		*(pwm + PWM_CTL) = 0;
		// needs some time until the PWM module gets disabled, without the delay the PWM module crashs
		usleep(10); 
		// set the number of counts that constitute a period with 0 for 20 milliseconds = 320 bits
		*(pwm + PWM_RNG1) = this->counts; usleep(10);
		//set  duty cycle
		*(pwm + PWM_DAT1) = (int) ((this->dutyCycle/100.0) * this->counts); usleep(10);
		// start PWM1 in
		if(this->mode == PWMMODE) //PWM mode 
		*(pwm + PWM_CTL) |= (1 << 0); 
		else // M/S Mode
		*(pwm + PWM_CTL) |= ( (1 << 7) | (1 << 0) );
	}
	//This function is responsible for the global configuration and initialixation
};
#endif

// Class to control GPIO
class GPIO{
	private:
		string gpioPinGPIVal;
	public:
		GPIO(string gpioPin = "4"){
			gpioPinGPIVal = gpioPin;
		}
		int export_gpio(){
			string str = "/sys/class/gpio/export";
			//cout<<str<<endl;
			ofstream exportgpio(str.c_str());
			if(exportgpio < 0){
				cout<<"Error: GPIO export failed: GPIO - "<<gpioPinGPIVal<<endl;
				return -1;
			}
			exportgpio << gpioPinGPIVal;
			exportgpio.close(); //close export file
		}
		int unexport_gpio(){
			string str = "/sys/class/gpio/unexport";
			//cout<<str<<endl;
			ofstream unexportgpio(str.c_str());
			if(unexportgpio < 0){
				cout<<"Error: GPIO unexport failed: GPIO - "<<gpioPinGPIVal<<endl;
				return -1;
			}
			unexportgpio << gpioPinGPIVal;
			unexportgpio.close(); //close export file
		}
		int setdir_gpio(string dir){
			string setdir_str ="/sys/class/gpio/gpio" + gpioPinGPIVal + "/direction";
			//cout<<setdir_str<<endl;
			ofstream setdirgpio(setdir_str.c_str()); // open direction file for gpio
			if (setdirgpio < 0){
				cout << "Error: Unable to set direction of GPIO "<< gpioPinGPIVal <<" ."<< endl;
				return -1;
			}
			setdirgpio << dir; //write direction to direction file
			setdirgpio.close(); // close direction file
			return 0;
		}
		int setval_gpio(string val){
			string setval_str = "/sys/class/gpio/gpio" + gpioPinGPIVal + "/value";
			//cout<<setval_str<<endl;
			ofstream setvalgpio(setval_str.c_str()); // open value file for gpio
			if (setvalgpio < 0){
				cout << "Error: Unable to set the value of GPIO"<< gpioPinGPIVal <<" ."<< endl;
				return -1;
			}

			setvalgpio << val ;//write value to value file
			setvalgpio.close();// close value file
			return 0;
		}
		int getval_gpio(string& val){
			string getval_str = "/sys/class/gpio/gpio" + gpioPinGPIVal + "/value";
			//cout<<getval_str<<endl;
			ifstream getvalgpio(getval_str.c_str());// open value file for gpio
			if (getvalgpio < 0){
				cout << " OPERATION FAILED: Unable to get value of GPIO"<< gpioPinGPIVal <<" ."<< endl;
				return -1;
			}

			getvalgpio >> val ;  //read gpio value

			if(val != "0")
				val = "1";
			else
				val = "0";

			getvalgpio.close(); //close the value file
			return 0;
		}
		string get_gpionum(){
			return gpioPinGPIVal;
		}
};

double getTime(){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    double sec = tv.tv_sec - startTimeVal.tv_sec;
    double usec = tv.tv_usec - startTimeVal.tv_usec;
    return (sec + usec/1000000.0);  
}
void manuallyCount(){
	//initializeGPIO(gpio22,"22","out");
	GPIO *gpio18;
	//wheel incoder input value GPIO
	gpio18 = new GPIO("18");
	gpio18->export_gpio();
	gpio18->setdir_gpio("in");
	int tick_counter = 0; // counts low to high changes on GPIO 18
    string wheel_current = "14"; // current state of GPio
    string wheel_next; // temporary value for checking when the value changes.
    int error = 0;
    //cout << "program starts here:" << endl;
    //error = gpio18->getval_gpio(wheel_current);
	//cout << wheel_current << endl;
	while(1){
		//Wheel encoder Program Start
		//error = gpio18->getval_gpio(wheel_current);
		//cout << wheel_current << endl;
		do {   
			error = gpio18->getval_gpio(wheel_current);  
		}while (wheel_current!= "0"); // polls for gpio to return 0;
		//cout << "at zero" << endl;
		do {
			error = gpio18->getval_gpio(wheel_current);
		}while (wheel_current != "1"); // polls for gpio to return 1;
		tick_counter++; // we have gone for zero to 1, increment counter
		cout << "Number of Ticks %i: " << tick_counter << endl;
	}
	
}
void goThisDistance(){
	//initializeGPIO(gpio22,"22","out");
	GPIO *gpio18;
	GPIO *gpio4,*gpio17;
	//initializeGPIO(gpio4,"4","out");
	gpio4 = new GPIO("4");
	gpio4->unexport_gpio();
	gpio4->export_gpio();
	gpio4->setdir_gpio("out");
	//initializeGPIO(gpio17,"17","out");
	gpio17 = new GPIO("17");
	gpio17->unexport_gpio();
	gpio17->export_gpio();
	gpio17->setdir_gpio("out");
	//wheel incoder input value GPIO
	gpio18 = new GPIO("18");
	gpio18->unexport_gpio();
	gpio18->export_gpio();
	gpio18->setdir_gpio("in");
	
	int tick_counter = 0; // counts low to high changes on GPIO 18
    string wheel_current = "14"; // current state of GPio
    string wheel_next; // temporary value for checking when the value changes.
    int error = 0;
    //cout << "program starts here:" << endl;
    //error = gpio18->getval_gpio(wheel_current);  
	//cout << wheel_current << endl;
	int distance = 0, n = 0;
	while(1){
		tick_counter = 0;
		cout<< "Enter Distance(unit: cm): ";
		cout<<">> ";
		cin >> distance;
		tick_counter = ceil((double)distance/1.55);
		tick_counter /= 2;
		cout<<"num of ticks: "<<tick_counter<<endl;
		cout<< "Moving Forward..." << endl;
		gettimeofday(&startTimeVal,NULL);
		gpio4->setval_gpio("1");
		gpio17->setval_gpio("0");
		while(1){
			//Wheel encoder Program Start
			//error = gpio18->getval_gpio(wheel_current);
			//cout << wheel_current << endl;
			gpio18->getval_gpio(wheel_current);
			if(wheel_current == "1"){
				do {   error = gpio18->getval_gpio(wheel_current);  }
				while (wheel_current!= "0"); // polls for gpio to return 0;
				//cout << "at zero" << endl;
			}else{
				do {   error = gpio18->getval_gpio(wheel_current);  }
				while (wheel_current != "1"); // polls for gpio to return 1;
			}
			if(tick_counter < 0){
				//Stop You have reached the destination
				cout<< "Stop..." << endl;
				gpio4->setval_gpio("0");
				gpio17->setval_gpio("1");
				usleep(500000);
				gpio4->setval_gpio("0");
				gpio17->setval_gpio("0");
				break;
			}
			cout << "Number of Ticks %i: " << tick_counter <<" Time: "<< getTime() << endl;
			tick_counter--; // we have gone for zero to 1, increment counter
		}
	}
}
int main(){
	const int forward = 8, backward = 2, turnRight = 6, turnLeft = 4,stop = 5;
	cout<< "-----Created By: RoboGlove-----"<<endl;
	//manuallyCount();
	goThisDistance();
	/*
	GPIO *gpio4,*gpio17,*gpio27,*gpio22;
	//Use the GPIO pins
	//initializeGPIO(gpio4,"4","out");
	gpio4 = new GPIO("4");
	gpio4->export_gpio();
	gpio4->setdir_gpio("out");
	//initializeGPIO(gpio17,"17","out");
	gpio17 = new GPIO("17");
	gpio17->export_gpio();
	gpio17->setdir_gpio("out");
	//initializeGPIO(gpio27,"27","out");
	gpio27 = new GPIO("27");
	gpio27->export_gpio();
	gpio27->setdir_gpio("out");
	//initializeGPIO(gpio22,"22","out");
	gpio22 = new GPIO("22");
	gpio22->export_gpio();
	gpio22->setdir_gpio("out");
	//GPIO 18, 23 are PWM pins

	int sec = 2*500000, keyPressed = 0;
	cout<<"Enter CMD: Forward: 8, Backward: 2, Right: 6, Left: 4, Stop: 5"<<endl;
	while(1){
		usleep(500000);
		keyPressed = 0;
		cout<<">> ";
		cin >> keyPressed;
        //cout<<"Enterd: "<< keyPressed << endl;
		switch((keyPressed)){
			case KEY_UP:
				//Go forward motion Code
				cout<< "Moving Forward..." << endl;
				gpio4->setval_gpio("1");
				gpio17->setval_gpio("0");
				//gpio27->setval_gpio("1");
				//gpio22->setval_gpio("0");
				break;
			case KEY_DOWN:
				//Go backward motion Code
				cout<< "Moving Backward..." << endl;				
				gpio4->setval_gpio("0");
				gpio17->setval_gpio("1");
				//gpio27->setval_gpio("0");
				//gpio22->setval_gpio("1");
				break;
			case KEY_RIGHT:
				//turn right Code
				cout<< "Tunrnnig Right..." << endl;
				gpio4->setval_gpio("1");
				gpio17->setval_gpio("0");
				//gpio27->setval_gpio("0");
				//gpio22->setval_gpio("1");
				break;
			case KEY_LEFT:
				//turn left Code
				cout<< "Tunrnnig Left..." << endl;
				gpio4->setval_gpio("0");
				gpio17->setval_gpio("1");
				//gpio27->setval_gpio("1");
				//gpio22->setval_gpio("0");
				break;
			case st85op:
				cout<< "Full Stop..." << endl;
				//turn left Code
				gpio4->setval_gpio("1");
				gpio17->setval_gpio("1");
				//gpio27->setval_gpio("0");
				//gpio22->setval_gpio("0");
				break;
			case 7:
				cout<< "Full Stop..." << endl;
				//turn left Code
				gpio4->setval_gpio("0");
				gpio17->setval_gpio("0");
				//gpio27->setval_gpio("0");
				//gpio22->setval_gpio("0");
				break;
		}
		/*
		cout<<"Info: GPIO 26 is on"<<endl;
		gpio26->setval_gpio("1");
		cout<<"Info: waiting 3 sec"<<endl;
		usleep(sec*3);
		cout<<"Info: GPIO 26 is off"<<endl;
		gpio26->setval_gpio("0");
		cout<<"Info: waiting 1 sec"<<endl;
		usleep(sec);
		
	}
	*/
}
