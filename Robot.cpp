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
#include<ncurses.h>

#define KEY_UP 72
#define KEY_DOWN 80
#define KEY_LEFT 75
#define KEY_RIGHT 77

using namespace std;
void sig_handler(int sig);

bool ctrl_c_pressed = false;
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
int initializeGPIO(GPIO* gpio,string gpioPinNum, string direction){
	gpio = new GPIO(gpioPinNum);
	gpio->export_gpio();
	gpio->setdir_gpio(direction);
	return 0;
}
int main(){
	const int forward = 8, backward = 2, turnRight = 6, turnLeft = 4,stop = 5;
	cout<< "-----Created By: RoboGlove-----"<<endl;
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

	int sec = 2*500000;
	cout<<"Enter CMD: Forward: 8, Backward: 2, Right: 6, Left: 4, Stop: 5"<<endl;
	int keyPressed = 0;
	do
    {
        int c = getchar();
        printf("c=%d\n", c);
    }
    while (1);
	
	while(1){
		keyPressed = 0;
		//cout<<">> ";
		//cin >> keyPressed;
        //cout<<"Enterd: "<< keyPressed << endl;
		switch((keyPressed = getchar())){
			case KEY_UP:
				//Go forward motion Code
				cout<< "Moving Forward..." << endl;
				gpio4->setval_gpio("1");
				gpio17->setval_gpio("0");
				gpio27->setval_gpio("1");
				gpio22->setval_gpio("0");
				break;
			case KEY_DOWN:
				//Go backward motion Code
				cout<< "Moving Backward..." << endl;				
				gpio4->setval_gpio("0");
				gpio17->setval_gpio("1");
				gpio27->setval_gpio("0");
				gpio22->setval_gpio("1");
				break;
			case KEY_RIGHT:
				//turn right Code
				cout<< "Tunrnnig Right..." << endl;
				gpio4->setval_gpio("1");
				gpio17->setval_gpio("0");
				gpio27->setval_gpio("0");
				gpio22->setval_gpio("1");
				break;
			case KEY_LEFT:
				//turn left Code
				cout<< "Tunrnnig Left..." << endl;
				gpio4->setval_gpio("0");
				gpio17->setval_gpio("1");
				gpio27->setval_gpio("1");
				gpio22->setval_gpio("0");
				break;
			case stop:
				cout<< "Full Stop..." << endl;
				//turn left Code
				gpio4->setval_gpio("0");
				gpio17->setval_gpio("0");
				gpio27->setval_gpio("0");
				gpio22->setval_gpio("0");
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
		*/
	}
}
