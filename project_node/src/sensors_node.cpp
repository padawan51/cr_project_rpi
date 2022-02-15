//INCLUDES
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <pigpio.h>
#include <chrono>
#include <ctime>
#include <random>
#include <iostream>
#include <signal.h>

#include "custom_msgs/Delay.h"
#include "custom_msgs/Heights.h"
#include "project_node/types.h"
#include "custom_msgs/DataSensors.h"
//FIN DES INCLUDES

//DEFINES
#define CLOCK_DURATION std::chrono::high_resolution_clock::duration
#define CLOCK_TIME_POINT std::chrono::high_resolution_clock::time_point
#define CLOCK_NOW std::chrono::high_resolution_clock::now()

#define DEBUGGING
#ifndef DEBUGGING
	#define RPI_CAN_CS_CE0		8  // Broadcom
	#define RPI_CAN_DOUT_MISO	9  // Broadcom
	#define RPI_CAN_DIN_MOSI	10 // Broadcom
#else
	#define LED			16
#endif

#define BAUD_RATE		500000    // Bits par seconde
#define SPI_CHANNEL_0		0
#define VDD			5         // Tension d'alimentation en Volt
#define CAN_MAX_VALUE		4096      // 2^12 
#define TEMP_ACCURACY		0.0142857 // en V/°C
//FIN DES DEFINES

//VARIABLES GLOBALES
ros::Publisher pub_measure_ok;
ros::Publisher pub_sensor_ok;

custom_msgs::Heights height;
custom_msgs::Delay delay;
custom_msgs::DataSensors sensorsData;

int32 type; //Type de sortie du capteur de vitesse aérodynamique

float32 Ei; //Tension minimale
float32 Ef; //Tension maximale

bool stop;
bool pigpioIsRunning = false;

unsigned int spi_handle;
//FIN DES VARIABLES GLOBALES

//PROTOTYPES
void pause_s(double seconde = 1.0);
void mySIGINTHandler(int sig);
bool manageGPIO();
void terminateGPIO();
//FIN DES PROTOTYPES

void printStars(){
	std::cout << "************************************************************************************************\n";
}

/**
 * @brief Gestion du signal SIGINT (Ctrl+C)
 * Utile, car l'utilisation de la bibliothèque pigpio masque le gestionnaire 
 * du signal SIGINT défini par ros. De ce, il est imposssible de stopper le noeud 
 * avec un Ctrl+C, d'où l'utilité de cette fonction qui permet de prendre en main ce signal.
 * 
 * @param sig Signal à gérer.
*/
void mySIGINTHandler(int sig){
	if(pigpioIsRunning) terminateGPIO();
	
	ros::shutdown();
}

/**
 * @brief Permet de faire une pause
 * @param seconde Durée de la pause en seconde
*/
void pause_s(double seconde)
{
    CLOCK_TIME_POINT ti;
    double duration = 0.0;
    std::chrono::duration<double> time_span;

    ti = CLOCK_NOW;
    do{
        time_span = CLOCK_DURATION(CLOCK_NOW - ti);
        duration = time_span.count();
    }while(duration <= seconde);
}


/**
 * @brief Initialise la bibliothèque pigpio et configure les broches du Raspberry Pi
 * @return Retourne un booléen : true si la bibliothèque a été correctement initialisée, false sinon.
*/
bool manageGPIO(){
	if(gpioInitialise() > 0){
#ifndef DEBUGGING
		//GPIO
		gpioSetMode(RPI_CAN_CS_CE0, PI_OUTPUT);
		gpioSetMode(RPI_CAN_DIN_MOSI, PI_OUTPUT);
		gpioSetMode(RPI_CAN_DOUT_MISO, PI_INPUT);
		
		gpioWrite(RPI_CAN_CS_CE0, PI_HIGH);
#else
		ROS_INFO("[ LED  ] : LED ROUGE ALLUMEE");
		gpioSetMode(LED, PI_OUTPUT);
		gpioWrite(LED, PI_HIGH);
#endif

		signal(SIGINT, mySIGINTHandler);
		pigpioIsRunning = true;
		
		return true;
	}else return false;
}

/**
 * @brief Arrête la bibliothèque pigpio
*/
void terminateGPIO(){
	pause_s();
#ifdef DEBUGGING
	ROS_INFO("[ LED  ] : LED ROUGE ETEINTE");
	gpioWrite(LED, PI_LOW);
#endif
	gpioTerminate();
	
	pigpioIsRunning = false;
}

/**
 * @brief Permet d'enregistrer le type de sortie du capteur de vitesse aérodynamique
 * @param msg Message lu sur le topic volt_out_type et qui contient l'information sur le type de sortie
*/
void voltOutputTypeCallback(const std_msgs::Int32::ConstPtr& msg){
	type = msg->data;
	
	if(type == VelocityVoltageOutputType::TYPE_0_5_V){
		Ei = 0.0; // volt
		Ef = 5.0; // volt
	}
	else if(type == VelocityVoltageOutputType::TYPE_1_5_V){
		Ei = 1.0; // volt
		Ef = 5.0; // volt
	}
	
	ROS_INFO("[ INFO ] : < Vitesse aérodynamique > Ei = %.2f  V ; Ef = %.2f V\n", Ei, Ef);
}

/**
 * @brief Permet d'enregistrer les temporisations
 * @param msg Message lu sur le topic delay et qui contient les informations de temporisations
*/
void delayCallback(const custom_msgs::Delay::ConstPtr& msg){
	delay.restDelay = msg->restDelay;
	delay.measureDelay = msg->measureDelay;
	delay.periodDelay = msg->periodDelay;
	
	if(stop){
		stop = false;
		if(!sensorsData.heights.empty()) sensorsData.heights.clear();
		if(!sensorsData.airSpeed.empty()) sensorsData.airSpeed.clear();
		if(!sensorsData.airSpeedTimer.empty()) sensorsData.airSpeedTimer.clear();
		if(!sensorsData.temperature.empty()) sensorsData.temperature.clear();
		if(!sensorsData.temperatureTimer.empty()) sensorsData.temperatureTimer.clear();
	}
	
	ROS_INFO("[ INFO ] : Temporisation");
	ROS_INFO("[ RECU ] : \t--> Temps de repos : %d s", delay.restDelay);
	ROS_INFO("[ RECU ] : \t--> Duree de la mesure : %d s", delay.measureDelay);
	ROS_INFO("[ RECU ] : \t--> Periode de la mesure : %.2f s\n", delay.periodDelay);
}

/**
 * @brief Enregistre la liste des hauteurs à atteindre
 * @param Message lu sur le topic heights_list et qui contient la liste des hauteurs à atteindre
*/
void heightsCallback(const custom_msgs::Heights::ConstPtr& msg){
	height.heightList = msg->heightList;
	
	ROS_INFO("[ INFO ] : Hauteurs a atteindre");
	for(unsigned int i = 0; i < height.heightList.size(); i++) ROS_INFO("[ RECU ] : \t--> H = %d mm", height.heightList[i]);
	std::cout << std::endl;
}

/**
 * @brief Envoi de l'ensemble des mesures effectuées vers l'IHM
 * @param msg Message lu sur le topic send_measure et qui contient l'autorisation d'envoi des mesures
*/
void sendMeasureCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data){
		pub_measure_ok.publish(sensorsData);
		
		sensorsData.heights.clear();
		sensorsData.airSpeed.clear();
		sensorsData.airSpeedTimer.clear();
		sensorsData.temperature.clear();
		sensorsData.temperatureTimer.clear();
		
		printStars();
	}
}

/**
 * @brief Arrête l'acquisition des données des capteurs de température et de vitesse aérodynamique
 * @param msg Message lu sur le topic stop_measure et qui contient l'autorisation d'arrêter la prise des mesures
*/
void stopCallback(const std_msgs::Bool::ConstPtr& msg){
	stop = msg->data;
	ROS_INFO("[ STOP ] : stop = %d", stop);
}

/**
 * @brief Effectue l'acquisition des données des capteurs de température et de vitesse aérodynamique
 * @param msg Message lu sur le topic raspi_sensor et qui contient la hauteur à laquelle se trouvent les capteur
*/
void raspiSensorCallback(const std_msgs::UInt32::ConstPtr& msg){
#ifndef DEBUGGING
	int val;
	float32 result;
	float32 Vin;
	CLOCK_TIME_POINT t_init;
	CLOCK_TIME_POINT start;
	double duration = 0.0;
	double limitTime;
	double end = 0.0;
   std::chrono::duration<double> time_span;
	
	unsigned char txBuf[3]; //Buffer de transimission des données sur le bus SPI
	unsigned char rxBuf[3]; //Buffer de lecture des données sur le bus SPI
	
	manageGPIO();
	
	if(!height.heightList.empty()){
			limitTime = delay.restDelay + delay.measureDelay; //Temps alloué à la réalisation d'une mesure à une hauteur donnée
			t_init = CLOCK_NOW; //initialisation du timer de la mesure globale
			ROS_INFO("[ INFO ] : H = %d mm -< Acquisition en cours ... >-\n", height.heightList[msg->data]);
			
			spi_handle = spiOpen(SPI_CHANNEL_0, BAUD_RATE, 1); //Ouverture d'un canal SPI pour la communication
			
			if(spi_handle >= 0){
				ROS_INFO("[ INFO ] : Liaison SPI ouverte ...");
				
				gpioWrite(RPI_CAN_CS_CE0, PI_HIGH);
				
				do{
					start = CLOCK_NOW; //initialisation de la période de la mesure 
					
					//Acquisition de la température
					txBuf[0] = 1;
					txBuf[1] = 160;
					txBuf[2] = 0;
					
					gpioWrite(RPI_CAN_CS_CE0, PI_LOW);
					
					spiXfer(spi_handle, txBuf, rxBuf, 3);
					
					gpioWrite(RPI_CAN_CS_CE0, PI_HIGH);
					
					//lecture de la valeur numérique de la mesure de la température
					val = ((rxBuf[1] & 3) << 8) | rxBuf[2];
					
					//conversion de la température en valeur analogique
					result = (static_cast<double>(val)) / (static_cast<double>(CAN_MAX_VALUE));
					result *= VDD;
					result /= TEMP_ACCURACY;
					result -= 50.;
					
					//enregistrement de la mesure de température et du temps de mesure
					time_span = CLOCK_DURATION(CLOCK_NOW - t_init);
					duration = time_span.count();
					
					sensorsData.temperature.push_back(result);
					sensorsData.temperatureTimer.push_back(duration);
					
					//Acquisition de la vitesse de l'air
					txBuf[0] = 1;
					txBuf[1] = 224;
					txBuf[2] = 0;
					
					gpioWrite(RPI_CAN_CS_CE0, PI_LOW);
					
					spiXfer(spi_handle, txBuf, rxBuf, 3);
					
					gpioWrite(RPI_CAN_CS_CE0, PI_HIGH);
					
					//lecture de la valeur numérique de la mesure de la vitesse
					val = ((rxBuf[1] & 3) << 8) | rxBuf[2];
					
					//conversion de la vitesse en valeur analogique
					Vin = (static_cast<double>(val)) / (static_cast<double>(CAN_MAX_VALUE));
					Vin *= VDD;
					result = (Vin - Ei) / (Ef - Ei)
					result *= 50. ;
					
					//enregistrement de la mesure de la vitesse et du temps de mesure
					time_span = CLOCK_DURATION(CLOCK_NOW - t_init);
					duration = time_span.count();
					
					sensorsData.airSpeed.push_back(result);
					sensorsData.airSpeedTimer.push_back(duration);
					
					do{
						time_span = CLOCK_DURATION(CLOCK_NOW - start);
						end = time_span.count();
					}while(end < delay.periodDelay && !stop);
					
					sensorsData.heights.push_back(height.heightList[msg->data]);
					
					time_span = CLOCK_DURATION(CLOCK_NOW - t_init);
					duration = time_span.count();
				}while(duration < limitTime && !stop);		
			}
			else if(spi_handle == PI_BAD_SPI_CHANNEL) ROS_INFO("[ ERREUR ] : Impossible d'ouvir la liaison SPI (PI_BAD_SPI_CHANNEL)");
			else if(spi_handle == PI_BAD_SPI_SPEED) ROS_INFO("[ ERREUR ] : Impossible d'ouvir la liaison SPI (PI_BAD_SPI_SPEED)");
			else if(spi_handle == PI_BAD_FLAGS) ROS_INFO("[ ERREUR ] : Impossible d'ouvir la liaison SPI (PI_BAD_FLAGS)");
			else if(spi_handle == PI_SPI_OPEN_FAILED) ROS_INFO("[ ERREUR ] : Impossible d'ouvir la liaison SPI (PI_SPI_OPEN_FAILED)");
			
			//fermeture de la liasion SPI
			if((spi_handle >= 0) && spiClose(spi_handle) == 0){
				std_msgs::Bool eom;
				eom.data = true;
				
				terminateGPIO();
				ROS_INFO("[ INFO ] : ... Liaison SPI fermee");
				pub_sensor_ok.publish(eom);
			}
			else terminateGPIO();
	}
#else
	manageGPIO();
	
	uint32 index = msg->data;
	std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
	std::uniform_real_distribution<double> distribution(20.00, 21.0);
	std::uniform_real_distribution<double> distributionVel(0.10, 0.18);
	float32 val;
	float32 val_vel;
	CLOCK_TIME_POINT t_init;
	CLOCK_TIME_POINT start;
	double duration = 0.0;
	double limitTime;
	double end = 0.0;
   std::chrono::duration<double> time_span;
	
	if(!height.heightList.empty()){
			limitTime = delay.restDelay + delay.measureDelay;
			t_init = CLOCK_NOW;
			ROS_INFO("[ INFO ] : H = %d mm -< Acquisition en cours ... >-", height.heightList[index]);
		
			do{
				start = CLOCK_NOW;
				
				val = static_cast<float32>(distribution(generator));
				time_span = CLOCK_DURATION(CLOCK_NOW - t_init);
				duration = time_span.count();
				sensorsData.temperature.push_back(val);
				sensorsData.temperatureTimer.push_back(duration);
				
				val_vel = static_cast<float32>(distributionVel(generator));
				time_span = CLOCK_DURATION(CLOCK_NOW - t_init);
				duration = time_span.count();
				sensorsData.airSpeed.push_back(val_vel);
				sensorsData.airSpeedTimer.push_back(duration);
				
				do{
					time_span = CLOCK_DURATION(CLOCK_NOW - start);
					end = time_span.count();
					ros::spinOnce();
				}while(end < delay.periodDelay && !stop);
				
				sensorsData.heights.push_back(height.heightList[index]);
				
				time_span = CLOCK_DURATION(CLOCK_NOW - t_init);
				duration = time_span.count();
			}while(duration < limitTime && !stop);
	}
	terminateGPIO();
	std_msgs::Bool eom;
	eom.data = true;
	pub_sensor_ok.publish(eom);
#endif
}

int main(int argc, char** argv){
		ros::init(argc, argv, "sensor_node");
		ros::NodeHandle nh;
		
		//Publicate
		pub_measure_ok = nh.advertise<custom_msgs::DataSensors>("raspi_eom", 5);
		pub_sensor_ok = nh.advertise<std_msgs::Bool>("sensor_ok", 5);
		
		//Subscribe
		ros::Subscriber sub_delay = nh.subscribe("delay", 5, delayCallback);
		ros::Subscriber sub_heights = nh.subscribe("heights_list", 5, heightsCallback);
		ros::Subscriber sub_send_measure = nh.subscribe("send_measure", 5, sendMeasureCallback);
		ros::Subscriber sub_raspi_sensor = nh.subscribe("raspi_sensor", 5, raspiSensorCallback);
		ros::Subscriber sub_volt_out_type = nh.subscribe("volt_out_type", 5, voltOutputTypeCallback);
		ros::Subscriber sub_stop_measure = nh.subscribe("stop_measure", 5, stopCallback);
		
		ROS_INFO("[ INFO ] : SENSOR NODE LAUNCHED ...\n");
		
		stop = false;
		
		ros::spin();
		
		return 0;
}
