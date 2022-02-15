//INCLUDES
#include <ros/ros.h>
#include <ros/rate.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <vector>
#include <pigpio.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <signal.h>

#include "project_node/types.h"
#include "custom_msgs/Heights.h"
#include "custom_msgs/Parameters.h"
//FIN DES INCLUDES

//DEFINES
#define CLOCK_DURATION std::chrono::high_resolution_clock::duration
#define CLOCK_TIME_POINT std::chrono::high_resolution_clock::time_point
#define CLOCK_NOW std::chrono::high_resolution_clock::now()
#define MY_PI				3.14159

#define DEBUGGING
#ifndef DEBUGGING
	#define RPI_MOTOR_CONTROLLER_CW 	17 // Broadcom 
	#define RPI_MOTOR_CONTROLLER_CCW 	27 // Broadcom
	#define RPI_MOTOR_CONTROLLER_PD		5  // Broadcom
	#define RPI_HIGH_LIMIT_SWITCH		23 // Broadcom
	#define RPI_LOW_LIMIT_SWITCH		24 // Broadcom
#else
	#define LED				17
#endif
//FIN DES DEFINES

//VARIABLES GLOBALES
bool init_sensor_pos; 
bool sensor_ok;
bool hls_is_enabled; 					//Permet d'enregistrer l'état du bouton de fin de course supérieur
bool raspi_go;
bool pigpioIsRunning = false;

int readGPIO;							//Permet de stocker la valeur lue sur un GPIO
int nbPulse; 							//Nombre d'impulsions requis pour le moteur pas à pas

unsigned int count;

double period_ON = 0.; 					//Temps (en seconde) durant lequel l'impulsion envoyée au moteur est à l'état haut
double period_OFF = 0.; 				//Temps (en seconde) durant lequel l'impulsion envoyée au moteur est à l'état bas

int32 processed_measure ; 				//Nombre de mesures effectuées
custom_msgs::Heights heights; 			//Hauteurs à atteindre
custom_msgs::Parameters parameters; 	//Paramètres du moteur pas à pas

ros::Publisher pub_processed_measure;
ros::Publisher pub_send_measure;
ros::Publisher pub_raspi_sensor;
ros::Publisher pub_sensor_init_pos;

std_msgs::Int32 msg_processed_measure;
//FIN DES VARIABLES GLOBALES

//PROTOTYPES
void pause_s(double seconde = 1.0);
void mySIGINTHandler(int sig);
bool manageGPIO();
void terminateGPIO();
void sendData();
//FIN DES PROTOTYPES

void printStars(){
	std::cout << "************************************************************************************************\n";
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
 * @brief Initialise la bibliothèque pigpio et configure les broches du Raspberry Pi
 * @return Retourne un booléen : true si la bibliothèque a été correctement initialisée, false sinon.
*/
bool manageGPIO(){
	if(gpioInitialise() > 0){
	#ifndef DEBUGGING
		//GPIO
		gpioSetMode(RPI_MOTOR_CONTROLLER_CW, PI_OUTPUT);
		gpioSetMode(RPI_MOTOR_CONTROLLER_CCW, PI_OUTPUT);
		gpioSetMode(RPI_MOTOR_CONTROLLER_PD, PI_OUTPUT);
		gpioSetMode(RPI_HIGH_LIMIT_SWITCH, PI_INPUT);
		gpioSetMode(RPI_LOW_LIMIT_SWITCH, PI_INPUT);
		
		gpioWrite(RPI_MOTOR_CONTROLLER_CW, PI_LOW);
		gpioWrite(RPI_MOTOR_CONTROLLER_CCW, PI_LOW);
		gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_HIGH);
	#else
		ROS_INFO("[ LED  ] : LED VERTE ALLUMEE");
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
	ROS_INFO("[ LED  ] : LED VERTE ETEINTE");
	gpioWrite(LED, PI_LOW);
#endif
	
	gpioTerminate();
	pigpioIsRunning = false;
}

/**
 *@brief Termine l'acquisition des données
*/
void sendData(){
	std_msgs::Bool send;
	send.data = true; 
	pub_send_measure.publish(send);
	
	ROS_INFO("[ INFO ] : ... ACQUISITION TERMINEE\n");
	count = 0;
	printStars();
}

/**
 * @brief Permet de monter les capteurs jusqu'à leur position cible.
 * @param T_on Durée de l'impulsion à l'état haut.
 * @param T_off Durée de l'impulsion à l'état bas.
 * @param pulse Nombre d'impulsions requis
 * @param height Hauteur à atteindre
 * @return Retourne un booléen : true si la position cible a été atteinte, false sinon.
*/
bool raiseSensors(double T_on, double T_off, int pulse = -1, uint32 height = 0){
#ifndef DEBUGGING
	gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_LOW);
	
	/*pulse = -1 permet de ramener les capteurs à leur position initiale, 
	 * c'est-à dire jusqu'à l'actionnement du bouton de fin de course supérieur
	*/
	if(pulse == -1){
		int result;
		while(1){
			if(gpioWrite(RPI_MOTOR_CONTROLLER_CW, PI_HIGH) != 0){
				gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_HIGH);
				return false;
			}
			time_sleep(T_on);
			if(gpioWrite(RPI_MOTOR_CONTROLLER_CW, PI_LOW) != 0){
				gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_HIGH);
				return false;
			}
			time_sleep(T_off);
			
			if(gpioRead(RPI_HIGH_LIMIT_SWITCH)) break;
		}
	}else{
		int cpt = 0;
		
		/*Rotation du moteur jusqu'à ce que le nombre d'impulsions requis soit atteint ou
		 * que le capteur de fin de course supérieur soit activé */
		while((cpt < pulse) && !hls_is_enabled){
			if(gpioWrite(RPI_MOTOR_CONTROLLER_CW, PI_HIGH) != 0){
				gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_HIGH);
				return false;
			}
			time_sleep(T_on);
			if(gpioWrite(RPI_MOTOR_CONTROLLER_CW, PI_LOW) != 0){
				gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_HIGH);
				return false;
			}
			time_sleep(T_off);
			
			hls_is_enabled = gpioRead(RPI_HIGH_LIMIT_SWITCH);
			cpt++;
		}
		
		/*Signalement dans le cas où le capteur de fin de course supérieur est à une position
		 * inférieure par rapport à la hauteur cible*/
		if((cpt < (pulse - 1)) && hls_is_enabled){
			ROS_INFO("[ ERREUR ] : La hauteur cible (%d mm) n'a pas ete atteinte. Le capteur de fin de course est trop proche", height);
			gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_HIGH);
			return false;
		}
	}
	gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_HIGH);
#endif
	
	return true;
}

/**
 * @brief Permet de déscendre les capteurs jusqu'au capteur de fin de course inférieur.
 * @param T_on Durée de l'impulsion à l'état haut.
 * @param T_off Durée de l'impulsion à l'état bas.
 * @return Retourne un booléen : true si la position cible a été atteinte, false sinon.
*/
bool bringDownSensors(double T_on, double T_off){
#ifndef DEBUGGING
	gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_LOW);
	
	while(1){
		if(gpioWrite(RPI_MOTOR_CONTROLLER_CCW, PI_HIGH) != 0){
			gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_HIGH);
			return false;
		}
		time_sleep(T_on);
		if(gpioWrite(RPI_MOTOR_CONTROLLER_CCW, PI_LOW) != 0){
			gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_HIGH);
			return false;
		}
		time_sleep(T_off);
		
		if(gpioRead(RPI_LOW_LIMIT_SWITCH)) break;
	}
	
	gpioWrite(RPI_MOTOR_CONTROLLER_PD, PI_HIGH);
#endif
	
	return true;
}

/**
 * @brief Permet de ramener les capteurs à leur position initiale.
 * @param msg Message lu sur le topic init_sensor_pos et qui contient l'ordre d'initialisation.
*/
void initSensorPositionCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data){
		manageGPIO();
		
#ifndef DEBUGGING
		readGPIO = gpioRead(RPI_HIGH_LIMIT_SWITCH);
		if((readGPIO != PI_BAD_GPIO) && (readGPIO != PI_HIGH)){
			if(raiseSensors(period_ON, period_OFF)){
				std_msgs::Bool isInit;
				isInit.data = true;
				pub_sensor_init_pos.publish(isInit);

				ROS_INFO("[ INFO ] : Capteurs ramenés à leur position initiale");
			}
			else ROS_INFO("[ ERREUR ] : Impossible de ramener les capteurs à leur position initiale");
		}
#else
		std_msgs::Bool isInit;
		isInit.data = true;
		pub_sensor_init_pos.publish(isInit);

		ROS_INFO("[ INFO ] : Capteurs ramenés à leur position initiale");
#endif
		
		if(count != 0) count = 0;

		terminateGPIO();
	}
}

/**
 * @brief Met à jour le nombre de mesures effectuées et l'envoi vers l'IHM à travers le topic processed_measure
 * @param msg Message lu sur le topic sensor_ok et qui indique que les mesures ont été effectuées
*/
void sensorCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data){
		msg_processed_measure.data = (++processed_measure);
		pub_processed_measure.publish(msg_processed_measure);
	}
}

/**
 * @brief Enregistre les paramètres du motor pas à pas et qui sont fournis par l'IHM
 * @param msg Message lu sur le topic parameters et qui contient les données de paramétrage du moteur pas à pas
*/
void paramCallback(const custom_msgs::Parameters::ConstPtr& msg){
	double angularVel = 0.;
	double frequency = 0.;
	double period = 0.;

	parameters.pulleyRadius = msg->pulleyRadius; //en mm
	parameters.motorAccuracy = msg->motorAccuracy; //en mm/pulse
	parameters.motorAccuracyDivider = msg->motorAccuracyDivider;
	parameters.dutyCycle = msg->dutyCycle; //en %
	parameters.linVelMotorMast = msg->linVelMotorMast; //en mm/s
	
	if(processed_measure != 0) processed_measure = 0;
	if(count != 0) count = 0;
	
	angularVel = parameters.linVelMotorMast / (2. * MY_PI * parameters.pulleyRadius); //en tr/s
	frequency = angularVel * parameters.motorAccuracyDivider * 200.; //en Hz
	period = 1./frequency; //en s
	period_ON = (period * (100 - parameters.dutyCycle))/100.; //en s
	period_OFF = period - period_ON; //en s
	
	ROS_INFO("[ INFO ] : Parametrage du moteur du mat vertical");
	ROS_INFO("[ RECU ] : \t--> Rayon de la poulie : %.2f mm", parameters.pulleyRadius);
	ROS_INFO("[ RECU ] : \t--> Precision du moteur : %f mm/pulse", parameters.motorAccuracy);
	ROS_INFO("[ RECU ] : \t--> Division du pas du moteur : %d", parameters.motorAccuracyDivider);
	ROS_INFO("[ RECU ] : \t--> Vitesse lineaire desiree : %d mm/s", parameters.linVelMotorMast);
	ROS_INFO("[ RECU ] : \t--> Rapport cyclique des pulsations : %d %%", parameters.dutyCycle);
	ROS_INFO("[ INFO ] :  \t--> Frequence des pulsations : %.2f Hz", frequency);
	ROS_INFO("[ INFO ] :  \t--> Periode des pulsations : %.2f us\n", period*1000000.); 
}

/**
 * @brief Enregistre la liste des hauteurs à atteindre
 * @param Message lu sur le topic heights_list et qui contient la liste des hauteurs à atteindre
*/
void heightListCallback(const custom_msgs::Heights::ConstPtr& msg){
	heights.heightList = msg->heightList;
	
	ROS_INFO("[ INFO ] : Hauteurs a atteindre");
	for(unsigned int i = 0; i < heights.heightList.size(); i++) ROS_INFO("[ RECU ] : \t--> H = %d mm", heights.heightList[i]);
	std::cout << std::endl;
}

/**
 * @brief Initialise la prise des mesures
 * @param msg Message lu sur le topic raspi_go et qui contient l'information de démarrage de la prise des mesures
*/
void raspiGoCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data){
#ifndef DEBUGGING
		if(count == 0){
			manageGPIO();
			
			if(bringDownSensors()) raspi_go = true;
			else ROS_INFO("[ ERROR ] : Descente des capteurs impossible");
			
			terminateGPIO();
		}else raspi_go = true;
#else
		manageGPIO();
		raspi_go = true;
		if(count == 0) ROS_INFO("[ INFO ] : ACQUISITION EN COURS ...");
		
		terminateGPIO();
#endif
	}
}

int main(int argc, char**argv){
		ros::init(argc, argv, "motor_mast_node");
		ros::NodeHandle nh;
		
		//Publicate
		pub_processed_measure = nh.advertise<std_msgs::Int32>("processed_measure", 5);
		pub_send_measure = nh.advertise<std_msgs::Bool>("send_measure", 5);
		pub_raspi_sensor = nh.advertise<std_msgs::UInt32>("raspi_sensor", 5);
		pub_sensor_init_pos = nh.advertise<std_msgs::Bool>("sensor_init_pos", 5);
		
		//Subscribe
		ros::Subscriber sub_init_motor_pos = nh.subscribe("init_sensor_pos", 5, initSensorPositionCallback);
		ros::Subscriber sub_IHM_params = nh.subscribe("parameters", 5, paramCallback);
		ros::Subscriber sub_raspi_go = nh.subscribe("raspi_go", 5, raspiGoCallback);
		ros::Subscriber sub_heights = nh.subscribe("heights_list", 5, heightListCallback);
		ros::Subscriber sub_sensor_ok = nh.subscribe("sensor_ok", 5, sensorCallback);
		
		//ros::Rate rate(5);
		
		processed_measure = 0;
		count = 0;
		raspi_go = false;
		
		ROS_INFO("[ INFO ] : MOTOR NODE LAUNCHED ...\n");
		
		while(ros::ok()){
			if(raspi_go){
				std_msgs::UInt32 rs;
				raspi_go = false;
				
				if(count < heights.heightList.size()){
					manageGPIO();
					rs.data = count;
					
#ifndef DEBUGGINNG
					if(count == 0){
						nbPulse = static_cast<int>(abs(heights.heightList[count] - 100)/parameters.motorAccuracy); // La hauteur minimale des capteurs sur le support est de 100 mm
					}
					else{
						nbPulse = static_cast<int>(abs(heights.heightList[count] - heights.heightList[count - 1])/parameters.motorAccuracy);
					}
					
					if(raiseSensors(period_ON, period_OFF, nbPulse, heights.heightList[count] )){
						terminateGPIO();
						pub_raspi_sensor.publish(rs);
						count++;
					}
					else sendData();
#else
					terminateGPIO();
					pub_raspi_sensor.publish(rs);
					count++;
#endif
				}
				else if(count == heights.heightList.size()) sendData();
			}
			ros::spinOnce();
		}
		//ROS_INFO("FINI ...");
		return 0;
}
