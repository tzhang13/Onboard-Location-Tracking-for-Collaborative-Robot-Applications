#include "stdafx.h"

#define DEFAULT_BUFLEN 48
#define PI 3.14159265359
#define DECLINATION -8.58

#pragma comment (lib, "Ws2_32.lib")

using namespace std;

DWORD WINAPI SocketHandler(void* lp);
DWORD WINAPI SocketHandler2(void* lp);
DWORD WINAPI SocketHandler3(void* lp);

void smoothSensor(struct passMe *data);
void getStates(struct passMe *pm, float dt);
void getStates2(struct passMe *pm, float dt);
void getStates3(struct passMe *pm, float dt);
void getKalmanPosition(struct kalman *imu, float p1, float v1, float dt);

struct kalman
{
	float Q_pos;
	float Q_vel;
    float R_measure;

	float pos_filtered; 
	float vel_filtered;
	float P[2][2];

	//! @brief Default constructor
	//!
	kalman ()
	{
		Q_pos = 0.001f;
		Q_vel = 0.003f;
		R_measure = 0.03f;	

		pos_filtered=0;
		vel_filtered=0;

		for (int i=0; i<2; i++) {
			for (int j=0; j<2; j++) {
				P[i][j]=0;
			}
		}

	}

};

struct passMe 
{
	HANDLE grabmutex; //! Semaphore used for data protection
	SOCKET ArduinoSocket, ArduinoSocket2, ArduinoSocket3;
	SOCKET AndroidSocket;

	int buffer_len;
    char buffer[1024];

	struct kalman *imu_1[3]; 
	struct kalman *imu_2[3]; 
	struct kalman *imu_3[3];

	//
	float accel_float_1[3]; 
	float mag_float_1[3]; 
	float gyro_float_1[3]; 

	float pos_1[3];
	float vel_1[3];

	float yaw[3], pitch[3], roll[3];
	float past_gyro_sum_1;

	//
	float accel_float_2[3]; 
	float mag_float_2[3]; 
	float gyro_float_2[3]; 

	float pos_2[3];
	float vel_2[3];

	//
	float accel_float_3[3]; 
	float mag_float_3[3]; 
	float gyro_float_3[3]; 

	float pos_3[3];
	float vel_3[3];

	int dist_sensor;
	int dist_sensor_filtered;

	int light_sensor;
	int light_sensor_filtered;

	bool thread1, thread2, thread3;

  //! @brief Default constructor
  //!
  passMe ()
  {
	grabmutex = CreateMutex (NULL, false, NULL);
    ArduinoSocket = INVALID_SOCKET;
	buffer_len = 1024;
	thread1 = thread2 = thread3 = false;
	*buffer = NULL;
	dist_sensor = 0;
	dist_sensor_filtered = -1;

	light_sensor = 0;
	light_sensor_filtered = -1;

	past_gyro_sum_1 = 1000;

	for (int i=0; i<3; i++) {
		accel_float_1[i] =0; 
		mag_float_1[i] =0; 
		gyro_float_1[i] =0; 

		pos_1[i]=0;
		vel_1[i]=0;

		accel_float_2[i] =0; 
		mag_float_2[i] =0; 
		gyro_float_2[i] =0; 

		pos_2[i]=0;
		vel_2[i]=0;

		accel_float_3[i] =0; 
		mag_float_3[i] =0; 
		gyro_float_3[i] =0; 

		pos_3[i]=0;
		vel_3[i]=0;
	}

	for (int i=0; i<3; i++) {
		yaw[i] = pitch[i] = roll[i] = 0;
	}

  }

};

int _tmain(int argc, _TCHAR* argv[]) {
    int host_port = 80; //The port you want the server to listen on	

    //Initialize socket support
    WSADATA wsaData;
    int err;
    err = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (err != 0 || ( LOBYTE( wsaData.wVersion ) != 2 || HIBYTE( wsaData.wVersion ) != 2)) {
        fprintf(stderr, "Could not find useable sock dll %d\n", WSAGetLastError());
        goto FINISH;
    }

    //Initialize sockets and set any options
    int hsock;
    int *p_int ;
    hsock = socket(AF_INET, SOCK_STREAM, 0);
    if(hsock == -1){
        printf("Error initializing socket %d\n",WSAGetLastError());
        goto FINISH;
    }
    
    p_int = (int*)malloc(sizeof(int));
    *p_int = 1;
    if ((setsockopt(hsock, SOL_SOCKET, SO_REUSEADDR, (char*)p_int, sizeof(int)) == -1 ) ||
        (setsockopt(hsock, SOL_SOCKET, SO_KEEPALIVE, (char*)p_int, sizeof(int)) == -1 )) {
        printf("Error setting options %d\n", WSAGetLastError());
        free(p_int);
        goto FINISH;
    }
    free(p_int);

    //Bind and listen
    struct sockaddr_in my_addr;

    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(host_port);
    
    memset(&(my_addr.sin_zero), 0, 8);
    my_addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(hsock, (struct sockaddr*)&my_addr, sizeof(my_addr)) == -1) {
        fprintf(stderr,"Error binding to socket, make sure nothing else is listening on this port %d\n",WSAGetLastError());
        goto FINISH;
    }
    if (listen(hsock, 10) == -1) {
        fprintf(stderr, "Error listening %d\n",WSAGetLastError());
        goto FINISH;
    }

	cout << "Listening socket" << endl;
	
	passMe pm; //! State variable used to communicate with the threads

	// If mutex was not created
	if (pm.grabmutex == NULL) {
        printf("CreateMutex error: %d\n", GetLastError());
        return 1;
    }
	
	// Accept client sockets
	cout << "Attempting to connect" << endl;
	pm.AndroidSocket = accept(hsock, NULL, NULL);
    if (pm.AndroidSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(hsock);
        WSACleanup();
        return 1;
    } 

	cout << "Android socket" << endl;

	pm.ArduinoSocket = accept(hsock, NULL, NULL);
    if (pm.ArduinoSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(hsock);
        WSACleanup();
        return 1;
    } else {
		CreateThread(NULL, 0, &SocketHandler, &pm, 0, 0);
	}

	cout << "Arduino socket 1" << endl;

	pm.ArduinoSocket2 = accept(hsock, NULL, NULL);
    if (pm.ArduinoSocket2 == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(hsock);
        WSACleanup();
        return 1;
    } else {
		CreateThread(NULL, 0, &SocketHandler2, &pm, 0, 0);
	}

	cout << "Arduino socket 2" << endl;

	pm.ArduinoSocket3 = accept(hsock, NULL, NULL);
    if (pm.ArduinoSocket3 == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(hsock);
        WSACleanup();
        return 1;
    } else {
		CreateThread(NULL, 0, &SocketHandler3, &pm, 0, 0);
	}

	cout << "Arduino socket 3" << endl;

	while (true) {

	}

FINISH:
;
}

DWORD WINAPI SocketHandler(void* lp) {
	passMe *pm = (passMe*) lp;

    int bytecount, counter = 0;
	char *temp;
	float curr_time = 0, begin_time = 0, print_time = 0;
	ofstream myfile;
	myfile.open ("pc_client_1.csv");

	ofstream myfile2;
	myfile2.open ("pc_client_1_kalman.csv");

	while (true) {
		WaitForSingleObject (pm->grabmutex, INFINITE);
		printf("#####################11111111111111\n");
		pm->thread1 = true;
		memset(pm->buffer, '\0', pm->buffer_len);

		if ((bytecount = recv(pm->ArduinoSocket, pm->buffer, pm->buffer_len, 0)) == SOCKET_ERROR) {
			fprintf(stderr, "Error receiving data %d\n", WSAGetLastError());
			goto FINISH;
		}

		//printf("Received bytes %d\nReceived string %s\n", bytecount, pm->buffer);

		temp = strtok(pm->buffer, ",");
		pm->accel_float_1[0] = atof(temp);

		temp = strtok(NULL, ",");
		pm->accel_float_1[1] = atof(temp);
		
		temp = strtok(NULL, ",");
		pm->accel_float_1[2] = atof(temp);

		temp = strtok(NULL, ",");
		pm->mag_float_1[0] = atof(temp);

		temp = strtok(NULL, ",");
		pm->mag_float_1[1] = atof(temp);

		temp = strtok(NULL, ",");
		pm->mag_float_1[2] = atof(temp);
		
		temp = strtok(NULL, ",");
		pm->gyro_float_1[0] = atof(temp);

		temp = strtok(NULL, ",");
		pm->gyro_float_1[1] = atof(temp);

		temp = strtok(NULL, ",");
		pm->gyro_float_1[2] = atof(temp);

		curr_time = (float) clock()/ CLOCKS_PER_SEC; // in seconds
		getStates(pm, curr_time-begin_time);

		if (curr_time - print_time > .01) {
			sprintf(pm->buffer, "%f, %f, %f,", pm->pos_1[0], pm->pos_1[1], pm->pos_1[2]);
			myfile << pm->buffer << endl;
			//myfile.close();

			sprintf(pm->buffer, "%f, %f, %f,", pm->imu_1[0]->pos_filtered, pm->imu_1[1]->pos_filtered, pm->imu_1[2]->pos_filtered);
			myfile2 << pm->buffer << endl;

			print_time = curr_time;
		}

		begin_time = curr_time;	

		memset(pm->buffer, '\0', pm->buffer_len);
    		
		pm->thread1 = false;
		ReleaseMutex (pm->grabmutex);

	}

FINISH:
    free((void *) pm->ArduinoSocket);
	free(pm);
    return 0;
}

DWORD WINAPI SocketHandler2(void* lp) {
	passMe *pm = (passMe*) lp;

    int bytecount, counter = 0;
	char *temp;
	float curr_time = 0, begin_time = 0, print_time = 0;
	ofstream myfile;
	myfile.open ("pc_client_2.csv");

	ofstream myfile2;
	myfile2.open ("pc_client_2_kalman.csv");

	while (true) {
		WaitForSingleObject (pm->grabmutex, INFINITE);
		printf("#####################22222222222\n");
		pm->thread2 = true;
		memset(pm->buffer, '\0', pm->buffer_len);

		if ((bytecount = recv(pm->ArduinoSocket2, pm->buffer, pm->buffer_len, 0)) == SOCKET_ERROR) {
			fprintf(stderr, "Error receiving data %d\n", WSAGetLastError());
			goto FINISH;
		}

		//printf("Received bytes %d\nReceived string %s\n", bytecount, pm->buffer);

		temp = strtok(pm->buffer, ",");
		pm->accel_float_2[0] = atof(temp);

		temp = strtok(NULL, ",");
		pm->accel_float_2[1] = atof(temp);
		
		temp = strtok(NULL, ",");
		pm->accel_float_2[2] = atof(temp);

		temp = strtok(NULL, ",");
		pm->mag_float_2[0] = atof(temp);

		temp = strtok(NULL, ",");
		pm->mag_float_2[1] = atof(temp);

		temp = strtok(NULL, ",");
		pm->mag_float_2[2] = atof(temp);
		
		temp = strtok(NULL, ",");
		pm->gyro_float_2[0] = atof(temp);

		temp = strtok(NULL, ",");
		pm->gyro_float_2[1] = atof(temp);

		temp = strtok(NULL, ",");
		pm->gyro_float_2[2] = atof(temp);

		curr_time = (float) clock()/ CLOCKS_PER_SEC; // in seconds
		getStates2(pm, curr_time-begin_time);

		if (curr_time - print_time > .01) {
			sprintf(pm->buffer, "%f, %f, %f,", pm->pos_2[0], pm->pos_2[1], pm->pos_2[2]);
			myfile << pm->buffer << endl;
			//myfile.close();

			sprintf(pm->buffer, "%f, %f, %f,", pm->imu_2[0]->pos_filtered, pm->imu_2[1]->pos_filtered, pm->imu_2[2]->pos_filtered);
			myfile2 << pm->buffer << endl;

			print_time = curr_time;
		}

		begin_time = curr_time;	

		memset(pm->buffer, '\0', pm->buffer_len);
    		
		pm->thread2 = false;
		ReleaseMutex (pm->grabmutex);

	}

FINISH:
    free((void *) pm->ArduinoSocket2);
	free(pm);
    return 0;
}

DWORD WINAPI SocketHandler3(void* lp) {
	passMe *pm = (passMe*) lp;

    int bytecount;
	int counter = 0;
	char *temp;

	float curr_time = 0, begin_time = 0, print_time = 0;
	ofstream myfile;
	myfile.open("pc_client_3.csv");

	ofstream myfile2;
	myfile2.open("pc_client_3_kalman.csv");

	while (true) {
		WaitForSingleObject (pm->grabmutex, INFINITE);
		printf("#####################333333333333333333\n");
		pm->thread3 = true;
		memset(pm->buffer, '\0', pm->buffer_len);

		if ((bytecount = recv(pm->ArduinoSocket3, pm->buffer, pm->buffer_len, 0)) == SOCKET_ERROR) {
			fprintf(stderr, "Error receiving data %d\n", WSAGetLastError());
			goto FINISH;
		}

		//printf("Received bytes %d\nReceived string %s\n", bytecount, pm->buffer);

		temp = strtok(pm->buffer, ",");
		pm->accel_float_3[0] = atof(temp);

		temp = strtok(NULL, ",");
		pm->accel_float_3[1] = atof(temp);
		
		temp = strtok(NULL, ",");
		pm->accel_float_3[2] = atof(temp);

		temp = strtok(NULL, ",");
		pm->mag_float_3[0] = atof(temp);

		temp = strtok(NULL, ",");
		pm->mag_float_3[1] = atof(temp);

		temp = strtok(NULL, ",");
		pm->mag_float_3[2] = atof(temp);
		
		temp = strtok(NULL, ",");
		pm->gyro_float_3[0] = atof(temp);

		temp = strtok(NULL, ",");
		pm->gyro_float_3[1] = atof(temp);

		temp = strtok(NULL, ",");
		pm->gyro_float_3[2] = atof(temp);

		temp = strtok(NULL, ",");
		pm->dist_sensor = atoi(temp);

		temp = strtok(NULL, ",");
		pm->light_sensor = atoi(temp);

		curr_time = (float) clock()/ CLOCKS_PER_SEC; // in seconds
		
		smoothSensor(pm);
		getStates3(pm, curr_time-begin_time);

		if (curr_time - print_time > .01) {
			sprintf(pm->buffer, "%f, %f, %f,", pm->pos_3[0], pm->pos_3[1], pm->pos_3[2]);
			myfile << pm->buffer << endl;
			//myfile.close();

			sprintf(pm->buffer, "%f, %f, %f,", pm->imu_3[0]->pos_filtered, pm->imu_3[1]->pos_filtered, pm->imu_3[2]->pos_filtered);
			myfile2 << pm->buffer << endl;

			sprintf(pm->buffer, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, 				%f, %f, %f, %f, %f, %d, %d\0", 
				pm->pos_1[0], pm->pos_1[1], pm->pos_1[2],
				pm->yaw[0], pm->pitch[0], pm->roll[0], 
				pm->pos_2[0], pm->pos_2[1], pm->pos_2[2],
				pm->yaw[1], pm->pitch[1], pm->roll[1], 
				pm->pos_3[0], pm->pos_3[1], pm->pos_3[2],
				pm->yaw[2], pm->pitch[2], pm->roll[2], 
				pm->imu_1[0]->pos_filtered, pm->imu_1[1]->pos_filtered, pm->imu_1[2]->pos_filtered,
				pm->dist_sensor_filtered, pm->light_sensor_filtered);

			if ((bytecount = send(pm->AndroidSocket, pm->buffer, strlen(pm->buffer), 0)) == SOCKET_ERROR) {
				fprintf(stderr, "Error sending data %d\n", WSAGetLastError());
				goto FINISH;
			}

			printf("Sent bytes %d\nSent string \"%s\"\n", bytecount, pm->buffer);

			print_time = curr_time;
		}				

		begin_time = curr_time;	

		memset(pm->buffer, '\0', pm->buffer_len);
    		
		pm->thread3 = false;
		ReleaseMutex (pm->grabmutex);
	}

FINISH:
    free((void *) pm->ArduinoSocket3);
	free(pm);
    return 0;
}

void getStates(struct passMe *pm, float dt) {
	float ax, ay, az;
	float curr_gyro_sum;
	float past_pos[3];
	float const ay_grav = -.06, ax_grav = 0, az_grav = -.99; // initialize accelerometer values

	ax = pm->accel_float_1[0] - ax_grav;
	ay = pm->accel_float_1[1] - ay_grav;
	az = pm->accel_float_1[2] - az_grav;

	past_pos[0] = pm->pos_1[0];
	past_pos[1] = pm->pos_1[1];
	past_pos[2] = pm->pos_1[2];

	// Sums up all gyroscope values to use for feedback
	curr_gyro_sum = pm->gyro_float_1[0] + pm->gyro_float_1[1] + pm->gyro_float_1[2];

	// If board not moving much
	if ((ax < .05 && ax > -.05) && (ay < .05 && ay > -.05) && (az < .10 && az > -.10)) {
		if (curr_gyro_sum < pm->past_gyro_sum_1 + 20) {
			printf("Not moving much\n");
			pm->vel_1[0] = pm->vel_1[1] = pm->vel_1[2] = 0;
			pm->past_gyro_sum_1 = pm->gyro_float_1[0] + pm->gyro_float_1[1] + pm->gyro_float_1[2];
			return;
		}
	} 

	pm->past_gyro_sum_1 = pm->gyro_float_1[0] + pm->gyro_float_1[1] + pm->gyro_float_1[2];

	// Calculate position coordinates
	if ((ax > .1 || ax < -.1) {
		pm->vel_1[0] += ax*dt;
		pm->pos_1[0] += pm->vel_1[0] * dt + .5*pow(dt,2)*ax;
	} else {
		pm->vel_1[0] = 0;
	}

	if ((ay > .1 || ay < -.1) {
		pm->vel_1[1] += ay*dt;
		pm->pos_1[1] += pm->vel_1[1] * dt + .5*pow(dt,2)*ay;
	} else {
		pm->vel_1[1] = 0;
	}

	if ((az > .2 || az < -.2) {
		pm->vel_1[2] += az*dt;
		pm->pos_1[2] += pm->vel_1[2] * dt + .5*pow(dt,2)*az;
	} else {
		pm->vel_1[2] = 0;
	}

	// If sensors moved too much, probably due to noise
	if (pm->pos_1[0] - past_pos[0] > 20 || pm->pos_1[0] - past_pos[0] < -20) { 
		pm->pos_1[0] = past_pos[0];
		pm->vel_1[0] = 0;
	}

	if (pm->pos_1[1] - past_pos[1] > 20 || pm->pos_1[1] - past_pos[1] < -20) {
		pm->pos_1[1] = past_pos[1];
		pm->vel_1[1] = 0;
	}

	if (pm->pos_1[2] - past_pos[2] > 20 || pm->pos_1[2] - past_pos[2] < -20) {
		pm->pos_1[2] = past_pos[2];
		pm->vel_1[2] = 0;
	}

	if (vel_1[0] != 0) 
		getKalmanPosition(pm->imu_1[0], pm->pos_1[0], pm->vel_1[0], curr_time-begin_time);

	if (vel_1[1] != 0) 
		getKalmanPosition(pm->imu_1[1], pm->pos_1[1], pm->vel_1[1], curr_time-begin_time);

	if (vel_1[2] != 0) 
		getKalmanPosition(pm->imu_1[2], pm->pos_1[2], pm->vel_1[2], curr_time-begin_time);
	
	printf("Pos 1: %f, %f, %f\n", pm->pos_1[0], pm->pos_1[1], pm->pos_1[2]);

	// Calculate orientation angles
	float roll = atan2(ay, az);
	roll = (pm->gyro_float_1[1] * dt) * 0.80 + roll * 0.20;

	float pitch = atan2(-ax, sqrt(ay * ay + az * az));
	pitch = (pm->gyro_float_1[0] * dt) * 0.80 + pitch * 0.20;
  
	float heading;
	float mx = pm->mag_float_1[0], my = pm->mag_float_1[1];
	float mz = pm->mag_float_1[2];

	if (my == 0)
		heading = (mx < 0) ? 180.0 : 0;
	else
		heading = atan2(mx, my);
    
	heading -= DECLINATION * PI / 180;
  
	if (heading > PI) heading -= (2 * PI);
	else if (heading < -PI) heading += (2 * PI);
	else if (heading < 0) heading += 2 * PI;

	float Bfy = mz * sin(roll) - my * cos(roll);
	float Bfx = mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll);
	float yaw = atan2(-Bfy, Bfx);
  
	// Convert everything from radians to degrees:
	heading *= 180.0 / PI;
	pitch *= 180.0 / PI;
	roll  *= 180.0 / PI;
	yaw *= 180.0 / PI;

	pm->yaw[0] = yaw;
	pm->roll[0] = roll;
	pm->pitch[0] = pitch;
}

void getStates2(struct passMe *pm, float dt) {
	float ax, ay, az;
	float curr_gyro_sum;
	float past_pos[3];
	float const ay_grav = -.06, ax_grav = 0, az_grav = -.99; // initialize accelerometer values

	ax = pm->accel_float_2[0] - ax_grav;
	ay = pm->accel_float_2[1] - ay_grav;
	az = pm->accel_float_2[2] - az_grav;

	past_pos[0] = pm->pos_2[0];
	past_pos[1] = pm->pos_2[1];
	past_pos[2] = pm->pos_2[2];

	// Sums up all gyroscope values to use for feedback
	curr_gyro_sum = pm->gyro_float_2[0] + pm->gyro_float_2[1] + pm->gyro_float_2[2];

	// If board not moving much
	if ((ax < .05 && ax > -.05) && (ay < .05 && ay > -.05) && (az < .10 && az > -.10)) {
		if (curr_gyro_sum < pm->past_gyro_sum_2 + 20) {
			printf("Not moving much\n");
			pm->vel_2[0] = pm->vel_2[1] = pm->vel_2[2] = 0;
			pm->past_gyro_sum_2 = pm->gyro_float_2[0] + pm->gyro_float_2[1] + pm->gyro_float_2[2];
			return;
		}
	} 

	pm->past_gyro_sum_2 = pm->gyro_float_2[0] + pm->gyro_float_2[1] + pm->gyro_float_2[2];

	// Calculate position coordinates
	if ((ax > .1 || ax < -.1) {
		pm->vel_2[0] += ax*dt;
		pm->pos_2[0] += pm->vel_2[0] * dt + .5*pow(dt,2)*ax;
	} else {
		pm->vel_2[0] = 0;
	}

	if ((ay > .1 || ay < -.1) {
		pm->vel_2[1] += ay*dt;
		pm->pos_2[1] += pm->vel_2[1] * dt + .5*pow(dt,2)*ay;
	} else {
		pm->vel_2[1] = 0;
	}

	if ((az > .2 || az < -.2) {
		pm->vel_2[2] += az*dt;
		pm->pos_2[2] += pm->vel_2[2] * dt + .5*pow(dt,2)*az;
	} else {
		pm->vel_2[2] = 0;
	}

	// If sensors moved too much, probably due to noise
	if (pm->pos_2[0] - past_pos[0] > 20 || pm->pos_2[0] - past_pos[0] < -20) { 
		pm->pos_2[0] = past_pos[0];
		pm->vel_2[0] = 0;
	}

	if (pm->pos_2[1] - past_pos[1] > 20 || pm->pos_2[1] - past_pos[1] < -20) {
		pm->pos_2[1] = past_pos[1];
		pm->vel_2[1] = 0;
	}

	if (pm->pos_2[2] - past_pos[2] > 20 || pm->pos_2[2] - past_pos[2] < -20) {
		pm->pos_2[2] = past_pos[2];
		pm->vel_2[2] = 0;
	}

	if (vel_2[0] != 0) 
		getKalmanPosition(pm->imu_2[0], pm->pos_2[0], pm->vel_2[0], curr_time-begin_time);

	if (vel_2[1] != 0) 
		getKalmanPosition(pm->imu_2[1], pm->pos_2[1], pm->vel_2[1], curr_time-begin_time);

	if (vel_2[2] != 0) 
		getKalmanPosition(pm->imu_2[2], pm->pos_2[2], pm->vel_2[2], curr_time-begin_time);
	
	printf("Pos 2: %f, %f, %f\n", pm->pos_2[0], pm->pos_2[1], pm->pos_2[2]);

	// Calculate orientation angles
	float roll = atan2(ay, az);
	roll = (pm->gyro_float_1[1] * dt) * 0.80 + roll * 0.20;

	float pitch = atan2(-ax, sqrt(ay * ay + az * az));
	pitch = (pm->gyro_float_1[0] * dt) * 0.80 + pitch * 0.20;
  
	float heading;
	float mx = pm->mag_float_2[0], my = pm->mag_float_2[1];
	float mz = pm->mag_float_2[2];

	if (my == 0)
		heading = (mx < 0) ? 180.0 : 0;
	else
		heading = atan2(mx, my);
    
	heading -= DECLINATION * PI / 180;
  
	if (heading > PI) heading -= (2 * PI);
	else if (heading < -PI) heading += (2 * PI);
	else if (heading < 0) heading += 2 * PI;

	float Bfy = mz * sin(roll) - my * cos(roll);
	float Bfx = mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll);
	float yaw = atan2(-Bfy, Bfx);
  
	// Convert everything from radians to degrees:
	heading *= 180.0 / PI;
	pitch *= 180.0 / PI;
	roll  *= 180.0 / PI;
	yaw *= 180.0 / PI;

	pm->yaw[1] = yaw;
	pm->roll[1] = roll;
	pm->pitch[1] = pitch;
}

void getStates3(struct passMe *pm, float dt) {
	float ax, ay, az;
	float curr_gyro_sum;
	float past_pos[3];
	float const ay_grav = -.06, ax_grav = 0, az_grav = -.99; // initialize accelerometer values

	ax = pm->accel_float_3[0] - ax_grav;
	ay = pm->accel_float_3[1] - ay_grav;
	az = pm->accel_float_3[2] - az_grav;

	past_pos[0] = pm->pos_3[0];
	past_pos[1] = pm->pos_3[1];
	past_pos[2] = pm->pos_3[2];

	// Sums up all gyroscope values to use for feedback
	curr_gyro_sum = pm->gyro_float_3[0] + pm->gyro_float_3[1] + pm->gyro_float_3[2];

	// If board not moving much
	if ((ax < .05 && ax > -.05) && (ay < .05 && ay > -.05) && (az < .10 && az > -.10)) {
		if (curr_gyro_sum < pm->past_gyro_sum_3 + 20) {
			printf("Not moving much\n");
			pm->vel_3[0] = pm->vel_3[1] = pm->vel_3[2] = 0;
			pm->past_gyro_sum_3 = pm->gyro_float_3[0] + pm->gyro_float_3[1] + pm->gyro_float_3[2];
			return;
		}
	} 

	pm->past_gyro_sum_3 = pm->gyro_float_3[0] + pm->gyro_float_3[1] + pm->gyro_float_3[2];

	// Calculate position coordinates
	if ((ax > .1 || ax < -.1) {
		pm->vel_3[0] += ax*dt;
		pm->pos_3[0] += pm->vel_3[0] * dt + .5*pow(dt,2)*ax;
	} else {
		pm->vel_3[0] = 0;
	}

	if ((ay > .1 || ay < -.1) {
		pm->vel_3[1] += ay*dt;
		pm->pos_3[1] += pm->vel_3[1] * dt + .5*pow(dt,2)*ay;
	} else {
		pm->vel_3[1] = 0;
	}

	if ((az > .2 || az < -.2) {
		pm->vel_3[2] += az*dt;
		pm->pos_3[2] += pm->vel_3[2] * dt + .5*pow(dt,2)*az;
	} else {
		pm->vel_3[2] = 0;
	}

	// If sensors moved too much, probably due to noise
	if (pm->pos_3[0] - past_pos[0] > 20 || pm->pos_3[0] - past_pos[0] < -20) { 
		pm->pos_3[0] = past_pos[0];
		pm->vel_3[0] = 0;
	}

	if (pm->pos_3[1] - past_pos[1] > 20 || pm->pos_3[1] - past_pos[1] < -20) {
		pm->pos_3[1] = past_pos[1];
		pm->vel_3[1] = 0;
	}

	if (pm->pos_3[2] - past_pos[2] > 20 || pm->pos_3[2] - past_pos[2] < -20) {
		pm->pos_3[2] = past_pos[2];
		pm->vel_3[2] = 0;
	}

	if (vel_3[0] != 0) 
		getKalmanPosition(pm->imu_3[0], pm->pos_3[0], pm->vel_3[0], curr_time-begin_time);

	if (vel_3[1] != 0) 
		getKalmanPosition(pm->imu_3[1], pm->pos_3[1], pm->vel_3[1], curr_time-begin_time);

	if (vel_3[2] != 0) 
		getKalmanPosition(pm->imu_3[2], pm->pos_3[2], pm->vel_3[2], curr_time-begin_time);
	
	printf("Pos 3: %f, %f, %f\n", pm->pos_3[0], pm->pos_3[1], pm->pos_3[2]);

	// Calculate orientation angles
	float roll = atan2(ay, az);
	roll = (pm->gyro_float_1[1] * dt) * 0.80 + roll * 0.20;

	float pitch = atan2(-ax, sqrt(ay * ay + az * az));
	pitch = (pm->gyro_float_1[0] * dt) * 0.80 + pitch * 0.20;
  
	float heading;
	float mx = pm->mag_float_3[0], my = pm->mag_float_3[1];
	float mz = pm->mag_float_3[2];

	if (my == 0)
		heading = (mx < 0) ? 180.0 : 0;
	else
		heading = atan2(mx, my);
    
	heading -= DECLINATION * PI / 180;
  
	if (heading > PI) heading -= (2 * PI);
	else if (heading < -PI) heading += (2 * PI);
	else if (heading < 0) heading += 2 * PI;

	float Bfy = mz * sin(roll) - my * cos(roll);
	float Bfx = mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll);
	float yaw = atan2(-Bfy, Bfx);
  
	// Convert everything from radians to degrees:
	heading *= 180.0 / PI;
	pitch *= 180.0 / PI;
	roll  *= 180.0 / PI;
	yaw *= 180.0 / PI;

	pm->yaw[2] = yaw;
	pm->roll[2] = roll;
	pm->pitch[2] = pitch;
}

void smoothSensor(struct passMe *data) { //low-pass filter
	if (data->dist_sensor_filtered == -1) {
		data->dist_sensor_filtered = data->dist_sensor;
	} else {
		data->dist_sensor_filtered = data->dist_sensor_filtered + .2 * (data->dist_sensor - data->dist_sensor_filtered);
	}

	if (data->light_sensor_filtered == -1) {
		data->light_sensor_filtered = data->light_sensor;
	} else {
		data->light_sensor_filtered = data->light_sensor_filtered + .2 * (data->light_sensor - data->light_sensor_filtered);
	}
}

void getKalmanPosition(struct kalman *imu, float p1, float v1, float dt) {
	imu->P[0][0] += dt * (dt*imu->P[1][1] + imu->P[0][1] + imu->P[1][0] + imu->Q_pos);
	imu->P[0][1] += dt * imu->P[1][1];
	imu->P[1][0] += dt * imu->P[1][1];
	imu->P[1][1] += imu->Q_vel * dt;

    float y = p1 - imu->pos_filtered; // Angle difference

    float S = imu->P[0][0] + imu->R_measure; // Estimate error

    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = imu->P[0][0] / S;
    K[1] = imu->P[1][0] / S;

	imu->pos_filtered += K[0] * y;
	imu->vel_filtered += K[1] * y;

    float P00_temp = imu->P[0][0];
    float P01_temp = imu->P[0][1];

    imu->P[0][0] -= K[0] * P00_temp;
    imu->P[0][1] -= K[0] * P01_temp;

    imu->P[1][0] -= K[1] * P00_temp;
    imu->P[1][1] -= K[1] * P01_temp;

	printf("Kalman filtering: ");
	for (int i=0; i<3; i++) {
		printf("%f, ", imu->pos_filtered);
	}
	printf("\n");
	
} 
