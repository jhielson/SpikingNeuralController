// Description:   Robot execution code for genetic algorithm

#include <webots/robot.h>
#include <webots/motor.h>
//#include <webots/receiver.h>
//#include <webots/emitter.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
//#include <unistd.h>
#include "mlp.c"

// Global defines
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define SIMULATION 0  // for wb_robot_get_mode() function
#define REALITY 2     // for wb_robot_get_mode() function
#define TIME_STEP 32  // [ms]

#define NUM_SENSORS 8
#define NUM_SENSORS_NN 8
#define NUM_WHEELS 4 // 2 but + and - values
#define NUM_HIDDEN 4
#define NUM_HIDDEN_INPUT 5 // 4 + 1 from input
#define GENOTYPE_SIZE 56
#define RANGE (1024 / 2)
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23

#define NB_GROUND_SENS 3
#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2

#define MAX_VEL 6

WbDeviceTag gs[NB_GROUND_SENS];
float gs_value[NB_GROUND_SENS]={0,0,0};

// float vectorWeights[GENOTYPE_SIZE];
// float vectorWeights[GENOTYPE_SIZE] = {-4.007387, 5.457971, -10.000000, 9.648352, -1.237472, 4.415173, 8.942057, 5.608282, 4.510410, 3.262457, 1.663498, -10.000000, -0.834150, 10.000000, 10.000000, 1.818801, 0.029110, -10.000000, -10.000000, 10.000000, -10.000000, -7.417870, -3.512601, 2.050406, 6.838716, 6.167096, -10.000000, -10.000000, -1.194183, -8.232101, 10.000000, 10.000000, -0.866501, -1.953355, -10.000000, 10.000000, -10.000000, -0.458687, 6.262162, 10.000000, 0.645230, -10.000000, 10.000000, -10.000000, -10.000000, 10.000000, 10.000000, 8.052084, 10.000000, -10.000000, -2.136456, -10.000000, -10.000000, 10.000000, 3.628583, 10.000000};
//float vectorWeights[GENOTYPE_SIZE] = {-4.0073, 5.4579, -10.0000, 9.6483, -1.2374, 4.4151, 8.9420, 5.6082, 4.5104, 3.2624, 1.6634, -10.0000, -0.8341, 10.0000, 10.0000, 1.8188, 0.0291, -10.0000, -10.0000, 10.0000, -10.0000, -7.4178, -3.5126, 2.0504, 6.8387, 6.1670, -10.0000, -10.0000, -1.1941, -8.2321, 10.0000, 10.0000, -0.8665, -1.9533, -10.0000, 10.0000, -10.0000, -0.4586, 6.2621, 10.0000, 0.6452, -10.0000, 10.0000, -10.0000, -10.0000, 10.0000, 10.0000, 8.0520, 10.0000, -10.0000, -2.1364, -10.0000, -10.0000, 10.0000, 3.6285, 10.0000};
//float matrix1[NUM_HIDDEN_INPUT][NUM_SENSORS_NN];
//float matrix2[NUM_WHEELS][NUM_HIDDEN];

//Variables for fitness function
bool found = false;
float avgRotation = 0;
float difRotation = 0;
float reward = 0;
float sensorHigh = 0;
float maxSensorAvg = 0;
int count = 0;
float sensor1avg, sensor2avg,  sensor3avg, sensor4avg, sensor5avg, sensor6avg, sensor7avg, sensor8avg;
float fitness = 0;

// Proximity Sensors
WbDeviceTag sensors[NUM_SENSORS];  
// Receive Genes from Supervisor
WbDeviceTag receiver;              

float bigger (float one, float two, float three, float four, float five, float six, float seven, float eight){
    float max = 0;
    if (one > max) {
        max = one;
    }
    if (two > max) {
        max = two;
    }
    if (three > max) {
        max = three;
    }
    if (four > max) {
        max = four;
    }
    if (six > max) {
        max = six;
    }
    if (five > max) {
        max = five;
    }
    if (seven > max) {
        max = seven;
    }
    if (eight > max) {
        max = eight;
    }
    return max;
}

/*void split(){ /////////////////////////////////////////////////////////////////////////////////
    int k, j;
    for (k = 0; k < 8; k++) { // input layer
        j = 0;
        matrix1[j][k] = vectorWeights[8*j+k];
    }
    
    for (j = 1; j < 5; j++) { // hidden layer
        for (k = 0; k < 8; k++) { // input layer
            matrix1[j][k] = vectorWeights[8*j+k];
        }
    }
    
    for (j = 0; j < 4; j++) {   // output layer
        for (k = 0; k < 4; k++) { // hidden layer
            matrix2[j][k] = vectorWeights[4*j+k+40];
        }
    }
}
*/
// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
/*void check_for_new_genes() {
        //WE COMMENTED THIS OUT TO HARD CODE VALUES
    // if (wb_receiver_get_queue_length(receiver) > 0) {
    if (true) {
        // check that the number of genes received match what is expected
        //WE COMMENTED THIS OUT TO HARD CODE VALUES
        // assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(float));

        // And wait untill the ga recieves them
        // copy new genes directly in the sensor/actuator matrix
        // it's the GA's responsability to find a functional mapping
        //WE COMMENTED THIS OUT TO HARD CODE VALUES
        // memcpy(vectorWeights, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(float));
        split();
        
        //Re start all measurement
        avgRotation = 0;
        difRotation = 0;
        sensorHigh = 0;
        reward = 0;
        maxSensorAvg = 0;
        count = 0;
        sensor1avg = 0;
        sensor2avg = 0;
        sensor3avg = 0;
        sensor4avg = 0;
        sensor5avg = 0;
        sensor6avg = 0;
        sensor7avg = 0;
        sensor8avg = 0;
        found = false;
        fitness = 0;
        
        // prepare for receiving next genes packet
        // wb_receiver_next_packet(receiver);
        //printf("Package received! \n");
        //fflush(stdout);
    }else{
        // Create empty file
        FILE *contact;
        contact=fopen("C:\\Users\\Jhielson\\Downloads\\Code\\Code\\webots\\controllers\\rnnGAsupervisor_Jh\\contact.txt","w");
        if(contact==NULL){
            printf("Error! \n");
            fflush(stdout);
            exit(1);
        }
        //First send the fitness values       
        float movement = ((avgRotation)/(count))/(2*MAX_VEL);
        float direction = ((difRotation)/(count))/(2*MAX_VEL);     
        float sensor = bigger(sensor1avg,sensor2avg,sensor3avg,sensor4avg,sensor5avg,sensor6avg,sensor7avg,sensor8avg)/(count);
        float fitnessValue = fitness/count;
        
        if (found){
            fprintf(contact,"%lf\n", (1.0));
        }else{
            fprintf(contact,"%lf\n", (0.0));
        }   
        
        fprintf(contact,"%lf\n", movement); 
        fprintf(contact,"%lf\n", direction);
        fprintf(contact,"%lf\n", sensor);     
        fprintf(contact,"%lf\n", fitnessValue);
        fclose(contact);   
    }
}*/

static float clip_value(float value, float min_max) {
    if (value > min_max)
        return min_max;
    else if (value < -min_max)
        return -min_max;
    
    return value;
}

void sense_compute_and_actuate(WbDeviceTag left_motor, WbDeviceTag right_motor) {

    // read IR sensor values
    int i,min,max;
    float sensor_values[NUM_SENSORS];
    for (i = 0; i < NUM_SENSORS; i++){
        /*
        if(wb_distance_sensor_get_value(sensors[i]) > 2211){
            sensor_values[i] = 1.0;
            max = sensor_values[i];
        }else if(wb_distance_sensor_get_value(sensors[i]) > 676){
            sensor_values[i] = 0.75;
        }else if(wb_distance_sensor_get_value(sensors[i]) > 306){
            sensor_values[i] = 0.50;
        }else if(wb_distance_sensor_get_value(sensors[i]) > 153){
            sensor_values[i] = 0.25;
        }else{
            sensor_values[i] = 0.0;
            min = sensor_values[i];
        }
        */       

        sensor_values[i] = wb_distance_sensor_get_value(sensors[i])/4095;
        if(sensor_values[i] < 0){
            min = 0;
            sensor_values[i] = min;
        }
        if(sensor_values[i] > 1){
            max = 1;
            sensor_values[i] = max;
        }

        //printf(">> %d : %f \n", i, sensor_values[i]);
        //fflush(stdout);

    }
    
    // read ground sensors
    for(i = 0; i < NB_GROUND_SENS; i++) 
        gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    
    sensor1avg += sensor_values[0];
    sensor2avg += sensor_values[1];
    sensor3avg += sensor_values[2];
    sensor4avg += sensor_values[3];
    sensor5avg += sensor_values[4];
    sensor6avg += sensor_values[5];
    sensor7avg += sensor_values[6];
    sensor8avg += sensor_values[7];
    
    float possibleMax = bigger(sensor_values[0], sensor_values[1], sensor_values[2], sensor_values[3], sensor_values[4], sensor_values[5], sensor_values[6], sensor_values[7]);    
    maxSensorAvg += (possibleMax-min)/(max-min);
    
    float * wheel_speed = NULL;
    wheel_speed = runInput2(sensor_values);
    
    // clip to e-puck max speed values to avoid warning
    wheel_speed[0] = clip_value(wheel_speed[0], MAX_VEL);
    wheel_speed[1] = clip_value(wheel_speed[1], MAX_VEL);
    
    //For the reward, if the robot finds the black we give him points.
    if (gs_value[GS_CENTER]<400){ 
        //Black
        found = true;
    }    
    
    //Velocity:
    //printf("Vel. L:  %f \n", wheel_speed[0]);
    //printf("Vel. R:  %f \n", wheel_speed[1]);    
    count++;
    avgRotation += (wheel_speed[1] + wheel_speed[0]);
    difRotation += fabs(wheel_speed[1] - wheel_speed[0]);
    
    if(found)
    {
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
    }else{
        wb_motor_set_velocity(left_motor, wheel_speed[0]);
        wb_motor_set_velocity(right_motor, wheel_speed[1]);
        //wb_motor_set_velocity(left_motor, 0);
        //wb_motor_set_velocity(right_motor, 0);
    }
    
    float currentMovement = (wheel_speed[1] + wheel_speed[0])/(2*MAX_VEL);
    //printf("Current Movement: %lf \n", (wheel_speed[1] + wheel_speed[0])/(2*MAX_VEL));
    //fflush(stdout);
    float currentDirection = (1-sqrt(abs(wheel_speed[1] - wheel_speed[0])/2*MAX_VEL));
    //printf("Current Direction: %lf \n", abs(wheel_speed[1] - wheel_speed[0])/MAX_VEL);
    //fflush(stdout);
    float currentSensor = (1-possibleMax);
    //printf("Current Sensor: %lf \n", (1-possibleMax));
    //fflush(stdout);
    fitness += currentMovement*currentDirection*currentSensor;    
    //printf("Velocities: %lf, %lf\n", wheel_speed[0], wheel_speed[1]);
    //flush(stdout);
}

int main(int argc, const char *argv[]) {
    // initialize Webots
    wb_robot_init();  
    int i;
    
    // find simulation step in milliseconds
    int time_step = 256;
    // int time_step;
    // printf("Calculating time_Step \n");
    // fflush(stdout);
    // if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    //     printf("e-puck2 robot\n");
    //     fflush(stdout);
    //     time_step = 64;
    // } else {  // original e-puck
    //     printf("e-puck robot\n");
    //     fflush(stdout);
    //     time_step = 256;
    // }
    //time_step = 1024;
    //time_step = wb_robot_get_basic_time_step();
    // printf("Time_Step = %d \n",time_step);
    // fflush(stdout);
    
    // get a handler to the motors and set target position to infinity (speed control).
    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    
    // get a handler to the position sensors and enable them. 
    WbDeviceTag left_position_sensor = wb_robot_get_device("left wheel sensor");
    WbDeviceTag right_position_sensor = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(left_position_sensor, time_step);
    wb_position_sensor_enable(right_position_sensor, time_step);
    
    char device_name[4];
    // find and enable proximity sensors   
    for (i = 0; i < NUM_SENSORS; i++) {
        sprintf(device_name, "ps%d", i);
        sensors[i] = wb_robot_get_device(device_name);
        wb_distance_sensor_enable(sensors[i], time_step);
    }
    
    // Find and enable ground sensors
    for (i = 0; i < NB_GROUND_SENS; i++) {
        sprintf(device_name, "gs%d", i);
        gs[i] = wb_robot_get_device(device_name); 
        wb_distance_sensor_enable(gs[i],time_step);
    }

    // find and enable receiver
    //receiver = wb_robot_get_device("receiver");
    //wb_receiver_enable(receiver, time_step);
     
    // run until simulation is restarted
    while (wb_robot_step(time_step) != -1) {          
        //check_for_new_genes();
        sense_compute_and_actuate(left_motor,right_motor);
    }
    
    // printf("Closing controller \n");  
    // fflush(stdout);  
    wb_robot_cleanup();  // cleanup Webots
    return 0;            // ignored
}