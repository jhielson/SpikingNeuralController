// Description:   Robot execution code for genetic algorithm

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "mlp.c"
#include <time.h> 

// Global defines
#define TRUE 1
#define FALSE 0
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1

#define NUM_SENSORS 8
#define NUM_SENSORS_NN 8
#define NUM_WHEELS 4 // 2 but + and - values
#define NUM_HIDDEN 4
#define NUM_HIDDEN_INPUT 5 // 4 + 1 from input

#define GENOTYPE_SIZE 88

#define NB_GROUND_SENS 3
#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2

int time_step;

WbDeviceTag gs[NB_GROUND_SENS];
double gs_value[NB_GROUND_SENS]={0,0,0};

double vectorWeights[GENOTYPE_SIZE];

//Variables for fitness function
bool reward = false;
bool penalty = false;
int count = 0;
double fitness = 0;
double fitnessValue = 0;
double message[3];

int new_genotype = 0;
bool flagWalls = false;

// Proximity Sensors
WbDeviceTag sensors[NUM_SENSORS];
  
// Receive Genes from Supervisor
WbDeviceTag receiver; 

// Webots: to send fitness to supervisor
static WbDeviceTag emitter;          

double bigger (double one, double two, double three, double four, double five, double six, double seven, double eight){
    double max = 0;
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

/*
void split(){ 

    for (int k = 0; k < 8; k++) { // input layer
        int j = 0;
        matrix1[j][k] = vectorWeights[8*j+k];
    }
    
    for (int j = 1; j < 5; j++) { // hidden layer
        for (int k = 0; k < 8; k++) { // input layer
            matrix1[j][k] = vectorWeights[8*j+k];
        }
    }
    
    for (int j = 0; j < 4; j++) {   // output layer
        for (int k = 0; k < 4; k++) { // hidden layer
            matrix2[j][k] = vectorWeights[4*j+k+40];
        }
    }
}
*/

// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
int check_for_new_genes() {
    if (wb_receiver_get_queue_length(receiver) > 0) {
        // Check if it is over
        if(wb_receiver_get_data_size(receiver) == sizeof(double)) return 0;
        // check that the number of genes received match what is expected
        assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double));
        // copy new genes directly in the sensor/actuator matrix
        memcpy(vectorWeights, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(double));
        // New Genotype
        new_genotype = 1;
        
        // Reset Variables
        count = 0;
        reward = false;
        penalty = false;
        fitness = 0;
        fitnessValue = 0;
        
        // prepare for receiving next genes packet
        wb_receiver_next_packet(receiver);
    }else{
        // Genotype
        new_genotype = 0;
        // Fitness
        message[0] = fitnessValue;
        // Reward and punishment
        if(reward) message[1] = 100;
        else if(penalty) message[1] = 0;
        else message[1] = 0.0;
        // Stop if hit a Wall
        if(flagWalls) message[2] = 1;
        else message[2] = 0; 
        // Stop if it ends task
        if(reward || penalty) message[2] = 1;
        
        //printf(">> F %lf : R %lf :  W %lf\n", message[0], message[1], message[2]);
        //fflush(stdout);
        
        // Send
        wb_emitter_send(emitter, message, 3 * sizeof(double));
    }
    return 1;
}

void sense_compute_and_actuate(WbDeviceTag left_motor, WbDeviceTag right_motor) {

    /////////////////////////////////////////////////////////////////////////
    // Position
    /////////////////////////////////////////////////////////////////////////
    WbNodeRef robot = wb_supervisor_node_get_from_def("EPUCK_2_MLP");
    const double *position = wb_supervisor_node_get_position(robot); 
  
    // Corridor
    double start_position = 2.1;
    double end_position = 1.4; 
    
    // Calculate distance inside the corridor (Vertical)
    double current_position_Vertical_adapted = 0;
    if(position[2] < end_position) current_position_Vertical_adapted = end_position;
    else if(position[2] > start_position) current_position_Vertical_adapted = start_position;
    else current_position_Vertical_adapted = position[2];
    double distance_corridor_Vertical = start_position - current_position_Vertical_adapted;
    
    // Calculate distance inside the corridor (Horizontal)
    double distance_corridor_Horizontal = 0;
    /*if(distance_corridor_Vertical == start_position - end_position){
        // Large Corridor
        if (position[0] < 0.5){
            distance_corridor_Horizontal =  fabs(position[0]-0.00);
            
        }
        // Narrow Corridor
        if (position[0] > 0.5){
            distance_corridor_Horizontal =  fabs(position[0]-1.03);
        }   
    }   
    
    // Calculate final distance
    double total_horizontal = 0.30;
    */
    double total_horizontal = 0;
    double total_vertical = start_position-end_position;
    double total_distance = total_vertical+total_horizontal;
    double distance_corridor = (distance_corridor_Horizontal+distance_corridor_Vertical)/total_distance;   
     
    // Check Reward
    if(reward == false){
        // Large Corridor
        if (position[0] < 2.7){
            reward = true;
        }
        // Narrow Corridor
        if (position[0] > 4.30){
            reward = true;
        }   
    }   
        
    // Wrong turn >> penalty
    if (position[0] < 3.70 && position[0] > 3.30){
        penalty = true;
    } 
            
    /////////////////////////////////////////////////////////////////////////
    // IR sensor values
    /////////////////////////////////////////////////////////////////////////    
    int i;
    double realValues[NUM_SENSORS];
    double sensor_values[NUM_SENSORS];
    
    int min = 50;
    int max = 250;
    flagWalls = false;
    
    for (i = 0; i < NUM_SENSORS; i++){        
        sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
        realValues[i] = sensor_values[i]/3800;
        // hit the wall   
        //if(sensor_values[i] > 1000 && position[2] > end_position){ 
        //    flagWalls = true;
        //}
        // min     
        if(sensor_values[i] < min){
            sensor_values[i] = min;
        }
        // max
        if(sensor_values[i] > max){
            sensor_values[i] = max;
        }
        sensor_values[i] = (sensor_values[i]-min)/(max-min);
        
        // Add Noise
        double noise = 0.0002*(rand()%100)-0.01;
        sensor_values[i] += noise;
        // min     
        if(sensor_values[i] < 0){
            sensor_values[i] = 0;
        }
        // max
        if(sensor_values[i] > 1){
            sensor_values[i] = 1;
        }
        
    }
          
    // Closest to obstacles
    double possibleMax = (bigger(realValues[0], realValues[1], realValues[2], realValues[3], realValues[4], realValues[5], realValues[6], realValues[7]));    
    
    /////////////////////////////////////////////////////////////////////////
    // Read ground sensors
    /////////////////////////////////////////////////////////////////////////
    for(i = 0; i < NB_GROUND_SENS; i++) 
        gs_value[i] = wb_distance_sensor_get_value(gs[i]);
        
    // Task reward
    if (gs_value[GS_CENTER]<400){ 
        //Black
        reward = true;
    }    
        
    /////////////////////////////////////////////////////////////////////////
    // Send velocity command:   
    /////////////////////////////////////////////////////////////////////////
    
    // Define wheels velocities based on sensor values
    double * wheel_speed = NULL;
    wheel_speed = runInput(vectorWeights, sensor_values, new_genotype);
    
    if(reward || penalty || flagWalls){
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
    }else{
        wb_motor_set_velocity(left_motor, wheel_speed[0]);
        wb_motor_set_velocity(right_motor, wheel_speed[1]);
    }
    
    //printf("Velocities: %lf, %lf\n", wheel_speed[0], wheel_speed[1]);
    //fflush(stdout);
        
    /////////////////////////////////////////////////////////////////////////
    // Fitness function
    /////////////////////////////////////////////////////////////////////////  
    double currentMovement = (fabs(wheel_speed[1] + wheel_speed[0]))/(2*MAX_VEL);
    //printf("Current Movement: %lf \n", currentMovement);
    //fflush(stdout);
    double variation = (fabs(wheel_speed[1]-wheel_speed[0]))/(2*MAX_VEL);
    double currentDirection = (1-(sqrt(variation)));
    //printf("Current Direction: %lf \n", currentDirection);
    //fflush(stdout);
    double currentSensor = (1-possibleMax);
    //printf("Current Sensor: %lf \n", currentSensor);
    //fflush(stdout);
    //printf("Current Fitness: %lf \n", currentMovement*currentDirection*currentSensor);
    //fflush(stdout);
    // Integrate fitness
    if(!reward && !penalty)
    {      
      count++;      
      fitness += currentMovement*currentDirection*currentSensor*distance_corridor;        
      fitnessValue = ((fitness/count)*1000);
      if(position[0] < 4.5) fitnessValue *= 0.6; // 2/3      
    }
           
}

int main(int argc, const char *argv[]) {
    // initialize Webots
    wb_robot_init();  
    int i;
    
    // find simulation step in milliseconds
    printf("Calculating time_Step \n");
    fflush(stdout);
    if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
        printf("e-puck2 robot\n");
        fflush(stdout);
        time_step = 64;
    } else {  // original e-puck
        printf("e-puck robot\n");
        fflush(stdout);
        time_step = 256;
    }
    time_step = 256;
    //time_step = wb_robot_get_basic_time_step();
    printf("Time_Step = %d \n",time_step);
    fflush(stdout);
    
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
    
    // the emitter to send fitness values
    emitter = wb_robot_get_device("emitter");
    
    // find and enable receiver
    receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, time_step);
    
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
        
    /* initialize random seed: */
    srand (time(NULL)); 
     
    // run until simulation is restarted
    while (wb_robot_step(time_step) != -1) {          
        if(!check_for_new_genes()) break;
        sense_compute_and_actuate(left_motor,right_motor);
    }
    
    printf("Closing controller \n");  
    fflush(stdout);  
    wb_robot_cleanup();  // cleanup Webots
    return 0;            // ignored
}