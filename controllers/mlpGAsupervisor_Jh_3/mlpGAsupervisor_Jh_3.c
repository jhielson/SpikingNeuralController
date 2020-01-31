//   Description:   Supervisor code for genetic algorithm

#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/display.h>
#include <webots/keyboard.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <random.h>
#include <time.h>

#include "genotype.c"
#include "population.c"
#include "Auxiliar.c"

// GA parameters
static const int POPULATION_SIZE = 20; 
static const int NUM_GENERATIONS = 150;

//Duration of the run in each maze
int NUM_SECONDS = 150;

// To calculate fitness using average
int iterations = 6;

// Based on epuck robot
#define NUM_SENSORS 8
#define NB_GROUND_SENS 3

// Based on the number of neurons and their connectivities
#define GENOTYPE_SIZE 88

// Webots: proximity sensors
WbDeviceTag sensors[NUM_SENSORS];  

// Webots: ground sensors
WbDeviceTag gs[NB_GROUND_SENS];
double gs_value[NB_GROUND_SENS]={0,0,0};

// Fitness
double reward = 0;
double fitnessWithoutReward = 0;

// index access
enum { X, Y, Z };

// Webots: time step
static int time_step;

// Webots: to send genes to robot
static WbDeviceTag emitter; 

// Webots: to display the fitness evolution  
static WbDeviceTag display;  

// Webots: for receiving values from robot
static WbDeviceTag receiver;  

// display            
static int display_width, display_height;

// GA population
static Population population;

// Webots: for reading or setting the robot's position and orientation
static WbFieldRef robot_translation;
static WbFieldRef robot_rotation;

 // a translation needs 3 doubles
static double robot_trans0[3];
 
// a rotation needs 4 doubles
static double robot_rot0[4];    

//Values for fitness
double values[3];

void saveFitnessBestIndividual(int iteration){
    // Save best individual
    Genotype fittest = population_get_fittest(population);
    // File Name
    char str[20];
    strcpy(str, "fittest");
    // convert number to string
    char snum[5] = "";
    sprintf(snum, "%d", iteration);
    strcat(str, snum);
    strcat(str, ".txt");
    char *FILE_NAME = str;
    // Generate file
    FILE *outfile = fopen(FILE_NAME, "w");
    if (outfile) {
        genotype_fwrite(fittest, outfile);
        fclose(outfile);
        printf("wrote best genotype into %s\n", FILE_NAME);
        fflush(stdout);
    }
    else{
        printf("unable to write %s\n", FILE_NAME);
        fflush(stdout);
    }
}

int compare(const void * a, const void * b)
{
    if (*(double*)a > *(double*)b)
      return 1;
    else if (*(double*)a < *(double*)b)
      return -1;
    else
      return 0;  
}

float float_rand( float min, float max ){
    float scale = rand() / (float) RAND_MAX; /* [0, 1.0] */
    return min + scale * ( max - min );      /* [min, max] */
}

void draw_scaled_line(int generation, double y1, double y2) {
    const double XSCALE = (double)display_width / NUM_GENERATIONS;
    const double YSCALE = 0.3;
    wb_display_draw_line(display, (generation - 0.5) * XSCALE, display_height - y1 * YSCALE,
                         (generation + 0.5) * XSCALE, display_height - y2 * YSCALE);
}

void plot_fitness(int generation, double best_fitness, double average_fitness) {
    static double prev_best_fitness = 0.0;
    static double prev_average_fitness = 0.0;
    if (generation > 0) {
        wb_display_set_color(display, 0xff0000); // red
        draw_scaled_line(generation, prev_best_fitness, best_fitness);
        
        wb_display_set_color(display, 0x00ff00); // green
        draw_scaled_line(generation, prev_average_fitness, average_fitness);
    }
    
    prev_best_fitness = best_fitness;
    prev_average_fitness = average_fitness;
}

int calculateFitness(int i){
    if (wb_receiver_get_queue_length(receiver) > 0) {
        double vectorMessage[3];
        memcpy(vectorMessage, wb_receiver_get_data(receiver), 3 * sizeof(double));
        //printf("\nFitness: %lf   Reward: %lf   Wall: %lf \n", vectorMessage[0],vectorMessage[1],vectorMessage[2]);
        //fflush(stdout);
        fitnessWithoutReward = vectorMessage[0];
        reward = vectorMessage[1];       
        wb_receiver_next_packet(receiver);
        if(i>10) return vectorMessage[2];
    }
    return 0;
}

void clearQueue(){
    while(wb_receiver_get_queue_length(receiver) > 0) wb_receiver_next_packet(receiver);
}

// run the robot simulation for the specified number of seconds
void run_seconds(double seconds) {
    int i, n = 1000.0 * seconds / time_step;
    for (i = 0; i < n; i++) { 
        if(i == 0){
            clearQueue();
        }else{    
            //printf("Time: %d \n", (int)((i*time_step)/1000.0));
            //fflush(stdout);
            if (calculateFitness(i) == 1){
              //printf("Break - Wall %d \n", i);
              //fflush(stdout);
              return;
            }
        }
        wb_robot_step(time_step);
    }
}

// compute fitness
double measure_fitness(int demo) {
    //printf("\nFitnessW: %lf \n", fitnessWithoutReward);
    //fflush(stdout);
    //printf("FitnessR: %lf \n", reward);
    //fflush(stdout);
    if(demo == 0) return (fitnessWithoutReward+reward);
    else{
      if(reward > 0) return reward; 
      else return 0;
    }
}

// evaluate one genotype at a time
double evaluate_genotype(Genotype genotype, int demo) {
       
    // Initial positions
    const double MAZE1[3] = { 3.0, 0, 4.1 };
    const double MAZE2[3] = { 4.03058, 0, 4.1 };  
      
    // Fitness
    double fitness = 0;
     
    // Experiments
    for (int i = 0; i<iterations; i++) {       
        // Orientation
        double angleRobot[6] = { -0.26,-0.26,0,0,0.26,0.26};
        double ROT[4];
        
        // demo or GA
        if(demo == 1){
            ROT[0] = 0;
            ROT[1] = 1;
            ROT[2] = 0;
            ROT[3] = angleRobot[i]; //float_rand(0,6.3);
        }else{
            ROT[0] = 0;
            ROT[1] = 1;
            ROT[2] = 0;
            ROT[3] = angleRobot[i];          
        }
        
        if (i % 2 == 0) {
            //Run in maze 1:
            // send genotype to robot for evaluation
            wb_emitter_send(emitter, genotype_get_genes(genotype), GENOTYPE_SIZE * sizeof(double));
            wb_supervisor_field_set_sf_vec3f(robot_translation, MAZE1);
            wb_supervisor_field_set_sf_rotation(robot_rotation, ROT);
            run_seconds(NUM_SECONDS);
            /// measure fitness
            fitness += measure_fitness(demo);
        }else{
            //Run in maze 2:
            wb_emitter_send(emitter, genotype_get_genes(genotype), GENOTYPE_SIZE * sizeof(double));
            wb_supervisor_field_set_sf_vec3f(robot_translation, MAZE2);
            wb_supervisor_field_set_sf_rotation(robot_rotation, ROT);
            run_seconds(NUM_SECONDS);
            /// measure fitness
            fitness += measure_fitness(demo);
        }
    }
        
    // Average    
    double avg = fitness/iterations;    

    // Save fitness on genotype
    genotype_set_fitness(genotype, avg);
    return avg;

}

void run_optimization() {    
    printf("---\n");
    fflush(stdout);
    printf("starting GA optimization ...\n");
    fflush(stdout);
    printf("population size is %d, genome size is %d\n", POPULATION_SIZE, GENOTYPE_SIZE);
    fflush(stdout);
            
    int i, j;
    double current_best_fitness = 0;
    double bestf[NUM_GENERATIONS];
    double avrgf[NUM_GENERATIONS];
    for  (i = 0; i < NUM_GENERATIONS; i++) {        
        for (j = 0; j < POPULATION_SIZE; j++) {
            // Get the genotype
            Genotype genotype = population_get_genotype(population, j);   
            // Evaluate 
            evaluate_genotype(genotype,0);  
        }
        
        ////////////////////////////////////////////////
        // Save Values related to current population 
        ////////////////////////////////////////////////
               
        // Get values
        double best_fitness = genotype_get_fitness(population_get_fittest(population));
        double average_fitness = population_compute_average_fitness(population);
        
        // display results
        plot_fitness(i, best_fitness, average_fitness);
        printf(">>>>>>>>>>>>>>>>>>> best fitness (%d): %g\n", i, best_fitness);
        fflush(stdout);
        printf(">>>>>>>>>>>>>>>>>>> average fitness (%d): %g\n\n", i, average_fitness);
        fflush(stdout);
        // Save on File: fitness evolution
        FILE *results1;
        FILE *results2;
        results1=fopen("bestFitness.txt","a");
        results2=fopen("avgFitness.txt","a");
        if(results1==NULL||results2==NULL){
            printf("Error!");
            fflush(stdout);
            exit(1);
        }
        fprintf(results1,"%lf\n", best_fitness);
        fprintf(results2,"%lf\n", average_fitness);
        fclose(results1);
        fclose(results2);
        
        // Add the average and best fitness to the vector
        bestf[i] = best_fitness;
        avrgf[i] = average_fitness;
        
        // Save the best individual every time-step
        if(i%25 == 0)
           saveFitnessBestIndividual(i);
       
        // Save the best individual so far
        if(current_best_fitness < best_fitness){
            current_best_fitness = best_fitness;
            // Save best individual
            Genotype fittest = population_get_fittest(population);
            static const char *FILE_NAME = "fittest.txt";
            FILE *outfile = fopen(FILE_NAME, "w");
            if (outfile) {
                genotype_fwrite(fittest, outfile);
                fclose(outfile);
                printf("wrote best genotype into %s\n", FILE_NAME);
                fflush(stdout);
            }
            else{
                printf("unable to write %s\n", FILE_NAME);
                fflush(stdout);
            }
        }
        
        ///////////////////////////////////////////////////////////
        // Reproduce Population (but not after the last generation)
        ///////////////////////////////////////////////////////////
                
        if (i < NUM_GENERATIONS - 1)
            population_reproduce(population);

    }
    
    /////////////////////////////////////////
    /// End 
    /////////////////////////////////////////
    
    // Save best individual fitness values
    FILE * temp1 = fopen("dataBest.temp", "w");
    for (i=0; i < NUM_GENERATIONS; i++)
    {
        fprintf(temp1, "%d %lf \n", i, bestf[i]);
    }
    
    // Save average fitness values
    FILE * temp2 = fopen("dataAverage.temp", "w");
    for (i=0; i < NUM_GENERATIONS; i++)
    {
        fprintf(temp2, "%d %lf \n", i, avrgf[i]);
    }
    printf("GA optimization terminated.\n");
    fflush(stdout);
    
    // Send message to close controller
    double end = 1;
    wb_emitter_send(emitter, &end, sizeof(double));
    
    // Save best individual
    Genotype fittest = population_get_fittest(population);
    // File to save the best genotype
    static const char *FILE_NAME = "fittest.txt";
    FILE *outfile = fopen(FILE_NAME, "w");
    if (outfile) {
        genotype_fwrite(fittest, outfile);
        fclose(outfile);
        printf("wrote best genotype into %s\n", FILE_NAME);
        fflush(stdout);
    }
    else{
        printf("unable to write %s\n", FILE_NAME);
        fflush(stdout);
    }
    population_destroy(population);
}

double run_demo() {
    wb_keyboard_enable(time_step);
    
    // Read best fitness individual
    // File to save the best genotype
    static const char *FILE_NAME = "fittest.txt";
    FILE *infile = fopen(FILE_NAME, "r");
    if (! infile) {
        printf("unable to read %s\n", FILE_NAME);
        fflush(stdout);
        return 0;
    }    
    Genotype genotype = genotype_create();
    genotype_fread(genotype, infile);
    fclose(infile);

    return evaluate_genotype(genotype,1); 
}

double calculateMean(double data[], int size){
    double sum = 0.0;
    int i;
    for(i = 0; i<size;i++){
        sum += data[i];
        //printf("Data %lf \n", data[i]);
        //fflush(stdout);
    }
    //printf("Sum %lf \n", sum);
    //fflush(stdout);//
    return sum/(double)(size);
}

double calculateSD(double data[], double mean, int size){
    double sd = 0.0;
    int i;  
    for(i = 0; i < size; i++){
        double temp = data[i]-mean;
        sd += pow((double)(temp),(double)2);       
    }
    return sqrt(sd/(double)(size));
}

int main(int argc, const char *argv[]) {
    // initialize Webots
    wb_robot_init();

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
    //time_step = 1024;
    //time_step = wb_robot_get_basic_time_step();
    printf("Time_Step = %d \n",time_step);
    fflush(stdout);
        
    // to display the fitness evolution
    display = wb_robot_get_device("display");
    display_width = wb_display_get_width(display);
    display_height = wb_display_get_height(display);
    wb_display_draw_text(display, "MLP fitness 03", 2, 2);
    
    // initial population
    population = population_create(POPULATION_SIZE, GENOTYPE_SIZE);
    
    // find robot node and store initial position and orientation
    WbNodeRef robot = wb_supervisor_node_get_from_def("EPUCK_3_MLP");
    robot_translation = wb_supervisor_node_get_field(robot, "translation");
    robot_rotation = wb_supervisor_node_get_field(robot, "rotation");
    
    memcpy(robot_trans0, wb_supervisor_field_get_sf_vec3f(robot_translation), sizeof(robot_trans0));
    memcpy(robot_rot0, wb_supervisor_field_get_sf_rotation(robot_rotation), sizeof(robot_rot0));
    
    // the emitter to send genotype to robot
    emitter = wb_robot_get_device("emitter_3");
    // find and enable receiver
    receiver = wb_robot_get_device("receiver_3");
    wb_receiver_enable(receiver, time_step);

    // Optimization or demosntration
    printf("Type 0 to evolve the network or 1 to run a demonstration. \n");
    fflush(stdout);
    wb_keyboard_enable(100);
    int new_key = 0;
    while(wb_robot_step(100) != -1){
        new_key = wb_keyboard_get_key();
        if(new_key == 48|| new_key == 49) break;
    }
    if (new_key == 48){  
        printf("Start Optimization \n");
        fflush(stdout);    
    
        #define BILLION  1000000000L;
        struct timespec requestStart, requestEnd;
        clock_gettime(CLOCK_REALTIME, &requestStart);
    
        // run GA optimization
        run_optimization();
        
        clock_gettime(CLOCK_REALTIME, &requestEnd);
        // Calculate time it took
        double accum = ( requestEnd.tv_sec - requestStart.tv_sec )
          + ( requestEnd.tv_nsec - requestStart.tv_nsec )/ BILLION;
        printf("Time of execution: %lf sec \n", accum);
        fflush(stdout); 
        
    }else if(new_key == 49){
    
        printf("Start Demo \n");    
        fflush(stdout);  
        double data[100];
        int count;
        count = 0;
        while(wb_robot_step(time_step) != -1){
            data[count] = run_demo();
            //printf("Result %d: %lf \n", count, data[count]);    
            //fflush(stdout);
            //printf("Result %d Accurace %f \n", count, data[count]);    
            //fflush(stdout);
            if(count == 99){ 
                double result[2];
                int size = count + 1;
                result[0] = calculateMean(data,size);
                result[1] = calculateSD(data,result[0],size);
                printf("Mean %f sd %f \n", result[0], result[1]);    
                fflush(stdout);
                count = 0;
                
                // Save on file
                FILE *results1;
                results1=fopen("result.txt","a");
                if(results1==NULL){
                    printf("Error!");
                    fflush(stdout);
                    exit(1);
                }
                fprintf(results1,"\r\n 05 >> SNN RS >> Iterations 50 >> Time 20s >> Population 30 >> Elite 20 >> Mutation 15 >> Deviation 30 \r\n Mean %lf SD %lf \r\n", result[0], result[1]);
                fclose(results1);
            }else count++;
        }
    }
    //*/
    // cleanup Webots 
    wb_robot_cleanup();
    return 0;  // ignored
}
