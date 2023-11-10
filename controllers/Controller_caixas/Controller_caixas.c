/*

 * File:          Controlador.c

 * Date:

 * Description:

 * Author:

 * Modifications:

 */



/*

 * You may need to add include files like <webots/distance_sensor.h> or

 * <webots/motor.h>, etc.

 */

#include <stdio.h>

#include <webots/robot.h>

#include <webots/motor.h>

#include <webots/distance_sensor.h>

#include <webots/led.h>



/*

 * You may want to add macros here.

 */

#define TIME_STEP 256

#define QtddSensoresProx 8

#define QtddLeds 10

#define qtdLeds 5

static WbDeviceTag Leds[qtdLeds];
static bool leds_value[qtdLeds];
static const char *leds_name[qtdLeds] = {"led0", "led1","led2","led3","led4"};

static void blink_leds(){
  static int cont = 0;
  cont++;
  leds_value[(cont/10)%qtdLeds] = true;
}

static void init_devices() {
int i;
for (i = 0; i < qtdLeds; i++)
Leds[i] = wb_robot_get_device(leds_name[i]);
}

/*

 * This is the main program.

 * The arguments of the main function can be specified by the

 * "controllerArgs" field of the Robot node

 */

int main(int argc, char **argv) {

  

  int i=0;

  char texto[256];

  double LeituraSensorProx[QtddSensoresProx];

  double AceleradorDireito=1.0, AceleradorEsquerdo=1.0;

  /* necessary to initialize webots stuff */

  

  for(i=0;i<257;i++) texto[i]='0';

  

  wb_robot_init();
  init_devices();

  

  //configurei MOTORES

  WbDeviceTag MotorEsquerdo, MotorDireito;

  

  MotorEsquerdo = wb_robot_get_device("left wheel motor");

  MotorDireito = wb_robot_get_device("right wheel motor");

  

  wb_motor_set_position(MotorEsquerdo, INFINITY);

  wb_motor_set_position(MotorDireito, INFINITY);

  

  wb_motor_set_velocity(MotorEsquerdo,0);

  wb_motor_set_velocity(MotorDireito,0);

  

  

   //configurei Sensores de Proximidade

   WbDeviceTag SensorProx[QtddSensoresProx];

   

   SensorProx[0] = wb_robot_get_device("ps0");

   SensorProx[1] = wb_robot_get_device("ps1");

   SensorProx[2] = wb_robot_get_device("ps2");

   SensorProx[3] = wb_robot_get_device("ps3");

   SensorProx[4] = wb_robot_get_device("ps4");

   SensorProx[5] = wb_robot_get_device("ps5");

   SensorProx[6] = wb_robot_get_device("ps6");

   SensorProx[7] = wb_robot_get_device("ps7");

   

   wb_distance_sensor_enable(SensorProx[0],TIME_STEP);

   wb_distance_sensor_enable(SensorProx[1],TIME_STEP);

   wb_distance_sensor_enable(SensorProx[2],TIME_STEP);

   wb_distance_sensor_enable(SensorProx[3],TIME_STEP);

   wb_distance_sensor_enable(SensorProx[4],TIME_STEP);

   wb_distance_sensor_enable(SensorProx[5],TIME_STEP);

   wb_distance_sensor_enable(SensorProx[6],TIME_STEP);

   wb_distance_sensor_enable(SensorProx[7],TIME_STEP);



    //config leds

    WbDeviceTag Leds[QtddLeds];

    Leds[0] = wb_robot_get_device("led0");

    wb_led_set(Leds[0],-1);
    
    Leds[1] = wb_robot_get_device("led1");

    wb_led_set(Leds[1],-1);
    
    Leds[2] = wb_robot_get_device("led2");

    wb_led_set(Leds[2],-1);
    
    Leds[3] = wb_robot_get_device("led3");

    wb_led_set(Leds[3],-1);
    
    Leds[4] = wb_robot_get_device("led4");

    wb_led_set(Leds[4],-1);

  

  

  /*

   * You should declare here WbDeviceTag variables for storing

   * robot devices like this:

   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");

   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");

   */



  /* main loop

   * Perform simulation steps of TIME_STEP milliseconds

   * and leave the loop when the simulation is over

   */
  
  LeituraSensorProx[0]= wb_distance_sensor_get_value(SensorProx[0])-60;
  // sprintf(texto,"%s|%d: %5.2f  ",texto,0,LeituraSensorProx[0]);

  while (wb_robot_step(TIME_STEP) != -1) {

    for(i=0;i<256;i++) texto[i]=0;

    //memcpy(texto,0,10);

    /*

     * Read the sensors :

     * Enter here functions to read sensor data, like:

     *  double val = wb_distance_sensor_get_value(my_sensor);

     */



    /* Process sensor data here */

    // for(i=0;i<QtddSensoresProx;i++){

       LeituraSensorProx[0]= wb_distance_sensor_get_value(SensorProx[0])-60;

       sprintf(texto,"%s|%d: %5.2f  ",texto,0,LeituraSensorProx[0]);

    // }

    // printf("%s\n",texto);

    wb_led_set(Leds[0], wb_led_get(Leds[0])*-1);

    /*

     * Enter here functions to send actuator commands, like:

     * wb_motor_set_position(my_actuator, 10.0);

     */

    

    // if(LeituraSensorProx[0]>100){

     // AceleradorDireito = -1;

      // AceleradorEsquerdo = 1;}

    // else {

      // AceleradorDireito = 1;

      // AceleradorEsquerdo = 1;}
    
    
    wb_motor_set_velocity(MotorEsquerdo,6.28);
    wb_motor_set_velocity(MotorDireito, 6.28);
    
    printf("%s\n",texto);
    
    wb_robot_step(2000);
    
    printf("%s\n",texto);
    
    wb_led_set(Leds[1], wb_led_get(Leds[1])*-1);
      wb_led_set(Leds[2], wb_led_get(Leds[2])*-1);
      wb_led_set(Leds[3], wb_led_get(Leds[3])*-1);
      wb_led_set(Leds[4], wb_led_get(Leds[4])*-1);
    
    if(SensorProx[0] > 300 && SensorProx[0] < 5000){
      wb_led_set(Leds[1], wb_led_get(Leds[0])*-1);
      wb_led_set(Leds[2], wb_led_get(Leds[0])*-1);
      wb_led_set(Leds[3], wb_led_get(Leds[0])*-1);
      wb_led_set(Leds[4], wb_led_get(Leds[0])*-1);
    }
     
    // wb_motor_set_velocity(MotorEsquerdo,-6.28*AceleradorEsquerdo);
    // wb_motor_set_velocity(MotorDireito, -6.28*AceleradorDireito);
    
    // wb_robot_step(1000);
    
    // wb_motor_set_velocity(MotorEsquerdo, 1*AceleradorEsquerdo);
    // wb_motor_set_velocity(MotorDireito, 6.28*AceleradorDireito);
    
    // wb_robot_step(2400);


  };



  /* Enter your cleanup code here */



  /* This is necessary to cleanup webots resources */

  wb_robot_cleanup();



  return 0;

}