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

#include <stdlib.h>

#include <webots/robot.h>

#include <webots/motor.h>

#include <webots/distance_sensor.h>

#include <webots/led.h>

#include <webots/supervisor.h>


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

int main(int argc, char **argv) {



  double LeituraSensorProx[QtddSensoresProx];

  double AceleradorDireito=1.0, AceleradorEsquerdo=1.0;


  wb_robot_init();
  init_devices();


  WbDeviceTag MotorEsquerdo, MotorDireito;

  

  MotorEsquerdo = wb_robot_get_device("left wheel motor");

  MotorDireito = wb_robot_get_device("right wheel motor");

  

  wb_motor_set_position(MotorEsquerdo, INFINITY);

  wb_motor_set_position(MotorDireito, INFINITY);

  

  wb_motor_set_velocity(MotorEsquerdo,0);

  wb_motor_set_velocity(MotorDireito,0);

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
    
    
    WbNodeRef c1 = wb_supervisor_node_get_from_def("c1");
    WbNodeRef c2 = wb_supervisor_node_get_from_def("c2");
    WbNodeRef c3 = wb_supervisor_node_get_from_def("c3");
    WbNodeRef c4 = wb_supervisor_node_get_from_def("c4");
    WbNodeRef c5 = wb_supervisor_node_get_from_def("c5");
    WbNodeRef c6 = wb_supervisor_node_get_from_def("c6");
    WbNodeRef c7 = wb_supervisor_node_get_from_def("c7");
    WbNodeRef c8 = wb_supervisor_node_get_from_def("c8");
    WbNodeRef c9 = wb_supervisor_node_get_from_def("c9");
    WbNodeRef c10 = wb_supervisor_node_get_from_def("c10");
    
    
    const double *c1p = wb_supervisor_node_get_position(c1);
    const double *c2p = wb_supervisor_node_get_position(c2);
    const double *c3p = wb_supervisor_node_get_position(c3);
    const double *c4p = wb_supervisor_node_get_position(c4);
    const double *c5p = wb_supervisor_node_get_position(c5);
    const double *c6p = wb_supervisor_node_get_position(c6);
    const double *c7p = wb_supervisor_node_get_position(c7);
    const double *c8p = wb_supervisor_node_get_position(c8);
    const double *c9p = wb_supervisor_node_get_position(c9);
    const double *c10p = wb_supervisor_node_get_position(c10);

     bool mudou = false;
     
  while (wb_robot_step(TIME_STEP) != -1) {

    wb_led_set(Leds[0], wb_led_get(Leds[0])*-1);

    
    while(!mudou){

      const double *c1n = wb_supervisor_node_get_position(c1);
      const double *c2n = wb_supervisor_node_get_position(c2);
      const double *c3n = wb_supervisor_node_get_position(c3);
      const double *c4n = wb_supervisor_node_get_position(c4);
      const double *c5n = wb_supervisor_node_get_position(c5);
      const double *c6n = wb_supervisor_node_get_position(c6);
      const double *c7n = wb_supervisor_node_get_position(c7);
      const double *c8n = wb_supervisor_node_get_position(c8);
      const double *c9n = wb_supervisor_node_get_position(c9);
      const double *c10n = wb_supervisor_node_get_position(c10);


      wb_motor_set_velocity(MotorEsquerdo,6.28*AceleradorEsquerdo);
      wb_motor_set_velocity(MotorDireito, 6.28*AceleradorDireito);
      
      int time2 = (rand() % (50)) + 1;
      
      wb_robot_step(time2*200);
      
      wb_motor_set_velocity(MotorEsquerdo, -6.28*AceleradorEsquerdo);
      wb_motor_set_velocity(MotorDireito, 6.28*AceleradorDireito);
      
      
      int time = (rand() % (10));
      
      wb_robot_step(time*100);
        
   if(c1p[0] != c1n[0] || c1p[1] != c1n[1] || c1p[2] != c1n[2]){
     mudou = true;
   }else if(c2p[0] != c2n[0] || c2p[1] != c2n[1] || c2p[2] != c2n[2]){
      mudou = true;
   }else if(c3p[0] != c3n[0] || c3p[1] != c3n[1] || c3p[2] != c3n[2]){
      mudou = true;
   }else if(c4p[0] != c4n[0] || c4p[1] != c4n[1] || c4p[2] != c4n[2]){
      mudou = true;
   }else if(c5p[0] != c5n[0] || c5p[1] != c5n[1] || c5p[2] != c5n[2]){
      mudou = true;
   }else if(c6p[0] != c6n[0] || c6p[1] != c6n[1] || c6p[2] != c6n[2]){
      mudou = true;
   }else if(c7p[0] != c7n[0] || c7p[1] != c7n[1] || c7p[2] != c7n[2]){
      mudou = true;
   }else if(c8p[0] != c8n[0] || c8p[1] != c8n[1] || c8p[2] != c8n[2]){
      mudou = true;
   }else if(c9p[0] != c9n[0] || c9p[1] != c9n[1] || c9p[2] != c9n[2]){
      mudou = true;
   }else if(c10p[0] != c10n[0] || c10p[1] != c10n[1] || c10p[2] != c10n[2]){
      mudou = true;
   }
    
    printf("%lf %lf %lf - %lf %lf %lf\n  ", c1p[0],c1p[1],c1p[2],c1n[0],c1n[1],c1n[2]);
    printf("estado : %d\n", mudou);
    
    }

    blink_leds();
  
    // printf("acabou!!!!!!!!!!!! pedro pode ir ewmbvor");
    
      wb_motor_set_velocity(MotorEsquerdo, 0*AceleradorEsquerdo);
      wb_motor_set_velocity(MotorDireito, 0*AceleradorDireito);  
    
  };
  
  
   // printf("saiiiiiiiiiiiiiiiiiiiiiiiii");
   

  wb_robot_cleanup();



  return 0;

}