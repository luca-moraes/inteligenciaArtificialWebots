/*

 * File:          Controlador.c

 * Date:

 * Description:

 * Author: @luca-moraes

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

  for (int i = 0; i < qtdLeds; i++) {
      wb_led_set(Leds[i], 1);
   }
}

static void apaga_leds(){
  static int cont = 0;
  cont++;
  leds_value[(cont/10)%qtdLeds] = false;

  for (int i = 0; i < qtdLeds; i++) {
      wb_led_set(Leds[i], 0);
   }
}

static void init_devices() {
int i;
for (i = 0; i < qtdLeds; i++)
Leds[i] = wb_robot_get_device(leds_name[i]);
}

const double *getPosicao(WbNodeRef node) {
    double *posicao = (double *)malloc(3 * sizeof(double));

    const double *tempPos = wb_supervisor_node_get_position(node);

    posicao[0] = tempPos[0];
    posicao[1] = tempPos[1];
    posicao[2] = tempPos[2];

    return posicao;
}

bool comparePositions(const double *p1, const double *p2) {
   const double EPSILON = 0.001;

    return fabs(p1[0] - p2[0]) < EPSILON && 
           fabs(p1[1] - p2[1]) < EPSILON && 
           fabs(p1[2] - p2[2]) < EPSILON;
}

int main(int argc, char **argv) {

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

     const double *c1p = getPosicao(c1);
     const double *c2p = getPosicao(c2);
     const double *c3p = getPosicao(c3);
     const double *c4p = getPosicao(c4);
     const double *c5p = getPosicao(c5);
     const double *c6p = getPosicao(c6);
     const double *c7p = getPosicao(c7);
     const double *c8p = getPosicao(c8);
     const double *c9p = getPosicao(c9);
     const double *c10p = getPosicao(c10);
    
     bool mudou = false;
     
  while (wb_robot_step(TIME_STEP) != -1) {

    wb_led_set(Leds[0], wb_led_get(Leds[0])*-1);

    
    while(!mudou){
   
      wb_motor_set_velocity(MotorEsquerdo,6.28*AceleradorEsquerdo);
      wb_motor_set_velocity(MotorDireito, 6.28*AceleradorDireito);
      
      int time2 = (rand() % (50)) + 1;
      
      wb_robot_step(time2*200);
      
      wb_motor_set_velocity(MotorEsquerdo, -6.28*AceleradorEsquerdo);
      wb_motor_set_velocity(MotorDireito, 6.28*AceleradorDireito);
      
      
      int time = (rand() % (10));
      
      wb_robot_step(time*100);

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

      if (!comparePositions(c1p, c1n) || 
         !comparePositions(c2p, c2n) ||
         !comparePositions(c3p, c3n) ||
         !comparePositions(c4p, c4n) ||
         !comparePositions(c5p, c5n) ||
         !comparePositions(c6p, c6n) ||
         !comparePositions(c7p, c7n) ||
         !comparePositions(c8p, c8n) ||
         !comparePositions(c9p, c9n) ||
         !comparePositions(c10p, c10n)) 
      {
         mudou = true;
      }
    
    printf("%lf %lf %lf - %lf %lf %lf\n  ", c1p[0],c1p[1],c1p[2],c1n[0],c1n[1],c1n[2]);
    printf("estado : %d\n", mudou);
    
    }

    blink_leds();
    wb_robot_step(200);
    apaga_leds();
  
    // printf("acabou!!!!!!!!!!!! pedro pode ir ewmbvor");
    
      wb_motor_set_velocity(MotorEsquerdo, 0*AceleradorEsquerdo);
      wb_motor_set_velocity(MotorDireito, 0*AceleradorDireito);  
    
  };
  
  
   // printf("saiiiiiiiiiiiiiiiiiiiiiiiii");
   

  wb_robot_cleanup();



  return 0;

}