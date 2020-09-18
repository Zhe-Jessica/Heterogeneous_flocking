#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <webots/supervisor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#define TIME_STEP 64
#define ROBOTS 10

double v = 0;
double w = 0;
double PI = 3.1415926;
double l = 0.05;
double L = 0.271756;
double R = 0.031;
double target[1][2] = { {-0.5,-1} };
int i;


int main(int argc, char** argv) {
    wb_robot_init();
    long int time_print = 0;
    long int time_number = 0;

    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);


    WbNodeRef robot_node = wb_supervisor_node_get_self();
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    WbFieldRef trans_angle = wb_supervisor_node_get_field(robot_node, "rotation");

    const char* list[10] = {"A1", "A2","A3","A4", "A5", "A6", "A7", "A8" ,"A9", "A10"};
    WbNodeRef node;
    WbFieldRef robot_translation_field[ROBOTS], robot_rotation_field[ROBOTS];
    const double* robot_translation[ROBOTS], * robot_rotation[ROBOTS];

    for (i = 0; i < ROBOTS; i++) {
        node = wb_supervisor_node_get_from_def(list[i]);
        robot_translation_field[i] = wb_supervisor_node_get_field(node, "translation");
        robot_rotation_field[i] = wb_supervisor_node_get_field(node, "rotation");
    }

    const char* name = wb_robot_get_name();
    int number = atoi(name);
    //FILE* fp = fopen("C:\\Users\\duric\\Desktop\\tests\\10.csv", "a");
    //FILE* fp = fopen("C:\Users\duric\Desktop\tests\10.csv", "w");


    //while (time_print<20673) {
    while (wb_robot_step(TIME_STEP) != -1) {

        time_print = TIME_STEP * time_number;

        for (i = 0; i < ROBOTS; i++) {
            robot_translation[i] = wb_supervisor_field_get_sf_vec3f(robot_translation_field[i]);
            robot_rotation[i] = wb_supervisor_field_get_sf_rotation(robot_rotation_field[i]);
            if (time_print < 20673) {
                printf("%d           %d             %g         %g           %g\n",time_print, i+1, robot_translation[i][0], robot_translation[i][2], robot_rotation[i][3]);
            }
            //fprintf(fp, "Time:%d  Robot:%d  %g  %g  %g\n", time_print, i + 1, robot_translation[i][0], robot_translation[i][2], robot_rotation[i][3]);

        }


        const double* trans = wb_supervisor_field_get_sf_vec3f(trans_field);
        //printf("MY_ROBOT is at position: %g %g \n", trans[2], trans[0]);

        const double* orient = wb_supervisor_field_get_sf_rotation(trans_angle);
        //printf("orientation is: %g \n", orient[3]);

        double real_orient = orient[3] - PI;
        double head_positionx = trans[2] + l * cos(real_orient);
        double head_positiony = trans[0] + l * sin(real_orient);
        double error1 = head_positionx - target[0][0];
        double error2 = head_positiony - target[0][1];
        double C = 15;
        double u1 = -C * error1;
        double u2 = -C * error2;

        if (cos(real_orient) * u1 + sin(real_orient) * u2 > 0.05) {
            v = 0.05;
        }
        else if (cos(real_orient) * u1 + sin(real_orient) * u2 < -0.05) {
            v = -0.05;
        }
        else {
            v = cos(real_orient) * u1 + sin(real_orient) * u2;
        }

        if (-1 / l * sin(real_orient) * u1 + 1 / l * cos(real_orient) * u2 > 0.2) {
            w = 0.2;
        }
        else if (-1 / l * sin(real_orient) * u1 + 1 / l * cos(real_orient) * u2 < -0.2) {
            w = -0.2;
        }
        else {
            w = -1 / l * sin(real_orient) * u1 + 1 / l * cos(real_orient) * u2;
        }


        // initialize motor speeds at 50% of MAX_SPEED.
        double left_speed = 1 / R * v + L / (2 * R) * w;
        double right_speed = 1 / R * v - L / (2 * R) * w;



        // write actuators inputs
        wb_motor_set_velocity(left_motor, right_speed);
        wb_motor_set_velocity(right_motor, left_speed);

        time_number += 1;

    };

    /* Enter your cleanup code here */

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();
    //fclose(fp);

    return 0;
}
