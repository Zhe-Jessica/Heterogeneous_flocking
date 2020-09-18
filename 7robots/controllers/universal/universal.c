#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>

#define random(x) (rand()%x)
//#include <ctime>

// time in [ms] of a simulation step
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define ROBOTS 4  // Read from+1
#define PI 3.14159265358979323846

int main(int argc, char** argv) {
	// initialize the Webots API
	wb_robot_init();
	double robot_initial_translation[5][3] = { {0.15, 0, -0.26},  {-0.3, 0, 0}, {0.3, 0, 0}, {-0.15, 0, 0.26}, {0.15, 0, 0.26} };
	double rotation[10];
	srand((int)time(0));
	for (int x = 0; x < 10; x++) {
		rotation[x] = random(100);
	}
	//printf("%d     %d\n", name_number, random(100));


	WbNodeRef node;
	WbFieldRef robot_translation_field[ROBOTS], robot_rotation_field[ROBOTS];
	const double* robot_translation[ROBOTS], * robot_rotation[ROBOTS];
	double neighbour_angle[ROBOTS - 1], neighbour_distance[ROBOTS - 1];

	int i, j;
	double l_ij = 0.3;
	double k = 1.5;


	//unique part. read its own name
	const char* name = wb_robot_get_name();
	int name_number = atoi(name);
	//srand((int)time(0));
	//for (int x = 0; x < 10; x++)
	//printf("%d     %d\n", name_number, random(100));

	if (name_number > 4) {
		name_number -= 1;
	}

	double position_set_angle = 6.28 * rotation[name_number] / 100 - 3.14;	//(-3.14,3.14)  change to pi
	const double INITIAL_ROT[4] = { 0, 1, 0, position_set_angle };
	//printf("%d    %g\n", name_number, position_set_angle);
	WbNodeRef robot_node = wb_supervisor_node_get_self();
	WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
	const double* trans = wb_supervisor_field_get_sf_vec3f(trans_field);
	double self_initial_translation[3] = { trans[0] + 0.2 * rotation[name_number+1] / 100 - 0.1, 0, trans[2] + 0.2 * rotation[name_number + 1] / 100 - 0.1 };

	//printf("%d\n", name_number);

	const char* list[6][4] = { { "A1", "A2","A3","A4" }, { "A2", "A1", "A4", "A5" },{"A3", "A1", "A4", "A6"},{"A5", "A2", "A4", "A7"},{"A6", "A3", "A4", "A7"},{"A7", "A4", "A5", "A6"} };
	const char* robot_name[4];
	for (i = 0; i < 4; i++) {
		robot_name[i] = list[name_number - 1][i];
		//robot_name[i] = list[0][i];

	}
	//printf("%d", name_number);



	// initialize devices
	WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
	WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	wb_motor_set_velocity(left_motor, 0.0);
	wb_motor_set_velocity(right_motor, 0.0);

	for (i = 0; i < ROBOTS; i++) {
		node = wb_supervisor_node_get_from_def(robot_name[i]);
		robot_translation_field[i] = wb_supervisor_node_get_field(node, "translation");
		robot_rotation_field[i] = wb_supervisor_node_get_field(node, "rotation");
	}
	//initialization
	wb_supervisor_field_set_sf_vec3f(robot_translation_field[0], self_initial_translation);
	wb_supervisor_field_set_sf_rotation(robot_rotation_field[name_number - 2], INITIAL_ROT);

	while (wb_robot_step(TIME_STEP) != -1) {
		//printf("%d", name_number);

		double total_x_distance = 0;
		double total_y_distance = 0;

		for (i = 0; i < ROBOTS; i++) {
			robot_translation[i] = wb_supervisor_field_get_sf_vec3f(robot_translation_field[i]);
			robot_rotation[i] = wb_supervisor_field_get_sf_rotation(robot_rotation_field[i]);
		}
		for (i = 0; i < ROBOTS - 1; i++) {
			double x_distance = robot_translation[0][0] - robot_translation[i + 1][0];
			double y_distance = robot_translation[0][2] - robot_translation[i + 1][2];
			neighbour_angle[i] = atan2(x_distance, y_distance);
			neighbour_distance[i] = sqrt(pow(x_distance, 2) + pow(y_distance, 2));
			total_x_distance -= sin(neighbour_angle[i]) * (neighbour_distance[i] - l_ij);	//calculate total x distance, right positive
			total_y_distance -= cos(neighbour_angle[i]) * (neighbour_distance[i] - l_ij);
		}
		double total_force = k * sqrt(pow(total_x_distance, 2) + pow(total_y_distance, 2)) / l_ij; //distance(x_distance, y_distance);
		double total_angle = atan2(-total_x_distance, -total_y_distance);
		//printf("%g", cos(total_angle - robot_rotation[0][3]));

		double alpha = 1.5;
		double beta = 100;
		double v_i = alpha * total_force * cos(total_angle - robot_rotation[0][3]);
		double w_i = beta * total_force * sin(total_angle - robot_rotation[0][3]);
		double left_speed = 48.78 * v_i - 3.46 * w_i;
		double right_speed = 48.78 * v_i + 3.46 * w_i;
		//printf("      %g  %g  %g\n", total_force, left_speed, right_speed);

		if (left_speed > MAX_SPEED) {
			left_speed = MAX_SPEED;
		}
		if (left_speed < -MAX_SPEED) {
			left_speed = -MAX_SPEED;
		}
		if (right_speed > MAX_SPEED) {
			right_speed = MAX_SPEED;
		}
		if (right_speed < -MAX_SPEED) {
			right_speed = -MAX_SPEED;
		}
		//write actuators inputs
		wb_motor_set_velocity(left_motor, left_speed);
		wb_motor_set_velocity(right_motor, right_speed);
	}
	wb_robot_step(32);
	wb_robot_cleanup();

	return 0;
}