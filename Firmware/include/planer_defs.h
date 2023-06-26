#define LABYRINTH_HEIGHT 6
#define LABYRINTH_WIDTH 6

#define CELL_SIZE 160 //in mm
#define WALL_WIDTH 12 //in mm

#define WIDTH LABYRINTH_HEIGHT*(CELL_SIZE+WALL_WIDTH) + WALL_WIDTH * 2
#define HEIGHT LABYRINTH_WIDTH*(CELL_SIZE+WALL_WIDTH) + WALL_WIDTH * 2

#define DISTANCE_WHEELS 50 //distance of one wheel from the center in mm
#define MOUSE_WIDTH 90 //in mm
#define MOUSE_HEIGHT 110

#define SENSOR_RANGE 130 //in mm

#define M_PI 3.14159265358979323846


#define WALL_RETAIN 0.99
#define POS_RETAIN 0.999
#define SENS_VAR 5.0
#define ENCODER_VAR 0.05
#define THRESHOLD_WALL_SEEN 0.6

#define MIN_CORNER_DIST 20
#define TOLERANCE_DIST 15
#define IGNORE_MAX 20

#define L_ERROR_BAND M_PI/6;
#define L_SMOOTHING 0.03; //smoothing for localization


#define KP 2
#define KI 1
#define KD 1
#define PID_MAX 1
#define PID_MIN -1
#define THRESHOLD_NO_WALL 0.2
