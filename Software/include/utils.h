#ifndef UTILS_H
#define UTILS_H

/* Define Point structure */
typedef struct {
    float x;
    float y;
} Point;


float distBetweenPoints(Point& p1, Point& p2);
void getCellFromPos(Point& rob_pos, Point& current_cell);
void findIntersection(Point starting_point, 
                      Point end_point, 
                      Point wall_point1, 
                      Point wall_point2,
                      float* distance_to_intersection, 
                      bool* intersection_found, 
                      Point* intersection_point);

//--------------------------------------------------------------------------------

void init_labyrinth(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
void init_corners(CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) ;



#endif
