#ifndef Maze_h
#define Maze_h

#define SIZE 16			// Size of one dimention of Map
#define TRUE 1
#define FALSE 0
// Directions
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// Stack Constants
#define SPI 1			// Stack Pointer Index
#define SSI 0 			// Stack Size Index
#define STACK_OFFSET 2
#define STACKSIZE 80

#define START_X 15
#define START_Y 0
#define LARGEVAL 301

// Shortcut Constants
#define MAPIJ this_maze->map[i][j]
#define MAP this_maze->map
#define M_DISTANCE this_cell->m_distance
#define ROW this_cell->pos_x
#define COL this_cell->pos_y
#define VISITED this_cell->visited
#define LEFT this_cell->left
#define RIGHT this_cell->right
#define UP this_cell->up
#define DOWN this_cell->down


typedef struct Cell{ 
    /* data fields */
    short m_distance;
    short pos_x;
    short pos_y;
    short visited;
  
    /* pointers to neighbors */
    struct Cell * left;
    struct Cell * right;
    struct Cell * up;
    struct Cell * down;
  } 
  Cell;
  
  typedef struct Maze {
    Cell * map [SIZE][SIZE];	
  } 
  Maze;

  typedef struct Stack {
    short properties [STACK_OFFSET];
    Cell * the_stack [STACKSIZE];
  } 
  Stack;
  
  // Node Functions
Cell * new_Cell (const short i, const short j);
void delete_Cell (Cell ** npp);
void flood_fill (Cell * this_cell, Stack * this_stack, const short reflood_flag);
void set_wall (Cell * this_cell, const short dir);
void set_value (Cell * this_cell, const short value);
void set_visited (Cell * this_cell);
short get_smallest_neighbor_dir (Cell * this_cell, const short preferred_dir);

// Floodfill Helper Functions
short get_smallest_neighbor (Cell * this_cell);
short floodval_check(Cell * this_cell) ;
void update_floodval (Cell * this_cell);
void push_open_neighbors (Cell * this_cell, Stack * this_stack);


// Maze Functions
Maze * new_Maze ();
void delete_Maze (Maze ** mpp);
void print_map (const Maze * this_maze);

#endif
