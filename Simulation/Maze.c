#include <stdlib.h>
#include <stdio.h>
#include "Maze.h"

/* Node Constructor */
Cell * new_Cell (const short i, const short j) {

    Cell * this_cell;
    short halfsize;
  
    this_cell = (Cell *) malloc(sizeof(Cell));
    halfsize = SIZE / 2;
  
    ROW = i;
    COL = j;
    VISITED = FALSE;
  
    /* Initializing the flood value at this coord
            NOTE : Right now this only works when SIZE is even -- which is ok */
    if (i < halfsize && j < halfsize)
      M_DISTANCE = (halfsize - 1 - i) + (halfsize - 1 - j) ;
  
    else if (i < halfsize && j >= halfsize)
    M_DISTANCE = (halfsize - 1 - i) + (j - halfsize) ;
  
    else if (i >= halfsize && j < halfsize)
    M_DISTANCE = (i - halfsize) + (halfsize - 1 - j) ;
  
    else
    M_DISTANCE = (i - halfsize) + (j - halfsize) ;
  
    return this_cell;
  }
  
  
  /* Maze Constructor */
  Maze * new_Maze () {
  
    Maze * this_maze;
    short i, j;
  
    this_maze = (Maze *) malloc(sizeof(Maze));
  
    /* Allocate a new Node for each coord of maze */
    for (i = 0; i < SIZE; ++i) 
      for (j = 0; j < SIZE; ++j) 
        MAPIJ = new_Cell (i, j);
  
    /* setting the neighbors ptrs... must be done after all cells allocated */
    for (i = 0; i < SIZE; i++)
      for (j = 0; j < SIZE; j++) {
        MAPIJ->left = (j == 0) ? NULL : (this_maze->map[i][j-1]);
        MAPIJ->right = (j == SIZE-1) ? NULL : (this_maze->map[i][j+1]);
        MAPIJ->up = (i == 0) ? NULL : (this_maze->map[i-1][j]);
        MAPIJ->down = (i == SIZE-1) ? NULL : (this_maze->map[i+1][j]);
      }
  
    return this_maze;
  }

  void delete_Cell (Cell ** npp) {  
    free (*npp);
    *npp = 0;
  }

  void delete_Maze (Maze ** mpp) {
    short i, j;
    for (i = 0; i < SIZE; i++) 
      for (j = 0; j < SIZE; j++) 
        delete_Cell (&((*mpp)->map[i][j])); 
      
    free(*mpp);
    *mpp = 0;
  }

  short get_smallest_neighbor (Cell * this_cell) {

    /* debug statements */
    // if (debug_on)
    //   printf("In get_smallest_neighbor\n");
  
    // The Node's floodval will be 1 higher than the neigboring cell
    short smallestneighbor = LARGEVAL;
  
    // NOTE: LEFT, RIGHT, etc, are substituting:
    // this_node->left, this_node->right, etc.
  
    if (LEFT != NULL && (LEFT->right != NULL) && (LEFT->m_distance) < smallestneighbor)
      smallestneighbor = LEFT->m_distance;
  
    if (RIGHT != NULL && (RIGHT->left != NULL) && (RIGHT->m_distance) < smallestneighbor)
      smallestneighbor = RIGHT->m_distance;	
  
    if (UP != NULL && (UP->down != NULL) && (UP->m_distance) < smallestneighbor)
      smallestneighbor = UP->m_distance;
  
    if (DOWN != NULL && (DOWN->up != NULL) && (DOWN->m_distance) < smallestneighbor)
      smallestneighbor = DOWN->m_distance;
  
    return smallestneighbor;
  }

  short get_smallest_neighbor_dir (Cell * this_cell, const short preferred_dir) {

    short smallestval; 		/* smallest neighbor value */
    short pathcount; 			/* number of available paths */
  
  
    /* get the smallest neighboring flood_val */
    smallestval = get_smallest_neighbor(this_cell);
  
    /* clear pathcount */
    pathcount = 0;
  
    /* count the number of available paths */
    if ((UP != NULL) && (UP->m_distance == smallestval)) 
      pathcount++;
  
    if ((RIGHT != NULL) && (RIGHT->m_distance == smallestval)) 
      pathcount++;
  
    if ((DOWN != NULL) && (DOWN->m_distance == smallestval)) 
      pathcount++;
  
    if ((LEFT != NULL) && (LEFT->m_distance == smallestval)) 
      pathcount++;
  
    // switch (preferred_dir)
    // {
    // case NORTH: 
    //   if ((UP != NULL) && (UP->m_distance == smallestval))
    //     return NORTH;
    //   break;
    // case EAST: 
    //   if ((RIGHT != NULL) && (RIGHT->m_distance == smallestval))
    //     return EAST;
    //   break;
    // case SOUTH: 
    //   if ((DOWN != NULL) && (DOWN->m_distance == smallestval))
    //     return SOUTH;
    //   break;
    // case WEST:  
    //   if ((LEFT != NULL) && (LEFT->m_distance == smallestval))
    //     return WEST;
    //   break;
    // }
  
    /* if there is only one path, return that direction */
    //if (pathcount > 1)
    //	return preferred_dir;
  
    /* if there are multiple available paths, choose the favorable path 
    prefer cells that have not been vistited*/
  
    if ((UP != NULL) && (UP->m_distance == smallestval) && (UP->visited == FALSE))
      return NORTH;
    else if ((RIGHT != NULL) && (RIGHT->m_distance == smallestval) && (RIGHT->visited == FALSE))
      return EAST;
    else if ((DOWN != NULL) && (DOWN->m_distance == smallestval) && (DOWN->visited == FALSE))
      return SOUTH;
    else if ((LEFT != NULL) && (LEFT->m_distance == smallestval) && (LEFT->visited == FALSE))
      return WEST;
  
    if ((UP != NULL) && (UP->m_distance == smallestval))
      return NORTH;
    else if ((RIGHT != NULL) && (RIGHT->m_distance == smallestval))
      return EAST;
    else if ((DOWN != NULL) && (DOWN->m_distance == smallestval))
      return SOUTH;
    else //if ((LEFT != NULL) && (LEFT->floodval) == smallestval)
    return WEST;
  
  }
  
  short floodval_check(Cell * this_cell) {
  
    /* return a flag determining wheter this node should be updated 
          aka, is this Node 1 + min open adj cell? */
    if (get_smallest_neighbor (this_cell) + 1 == this_cell->m_distance)
      return TRUE;

    return FALSE;
  }

  void update_floodval (Cell * this_cell) {
  
    /* set this node's value to 1 + min open adjascent cell */
    this_cell->m_distance = get_smallest_neighbor (this_cell) + 1;
  
  }
  
  /* pushes the open neighboring cells of this node to the stack */
  void push_open_neighbors (Cell * this_cell, Stack * this_stack) {
  
    /* A NULL neighbor represents a wall.
          if neighbor is accessible, push it onto stack! */
    if (LEFT != NULL && LEFT->right != NULL) 
      push(this_stack, LEFT);
  
    if (RIGHT != NULL && RIGHT->left != NULL) 
      push(this_stack, RIGHT);
  
    if (UP != NULL && UP->down != NULL) 
      push(this_stack, UP);
  
    if (DOWN != NULL && DOWN->up != NULL) 
      push(this_stack, DOWN);
  
  }
  
  /* main function for updating the flood values of this node */
  void flood_fill (Cell * this_cell, Stack * this_stack, const short reflood_flag) {
  
    short status;  /* Flag for updating the flood value or not */
  
    /* we want to avoid flooding the goal values - this is for non-reverse */
    if (!reflood_flag) 
      if (ROW == SIZE / 2 || ROW == SIZE / 2 - 1) 
        if (COL == SIZE / 2 || COL == SIZE / 2 - 1) 
          return;
  
    /* we want to avoid flooding the goal values - this is reverse */
    if (reflood_flag) 
      if (ROW == START_X && COL == START_Y)
        return;
  
    /* is the cell (1 + minumum OPEN adjascent cell) ? */
    status = floodval_check (this_cell);
  
    /* if no, update curent cell's flood values, 
          then push open adjascent neighbors to stack. */
    if (!status) {
      update_floodval(this_cell); /* Update floodval to 1 + min open neighbor */
      push_open_neighbors(this_cell, this_stack); /* pushed, to be called later */
    }
  
  }

  /* Function for setting this node's floodval to a specific value */
void set_value (Cell * this_cell, const short value) {
  /* set the flood value to specified value */
  M_DISTANCE = value;
}

/* Function for setting this node's floodval to a specific value */
void set_visited (Cell * this_cell) {
  /* set the flood value to specified value */
  VISITED = TRUE;
}

/* Function for setting the walls of this node */
void set_wall (Cell * this_cell, const short dir) {
  /* set a wall, of this node, of the specified direction  */
  switch (dir) 
  {
    case NORTH :
      if (ROW != 0) {
        UP = NULL;
      } 
      break;

    case SOUTH :
      if (ROW != SIZE -1) {
        DOWN = NULL;
      } 
      break; 

    case EAST : 
      if (COL != SIZE - 1) {
        RIGHT = NULL;
      } 
      break;

    case WEST :
      if (COL != 0) { 
        LEFT = NULL;
      } 
      break;
  }
}




