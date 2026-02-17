#include <stdlib.h>
#include <stdio.h>
#include "maze.h"

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
        MAPIJ = new_Node (i, j);
  
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
	
    /* debug statements */
    // if (debug_on) 
    //   printf("deallocating %d, %d\n", (*npp)->row, (*npp)->column);
  
    free (*npp);
    *npp = 0;
  }

  void delete_Maze (Maze ** mpp) {

    short i, j;
  
    for (i = 0; i < SIZE; i++) 
      for (j = 0; j < SIZE; j++) 
        delete_Node (&((*mpp)->map[i][j])); 
      
    free(*mpp);
    *mpp = 0;
  }