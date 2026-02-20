
#ifndef STACK_H
#define STACK_H

#include "Maze.h"  

Stack* new_Stack(void);
void delete_Stack(Stack** spp);

int is_empty_Stack(Stack* this_stack);
void pop(Stack* this_stack, Cell** npp);
void push(Stack* this_stack, Cell* this_node);

#endif
