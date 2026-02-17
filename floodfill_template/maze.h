
typedef struct Node { 
    /* data fields */
    short m_distance;
    short pos_x;
    short pos_y;
    short visited;
  
    /* pointers to neighbors */
    struct Node * left;
    struct Node * right;
    struct Node * up;
    struct Node * down;
  
  } 
  Node;
  
  typedef struct Maze {
  
    Node * map [SIZE][SIZE];	
  } 
  Maze;