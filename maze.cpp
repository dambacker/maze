// code by Dam Backer
// example maze implementation
// 1) Discover the walls of the maze (depth first search)
//      - Assume the robot has to physically drive around to discover the walls.
//      - Explore a cell, make note of the walls and mark the cell as visited. If the cell has unexplored directions (no walls, not previously visited)
//        explore those directions. At a dead end, back track until you reach a fork that has unexplored paths. 
//        Keep doing until you return at the origin having explored all cells that can be reached in the maze.
//      - Solve 1) using a stack to track the path, unexplored cells and the ability to backtrack.
//      - Solve 2) using recursive call for each unexplored direction at a cell
//      - Note: Both approaches are the same algorithm (depth first search). In compute science, recursion is preferred because it is cool. In the
//              real world the stack based approach is preferred because it is easier to debug.
// 2) Solve the fastest path through the maze (breadth first search)
//      - Shortest Path Algorithm: Dijkstra's Algorithm, using a weighted graph representation of the maze.
//      - Solve 1) Simple assuming the distance (weight) from each cell to the next is 1
//      - Solve 2) Include the speed that the robot can gain in a straight line by adding edges to the graph representing
//                 the (improved) time to travel straight sections across multiple cells.
//      - Note: For this algorithm we use the information from 1) without the need for the robot to travel physically again.

#include <string>

//our maze dimensions
#define DIMX    10
#define DIMY    10

#define MAX_DISTANCE (DIMX*DIMY)

//maze example (assumed to match the dimensions)
typedef char* TextMaze[DIMY*2+1];  // X/Y flipped
TextMaze textMaze = {
    "+-+-+-+-+-+-+-+-+-+-+",
    "|                   |",
    "+ +-+-+-+-+-+-+-+-+ +",
    "| | | | | | | | |   |",
    "+ +-+-+-+-+-+-+-+ + +",
    "| | | | | | | |   | |",
    "+ +-+-+-+-+-+-+ +-+ +",
    "| | | | | | |   | | |",
    "+ +-+-+-+-+-+ +-+-+ +",
    "| | | | |           |",
    "+ +-+-+-+ o +-+-+-+-+",
    "| | | | |         | |",
    "+ +-+-+-+-+-+-+-+ +-+",
    "| | |         | | | |",
    "+ +-+ +-+-+-+ +-+ +-+",
    "| | | | | | |     | |",
    "+ +-+ +-+-+-+-+-+-+-+",
    "|     | | | | | | | |",
    "+ +-+ +-+-+-+-+-+-+-+",
    "| | | | | | | | | | |",
    "+-+-+-+-+-+-+-+-+-+-+",
};

enum 
{
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};

//-------------------------------------
// Maze class implementation, grid of cells with info per cell
//-------------------------------------

//The cell class contains useful information about each cell during wall discovery and fastest path solver
class Cell
{
public:
    int         x;          //location
    int         y;
    bool        discovered; //has cell been discovered
    Cell*       north;
    Cell*       east;
    Cell*       south;
    Cell*       west;
    float       distance;   //fastest path to origin 
};

class Maze
{
public:
    Maze::Maze();
    Maze::~Maze() {};
public:
    void        Discover(Cell* cell);
    void        Solve();
    void        Print();
public:
    Cell        cells[DIMX][DIMY];
};

//-------------------------------------
// basic push/pop queue (first-in-first-out = FIFO)
//-------------------------------------

class QueueElement
{
public:
    Cell* cell;
    QueueElement* next;
};

class Queue
{
public:
    Queue::Queue()  { top = NULL; bottom = NULL; }
    Queue::~Queue() { _ASSERT((top == NULL) && (bottom == NULL)); }; //check that queue was empty when we are done with it

public:
    void Push(Cell* cell) 
    {
        QueueElement* elem = new(QueueElement);
        elem->cell = cell;
        elem->next = NULL;
        if (top != NULL)
            top->next = elem;
        if (bottom == NULL)
            bottom = elem;
        top = elem;
    };
    Cell* Pop()
    {
        _ASSERT(bottom != NULL);
        Cell* cell = bottom->cell;
        QueueElement* elem = bottom;
        bottom = bottom->next;
        if (bottom == NULL)
            top = NULL;
        delete(elem);
        return cell;
    };
    bool IsEmpty() { return ((top == NULL) && (bottom == NULL)); };

private:
    QueueElement* top;
    QueueElement* bottom;
};

//=====================================
// helper functions
//=====================================

//-----------------------------------------------------------------------------
bool Wall(int x, int y, int direction)
//-----------------------------------------------------------------------------
{
    switch (direction)
    {
        case NORTH: return textMaze[DIMY*2 - 1 - y*2 - 1][x*2+1    ] != ' ';
        case EAST:  return textMaze[DIMY*2 - 1 - y*2    ][x*2+1 + 1] != ' ';
        case SOUTH: return textMaze[DIMY*2 - 1 - y*2 + 1][x*2+1    ] != ' ';
        case WEST:  return textMaze[DIMY*2 - 1 - y*2    ][x*2+1 - 1] != ' ';
    }

    return true;
}

//-----------------------------------------------------------------------------
float Distance(Cell* cell1, Cell* cell2)
//-----------------------------------------------------------------------------
{
    //Calculate the time it takes to travel from cell1 to cell2 in a straight line (could be over more than a cell)
    //Accelerate up to max speed or half-distance, whichever comes first.
    //If not at half-distance (hence at max speed), keep going at constant (max) speed till final distance needed to decelerate,
    //decelerate until end point

    /* physics 101
        a = acceleration
        v = speed
        s = distance
        t = time

        v(t) = a*t              //speed v at time t
        t(v) = v/a              //time t to reach speed v
        s(t) = a/2*t^2          //distance s at time t
        t(s) = sqrt(s/(a/2))    //time t to distance s
    */

    //acceleration and maximum speed need to be based on the capabilities of the actual robot
    float acceleration          = 2.0f; //a = acceleration  Example: 2 cells/sec^2
    float maxSpeed              = 4.0f; //m = max speed     Example: 2 cells/sec

    //euclidean distance between the two cells
    float distance              = sqrt(pow(cell1->x - cell2->x, 2.0) + pow(cell1->y - cell2->y, 2.0));

    float halfDistance          = distance / 2.0f;
    float timeToHalfDistance    = sqrt(halfDistance/(acceleration/2));          //based on t(s) = sqrt(s/(a/2))
    float timeToMaxSpeed        = maxSpeed / acceleration;                      //based on t(v) = v/a
    float distanceToMaxSpeed    = acceleration/2.0*pow(timeToMaxSpeed, 2.0);    //based on s(t) = a/2*t^2
    float constantSpeedDistance = distance - distanceToMaxSpeed * 2.0f;
    float timeAtConstantSpeed   = constantSpeedDistance / maxSpeed;

    if (halfDistance <= distanceToMaxSpeed)
    {
        return timeToHalfDistance + timeToHalfDistance; //accelerate till half distance + decelerate second half
    }

    //accelerate, travel at constant speed, decelerate
    return timeToMaxSpeed + timeAtConstantSpeed + timeToMaxSpeed;
}

//=====================================
// Maze implementation
//=====================================

//-----------------------------------------------------------------------------
Maze::Maze() 
//-----------------------------------------------------------------------------
{
    //initialize the cell info
    for (int x=0; x<DIMX; x++)
    {
        for (int y=0; y<DIMY; y++)
        {
            cells[x][y].x               = x;
            cells[x][y].y               = y;
            cells[x][y].discovered      = false;
            cells[x][y].north           = NULL;
            cells[x][y].east            = NULL;
            cells[x][y].south           = NULL;
            cells[x][y].west            = NULL;
            cells[x][y].distance        = MAX_DISTANCE;
        }
    }
};

//-----------------------------------------------------------------------------
void Maze::Print()
//-----------------------------------------------------------------------------
{
	char black[]     = "\033[22;30m";
	char red[]       = "\033[22;31m";
	char l_red[]     = "\033[01;31m";
	char green[]     = "\033[22;32m";
	char l_green[]   = "\033[01;32m";
	char orange[]    = "\033[22;33m";
	char yellow[]    = "\033[01;33m";
	char blue[]      = "\033[22;34m";
	char l_blue[]    = "\033[01;34m";
	char magenta[]   = "\033[22;35m";
	char l_magenta[] = "\033[01;35m";
	char cyan[]      = "\033[22;36m";
	char l_cyan[]    = "\033[01;36m";
	char gray[]      = "\033[22;37m";
	char white[]     = "\033[01;37m";

    printf("\033[2J  %s+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+", blue); //clear screen, move to (0,0), print top line in blue
    for (int y=DIMY-1; y>=0; y--)
    {
        printf("\n%s%d %s|", white, y,blue);
        for (int x=0; x<DIMX; x++)
        {
            if (cells[x][y].distance != MAX_DISTANCE)
                printf("%s%4.1f %s", green, cells[x][y].distance, blue);
            else
                printf("     ");
            printf("%c", (!cells[x][y].east) ? '|' : ' ');
        }
        printf("\n  +");
        for (int x=0; x<DIMX; x++)
            printf("%s%c", (!cells[x][y].south) ? "-----" : "     ", (x==4 && y==5) ? 'o' : '+');
    }
    printf("\n%s     0     1     2     3     4     5     6     7     8     9   \n\n", white);
    printf("\033[%d;%dH%so%s\033[%d;%dH", 11, 33, red, white, 24, 0);
}

//-----------------------------------------------------------------------------
void Maze::Discover(Cell* cell)
//-----------------------------------------------------------------------------
{
    //recursively discover the maze. Call this function (recursively) for all 4 directions if there is no wall an the direction
    //wasn't already discovered.

    //NOTE: This is the same algorithm as the non-recursive one, but instead of stracking the undiscovered paths through a push/pop stack
    //      we use a 'call-stack' of recursive function calls. Fun, but a mess to debug if it goes wrong ;-)

    //mark the current cell as discovered and store the wall info for the 4 directions
    cell->discovered = true;
    if (!Wall(cell->x, cell->y, NORTH)) cell->north = &cells[cell->x  ][cell->y+1];
    if (!Wall(cell->x, cell->y, EAST))  cell->east  = &cells[cell->x+1][cell->y  ];
    if (!Wall(cell->x, cell->y, SOUTH)) cell->south = &cells[cell->x  ][cell->y-1];
    if (!Wall(cell->x, cell->y, WEST))  cell->west  = &cells[cell->x-1][cell->y  ];

    if ((cell->north) && (!cell->north->discovered)) Discover(cell->north);
    if ((cell->east)  && (!cell->east->discovered))  Discover(cell->east);
    if ((cell->south) && (!cell->south->discovered)) Discover(cell->south);
    if ((cell->west)  && (!cell->west->discovered))  Discover(cell->west);
}

//-----------------------------------------------------------------------------
void Maze::Solve()
//-----------------------------------------------------------------------------
{
    Queue queue;

    //start at (0,0)
    cells[0][0].distance = 0;
    queue.Push(&cells[0][0]);

    while (!queue.IsEmpty())
    {
        Cell* cell = queue.Pop();

//        float d;
//        for (Cell* c=cell; c!=NULL && (c->west || c->east) && ((d=cell->distance + Distance(cell, c)) < c->distance); c = c->north) {c->distance = d; queue.Push(c);}

        for (Cell* c = cell; c != NULL; c = c->north) if ((c->west  || c->east)  && (cell->distance + Distance(cell, c) < c->distance)) { c->distance = cell->distance + Distance(cell, c); queue.Push(c);}
        for (Cell* c = cell; c != NULL; c = c->east)  if ((c->north || c->south) && (cell->distance + Distance(cell, c) < c->distance)) { c->distance = cell->distance + Distance(cell, c); queue.Push(c);}
        for (Cell* c = cell; c != NULL; c = c->south) if ((c->east  || c->west)  && (cell->distance + Distance(cell, c) < c->distance)) { c->distance = cell->distance + Distance(cell, c); queue.Push(c);}
        for (Cell* c = cell; c != NULL; c = c->west)  if ((c->south || c->north) && (cell->distance + Distance(cell, c) < c->distance)) { c->distance = cell->distance + Distance(cell, c); queue.Push(c);}
    }
}

//-----------------------------------------------------------------------------
int main()
//-----------------------------------------------------------------------------
{
    Maze maze;

    maze.Discover(&maze.cells[0][0]);
    maze.Solve();
    maze.Print();

    return 0;
}
