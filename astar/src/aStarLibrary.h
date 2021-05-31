#include "math.h"
#include <stdlib.h>
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "time.h"
#include <string.h>
#include <stdio.h>

#define	PI 3.1415926535897931

/*
Visual C++ version
 */

	//Declare constants
        int const mapWidth = 1000, mapHeight = 1000, tileSize = 10, numberPeople = 1;
	int onClosedList = 10;
	int const notfinished = 0, notStarted = 0;// path-related constants
	int const found = 1, nonexistent = 2;
	int const walkable = 0, unwalkable = 1 ;// walkability array constants

	//Create needed arrays
	int walkability [mapWidth][mapHeight] = {0} ;
        int OriginalMap [mapWidth][mapHeight] = {0} ;
        int DistanceField [mapWidth][mapHeight] = {0} ;
	int openList[mapWidth*mapHeight+2]; //1 dimensional array holding ID# of open list items
	int whichList[mapWidth+1][mapHeight+1];  //2 dimensional array used to record
// 		whether a cell is on the open list or on the closed list.
	int openX[mapWidth*mapHeight+2]; //1d array stores the x location of an item on the open list
	int openY[mapWidth*mapHeight+2]; //1d array stores the y location of an item on the open list
	int parentX[mapWidth+1][mapHeight+1]; //2d array to store parent of each cell (x)
	int parentY[mapWidth+1][mapHeight+1]; //2d array to store parent of each cell (y)
	int Fcost[mapWidth*mapHeight+2];	//1d array to store F cost of a cell on the open list
        int Gcost[mapWidth+1][mapHeight+1]; 	//Exact cost fo the path from the starting point to any vertex n
        int Hcost[mapWidth*mapHeight+2];	//Heuristic estimatd cost from vertex n to the goal.
	int pathLength[numberPeople+1];     //stores length of the found path for critter
	int pathLocation[numberPeople+1];   //stores current position along the chosen path for critter
	int* pathBank [numberPeople+1];

	//Path reading variables
	int pathStatus[numberPeople+1];
	int xPath[numberPeople+1];
	int yPath[numberPeople+1];

	int x_pre[10000] = {0}, y_pre[10000] = {0}, x[10000] = {0}, y[10000] = {0} ;
	int step = 0;

	double path1_x[10000]= {0}, path1_y[10000] = {0} ;
	double path1_wayx[1000] = {0}, path1_wayy[1000] = {0}, waypoint[2000] = {0}, waypoint_nonexistent[1] = {0} ;

	double obs_num, obs_x, obs_y, obs_r ;
	int obs_xpos, obs_ypos, obs_range, margin_r ;
	int plot_chk =1 ;
	FILE *outfpt1, *wayfpt1, *MapData, *OutputMap, *OutputMap2 ;
	char mystring[5000], *token[100], *ptr ;
	int xDis, yDis ;
        const int DisStr = 100000, DisDia = 141400 ;
        const int wDistance = 150000;

        //const int DisStr = 10, DisDia = 14 ;
	FILE *fptres ;
	double CompTime ;

//-----------------------------------------------------------------------------
// Function Prototypes: where needed
//-----------------------------------------------------------------------------

void ReadPath(int pathfinderID,int currentX,int currentY, int pixelsPerFrame);
int ReadPathX(int pathfinderID,int pathLocation);
int ReadPathY(int pathfinderID,int pathLocation);
void InitializePathfinder (void) ;
void EndPathfinder (void) ;
int FindPath (int pathfinderID,int startingX, int startingY,int targetX, int targetY);


double* GenPath(int sx1, int sy1, int gx1, int gy1, int **obs_map, int MaxX, int MaxY)
{
	// if (plot_chk ==1)
	// {
	// 	fopen_s(&outfpt1, "Path1.txt", "w") ;
	// }

	int path ;
	clock_t start_time, end_time;      // clock_t
	int start_x, start_y, goal_x = 0, goal_y = 0 ;
	int pre_x = 0, pre_y = 0 ;
	int now_x = goal_x ;
	int now_y = goal_y ;
	int i = 0, waynum = 0  ;
	int scale = 10;
	double angle_chk = 0, pre_angle_chk = 0 ;
	int obs_range = 50 ;

	InitializePathfinder();
	step = 0 ;

        start_x = sx1*10 ;  //current pose * 10
	start_y = sy1*10 ;
        goal_x = gx1*10 ;
        goal_y = gy1*10 ;  //goal pose * 10


        for ( int num = 0 ; num < MaxX ; num++)  //Max from current Map size
	{
		for ( int num2 = 0 ; num2 < MaxY ; num2++)
		{
			walkability[num][num2] = obs_map[num][num2] ;
		}
	}

	start_time = clock();                  // Start_Time
	for ( int j = 0 ; j < 1 ; j++ )
	{
        path = FindPath (1,start_x,start_y,goal_x,goal_y);
	}
	end_time = clock();
	CompTime = (double)(end_time - start_time) / CLOCKS_PER_SEC;
	EndPathfinder();


	// fopen_s(&fptres, "CompTime.txt", "w") ; // Astar�� ���������� path ����
	// fprintf(fptres, "%.4f\n",CompTime) ;
	// fclose(fptres) ;

	if (path ==2)
	{
		return waypoint_nonexistent ;
	}
	else
	{
		path1_wayx[waynum] = (double)gx1 ;
		path1_wayy[waynum] = (double)gy1 ;
		waynum++ ;


		while(!( (now_x == start_x*0.1) && (now_y == start_y*0.1) ) )
		{


			path1_x[i] = x_pre[i];
			path1_y[i] = y_pre[i];
			// ROS_INFO("[%f, %f]",path1_x[i],path1_y[i]);

			// if (plot_chk ==1)
			// {
			// 	fprintf(outfpt1, "%.3f\t", path1_x[i]) ;
			// 	fprintf(outfpt1, "%.3f\n", path1_y[i]) ;
			// }

			now_x = x[i];
			now_y = y[i];

			if ( i > 0)
			{
				angle_chk = atan2(path1_y[i] - path1_y[i-1], path1_x[i] - path1_x[i-1]) ;
				// ROS_INFO("%f",angle_chk);
				if ( i > 1)
				{
					// ROS_INFO("[%f, %f]",path1_x[i],path1_y[i]);
					// ROS_INFO("Angles = [%f, %f]",pre_angle_chk*180.0/PI, angle_chk*180.0/PI);

					// ROS_INFO("%f",abs(pre_angle_chk - angle_chk)*180.0/PI);
                                        /*
					if ( (pre_angle_chk - angle_chk)*(pre_angle_chk - angle_chk) > (PI/18.0f)*(PI/18.0f) )
					{
                                                path1_wayx[waynum] = path1_x[i-1] ;
                                                path1_wayy[waynum] = path1_y[i-1] ;
                                                waynum++ ;
						// ROS_INFO("Waypoint Generated!");
                                        }
                                        */
                                        path1_wayx[waynum] = path1_x[i-1] ;
                                        path1_wayy[waynum] = path1_y[i-1] ;
                                        waynum++ ;

				}
				pre_angle_chk = angle_chk ;

			}
			i++ ;
		}

		path1_x[i] = now_x ;
		path1_y[i] = now_y ;
		path1_wayx[waynum] = path1_x[i] ;
		path1_wayy[waynum] = path1_y[i] ;


		// if (plot_chk ==1)
		// {
		// 	fprintf(outfpt1, "%.3f\t", path1_x[i]) ;
		// 	fprintf(outfpt1, "%.3f\n", path1_y[i]) ;
		// 	fclose(outfpt1) ;
		// }


                waypoint[0] = waynum+1 ; // WP0
		for (int i = 0; i < waynum+1; i++)
		{
                        waypoint[2*i+1] = path1_wayx[i] ; //WP1,3,5
                        waypoint[2*i+2] = path1_wayy[i] ; //WP2,4,6
		}


		return waypoint ;

	}
}


void InitializePathfinder (void)
{
	for (int x = 0; x < numberPeople+1; x++)
		pathBank [x] = (int*) malloc(4);
}


//-----------------------------------------------------------------------------
// Name: EndPathfinder
// Desc: Frees memory used by the pathfinder.
//-----------------------------------------------------------------------------
void EndPathfinder (void)
{
	for (int x = 0; x < numberPeople+1; x++)
	{
		free (pathBank [x]);
	}
}


//-----------------------------------------------------------------------------
// Name: FindPath
// Desc: Finds a path using A*
//-----------------------------------------------------------------------------
int FindPath (int pathfinderID,int startingX, int startingY,
                          int targetX, int targetY)
{

	for ( int num2 = 0 ; num2 < 10000 ; num2++)
	{
		x_pre[num2] = 0 ;
		y_pre[num2] = 0 ;
		x[num2] = 0 ;
		y[num2] = 0 ;

	}

	int onOpenList=0, parentXval=0, parentYval=0,
	a=0, b=0, m=0, u=0, v=0, temp=0, corner=0, numberOfOpenListItems=0,
	addedGCost=0, tempGcost = 0, path = 0,
	tempx, pathX, pathY, cellPosition,
	newOpenListItemID=0;

//1. Convert location data (in pixels) to coordinates in the walkability array.
        int startX = startingX/tileSize;
        int startY = startingY/tileSize;
	targetX = targetX/tileSize;
	targetY = targetY/tileSize;

//2.Quick Path Checks: Under the some circumstances no path needs to
//	be generated ...

//	If starting location and target are in the same location...
	if (startX == targetX && startY == targetY && pathLocation[pathfinderID] > 0)
		return found;
	if (startX == targetX && startY == targetY && pathLocation[pathfinderID] == 0)
		return nonexistent;

//	If target square is unwalkable, return that it's a nonexistent path.
	if (walkability[targetX][targetY] == unwalkable)
		goto noPath;

//3.Reset some variables that need to be cleared
	if (onClosedList > 1000000) //reset whichList occasionally
	{
		for (int x = 0; x < mapWidth;x++) {
			for (int y = 0; y < mapHeight;y++)
				whichList [x][y] = 0;
		}
		onClosedList = 10;
	}
	onClosedList = onClosedList+2; //changing the values of onOpenList and onClosed list is faster than redimming whichList() array
	onOpenList = onClosedList-1;
	pathLength [pathfinderID] = notStarted;//i.e, = 0
	pathLocation [pathfinderID] = notStarted;//i.e, = 0
	Gcost[startX][startY] = 0; //reset starting square's G value to 0

//4.Add the starting location to the open list of squares to be checked.
	numberOfOpenListItems = 1;
	openList[1] = 1;//assign it as the top (and currently only) item in the open list, which is maintained as a binary heap (explained below)
	openX[1] = startX ; openY[1] = startY;

//5.Do the following until a path is found or deemed nonexistent.
	do
	{

//6.If the open list is not empty, take the first cell off of the list.
//	This is the lowest F cost cell on the open list.
	if (numberOfOpenListItems != 0)
	{

//7. Pop the first item off the open list.
	parentXval = openX[openList[1]];
	parentYval = openY[openList[1]]; //record cell coordinates of the item
	whichList[parentXval][parentYval] = onClosedList;//add the item to the closed list

//	Open List = Binary Heap: Delete this item from the open list, which
//  is maintained as a binary heap. For more information on binary heaps, see:
//	http://www.policyalmanac.org/games/binaryHeaps.htm
	numberOfOpenListItems = numberOfOpenListItems - 1;//reduce number of open list items by 1

//	Delete the top item in binary heap and reorder the heap, with the lowest F cost item rising to the top.
	openList[1] = openList[numberOfOpenListItems+1];//move the last item in the heap up to slot #1
	v = 1;

//	Repeat the following until the new item in slot #1 sinks to its proper spot in the heap.
	do
	{
	u = v;
	if (2*u+1 <= numberOfOpenListItems) //if both children exist
	{
	 	//Check if the F cost of the parent is greater than each child.
		//Select the lowest of the two children.
		if (Fcost[openList[u]] >= Fcost[openList[2*u]])
			v = 2*u;
		if (Fcost[openList[v]] >= Fcost[openList[2*u+1]])
			v = 2*u+1;
	}
	else
	{
		if (2*u <= numberOfOpenListItems) //if only child #1 exists
		{
	 	//Check if the F cost of the parent is greater than child #1
			if (Fcost[openList[u]] >= Fcost[openList[2*u]])
				v = 2*u;
		}
	}

	if (u != v) //if parent's F is > one of its children, swap them
	{
		temp = openList[u];
		openList[u] = openList[v];
		openList[v] = temp;
	}
	else
		break; //otherwise, exit loop

	}
	while (1);//reorder the binary heap


//7.Check the adjacent squares. (Its "children" -- these path children
//	are similar, conceptually, to the binary heap children mentioned
//	above, but don't confuse them. They are different. Path children
//	are portrayed in Demo 1 with grey pointers pointing toward
//	their parents.) Add these adjacent child squares to the open list
//	for later consideration if appropriate (see various if statements
//	below).
	for (b = parentYval-1; b <= parentYval+1; b++){
	for (a = parentXval-1; a <= parentXval+1; a++){

	//for (b = parentYval+1; b >= parentYval-1; b--){
	//for (a = parentXval+1; a >= parentXval-1; a--){

//	If not off the map (do this first to avoid array out-of-bounds errors)
	if (a != -1 && b != -1 && a != mapWidth && b != mapHeight){

//	If not already on the closed list (items on the closed list have
//	already been considered and can now be ignored).
	if (whichList[a][b] != onClosedList) {

//	If not a wall/obstacle square.
	if (walkability [a][b] != unwalkable) {

//	Don't cut across corners
	corner = walkable;
	if (a == parentXval-1)
	{
		if (b == parentYval-1)
		{
			if (walkability[parentXval-1][parentYval] == unwalkable
				|| walkability[parentXval][parentYval-1] == unwalkable)
				corner = unwalkable;
		}
		else if (b == parentYval+1)
		{
			if (walkability[parentXval][parentYval+1] == unwalkable
				|| walkability[parentXval-1][parentYval] == unwalkable)
				corner = unwalkable;
		}
	}
	else if (a == parentXval+1)
	{
		if (b == parentYval-1)
		{
			if (walkability[parentXval][parentYval-1] == unwalkable
				|| walkability[parentXval+1][parentYval] == unwalkable)
				corner = unwalkable;
		}
		else if (b == parentYval+1)
		{
			if (walkability[parentXval+1][parentYval] == unwalkable
				|| walkability[parentXval][parentYval+1] == unwalkable)
				corner = unwalkable;
		}
	}
	if (corner == walkable) {

		//	If not already on the open list, add it to the open list.
		if (whichList[a][b] != onOpenList)
		{

			//Create a new open list item in the binary heap.
			newOpenListItemID = newOpenListItemID + 1; //each new item has a unique ID #
			m = numberOfOpenListItems+1;
			openList[m] = newOpenListItemID;//place the new open list item (actually, its ID#) at the bottom of the heap
			openX[newOpenListItemID] = a;
			openY[newOpenListItemID] = b;//record the x and y coordinates of the new item

			//Figure out its G cost
			if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
				//addedGCost = 14;//cost of going to diagonal squares
				addedGCost = DisDia;//cost of going to diagonal squares
			else
				//addedGCost = 10;//cost of going to non-diagonal squares
				addedGCost = DisStr;//cost of going to non-diagonal squares
                        Gcost[a][b] = Gcost[parentXval][parentYval] + addedGCost;

                        //std::cout <<Gcost[a][b] << "  , " << DistanceField[a][b]*1000 <<"\n";
			//Figure out its H and F costs and parent
			//xDis = abs(a-targetX) ;
			//yDis = abs(b-targetY) ;
			//if ( xDis > yDis )
			//{
			//	 Hcost[openList[m]] = DisDia*yDis + DisStr*(xDis-yDis) ;
			//}
			//else
                        //{
			//	 Hcost[openList[m]] = DisDia*xDis + DisStr*(yDis-xDis) ;
			//}

                        Hcost[openList[m]] = (int)sqrt( (double)(a-targetX)*(a-targetX) + (double)(b-targetY)*(b-targetY) ) ;
                        //Hcost[openList[m]] = 0 ;

                        Fcost[openList[m]] = Gcost[a][b] + Hcost[openList[m]] - DistanceField[b][a]*wDistance;

                        //ROS_INFO("Gcost:  %d, Hcost: %d",Gcost[a][b],DistanceField[b][a]*wDistance);



			parentX[a][b] = parentXval ; parentY[a][b] = parentYval;

                        //Move the new open list item to the proper place in the binary heap.
			//Starting at the bottom, successively compare to parent items,
			//swapping as needed until the item finds its place in the heap
			//or bubbles all the way to the top (if it has the lowest F cost).
			while (m != 1) //While item hasn't bubbled to the top (m=1)
			{
				//Check if child's F cost is < parent's F cost. If so, swap them.
				if (Fcost[openList[m]] <= Fcost[openList[m/2]])
				{
					temp = openList[m/2];
					openList[m/2] = openList[m];
					openList[m] = temp;
					m = m/2;

				}
				else
					break;
			}
			numberOfOpenListItems = numberOfOpenListItems+1;//add one to the number of items in the heap

			//Change whichList to show that the new item is on the open list.
			whichList[a][b] = onOpenList;
		}

		//8.If adjacent cell is already on the open list, check to see if this
		//	path to that cell from the starting location is a better one.
		//	If so, change the parent of the cell and its G and F costs.
			else //If whichList(a,b) = onOpenList
			{
                                //F(n) = G(n) + H(n)
				//Figure out the G cost of this possible new path
                                //G cost = Actual cost the time distance from beginning to Here
                                //H cost = Actual cost the time distance from Here to End
				if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
					//addedGCost = 14;//cost of going to diagonal tiles
					addedGCost = DisDia;//cost of going to diagonal tiles
				else
					//addedGCost = 10;//cost of going to non-diagonal tiles
					addedGCost = DisStr;//cost of going to non-diagonal tiles
                                tempGcost = Gcost[parentXval][parentYval] + addedGCost ;

				//If this path is shorter (G cost is lower) then change
				//the parent cell, G cost and F cost.
				if (tempGcost < Gcost[a][b]) //if G cost is less,
				{
					parentX[a][b] = parentXval; //change the square's parent
					parentY[a][b] = parentYval;
					Gcost[a][b] = tempGcost;//change the G cost

					//Because changing the G cost also changes the F cost, if
					//the item is on the open list we need to change the item's
					//recorded F cost and its position on the open list to make
					//sure that we maintain a properly ordered open list.
					for (int x = 1; x <= numberOfOpenListItems; x++) //look for the item in the heap
					{
					if (openX[openList[x]] == a && openY[openList[x]] == b) //item found
					{
                                                Fcost[openList[x]] = Gcost[a][b] + Hcost[openList[x]] - DistanceField[b][a]*wDistance;//change the F cost

						//See if changing the F score bubbles the item up from it's current location in the heap
						m = x;
						while (m != 1) //While item hasn't bubbled to the top (m=1)
						{
							//Check if child is < parent. If so, swap them.
							if (Fcost[openList[m]] < Fcost[openList[m/2]])
							{
								temp = openList[m/2];
								openList[m/2] = openList[m];
								openList[m] = temp;
								m = m/2;
							}
							else
								break;
						}
						break; //exit for x = loop
					} //If openX(openList(x)) = a
					} //For x = 1 To numberOfOpenListItems
				}//If tempGcost < Gcost(a,b)

			}//else If whichList(a,b) = onOpenList
		}//If not cutting a corner
	}//If not a wall/obstacle square.
	}//If not already on the closed list
	}//If not off the map
	}//for (a = parentXval-1; a <= parentXval+1; a++){
	}//for (b = parentYval-1; b <= parentYval+1; b++){

	}//if (numberOfOpenListItems != 0)

//9.If open list is empty then there is no path.
	else
	{
		path = nonexistent; break;
	}

	//If target is added to open list then path has been found.
	if (whichList[targetX][targetY] == onOpenList)
	{
		path = found; break;
	}

	}
	while (1);//Do until path is found or deemed nonexistent

//10.Save the path if it exists.
	if (path == found)
	{

//a.Working backwards from the target to the starting location by checking
//	each cell's parent, figure out the length of the path.
	pathX = targetX; pathY = targetY;
	do
	{
		//Look up the parent of the current cell.
		tempx = parentX[pathX][pathY];
		pathY = parentY[pathX][pathY];
		pathX = tempx;

		//Figure out the path length
		pathLength[pathfinderID] = pathLength[pathfinderID] + 1;
	}
	while (pathX != startX || pathY != startY);

//b.Resize the data bank to the right size in bytes
	pathBank[pathfinderID] = (int*) realloc (pathBank[pathfinderID],
		pathLength[pathfinderID]*8);

//c. Now copy the path information over to the databank. Since we are
//	working backwards from the target to the start location, we copy
//	the information to the data bank in reverse order. The result is
//	a properly ordered set of path data, from the first step to the
//	last.
	pathX = targetX ; pathY = targetY;
	cellPosition = pathLength[pathfinderID]*2;//start at the end


	do
	{
	cellPosition = cellPosition - 2;//work backwards 2 integers
	pathBank[pathfinderID] [cellPosition] = pathX;
	pathBank[pathfinderID] [cellPosition+1] = pathY;

	x_pre[step] = pathX ;
	y_pre[step] = pathY ;


//d.Look up the parent of the current cell.
	tempx = parentX[pathX][pathY];
	pathY = parentY[pathX][pathY];
	pathX = tempx;

	x[step] = pathX ;
	y[step] = pathY ;


	step++;



//e.If we have reached the starting square, exit the loop.
	}
	while (pathX != startX || pathY != startY);

//11.Read the first path step into xPath/yPath arrays
	ReadPath(pathfinderID,startingX,startingY,1);

	}
	return path;


//13.If there is no path to the selected target, set the pathfinder's
//	xPath and yPath equal to its current location and return that the
//	path is nonexistent.
noPath:
	xPath[pathfinderID] = startingX;
	yPath[pathfinderID] = startingY;
	return nonexistent;
}




//==========================================================
//READ PATH DATA: These functions read the path data and convert
//it to screen pixel coordinates.
void ReadPath(int pathfinderID,int currentX,int currentY,
			  int pixelsPerFrame)
{
/*
;	Note on PixelsPerFrame: The need for this parameter probably isn't
;	that obvious, so a little explanation is in order. This
;	parameter is used to determine if the pathfinder has gotten close
;	enough to the center of a given path square to warrant looking up
;	the next step on the path.
;
;	This is needed because the speed of certain sprites can
;	make reaching the exact center of a path square impossible.
;	In Demo #2, the chaser has a velocity of 3 pixels per frame. Our
;	tile size is 50 pixels, so the center of a tile will be at location
;	25, 75, 125, etc. Some of these are not evenly divisible by 3, so
;	our pathfinder has to know how close is close enough to the center.
;	It calculates this by seeing if the pathfinder is less than
;	pixelsPerFrame # of pixels from the center of the square.

;	This could conceivably cause problems if you have a *really* fast
;	sprite and/or really small tiles, in which case you may need to
;	adjust the formula a bit. But this should almost never be a problem
;	for games with standard sized tiles and normal speeds. Our smiley
;	in Demo #4 moves at a pretty fast clip and it isn't even close
;	to being a problem.
*/


	int ID = pathfinderID; //redundant, but makes the following easier to read

	//If a path has been found for the pathfinder	...
	if (pathStatus[ID] == found)
	{

		//If path finder is just starting a new path or has reached the
		//center of the current path square (and the end of the path
		//hasn't been reached), look up the next path square.
		if (pathLocation[ID] < pathLength[ID])
		{
			//if just starting or if close enough to center of square
			if (pathLocation[ID] == 0 ||
				(abs(currentX - xPath[ID]) < pixelsPerFrame && abs(currentY - yPath[ID]) < pixelsPerFrame))
					pathLocation[ID] = pathLocation[ID] + 1;
		}

		//Read the path data.
		xPath[ID] = ReadPathX(ID,pathLocation[ID]);
		yPath[ID] = ReadPathY(ID,pathLocation[ID]);

		//If the center of the last path square on the path has been
		//reached then reset.
		if (pathLocation[ID] == pathLength[ID])
		{
			if (abs(currentX - xPath[ID]) < pixelsPerFrame
				&& abs(currentY - yPath[ID]) < pixelsPerFrame) //if close enough to center of square
					pathStatus[ID] = notStarted;
		}
	}

	//If there is no path for this pathfinder, simply stay in the current
 	//location.
	else
	{
		xPath[ID] = currentX;
		yPath[ID] = currentY;
	}
}


//The following two functions read the raw path data from the pathBank.
//You can call these functions directly and skip the readPath function
//above if you want. Make sure you know what your current pathLocation
//is.

//-----------------------------------------------------------------------------
// Name: ReadPathX
// Desc: Reads the x coordinate of the next path step
//-----------------------------------------------------------------------------
int ReadPathX(int pathfinderID,int pathLocation)
{
	int x;
	if (pathLocation <= pathLength[pathfinderID])
	{

	//Read coordinate from bank
	x = pathBank[pathfinderID] [pathLocation*2-2];

	//Adjust the coordinates so they align with the center
	//of the path square (optional). This assumes that you are using
	//sprites that are centered -- i.e., with the midHandle command.
	//Otherwise you will want to adjust this.
	x = (int)(tileSize*x + .5*tileSize) ;

	}
	return x;
}


//-----------------------------------------------------------------------------
// Name: ReadPathY
// Desc: Reads the y coordinate of the next path step
//-----------------------------------------------------------------------------
int ReadPathY(int pathfinderID,int pathLocation)
{
	int y;
	if (pathLocation <= pathLength[pathfinderID])
	{

	//Read coordinate from bank
	y = pathBank[pathfinderID] [pathLocation*2-1] ;

	//Adjust the coordinates so they align with the center
	//of the path square (optional). This assumes that you are using
	//sprites that are centered -- i.e., with the midHandle command.
	//Otherwise you will want to adjust this.
	y = (int)(tileSize*y + .5*tileSize) ;

	}
	return y;
}
