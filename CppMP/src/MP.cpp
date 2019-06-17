 #include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>
using namespace std;
#define CACHESIZE 10000
#define GOALPROB 0.005
#define WAYPOINTPROB 0.0005

//Created by Kevin Silvera and Haseeb Shuaib

//cache of valid states
double cache[CACHESIZE][2];
int rowIndex = 0;
int cacheBuilt = 0;

MotionPlanner::MotionPlanner(Simulator * const simulator)
{
    m_simulator = simulator;   

    Vertex *vinit = new Vertex();
    vinit->m_parent   = -1;   
    vinit->m_nchildren= 0;    
    vinit->m_state[0] = m_simulator->GetRobotCenterX();
    vinit->m_state[1] = m_simulator->GetRobotCenterY();

    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;
    openFile();
}

MotionPlanner::~MotionPlanner(void)
{
    //do not delete m_simulator  

    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
	delete m_vertices[i];
}

//Function should extend tree from vertex with index vid 
//to state sto in small steps (each step is added to tree)
/*
The code below is good for 2 cases:
    1) library is empty
    2) no paths in the library is worth using

    When there is an appliciable library path:
        Use the same code except loop through each valid vertices as a goal and
            until the final goal is found.
                (special case: what if a vertex is inside an obstacle.....)
*/
void MotionPlanner::ExtendTree(const int    vid, 
			       const double sto[])
{

    double distX = sto[0] - m_vertices[vid]->m_state[0];
    double distY = sto[1] - m_vertices[vid]->m_state[1];
    double dist = sqrt(pow(distX,2) + pow(distY,2));

    double vx = m_vertices[vid]->m_state[0];
    double vy = m_vertices[vid]->m_state[1];
    double step = m_simulator->GetDistOneStep();

    double dx = distX/dist * step;
    double dy = distY/dist * step;

    const int n = m_simulator->GetNrObstacles();

    double radius = m_simulator->GetRobotRadius();

    for(int j = 0; j<dist/step; j++){
        vx += dx;
        vy += dy;

        for(int i = 0; i < n; i++){
            double obs_x = m_simulator->GetObstacleCenterX(i);
            double obs_y = m_simulator->GetObstacleCenterY(i);
            double radi = m_simulator->GetObstacleRadius(i);
            double distance = sqrt(pow((vx-obs_x),2)+ pow((vy-obs_y),2));

            if((radius+radi) > distance){
                return; //collision with obstacle
            }
        }
        Vertex *v = new Vertex();

        if(j == 0){
            v->m_parent = vid;
        }
        else{
            v->m_parent = m_vertices.size() - 1;
        }
        v->m_nchildren = 0;
        v->m_state[0] = vx;
        v->m_state[1] = vy;

        AddVertex(v);
        
        double xToGoal = m_simulator->GetGoalCenterX() - vx;
        double yToGoal = m_simulator->GetGoalCenterY() - vy;
        double vToGoal = sqrt(pow(xToGoal,2) + pow(yToGoal,2));
        double radius = m_simulator->GetRobotRadius();

        if(vToGoal <= radius){
            m_vidAtGoal = v->m_parent;
            break;
        }
    }

			
}

void MotionPlanner::addToCache(struct waypoint v){

    double sto[2];
    if (rowIndex <= CACHESIZE-1){
        cache[rowIndex][0] = v.x;
        cache[rowIndex][1] = v.y;
        //printf("ADDING %d :[%lf][%lf]\n",rowIndex, cache[rowIndex][0],cache[rowIndex][1]);
        rowIndex++;
    }
    else{
            int i = PseudoRandomUniformReal(0,CACHESIZE-1);
            cache[i][0] = v.x;
            cache[i][1] = v.y;
           // printf("replacing i :[%lf][%lf]\n", i, cache[i][0],cache[i][1]);

    }
   
}
 
struct waypoint MotionPlanner::ChooseTarget(void){
     double i = PseudoRandomUniformReal(0,1);
     int y = PseudoRandomUniformReal(0,CACHESIZE-1);
     struct waypoint sto;

     if(i <= GOALPROB){
        sto.x = m_simulator->GetGoalCenterX();
        sto.y = m_simulator->GetGoalCenterY();
     }
    else if(i > GOALPROB && i <= WAYPOINTPROB){
        sto.x = cache[y][0];
        sto.y = cache[y][1];
    }
    else {
        double temp[2];
        m_simulator->SampleState(temp);
        sto.x = temp[0];
        sto.y = temp[1];
    }
    return sto;
}
void MotionPlanner::openFile(void){
    ifstream infile; 
   infile.open("cache.txt");
   if(infile.is_open()){
        //cout << "Opened" << endl;
        for(int i = 0; i<CACHESIZE; i++){
            for(int j = 0; j<2; j++){
                string s1;
                infile >> s1;
                double value = atof(s1.c_str());
             //   printf("Converstion  %f, placed at [%d][%d]\n",value, i,j);
                cache[i][j] = value;

         }
        }
        cacheBuilt = 1;
    }
    // else
    //     cout << "Error opening file";
    infile.close();
}

void MotionPlanner::writeFile(void){
     ofstream outfile;
   outfile.open("cache.txt");
   for(int i=0; i < rowIndex; i++){
    for(int j = 0; j < 2; j++){
        if(j==0)
            outfile << cache[i][j] << " ";
        else
            outfile << cache[i][j] << endl;
    }
   }
   
   outfile.close();
}

void MotionPlanner::ExtendERRT(void)
{
    Clock clk;
    StartTime(&clk);

    double sto[2];
    //m_simulator->SampleState(sto); //pick a state (REGULAR RRT)
    
    if (cacheBuilt){
       //printf("Cache Built. Using ERRT\n");
        struct waypoint temp =  ChooseTarget(); //this might not be waypoint, could also be goal or random
        sto[0] = temp.x;
        sto[1] = temp.y;
        double min = 10000;
        double index,x,y,distance;
        for(int i = 0; i <m_vertices.size();i++){
        x = m_vertices[i]->m_state[0];
        y = m_vertices[i]->m_state[1];
        distance = sqrt(pow(x-sto[0],2) + pow(y-sto[1],2));
        if(distance<min){
            index = i;
            min = distance;
        }
    }        ExtendTree(index,sto); //build the tree to this point
    }
    else{
        //printf("Using RRT\n");
        ExtendRRT();
    }
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
//    printf("IN\n");
    double sto[2];
    int prob = PseudoRandomUniformReal(0,10);
    if(prob == 1){
    	sto[0] = m_simulator->GetRobotCenterX();
    	sto[1] = m_simulator->GetRobotCenterY(); 
    }
    else
    	m_simulator->SampleState(sto);

    double min = 10000;
    double index,x,y,distance;
    for(int i = 0; i <m_vertices.size();i++){
    	x = m_vertices[i]->m_state[0];
    	y = m_vertices[i]->m_state[1];
    	distance = sqrt(pow(x-sto[0],2) + pow(y-sto[1],2));
    	if(distance<min){
    		index = i;
    		min = distance;
    	}
    }
 	ExtendTree(index, sto);
    
    m_totalSolveTime += ElapsedTime(&clk);
      //  printf("OUT\n");

}


void MotionPlanner::AddVertex(Vertex * const v)
{
    if(v->m_type == Vertex::TYPE_GOAL)
	m_vidAtGoal = m_vertices.size();
    m_vertices.push_back(v); 
    if(v->m_parent >= 0)
	(++m_vertices[v->m_parent]->m_nchildren);
}

///LEET
void MotionPlanner::GetPathFromInitToGoal(std::vector<int> *path)
{
    
    std::vector<int> rpath;
    
    rpath.clear();
    struct waypoint vert;

    int i = m_vidAtGoal;
    do
    {
	rpath.push_back(i);
    
        vert.x = m_vertices[i]->m_state[0];
        vert.y = m_vertices[i]->m_state[1];
        addToCache(vert);
	i = m_vertices[i]->m_parent;	
    } 
    while(i >= 0);
    
    path->clear();
    for(int i = rpath.size() - 1; i >= 0; --i){
	path->push_back(rpath[i]);
    }
    writeFile();
}
