#include <cstdlib>
#include <iostream>
#include <cutil.h>
#include "rrt.hpp"

int main(int argc, char * argv[])
{
  printf("\n\n**===-------------------------------------------------===**\n");
  printf("\n\n**===--- RRT ---===**\n");
  printf("\n\n**===-------------------------------------------------===**\n");
  double start[] = {5,50};
  double goal[] = {70,50};

  RRT rrt(start, goal, std::atoi(argv[1]));
  rrt.randomCircles(8192, 0.0, 0.5);

  // rrt.explore();
  // rrt.exploreObstacles();
  // rrt.exploreCuda();


  unsigned int timer;
  float host_time;

  CUT_SAFE_CALL(cutCreateTimer(&timer));
  cutStartTimer(timer);

  rrt.exploreObstacles();

  cutStopTimer(timer);
  printf("\n\n**===-------------------------------------------------===**\n");
  printf("Host CPU Processing time: %f (ms)\n", cutGetTimerValue(timer));
  host_time = cutGetTimerValue(timer);
  CUT_SAFE_CALL(cutDeleteTimer(timer));



  float device_time;

  CUT_SAFE_CALL(cutCreateTimer(&timer));
  cutStartTimer(timer);

  rrt.exploreCuda();

  cutStopTimer(timer);
  printf("\n\n**===-------------------------------------------------===**\n");
  printf("CUDA Processing time: %f (ms)\n", cutGetTimerValue(timer));
  device_time = cutGetTimerValue(timer);
  printf("Speedup: %fX\n", host_time/device_time);

  CUT_SAFE_CALL(cutDeleteTimer(timer));


  rrt.visualizeGraph();
  // rrt.printGraph();




  // find path
  // std::vector<vertex> path;
  // rrt.traverseGraph(path);
  //
  // std::cout << "-----------------------" << std::endl;
  // for(unsigned int i = 0; i < path.size(); i++)
  // {
  //   std::cout << "[" << path.at(i).x << " " << path.at(i).y << "]" << std::endl;
  // }
  // std::cout << "-----------------------" << std::endl;



  return 0;
}
