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

  int num_circs;
  int rand_num;

  //expecting command line args to be rand num, then num circs
  if (argc > 2)
  {
    num_circs = std::atoi(argv[2]);
    rand_num = std::atoi(argv[1]);
  } else if (argc > 1) {
    rand_num = std::atoi(argv[1]);
    num_circs = 2048;
  } else {
    num_circs = 2048;
    rand_num = 10;
  }

  RRT rrt(start, goal, rand_num);
  rrt.randomCircles(num_circs, 0.0, 0.5);

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
  std::vector<vertex> path;
  rrt.traverseGraph(path);


  // rrt.printGraph();
  rrt.visualizeGraph();




  // find path
  //
  // std::cout << "-----------------------" << std::endl;
  // for(unsigned int i = 0; i < path.size(); i++)
  // {
  //   std::cout << "[" << path.at(i).x << " " << path.at(i).y << "]" << std::endl;
  // }
  // std::cout << "-----------------------" << std::endl;



  return 0;
}
