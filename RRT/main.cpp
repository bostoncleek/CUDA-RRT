#include <cstdlib>
#include <iostream>
#include "rrt.hpp"

int main(int argc, char * argv[])
{

  // std::cout << "RRT" << std::endl;
  double start[] = {0,0};
  double goal[] = {5,5};

  RRT rrt(start, goal, std::atoi(argv[1]));
  rrt.randomCircles(100, 0.0, 0.3);

  // rrt.explore();
  // rrt.exploreObstacles();
  rrt.exploreCuda();


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
