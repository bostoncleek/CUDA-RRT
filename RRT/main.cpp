
#include <iostream>
#include "rrt.hpp"

int main()
{

  // std::cout << "RRT" << std::endl;
  double start[] = {0,0};
  double goal[] = {2.5,2.5};

  RRT rrt(start, goal);
  rrt.randomCircles(5, 0.0, 0.1);

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
