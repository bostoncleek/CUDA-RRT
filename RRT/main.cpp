
#include <iostream>
#include "rrt.hpp"

int main()
{

  // std::cout << "RRT" << std::endl;
  double start[] = {0,0};
  double goal[] = {2,2};

  RRT rrt(start, goal);
  rrt.randomCircles(5, 0.0, 0.1);

  rrt.explore();
  // rrt.exploreObstacles();


  for(unsigned int i = 0; i < rrt.vertices_.size(); i++)
  {
    std::cout << "vertex: [" << rrt.vertices_.at(i).x << " " << rrt.vertices_.at(i).y << "] -> ";
    for(unsigned int j = 0; j < rrt.vertices_.at(i).Edges.size(); j++)
    {
      std::cout << "[" << rrt.vertices_.at(i).Edges.at(j).v->x << " " << rrt.vertices_.at(i).Edges.at(j).v->y << "] ";
    }
    std::cout << std::endl;
  }



  // find path
  std::vector<vertex> path;
  rrt.traverseGraph(path);


  std::cout << "-----------------------" << std::endl;
  for(unsigned int i = 0; i < path.size(); i++)
  {
    std::cout << "[" << path.at(i).x << " " << path.at(i).y << "]" << std::endl;
  }
  std::cout << "-----------------------" << std::endl;



  return 0;
}
