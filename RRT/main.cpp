
#include <iostream>
#include "rrt.hpp"

int main()
{

  // std::cout << "RRT" << std::endl;
  double start[] = {0,0};
  double goal[] = {10,10};

  RRT rrt(start, goal);
  rrt.randomCircles(5, 0.0, 0.1);

  // rrt.explore();
  rrt.exploreObstacles();


  rrt.printGraph();




  // for(unsigned int i = 0; i < rrt.vertices_.size(); i++)
  // {
  //   std::cout << rrt.vertices_.at(i).adjacent_vertices.size() << " ";
  // }



  // vertex v = rrt.vertices_.at(28);
  //
  // std::cout << "vertex: [" << v.x << " " << v.y << "]\n";
  //
  // int idx = rrt.findParent(v);

// vertex: [4.68843 3.40946] -> [5.18697 3.44763]

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
