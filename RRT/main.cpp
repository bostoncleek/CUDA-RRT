
#include <iostream>
#include "rrt.hpp"




int main()
{

  // std::cout << "RRT" << std::endl;
  double start[] = {0,0};
  double goal[] = {10,10};

  RRT rrt(start, goal);

  rrt.explore();


  return 0;
}
