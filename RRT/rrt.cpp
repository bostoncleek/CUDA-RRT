
#include <cstdlib>
#include <algorithm>
#include "rrt.hpp"
#include <iostream>



double distance(const double *p1, const double *p2)
{
  const auto dx = p1[0] - p2[0];
  const auto dy = p1[1] - p2[1];

  return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}


RRT::RRT(double *start, double *goal)
        : start_(start),
          goal_(goal),
          delta_(0.5),
          epsilon_(1),
          xmin_(0),
          xmax_(10),
          ymin_(0),
          ymax_(10),
          max_iter_(1000)
{
  // add start to graph
  addVertex(start[0], start[1]);
}



void RRT::addVertex(double x, double y)
{
  // check if already in graph;
  bool found = false;

  for(unsigned int i = 0; i < vertices_.size(); i++)
  {
    if (vertices_[i].x == x and vertices_[i].y == y)
    {
      found = true;
    }
  }

  if (!found)
  {
    vertex v;
    v.x = x;
    v.y = y;

    vertices_.push_back(v);
  }
}



void RRT::addEdge(double x1, double y1, double x2, double y2)
{
  // search for node1 and node2
  // addes edge btw both

  for(unsigned int i = 0; i < vertices_.size(); i++)
  {
    // found node 1
    if (vertices_[i].x == x1 and vertices_[i].y == y1)
    {
      for(unsigned int j = 0; j < vertices_.size(); j++)
      {
        // do not add vertex to itself
        // found node 2
        if(vertices_[j].x == x2 && vertices_[j].y == y2 && i != j)
        {
          // edge connecting node 1 to node 2
          Edge edge1;
          edge1.v = &vertices_[j];
          vertices_[i].Edges.push_back(edge1);

          // edge connecting node 2 to node 1
          Edge edge2;
          edge2.v = &vertices_[i];
          vertices_[j].Edges.push_back(edge2);
        }
      } // end inner loop
    }
  } // end outer loop
}



bool RRT::explore()
{
  bool success = false;
  int ctr = 0;


  while(!success)
  {
    if (ctr == max_iter_)
    {
      std::cout << "Goal not achieved" << std::endl;
      return false;
    }

    // 1) random point
    double q_rand[2];
    randomConfig(q_rand);

    // 2) nearest node in graph
    vertex v_near = nearestVertex(q_rand);

    // 3) new node
    vertex v_new;
    if(!newConfiguration(v_new, v_near, q_rand))
    {
      continue;
    }

    std::cout << v_new.x << " " << v_new.y << "\n";

    // 4) add new node
    addVertex(v_new.x, v_new.y);
    addEdge(v_new.x, v_new.y, v_near.x, v_near.y);

    // 5) goal reached
    double p1[] = {v_new.x, v_new.y};
    const auto d = distance(p1, goal_);

    if (d <= epsilon_)
    {
      std::cout << "Goal reached" << std::endl;
      break;
    }

    ctr++;
  }

  return true;
}



bool RRT::newConfiguration(vertex &v_new, const vertex &v_near, const double *q_rand) const
{

  // difference btw q_rand and v_near
  const auto vx = q_rand[0] - v_near.x;
  const auto vy = q_rand[1] - v_near.y;

  // distance between v_near and q_rand
  const auto magnitude = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2));

  if (magnitude == 0)
  {
    return false;
  }

  // unit vector in driection of q_rand
  const auto ux = vx / magnitude;
  const auto uy = vy / magnitude;

  // place v_new a delta away from v_near
  v_new.x = v_near.x + delta_ * ux;
  v_new.y = v_near.y + delta_ * uy;

  return true;
}



vertex &RRT::nearestVertex(double *q_rand)
{
  double point[2];
  std::vector<double> d;

  for(unsigned int i = 0; i < vertices_.size(); i++)
  {
    point[0] = vertices_[i].x;
    point[1] = vertices_[i].y;

    d.push_back(distance(point, q_rand));
  }

  // index of nearest node
  const auto idx = std::min_element(d.begin(), d.end()) - d.begin();

  // std::cout << "minElementIndex:" << idx
  //     << ", minElement: [" << vertices_[idx].x << " " << vertices_[idx].y << "]\n";

  return vertices_[idx];
}



void RRT::randomConfig(double *q_rand) const
{
  // x position
  q_rand[0] = xmin_+static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX/(xmax_-xmin_)));

  // y position
  q_rand[1] = ymin_+static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX/(ymax_-ymin_)));
}













// end file
