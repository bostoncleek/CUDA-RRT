
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>

#include "rrt.hpp"

#include <cuda.h>
#include "collision_check.h"

double distance(const double *p1, const double *p2)
{
  const double dx = p1[0] - p2[0];
  const double dy = p1[1] - p2[1];

  return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}


double closestPointDistance(const double *p1, const double *p2, const double *p3)
{
  const double num = std::fabs((p2[1] - p1[1]) * p3[0] - \
                   (p2[0] - p1[0]) * p3[1] + \
                    p2[0] * p1[1] - p2[1] * p1[0]);

  const double denom = std::sqrt(std::pow(p2[0] - p1[0], 2) + \
                               std::pow(p2[1] - p1[1], 2));

  return num / denom;
}



RRT::RRT(double *start, double *goal)
        : start_(start),
          goal_(goal),
          delta_(0.5),
          epsilon_(1),
          xmin_(0),
          xmax_(5),
          ymin_(0),
          ymax_(5),
          max_iter_(1000),
          vertex_count_(0)
{
  // add start to graph
  vertex v_start;
  v_start.x = start[0];
  v_start.y = start[1];
  addVertex(v_start);

  // seed random generator
  std::srand(40);
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
    vertex v_near;
    nearestVertex(v_near, q_rand);

    // 3) new node
    vertex v_new;
    if(!newConfiguration(v_new, v_near, q_rand))
    {
      continue;
    }

    std::cout << v_new.x << " " << v_new.y << "\n";

    // 4) add new node
    addVertex(v_new);
    addEdge(v_near, v_new);

    // 5) goal reached
    double p1[] = {v_new.x, v_new.y};
    double d = distance(p1, goal_);

    if (d <= epsilon_)
    {
      std::cout << "Goal reached" << std::endl;

      // add goal to graph
      vertex v_goal;
      v_goal.x = goal_[0];
      v_goal.y = goal_[1];
      addVertex(v_goal);
      addEdge(v_new, v_goal);

      break;
    }

    ctr++;
  }

  return true;
}



bool RRT::exploreObstacles()
{
  bool success = false;
  int ctr = 0;


  while(!success)
  {
    if (ctr > max_iter_)
    {
      std::cout << "Goal not achieved" << std::endl;
      return false;
    }

    std::cout << "Iter: " << ctr << std::endl;

    // 1) random point
    double q_rand[2];
    randomConfig(q_rand);

    // 2) nearest node in graph
    vertex v_near;
    nearestVertex(v_near, q_rand);

    // 3) new node
    vertex v_new;
    if(!newConfiguration(v_new, v_near, q_rand))
    {
      continue;
    }

    std::cout << "v new at: " << v_new.x << ", " << v_new.y <<std::endl;
    // 4) collision btw new vertex and circles
    if (objectCollision(v_new))
    {
      std::cout << "Obstacle Collision" << std::endl;
      continue;
    }

    // 5) collision btw edge form v_new to v_near and a circle
    if (pathCollision(v_new, v_near))
    {
      std::cout << "Path Collision" << std::endl;
      continue;
    }


    std::cout << v_new.x << " " << v_new.y << "\n";

    // 6) add new node
    addVertex(v_new);
    addEdge(v_near, v_new);

    // 7) win check
    bool win_flag = win_check(v_new, goal_);

    if (win_flag)
    {
      std::cout << "Goal reached" << std::endl;
      // add goal to graph
      vertex v_goal;
      v_goal.x = goal_[0];
      v_goal.y = goal_[1];
      addVertex(v_goal);
      addEdge(v_new, v_goal);

      success = true;
      break;
    }

    ctr++;
  }

  return success;

}

bool RRT::win_check(const vertex &v_new, const double *goal)
{
  //cast goal to vertex //TODO: overlead collision to optionally take double as second arg
  vertex v_goal(goal[0],goal[1]);
  bool collis_check = pathCollision(v_new, v_goal);
  return collis_check;
}


bool RRT::exploreCuda()
{
  // TODO: copy q_new when it becomes availble


  ////////////////////////////////////////////////////////////////////////////
  // set up variables for host
  uint32_t num_circles = circles_.size();
  float *h_x = (float *)malloc(num_circles * sizeof(float));
  float *h_y = (float *)malloc(num_circles * sizeof(float));
  float *h_r = (float *)malloc(num_circles * sizeof(float));
  float *h_q = (float *)malloc(2 * sizeof(float));
  uint32_t *h_obs_flag = (uint32_t *)malloc(sizeof(uint32_t));

  // fill circles with data
  circleData(h_x, h_y, h_r);
  // for (unsigned int i = 0; i < num_circles; i++)
  // {
  //   printf("%f %f %f\n", h_x[i], h_y[i], h_r[i]);
  // }


  /////////////////////_///////////////////////////////////////////////////////
  // set up variables for device
  float *d_x = (float *)allocateDeviceMemory(num_circles * sizeof(float));
  float *d_y = (float *)allocateDeviceMemory(num_circles * sizeof(float));
  float *d_r = (float *)allocateDeviceMemory(num_circles * sizeof(float));
  float *d_q = (float *)allocateDeviceMemory(2 * sizeof(float));
  uint32_t *d_obs_flag = (uint32_t *)allocateDeviceMemory(sizeof(uint32_t));


  copyToDeviceMemory(d_x, h_x, num_circles * sizeof(float));
  copyToDeviceMemory(d_y, h_y, num_circles * sizeof(float));
  copyToDeviceMemory(d_r, h_r, num_circles * sizeof(float));



  ////////////////////////////////////////////////////////////////////////////
  // start RRT


  bool success = false;
  int ctr = 0;


  while(!success)
  {
    if (ctr == max_iter_)
    {
      std::cout << "Goal not achieved" << std::endl;
    }


    // 1) random point
    double q_rand[2];
    randomConfig(q_rand);

    // 2) nearest node in graph
    vertex v_near;
    nearestVertex(v_near, q_rand);

    // 3) new node
    vertex v_new;
    if(!newConfiguration(v_new, v_near, q_rand))
    {
      continue;
    }


    ////////////////////////////////////////////////////////////////////////////
    // call device for obstacle collisions
    // 4) collision btw new vertex and circles

    h_q[0] = v_new.x;
    h_q[1] = v_new.y;

    // copy nominal new vertex
    copyToDeviceMemory(d_q, h_q, 2 * sizeof(float));

    // calls obstalce kernel
    obstacle_collision(d_x, d_y, d_r, d_q, d_obs_flag);

    // copy flag to host
    copyToHostMemory(h_obs_flag, d_obs_flag, sizeof(uint32_t));


    if (*h_obs_flag)
    {
      std::cout << "Obstacle Collision" << std::endl;
      continue;
    }





    ////////////////////////////////////////////////////////////////////////////
    // call device for path collisions
    // 5) collision btw edge form v_new to v_near and a circle
    if (pathCollision(v_new, v_near))
    {
      // std::cout << "Path Collision" << std::endl;
      continue;
    }


    // std::cout << v_new.x << " " << v_new.y << "\n";

    // 6) add new node
    addVertex(v_new);
    addEdge(v_near, v_new);

    // 7) goal reached
    double p1[] = {v_new.x, v_new.y};
    double d = distance(p1, goal_);

    if (d <= epsilon_)
    {
      std::cout << "Goal reached" << std::endl;

      // add goal to graph
      vertex v_goal;
      v_goal.x = goal_[0];
      v_goal.y = goal_[1];
      addVertex(v_goal);
      addEdge(v_new, v_goal);

      success = true;
      break;
    }

    ctr++;
  }

  ////////////////////////////////////////////////////////////////////////////
  // tear down host variables
  free(h_x);
  free(h_y);
  free(h_r);
  free(h_q);

  ////////////////////////////////////////////////////////////////////////////
  // tear down device variables
  freeDeviceMemory(d_x);
  freeDeviceMemory(d_y);
  freeDeviceMemory(d_r);
  freeDeviceMemory(d_q);

  return success;
}

void RRT::randomCircles(int num_cirles, double r_min, double r_max)
{

  for(int i = 0; i < num_cirles; i++)
  {
    // circle center within bounds of world
    const double x = xmin_+static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX/(xmax_-xmin_)));
    const double y = xmin_+static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX/(xmax_-xmin_)));

    // radius between r_min and r_max;
    const double r = r_min+static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX/(r_max-r_min)));

    const double center[] = {x, y};

    // make sure start and goal are not within an obstacle
    const double d_init = distance(center, start_);
    const double d_goal = distance(center, goal_);


    if (d_init > r + epsilon_ and d_goal > r + epsilon_)
    {
      Circle c;
      c.x = x;
      c.y = y;
      c.r = r;

      circles_.push_back(c);
    }
  }

  // for(const auto &circle: circles_)
  // {
  //   std::cout << "Circle: " << circle.r << " [" << circle.x << " " << circle.y << "]" << std::endl;
  // }
}



void RRT::circleData(float *h_x, float *h_y, float *h_r)
{
  for(unsigned int i = 0; i < circles_.size(); i++)
  {
    h_x[i] = ((float)circles_.at(i).x);

    h_y[i] = ((float)circles_.at(i).y);

    h_r[i] = ((float)circles_.at(i).r);
  }

}


void RRT::traverseGraph(std::vector<vertex> &path) const
{
  // path.reserve(vertices_.size());

  int start_idx = 0;                  // first vertex added
  int goal_idx = vertex_count_-1;  // last vertex added


  // std::cout << "start: " << start_idx << std::endl;
  // std::cout << "goal: " << goal_idx << std::endl;


  // path is backwards
  path.push_back(vertices_.at(goal_idx));

  // current vertex is the goal
  vertex curr_v = vertices_.at(goal_idx);
  int curr_idx = goal_idx;


  while(curr_idx != start_idx)
  {

    int parent_idx = findParent(curr_v);

    path.push_back(vertices_.at(parent_idx));

    // update current node and current index
    curr_v = vertices_.at(parent_idx);
    curr_idx = parent_idx;
  }
}



void RRT::printGraph() const
{
  for(unsigned int i = 0; i < vertices_.size(); i++)
  {
    std::cout << "vertex: " << vertices_.at(i).id << " -> ";
    for(unsigned int j = 0; j < vertices_.at(i).adjacent_vertices.size(); j++)
    {
      std::cout << vertices_.at(i).adjacent_vertices.at(j) << " ";
    }
    std::cout << std::endl;
  }
}

void RRT::visualizeGraph() const
{
  std::ofstream obstacles;
  std::ofstream graph;
  obstacles.open("rrtout/obstacles.csv");  
  graph.open("rrtout/graph.csv");  
  double x1, y1;
  int mark;

  //log obstacles
  for (unsigned int i = 0; i < circles_.size(); i++){
    obstacles << circles_.at(i).x << "," << circles_.at(i).y << "," << circles_.at(i).r << "\n";
  }

  //log graph (nodes and vertices)
  for(unsigned int i = 0; i < vertices_.size(); i++)
  {

    //mark if root or goal -1 for root, 1 for goal, 0 for all else
    if (i == 0){
      mark = -1;
    } else if (i == (vertices_.size() -1)){ //TODO: figure out why its not setting last el to 1
      mark = 1;
    } else {
      mark = 0;
    }

    x1 = vertices_.at(i).x;
    y1 = vertices_.at(i).y;

    for(unsigned int j = 0; j < vertices_.at(i).adjacent_vertices.size(); j++)
    {
      int v_id = vertices_.at(i).adjacent_vertices.at(j);
      graph << vertices_.at(v_id).x << "," << vertices_.at(v_id).y << "," << x1 << "," << y1 << "," << mark << "\n";
    }

  }

  graph << goal_[0] << "," << goal_[1] << "," << vertices_.back().x << "," << vertices_.back().y << "," << 1 <<"\n"; 

  obstacles.close();
  graph.close();
}


void RRT::addVertex(vertex &v)
{
  v.id = vertex_count_;

  vertices_.push_back(v);
  vertex_count_++;

  // std::cout << "New vertex count: " << vertex_count_ << std::endl;
}



void RRT::addEdge(const vertex &v_near, const vertex &v_new)
{
  // search for node1 and node2
  // addes edge btw both
  bool added = false;


  for(unsigned int i = 0; i < vertices_.size(); i++)
  {
    // found node 1
    if (vertices_.at(i).id == v_near.id)
    {
      for(unsigned int j = 0; j < vertices_.size(); j++)
      {
        // do not add vertex to itself
        // found node 2
        if(vertices_.at(j).id == v_new.id && i != j)
        {
          // edge connecting node 1 to node 2
          // std::cout << "adding edge " << v_near.id << "->" << v_new.id << std::endl;
          // v_near.adjacent_vertices.push_back(v_new.id);
          vertices_.at(v_near.id).adjacent_vertices.push_back(v_new.id);
          added = true;
        }

      } // end inner loop
    }
  } // end outer loop

  if (!added)
  {
    std::cout << "Error: 'addEdge' edge not added" << std::endl;
  }
}


bool RRT::newConfiguration(vertex &v_new, const vertex &v_near, const double *q_rand) const
{

  // difference btw q_rand and v_near
  const double vx = q_rand[0] - v_near.x;
  const double vy = q_rand[1] - v_near.y;

  // distance between v_near and q_rand
  const double magnitude = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2));

  if (magnitude == 0)
  {
    return false;
  }

  // unit vector in driection of q_rand
  const double ux = vx / magnitude;
  const double uy = vy / magnitude;

  // place v_new a delta away from v_near
  v_new.x = v_near.x + delta_ * ux;
  v_new.y = v_near.y + delta_ * uy;

  return true;
}



void RRT::nearestVertex(vertex &v, double *q_rand) const
{
  double point[2];
  std::vector<double> d;

  for(unsigned int i = 0; i < vertices_.size(); i++)
  {
    point[0] = vertices_.at(i).x;
    point[1] = vertices_.at(i).y;

    d.push_back(distance(point, q_rand));
  }

  // index of nearest node
  const int idx = std::min_element(d.begin(), d.end()) - d.begin();

  // int idx = 0;
  // double smallest = d.at(0);
  //
  // for(unsigned int i = 1; i < d.size(); i++)
  // {
  //   if(d.at(i) < smallest)
  //   {
  //     smallest = d.at(i);
  //     idx = i;
  //   }
  // }

  // std::cout << "minElementIndex:" << idx
  //     << ", minElement: [" << vertices_[idx].x << " " << vertices_[idx].y << "]\n";

  // vertex v_near = vertices_.at(idx);
  v = vertices_.at(idx);
}



void RRT::randomConfig(double *q_rand) const
{
  // x position
  q_rand[0] = xmin_+static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX/(xmax_-xmin_)));

  // y position
  q_rand[1] = ymin_+static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX/(ymax_-ymin_)));
}



bool RRT::objectCollision(const vertex &v_new) const
{
  const double v[2] = {v_new.x, v_new.y};

  for(unsigned int i = 0; i < circles_.size(); i++)
  {
    const double c[2] = {circles_.at(i).x, circles_.at(i).y};
    const double d = distance(c, v);

    if (d < circles_.at(i).r + epsilon_)
    {
      return true;
    }
  }

  return false;
}


// bool RRT::pathCollision(const vertex &v_new, const double * goal) const
// {
//   const double vnew[2] = {v_new.x, v_new.y};

//   for(unsigned int i = 0; i < circles_.size(); i++)
//   {
//     const double c[2] = {circles_.at(i).x, circles_.at(i).y};
//     const double d = closestPointDistance(vnew, goal, c);

//     if (d < circles_.at(i).r + epsilon_)
//     {
//       return true;
//     }
//   }

//   return false;

// }


bool RRT::pathCollision(const vertex &v_new, const vertex &v_near) const
{
  const double vnew[2] = {v_new.x, v_new.y};
  const double vnear[2] = {v_near.x, v_near.y};

  for(unsigned int i = 0; i < circles_.size(); i++)
  {
    const double c[2] = {circles_.at(i).x, circles_.at(i).y};
    const double d = closestPointDistance(vnew, vnear, c);

    if (d < circles_.at(i).r + epsilon_)
    {
      return true;
    }
  }

  return false;
}


int RRT::findParent(const vertex &v) const
{
  // iterate over vertices
  for(unsigned int i = 0; i < vertices_.size(); i++)
  {
    for(unsigned int j = 0; j < vertices_.at(i).adjacent_vertices.size(); j++)
    {
      if (vertices_.at(i).adjacent_vertices.at(j) == v.id)
      {
        // std::cout << "Parent found" << std::endl;
        return i;
      }
    } // end inner loop
  } // end outer loop

  std::cout << "Parent not found" << std::endl;
  return -1;
}










// end file
