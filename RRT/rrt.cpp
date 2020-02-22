
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "rrt.hpp"



double distance(const double *p1, const double *p2)
{
  const auto dx = p1[0] - p2[0];
  const auto dy = p1[1] - p2[1];

  return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}


double closestPointDistance(const double *p1, const double *p2, const double *p3)
{
  const auto num = std::fabs((p2[1] - p1[1]) * p3[0] - \
                   (p2[0] - p1[0]) * p3[1] + \
                    p2[0] * p1[1] - p2[1] * p1[0]);

  const auto denom = std::sqrt(std::pow(p2[0] - p1[0], 2) + \
                               std::pow(p2[1] - p1[1], 2));

  return num / denom;
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

  // seed random generator
  std::srand(1);
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
    addEdge(v_near.x, v_near.y, v_new.x, v_new.y);

    // 5) goal reached
    double p1[] = {v_new.x, v_new.y};
    const auto d = distance(p1, goal_);

    if (d <= epsilon_)
    {
      std::cout << "Goal reached" << std::endl;

      // add goal to graph
      addVertex(goal_[0], goal_[1]);
      addEdge(v_new.x, v_new.y, goal_[0], goal_[1]);

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
    addVertex(v_new.x, v_new.y);
    addEdge(v_near.x, v_near.y, v_new.x, v_new.y);

    // 7) goal reached
    double p1[] = {v_new.x, v_new.y};
    const auto d = distance(p1, goal_);

    if (d <= epsilon_)
    {
      std::cout << "Goal reached" << std::endl;

      // add goal to graph
      addVertex(goal_[0], goal_[1]);
      addEdge(v_new.x, v_new.y, goal_[0], goal_[1]);

      break;
    }

    ctr++;
  }

  return true;

}


void RRT::randomCircles(int num_cirles, double r_min, double r_max)
{

  for(int i = 0; i < num_cirles; i++)
  {
    // circle center within bounds of world
    const auto x = xmin_+static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX/(xmax_-xmin_)));
    const auto y = xmin_+static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX/(xmax_-xmin_)));

    // radius between r_min and r_max;
    const auto r = r_min+static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX/(r_max-r_min)));

    const double center[] = {x, y};

    // make sure start and goal are not within an obstacle
    const auto d_init = distance(center, start_);
    const auto d_goal = distance(center, goal_);


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


void RRT::traverseGraph(std::vector<vertex> &path)
{
  // find start and goal vertices
  vertex s_v, g_v;
  if (!findVertex(start_, s_v) || !findVertex(goal_, g_v))
  {
    std::cout << "Start or Goal not found" << std::endl;
    return;
  }


  // path is backwards
  path.push_back(g_v);

  // current vertex is the goal
  vertex curr_v = g_v;


  while(!almost_equal(curr_v.x, s_v.x) && !almost_equal(curr_v.y, s_v.y))
  {
    vertex p;
    findParent(curr_v, p);

    path.push_back(p);

    curr_v = p;

    // std::cout << "[" << p.x << " " << p.y << "]" << std::endl;

  }
}




void RRT::addVertex(double x, double y)
{
  // check if already in graph;
  bool found = false;

  for(unsigned int i = 0; i < vertices_.size(); i++)
  {
    if (almost_equal(vertices_.at(i).x, x) && almost_equal(vertices_.at(i).y, y))
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
    if (almost_equal(vertices_.at(i).x, x1) && almost_equal(vertices_.at(i).y, y1))
    {
      for(unsigned int j = 0; j < vertices_.size(); j++)
      {
        // do not add vertex to itself
        // found node 2
        if(almost_equal(vertices_.at(j).x, x2) && almost_equal(vertices_.at(j).y, y2) && i != j)
        {
          // edge connecting node 1 to node 2
          Edge edge1;
          edge1.v = &vertices_.at(j);
          vertices_.at(i).Edges.push_back(edge1);

          // edge connecting node 2 to node 1
          // Edge edge2;
          // edge2.v = &vertices_.at(i);
          // vertices_.at(j).Edges.push_back(edge2);
        }
      } // end inner loop
    }
  } // end outer loop
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
    point[0] = vertices_.at(i).x;
    point[1] = vertices_.at(i).y;

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
  q_rand[0] = xmin_+static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX/(xmax_-xmin_)));

  // y position
  q_rand[1] = ymin_+static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX/(ymax_-ymin_)));
}



bool RRT::objectCollision(const vertex &v_new)
{
  double v[2] = {v_new.x, v_new.y};

  for(const auto &circle: circles_)
  {
    double c[2] = {circle.x, circle.y};
    const auto d = distance(c, v);

    if (d < circle.r + epsilon_)
    {
      return true;
    }
  }

  return false;
}


bool RRT::pathCollision(const vertex &v_new, const vertex &v_near)
{
  double vnew[2] = {v_new.x, v_new.y};
  double vnear[2] = {v_near.x, v_near.y};

  for(const auto &circle: circles_)
  {
    double c[2] = {circle.x, circle.y};
    const auto d = closestPointDistance(vnew, vnear, c);

    if (d < circle.r + epsilon_)
    {
      return true;
    }
  }

  return false;
}


bool RRT::findVertex(const double *p, vertex &v)
{
  for(unsigned int i = 0; i < vertices_.size(); i++)
  {
    if (almost_equal(vertices_.at(i).x, p[0]) && almost_equal(vertices_.at(i).y, p[1]))
    {
      v = vertices_.at(i);
      return true;
    }
  }

  std::cout << "Vertex not found" << std::endl;
  return false;
}



bool RRT::findParent(const vertex &v, vertex &parent)
{
  // iterate over vertices
  for(unsigned int i = 0; i < vertices_.size(); i++)
  {
    // iterate over children
    for(unsigned int j = 0; j < vertices_.at(i).Edges.size(); j++)
    {
      if (almost_equal(vertices_.at(i).Edges.at(j).v->x, v.x) && \
              almost_equal(vertices_.at(i).Edges.at(j).v->y, v.y))
      {
        parent = vertices_.at(i);
        return true;
      }
    } // end inner loop
  } // end outer loop

  std::cout << "Parent not found" << std::endl;
  return false;
}





























// end file








// end file
