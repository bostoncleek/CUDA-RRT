#ifndef RRT_INCLUDE_GUARD_HPP
#define RRT_INCLUDE_GUARD_HPP

#include <cmath>
#include <vector>


struct vertex;


struct Circle
{
  double x = 0.0;
  double y = 0.0;
  double r = 0.0;
};


/// \brief edges connecting nodes
struct Edge
{
    vertex *v = nullptr;
};


/// \brief nodes in the graph
struct vertex
{
    bool visited = false;
    double x = 0.0;
    double y = 0.0;
    std::vector<Edge> Edges; // adjacent vertices
};


/// \brief approximately compare two floating-point numbers using
///        an absolute comparison
/// \param d1 - a number to compare
/// \param d2 - a second number to compare
/// \param epsilon - absolute threshold required for equality
/// \return true if abs(d1 - d2) < epsilon
constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-6)
{
  return std::fabs(d1 - d2) < epsilon ? true : false;
}


/// \brief Distance between vertices
/// \param p1 - point 1
/// \param p2 - point 2
double distance(const double *p1, const double *p2);

/// \brief Finds the closest point on a line (p2 to p1) to p3
/// \param p1 - point 1 connected to point 2
/// \param p2 - point 2 connected to point 1
/// \param p3 - center point of circle
/// \returns - distance between circle and the closest point on the line
double closestPointDistance(const double *p1, const double *p2, const double *p3);



/// \brief RRT search in continous space
class RRT
{
public:
  /// \brief Constructs RRT search
  RRT(double *start, double *goal);


  /// \brief RRT from start to goal with no obstacles
  /// \returns true if goal reached
  bool explore();


  /// \brief RRT from start to goal with with obstacles
  /// \returns true if goal reached
  bool exploreObstacles();


  /// \bried Generate random circles
  /// \param num_cirles - number of circles
  /// \param r_min - min radius
  /// \param r_max - max radius
  void randomCircles(int num_cirles, double r_min, double r_max);



  void traverseGraph(std::vector<vertex> &path);


  void printGraph();



private:
  /// \brief Adds new nodes to graph
  /// \param x - x position
  /// \param y - y position
  void addVertex(double x, double y);

  /// \brief Adds Edge btw two nodes
  /// \param x1 - x position of node 1
  /// \param y1 - y position of node 1
  /// \param x2 - x position of node 2
  /// \param y2 - y position of node 2
  void addEdge(double x1, double y1, double x2, double y2);

  /// \brief Creates a new vertex by moveing a delta away from
  ///        the random point selected in the world
  /// \param v_near - nearest vertex to random point
  /// \param q_rand - random point
  /// \return true if the new vertex is created
  /// v_new[out] - newly created vertex
  bool newConfiguration(vertex &v_new,
                        const vertex &v_near,
                        const double *q_rand) const;

  /// \brief Find the nearest vertex in graph
  /// \param q_rand - random point
  /// \return nearest vertex
  vertex &nearestVertex(double *q_rand);

  /// \brief Creates a random point in the world
  /// q_rand[out] x and y coordinates of point
  void randomConfig(double *q_rand) const;

  /// \brief Test whether the new vertex collides with an obstacle
  /// \param v_new - potential new vertex to add to graph
  /// \returns true if collision between vertex and an obstacle
  bool objectCollision(const vertex &v_new);

  /// \brief Test whether the new edge collides with an obstacle
  /// \param v_new - potential new vertex to add to graph
  /// \param v_near - closest vertex in graph to v_new
  /// \returns true if collision between edge and an obstacle
  bool pathCollision(const vertex &v_new, const vertex &v_near);


  /// \brief - Finds vertex in graph
  /// \param p - coordinates of vertex searching for
  /// \param v - the found vertex
  /// \returns - true if vertex found
  int findVertex(const double *p);


  /// \brief - Finds parent vertex in graph
  /// \param v - the child vertex
  /// \param parent - the parent vertex
  /// \returns - true if parent vertex found
  int findParent(const vertex &v);



  double *start_;                                 // start config
  double *goal_;                                  // goal config
  double delta_;                                  // distance to place new node
  double epsilon_;                                // away from goal and obstacles
  double xmin_, xmax_, ymin_, ymax_;             // world bounds
  int max_iter_;                                  // max iterations
  std::vector<vertex> vertices_;                   // all nodes in graph

  std::vector<Circle> circles_;


};










#endif
