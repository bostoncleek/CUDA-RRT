#ifndef RRT_INCLUDE_GUARD_HPP
#define RRT_INCLUDE_GUARD_HPP

#include <cmath>
#include <vector>


struct vertex;


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


/// \brief Distance between vertices
/// \param p1 - point 1
/// \param p2 - point 2
double distance(const double *p1, const double *p2);



/// \brief RRT search in continous space
class RRT
{
public:
  /// \brief Constructs RRT search
  RRT(double *start, double *goal);

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


  bool explore();



private:
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

  double *start_;                                 // start config
  double *goal_;                                  // goal config
  double delta_;                                  // distance to place new node
  double epsilon_;                                // away from goal
  double xmin_, xmax_, ymin_, ymax_;             // world bounds
  int max_iter_;                                  // max iterations
  std::vector<vertex> vertices_;                   // all nodes in graph


};










#endif
