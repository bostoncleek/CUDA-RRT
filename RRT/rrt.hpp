#ifndef RRT_INCLUDE_GUARD_HPP
#define RRT_INCLUDE_GUARD_HPP

#include <cmath>
#include <vector>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

struct vertex;


struct Circle
{
  double x;
  double y;
  double r;

  Circle() : x(0.0), y(0.0), r(0.0) {}
};


// /// \brief edges connecting nodes
// struct Edge
// {
//     vertex *v = nullptr;
// };


/// \brief nodes in the graph
struct vertex
{
    int id;
    double x;
    double y;
    // std::vector<Edge> Edges; // adjacent vertices
    std::vector<int> adjacent_vertices;

    vertex() : id(-1), x(0.0), y(0.0) {}
    vertex(double x, double y): id(-1), x(x), y(y) {}
};


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
  RRT(double *start, double *goal, int rando);

  /// \brief RRT from start to goal with no obstacles
  /// \returns true if goal reached
  bool explore();

  /// \brief RRT from start to goal with with obstacles
  /// \returns true if goal reached
  bool exploreObstacles();

  /// \brief RRT from start to goal with with obstacles
  bool exploreCuda();

  /// \brief fills arrays with the obstacle data
  /// \param h_x - host array with circle's x position
  /// \param h_y - host array with circle's y position
  /// \param h_r - host array with circle's radius
  void circleData(float *h_x, float *h_y, float *h_r);

  /// \bried Generate random circles
  /// \param num_cirles - number of circles
  /// \param r_min - min radius
  /// \param r_max - max radius
  void randomCircles(int num_cirles, double r_min, double r_max);

  /// \brief Traverse graph to get path
  /// \param path - the vector of vertex from start to goal
  void traverseGraph(std::vector<vertex> &path) const;

  /// \brief print the entire graph
  void printGraph() const;

  /// \brief write obsctacles and graph to csvs for visualization
  void visualizeGraph() const;

private:
  /// \brief Adds new nodes to graph
  /// \param x - x position
  /// \param y - y position
  void addVertex(vertex &v);

  /// \brief Adds Edge btw two nodes
  /// \param v_near- parent
  /// \param v_new - child
  void addEdge(const vertex &v_near, const vertex &v_new);

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
  /// v[out] - nearest vertex
  void nearestVertex(vertex &v, double *q_rand) const;

  /// \brief Creates a random point in the world
  /// q_rand[out] x and y coordinates of point
  void randomConfig(double *q_rand) const;

  /// \brief - Finds parent vertex in graph
  /// \param v - the child vertex
  /// \returns - index of parent
  int findParent(const vertex &v) const;

  /// \brief - Checks if there is a straightline path between node and goal
  /// \param vnew - newly added vertex
  /// \param goal - goal location
  /// \returns true if straightline path to goal false if obstructed
  bool win_check(const vertex &v_new, const double *goal);

  /// \brief Test whether the new vertex would collide with an obstacle
  ///     or if the path to the new vertex intersects with an obstacle
  /// \param v_new - potential new vertex to add to graph
  /// \param v_near - closest vertex in graph to v_new
  /// \returns true if collision between edge and an obstacle
  bool collision_check(const vertex &v_new, const vertex &v_near);


  double *start_;                                 // start config
  double *goal_;                                  // goal config
  double delta_;                                  // distance to place new node
  double epsilon_;                                // away from goal and obstacles
  double xmin_, xmax_, ymin_, ymax_;             // world bounds
  int max_iter_;                                  // max iterations
  int vertex_count_;                              // counts which vertex
  std::vector<vertex> vertices_;                   // all nodes in graph

  std::vector<Circle> circles_;


};


#define TIME_IT(ROUTINE_NAME__, LOOPS__, ACTION__)\
{\
    printf("    Timing '%s' started\n", ROUTINE_NAME__);\
    struct timeval tv;\
    struct timezone tz;\
    const clock_t startTime = clock();\
    gettimeofday(&tv, &tz); long GTODStartTime =  tv.tv_sec * 1000 + tv.tv_usec / 1000 ;\
    for (int loops = 0; loops < (LOOPS__); ++loops)\
    {\
        ACTION__;\
    }\
    gettimeofday(&tv, &tz); long GTODEndTime =  tv.tv_sec * 1000 + tv.tv_usec / 1000 ;\
    const clock_t endTime = clock();\
    const clock_t elapsedTime = endTime - startTime;\
    const double timeInSeconds = (elapsedTime/(double)CLOCKS_PER_SEC);\
    printf("        GetTimeOfDay Time (for %d iterations) = %g\n", LOOPS__, (double)(GTODEndTime - GTODStartTime) / 1000. );\
    printf("        Clock Time        (for %d iterations) = %g\n", LOOPS__, timeInSeconds );\
    printf("    Timing '%s' ended\n", ROUTINE_NAME__);\
}







#endif
