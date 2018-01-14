#include <CImg.h>
using std::vector;
struct Node {
  // X,Y coordinates of node in occupancy grid.
  int x;
  int y;
  int id;
  int parent_id;

  Node() = delete;
  // Initialization constructor.
  Node(const int& x,
       const int& y) :
       x(x),
       y(y) {}
};

class RRT {
public:
  cimg_library::CImg<unsigned char> image;

  RRT() = delete;
  // Initialization constructor.
  RRT(const cimg_library::CImg<unsigned char>& image) :
      image(image) {}
  Node NearestNeighbor(std::vector<Node> node_vector, Node input_node);
  bool CheckCollision(Node start, Node end);
  vector<Node> FindPath(Node start, Node end, int goal_radius, int extension_distance);
  float Dist(Node Node1, Node Node2);
  int FindOctant(float slope, int x_diff, int y_diff);
  void SwitchOctantStart(int octant, int &x, int &y);
  void SwitchOctantEnd(int octant, int &x, int &y);
  Node Project(Node start, Node end, int extension_distance);
  void DrawRRT(vector<Node> existing_nodes);
  void DrawPath(vector<Node> Path);

};