#include "assignment8.h"
#include <math.h>
#include <algorithm>
#include <stdlib.h>
#include <iostream>
#include <CImg.h>

using cimg_library::CImg;
using std::vector;
using std::cout;
using std::endl;

//Finding the distance between two nodes
float RRT::Dist(Node Node1, Node Node2){
  float x_1, x_2, y_1, y_2, x_square, y_square, dist, min_dist, size;
  x_1 = Node1.x;
  y_1 = Node1.y;
  x_2 = Node2.x;
  y_2 = Node2.y;
  x_square = pow(x_2-x_1, 2);
  y_square = pow(y_2-y_1, 2);
  dist = sqrt(x_square + y_square);
  return dist;    
  }

//Finding the octant of the slope pbetween two points
int RRT::FindOctant(float slope, int x_diff, int y_diff){
  if(0 <= slope && slope <= 1) {
    if(x_diff >= 0) {return 0;}
    else {return 4;}
  }
  if(slope > 1) {
    if(x_diff >= 0) {return 1;}
    else {return 5;}
  }
  if(slope < -1) {
    if(x_diff <= 0) {return 2;}
    else {return 6;}
  }
  if(slope < 0 && slope >= -1) {
    if(x_diff <= 0) {return 3;}
    else {return 7;}
  }
  return -1;
} 


//Finding the nearest existing neighbor in node_vector to input_node
Node RRT::NearestNeighbor(vector<Node> node_vector, Node input_node) {
	Node closest_node = node_vector.at(0);
  int min_dist = 999999; //arbitrary large number for initialization
 	int size = node_vector.size();
  int dist;
 	for(int i = 0; i < size; ++i) {
     Node cur_node = node_vector.at(i);
 		dist = Dist(input_node, cur_node);
 		if(dist < min_dist) {
 			min_dist = dist;
 			closest_node = node_vector.at(i);
 		}
 	}
 	return closest_node;
}

//Finding if there is a wall between the two given nodes
bool RRT::CheckCollision(Node start, Node end) {
 	/*
 	// begin by drawing a line of nodes with proper x and y values
 	// using the Bresenham Line Algorithm, then proceed to check each
 	// node on that line to see if it is black on the map. if it's never
 	// black then return true, else return false.
 	*/
  int x0, x1, y0, y1, dx, dy, D, x, y, octant;
  float slope;
  x0 = start.x;
  y0 = start.y;
  x1 = end.x;
  y1 = end.y;
  dy = y1-y0;
  dx = x1-x0;
  if(dx == 0) {
    x = x0;
    int start_point, end_point;
    if(dy > 0) {
      start_point = y0;
      end_point = y1;
    }
    else{
      start_point = y1;
      end_point = y0;
    }
    for(y = start_point; y <= end_point; ++y){
      if(this->image(x, y) == 0) {
        return true;
      }
    }
  }
  else{
    float fdy = dy;
    float fdx = dx;
    slope = fdy/fdx;
    octant = FindOctant(slope, dx, dy);
    int start_x, end_x, start_y, end_y;
    start_x = x0;
    end_x = x1;
    start_y = y0;
    end_y = y1;
    SwitchOctantStart(octant, start_x, start_y);
    SwitchOctantStart(octant, end_x, end_y);
    dy = end_y-start_y;
    dx = end_x-start_x;
    D = 2*dy - dx;

    for(x = x0, y = y0; start_x <= end_x; ++start_x){
      if(this->image(x, y) == 0) {
        return true;
      }
      SwitchOctantStart(octant, x, y);
      if(D >= 0) {
        ++y;
        D = D - 2*dx;
      }
      ++x;
      D = D + 2*dy;
      SwitchOctantEnd(octant, x , y);
    }

 }
  return false;
}

  
//Finds a path between two nodes given the locations of the nodes, how close it needs to get to the end node,
//and how far each intermediate node can travel
vector<Node> RRT::FindPath(Node start, Node end, int goal_radius, int extension_distance) {
  vector<Node> existing_nodes; 
  vector<Node> final_path;
  Node new_node(0,0);
  int index = 0;
  start.id = index;
  start.parent_id = -1;
  existing_nodes.push_back(start);
  Node nearest_node = start;
  int dist = Dist(start, end);
  while(dist > goal_radius){
    int x_rand = rand() % image.width();
    int y_rand = rand() % image.height();
    Node rand_sample(x_rand, y_rand);
    nearest_node = NearestNeighbor(existing_nodes, rand_sample);

    if(Dist(rand_sample,nearest_node) > extension_distance){
      new_node = Project(rand_sample, nearest_node, extension_distance);
      if(!(CheckCollision(nearest_node, new_node))){
        ++index;
        new_node.parent_id = nearest_node.id;
        new_node.id = index;
        existing_nodes.push_back(new_node);
        dist = Dist(new_node, end);

      }
    }
  } 
  while(new_node.parent_id != -1){
    final_path.push_back(new_node);
    new_node = existing_nodes.at(new_node.parent_id);
  }
  final_path.push_back(start);
  DrawRRT(existing_nodes);
  return final_path;
}

//Placing a new node a set distance away from the root node
Node RRT::Project(Node project, Node root, int radius) {
  int x, y, dx, dy;
  dx = project.x - root.x;
  dy = project.y - root.y;
  float sin_ = dy / Dist(project, root);
  float cos_ = dx / Dist(project, root);
  x = root.x + cos_ * radius;
  y = root.y + sin_ * radius;
  Node new_node(x,y);
  return new_node;
}

//Changing the octant of the slope in order to use Bresenham's Line Algorithm
void RRT::SwitchOctantStart(int octant, int &x, int &y){
  int temp;
  switch(octant){ 
    case 0:
      break;

    case 1:
      temp = x;
      x = y;
      y = temp;
      break;

    case 2:
      temp = x;
      x = y;
      y = -1 * temp;
      break;

    case 3: 
      x = x * -1;
      break;
    
    case 4:
      x = x * -1;
      y = y * -1;
      break;
      
    case 5:
      temp = x;
      x = y * -1;
      y = temp * -1;
      break;
      
    case 6: 
      temp = x;
      x = -1 * y;
      y = temp;
      break;

    case 7:
      y = -1 * y;
      break;
    }
}
//Switching back to original octant
void RRT::SwitchOctantEnd(int octant, int &x, int &y){
  int temp;
  switch(octant) { 
    case 0:
      break;

    case 1:
      temp = x;
      x = y;
      y = temp;
      break;

    case 2:
      temp = x;
      x = -1 * y;
      y = temp;
      break;

    case 3: 
      x = x * -1;
      break;
      
    case 4:
      x = x * -1;
      y = y * -1;
      break;
      
    case 5:
      temp = x;
      x = y * -1;
      y = temp * -1;
      break;
      
    case 6: 

      temp = x;
      x = y;
      y = -1 * temp;
      break;

    case 7:
      y = -1 * y;
      break;
    }
}
//Viewing the completed path using CImg class
void RRT::DrawRRT(vector<Node> existing_nodes){
  const unsigned char color[] = {200};
  const unsigned char dColor[] = {100};
  for(int i = 0; i < existing_nodes.size(); ++i) {
    int x = existing_nodes.at(i).x;
    int y = existing_nodes.at(i).y;
    this->image.draw_circle(x,y,3,color);
    if(existing_nodes.at(i).parent_id != -1) {
      int pid = existing_nodes.at(i).parent_id;
      int x1 = existing_nodes.at(pid).x;
      int y1 = existing_nodes.at(pid).y;
      this->image.draw_line(x, y, x1, y1, dColor);

    }
  }
  cimg_library::CImgDisplay display(image);
  while(!display.is_closed()){
    display.wait();
  }
}

//Viewing the path as it gets created
void RRT::DrawPath(vector<Node> Path){
  const unsigned char color[] = {200};
  const unsigned char dColor[] = {100};
  for(int i = 0; i < Path.size(); ++i) {
    int x = Path.at(i).x;
    int y = Path.at(i).y;
    this->image.draw_circle(x,y,3,color);
    if(i != 0) {
      int x1 = Path.at(i-1).x;
      int y1 = Path.at(i-1).y;
      this->image.draw_line(x,y,x1,y1,dColor);
    }
  }
  cimg_library::CImgDisplay display(image);
  while(!display.is_closed()) {
    display.wait();
  }
}
