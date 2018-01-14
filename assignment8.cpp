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





//The distance formula is d = sqrt((x_2-x_1)^2 + (y_2-y_1)^2)
Node RRT::NearestNeighbor(vector<Node> node_vector, Node input_node) {
  // cout << "node_vector.at(0) id and pid are: " << node_vector.at(0).id << " " << node_vector.at(0).parent_id << endl;
	Node closest_node = node_vector.at(0);
  // cout << "closest_node's id and parent id at start are: " << closest_node.id << " " << closest_node.parent_id << endl;
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
  // cout << "closest_node's id and parent id at end are: " << closest_node.id << " " << closest_node.parent_id << endl;
 	return closest_node;
}

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
  // cout <<"dy and dx are: " << dy << " " << dx << endl;
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
    // cout << "starting at: " << start_point << " and ending at: " << end_point << endl;
    for(y = start_point; y <= end_point; ++y){
      // cout << " current pixel is: (" << x << ", " << y << ")" << endl;
      if(this->image(x, y) == 0) {
        // cout << "vertical line collision!" << endl;
        return true;
      }
    }
  }
  else{
    // cout << "non vertical line!" << endl;
    float fdy = dy;
    float fdx = dx;
    slope = fdy/fdx;
    // cout << "the slope is: " << slope << endl;
    octant = FindOctant(slope, dx, dy);
    // cout << "start pixel: (" << x0 << ", " << y0 << "), end pixel: (" << x1 << ", " << y1 << ")" << endl;
    // cout << "current slope and octant: " << slope << " " << octant << endl;
    int start_x, end_x, start_y, end_y;
    start_x = x0;
    end_x = x1;
    start_y = y0;
    end_y = y1;
    SwitchOctantStart(octant, start_x, start_y);
    SwitchOctantStart(octant, end_x, end_y);
    dy = end_y-start_y;
    dx = end_x-start_x;
    // cout <<"dy and dx are: " << dy << " " << dx << endl;
    D = 2*dy - dx;

    for(x = x0, y = y0; start_x <= end_x; ++start_x){
      // cout << " proper pixel is: (" << x << ", " << y << ")" << endl;
      if(this->image(x, y) == 0) {
        // cout << "there is a collision!" << endl;
        return true;
      }
      SwitchOctantStart(octant, x, y);
      // cout << " bresenham pixel is: (" << x << ", " << y << ")" << endl;

      // cout << "D = " << D << endl;
      if(D >= 0) {
        ++y;
        D = D - 2*dx;
      }
      ++x;
      D = D + 2*dy;
      SwitchOctantEnd(octant, x , y);
    }

 }
  // cout << "no collision found!" << endl;
  return false;
}

  

vector<Node> RRT::FindPath(Node start, Node end, int goal_radius, int extension_distance) {
  // cimg_library::CImg<unsigned char> new_image = this->image;
  // cimg_library::CImgDisplay display(new_image);
  vector<Node> existing_nodes; 
  vector<Node> final_path;
  Node new_node(0,0);
  int index = 0;
  start.id = index;
  start.parent_id = -1;
  existing_nodes.push_back(start);
  Node nearest_node = start;
  // cout << nearest_node.id << " " << nearest_node.parent_id << endl;
  int dist = Dist(start, end);
  while(dist > goal_radius){
    int x_rand = rand() % image.width();
    int y_rand = rand() % image.height();
    Node rand_sample(x_rand, y_rand);
    nearest_node = NearestNeighbor(existing_nodes, rand_sample);
  // cout << nearest_node.id << " " << nearest_node.parent_id << endl;

    if(Dist(rand_sample,nearest_node) > extension_distance){
      new_node = Project(rand_sample, nearest_node, extension_distance);
      // float test = Dist(nearest_node, new_node);
      // cout << test << endl;
      if(!(CheckCollision(nearest_node, new_node))){
        ++index;
        // cout << "current index is: " << index << endl;
        new_node.parent_id = nearest_node.id;
        // cout << "id of nearest_node: " << nearest_node.id << endl;
        new_node.id = index;
        existing_nodes.push_back(new_node);
        dist = Dist(new_node, end);
        // const unsigned char color[] = {200};
        // const unsigned char dColor[] = {100};
        // int x = new_node.x;
        // int y = new_node.y;
        // new_image.draw_circle(x,y,3,color);
        // if(nearest_node.parent_id != -1) {
        //   int x1 = nearest_node.x;
        //   int y1 = nearest_node.y;
        //   new_image.draw_line(x, y, x1, y1, dColor).display(display);
        // }

      }
    }
  } 
  // cout << "finished creating the node vector" << endl;
  while(new_node.parent_id != -1){
    // cout << "current node id is: " << new_node.id << endl;
    // cout << "current parent id is: " << new_node.parent_id << endl;
    final_path.push_back(new_node);
    new_node = existing_nodes.at(new_node.parent_id);
  }
  final_path.push_back(start);
  DrawRRT(existing_nodes);
  // DrawPath(final_path);
  return final_path;
}

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
