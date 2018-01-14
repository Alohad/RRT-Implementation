#include <gtest/gtest.h>
#include "assignment8.h"
#include <algorithm>
#include <iostream>
#include "CImg.h"


using std::cout;
using std::endl;
using std::vector;

int main(int argc, char * argv[]) {
	cimg_library::CImg<unsigned char> image;
    image.load("MediumMap.png");
    RRT test(image);
    int goal_radius = 10;
    int extension_distance = 10;
    Node start(9,9);
    Node end(250,250);
    bool test_collision = test.CheckCollision(start,end);
    vector<Node> help = test.FindPath(start,end,goal_radius,extension_distance);

}
