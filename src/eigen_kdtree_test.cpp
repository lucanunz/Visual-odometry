#include "eigen_01_point_loading.h"
#include "eigen_kdtree.h"
#include <iostream>
#include <fstream>
#include <random>
using namespace std;

using ContainerType = std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >;
using TreeNodeType = TreeNode_<ContainerType::iterator>;

int main(int argc, char** argv) {
  // generate 1000 random points
  ContainerType kd_points(50);
  for (size_t i=0;i<kd_points.size();i++) {
    kd_points[i] = (Vector3f() << float(i),Vector2f::Random()*10).finished();
  }
  for(const auto& v : kd_points)
    std::cout << v.transpose() << std::endl; 
  
  std::cout << "tree construction " << std::endl;
  // construct a kd_tree, with leaf size 10
  // the construction reshuffles the items in the container
  TreeNodeType  kd_tree(kd_points.begin(), kd_points.end(), 10);

  std::cout << "tree ok" << std::endl;
  float ball_radius=0.1f;
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_int_distribution<std::mt19937::result_type> dist(1,100);
  ContainerType query_points(10);
  for (size_t i=0;i<query_points.size();i++) {
    if(dist(rng)>50)
      query_points[i] << (Vector3f() << float(dist(rng)),Vector2f::Random()*10).finished();
    else{
      std::cout << "adding: " << kd_points[i%10].tail<2>().transpose() << std::endl;
      query_points[i] <<  (Vector3f() << float(dist(rng)),kd_points[i%10].tail<2>()).finished();
    }
  }
  for(const auto& v : query_points)
    std::cout << v.transpose() << std::endl; 
  // we search each point in the input set. We need to find a match
  for (auto p: query_points) {
    TreeNodeType::AnswerType neighbors;
    Vector3f* match_fast=kd_tree.bestMatchFast(p, ball_radius);
    Vector3f* match_full=kd_tree.bestMatchFull(p, ball_radius);

    if (match_fast==match_full)
      if(match_fast)
        cout << "FAST Correct: " << p.transpose() <<", full: " << match_full->transpose() << endl;
      else
        cout << "FAST Correct, no match" << endl;
    else {
      cout << "FAST Not Correct: ";
      if (! match_fast) {
        cout << " NONE ";
      } else {
        cout << match_fast->transpose() << " ";
      }
      cout << " - ";
      if (! match_full) {
        cout << " NONE ";
      } else {
        cout << match_full->transpose() << " ";
      }
      cout << endl;
    }
  }
  cout << endl;


  
}



