// Offline bridge to upstream BSD-licensed ROS leg cluster features/classifier.
// Upstream implementation/weights are supplied explicitly; no ROS graph is used.
#include <iostream>
#include <sstream>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <leg_detector/cluster_features.h>
int main(int argc,char**argv) {
  if(argc<2 || argc>3) return 2;
  double threshold=argc==3?std::stod(argv[2]):-1.;
  auto forest=cv::ml::RTrees::load(argv[1]);
  ClusterFeatures features;
  std::string line;
  while(std::getline(std::cin,line)) {
    std::istringstream input(line);
    sensor_msgs::msg::LaserScan scan;
    size_t count;
    if(!(input>>count>>scan.angle_min>>scan.angle_increment>>scan.range_min>>scan.range_max)) return 3;
    scan.ranges.resize(count);
    for(auto &r:scan.ranges) if(!(input>>r)) return 4;
    laser_processor::ScanProcessor processor(scan);
    processor.splitConnected(.13f);processor.removeLessThan(3);
    std::ostringstream result;size_t found=0;
    for(auto cluster:processor.getClusters()) {
      auto values=features.calcClusterFeatures(cluster,scan);
      auto p=cluster->getPosition();
      if(std::hypot(p.x,p.y)>=10. || values.size()!=static_cast<size_t>(forest->getVarCount())) continue;
      cv::Mat sample(1,values.size(),CV_32F,values.data()),votes;
      forest->getVotes(sample,votes,0);
      double positive=0.,total=0.;
      for(int c=0;c<votes.cols;++c) {double n=votes.at<int>(1,c); total+=n; if(votes.at<int>(0,c)>0) positive+=n;}
      double score=total>0?positive/total:0.;
      if(score>threshold) {result<<' '<<p.x<<' '<<p.y; ++found;}
    }
    std::cout<<found<<result.str()<<std::endl;
  }
}
