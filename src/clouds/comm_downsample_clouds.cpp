#include <iostream>
#include <string>
#include "comm/comm.h"
#include "comm/comm_eigen.h"

#include "comm_pcl.h"
#include "comm_cv.h"
#include "filtering.h"
#include "utils/vector_io.h"
#include "utils/my_exceptions.h"
#include <boost/program_options.hpp>
#include <opencv2/imgproc/imgproc.hpp>


namespace po = boost::program_options;
using namespace std;

int main(int argc, char* argv[]) {

  string cloudTopic;
  string labelTopic;
  string outTopic;
  bool doPause;
  vector<int> labels;
  labels.push_back(1);
  float voxelSize;

  po::options_description opts("Allowed options");
  opts.add_options()
    ("help,h", "produce help message")
    ("cloudTopic,c", po::value< string >(&cloudTopic)->default_value("kinect"),"cloud topic")
    ("labelTopic,l", po::value< string >(&labelTopic),"label topic")
    ("outTopic,o", po::value< string >(&outTopic),"output topic")
    ("labels,n", po::value< vector<int> >(&labels)->multitoken(),"label to extract")
    ("voxelSize,v", po::value<float>(&voxelSize)->default_value(.01),"voxel side length (meters)");
  po::variables_map vm;        
  po::store(po::command_line_parser(argc, argv)
	    .options(opts)
	    .run()
	    , vm);
  if (vm.count("help")) {
    cout << "usage: comm_downsample_clouds [options]" << endl;
    cout << opts << endl;
    return 0;
  }
  po::notify(vm);


 
  initComm();

  CloudMessage cloudMsg;
  ImageMessage labelMsg;
  FileSubscriber cloudSub(cloudTopic,"pcd");
  FileSubscriber labelSub(labelTopic,"bmp");
  FilePublisher cloudPub(outTopic,"pcd");

  while (true) { 
    bool gotOne = cloudSub.recv(cloudMsg);
    if (!gotOne) break;

    ENSURE(labelSub.recv(labelMsg));

    cv::Mat& labelImg = labelMsg.m_data;
    cv::Mat mask = (labelImg == labels[0]);
    for (int i = 1; i < labels.size(); i++) mask += (labelImg == labels[i]);

    ColorCloudPtr maskedCloud = maskCloud(cloudMsg.m_data, mask);
    ColorCloudPtr downedCloud = downsampleCloud(maskedCloud, voxelSize);
    ColorCloudPtr finalCloud = removeOutliers(downedCloud);
    labelCloud(finalCloud, labelImg);

    cloudPub.send(CloudMessage(finalCloud));

  }


}
