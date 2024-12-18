#include<tocg/extract_static_env.h>
#include<tocg/chicken_game_with_dynamic_veh.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "extract_static_env");
  ros::NodeHandle nodeHandle("");
  ros::NodeHandle privateHandle("~");
  ExtractEnv ee(nodeHandle, privateHandle);
  ChickenGame cg;
  ee.initializeExt();
  // 打开while循环和pubsher循环
  double updateRate_ = 25.0;
  privateHandle.param<double>("update_rate", updateRate_, 25.0);
  bool is_extracted = false;
  ros::Rate r(updateRate_);
  int i=0;
  while (ros::ok()) {
    if (ee.isCompleted() && (!is_extracted)){
      // std::cout<<1<<std::endl;
      cg.initialize(nodeHandle, privateHandle, ee);
      // std::cout<<2<<std::endl;
      cg.extractGaps();
      // std::cout<<3<<std::endl;
      cg.combineGaps();
      // std::cout<<4<<std::endl;
      is_extracted = true;
    }
    if (ee.isCompleted() && is_extracted){
      // std::cout<<5<<std::endl;
      ee.pubBorderlines();
      // std::cout<<6<<std::endl;
      ee.pubIntermediateFootprints();      
      // std::cout<<7<<std::endl;
      ee.pubBorderInformationForTEB();
      // std::cout<<8<<std::endl;
      cg.pubSegmentArea();
      // std::cout<<9<<std::endl;
    }
    ros::spinOnce();
    r.sleep();
  }
}