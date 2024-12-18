#include <tocg/chicken_game_with_dynamic_veh.h>

/** 
 * 假设自车沿着x轴正方向行驶
 * **/
int convertSign(const double& input){
  if (input>0) return 1;
  else if (input<0) return -1;
  return 0;
}

bool compareGap(Gap g1, Gap g2){
  return g1.x_start<g2.x_start;
}

bool comparePotentialGap(potentialGap g1, potentialGap g2){
  return g1.distance<g2.distance;
}

void ChickenGame::initialize(const ros::NodeHandle& publicHandle, const ros::NodeHandle& privateHandle, const ExtractEnv env){
  nh_ = publicHandle; 
  privatenh_ = privateHandle; 
  env_ = env;
  nh_.param<std::string>("frame_id", frame_id_, "odom");
  isAvInitialized_ = false;
  isAVGoalRecivied_ = false;

  sub_dynamic_social_vehicles_= nh_.subscribe<vehicle_simulator::SocialVehicles>("/simulate_social_car/dynamic_sv_states", 3, boost::bind(&ChickenGame::dynamicVehCB, this ,_1));
  sub_automated_vehicle_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 3, boost::bind(&ChickenGame::odomCB, this ,_1));
  sub_automated_vehicle_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 3, boost::bind(&ChickenGame::goalCB, this ,_1));

  pub_paths_for_show_ = privatenh_.advertise<visualization_msgs::MarkerArray>("paths_for_show",3);
  pub_path_for_teb_ = privatenh_.advertise<visualization_msgs::Marker>("paths_for_teb",3);
  pub_states_right_ = privatenh_.advertise<visualization_msgs::Marker>("states_right",3);
  pub_states_right_pure_ = privatenh_.advertise<visualization_msgs::Marker>("states_right_pure",3);
  pub_states_left_ = privatenh_.advertise<visualization_msgs::Marker>("states_left",3);
  pub_states_back_ = privatenh_.advertise<visualization_msgs::Marker>("states_back",3);
  pub_states_cut_ = privatenh_.advertise<visualization_msgs::Marker>("states_cut",3);
  pub_gaps_ = privatenh_.advertise<visualization_msgs::Marker>("gaps",3);
  pub_meeting_gap_ = privatenh_.advertise<visualization_msgs::Marker>("meeting_gaps",3);
  pub_potential_gaps_cuts_and_backs_ = privatenh_.advertise<visualization_msgs::Marker>("potential_gaps_cuts_and_backs",3);
  pub_potential_ends_ = privatenh_.advertise<visualization_msgs::MarkerArray>("potential_end_points",3);
}

void ChickenGame::dynamicVehCB(const vehicle_simulator::SocialVehicles::ConstPtr& msg){ 
  // 必须保证自车的odom和goal是有的
  if (!isAvInitialized_)
    return;
  if (!isAVGoalRecivied_)
    return;

  float footprint_supplement_rear = 1.5 * abs(env_.y_negative_footprint_[0].first + env_.y_positive_footprint_[0].first); // 定值 用于补充两个车屁股，在即将完成会车时 让自车多等一会会
  float footprint_supplement_front = 1.5 * abs(env_.y_negative_footprint_[2].first + env_.y_positive_footprint_[2].first);  // 定值 用于补充两个车头，在即将完成会车时 让自车多等一会会

  static int last_oncoming_vehs_num = -1;
  static std::vector<std::vector<double>> selected_path;
  static std::string selected_type;
  // 找离自车最近的follower和oncoming
  auto automated_vehicle_odom = automated_vehicle_odom_;
  
  double followed_vehs_num_ = 0;
  bool is_exist_followed_veh = false;
  nav_msgs::Odometry nearest_follower; nearest_follower.pose.pose.position.x = -999;

  double oncoming_vehs_num_ = 0;
  bool is_exist_oncoming_veh = false;
  nav_msgs::Odometry nearest_oncoming; nearest_oncoming.pose.pose.position.x = 999;
  
  for (auto veh:msg->vehicles){
    double theta = tf::getYaw(veh.pose.pose.orientation);
    while (theta>M_PI) theta-=M_PI;
    while (theta<-M_PI) theta+=M_PI;
    double x = veh.pose.pose.position.x;
    // horizon m内后续有跟车
    float horizon = 25;
    if ( x<automated_vehicle_odom.pose.pose.position.x && automated_vehicle_odom.pose.pose.position.x-x<horizon && (theta>-M_PI_2 && theta<M_PI_2) ){
      followed_vehs_num_ ++;
      is_exist_followed_veh = true;
      if (x>nearest_follower.pose.pose.position.x)
        nearest_follower = veh;
    }
    if ( x>automated_vehicle_odom.pose.pose.position.x-footprint_supplement_rear && x-automated_vehicle_odom.pose.pose.position.x<horizon && (theta>M_PI_2 || theta<-M_PI_2) ){
      oncoming_vehs_num_ ++;
      is_exist_oncoming_veh = true;
      if (x<nearest_oncoming.pose.pose.position.x)
        nearest_oncoming = veh;
    }
  }

  // std::cout<<"------ "<<std::endl;
  // std::cout<<"("<<nearest_follower.pose.pose.position.x<<","<<nearest_follower.pose.pose.position.y<<") "<<followed_vehs_num_<<std::endl;
  // std::cout<<"("<<nearest_oncoming.pose.pose.position.x<<","<<nearest_oncoming.pose.pose.position.y<<") "<<oncoming_vehs_num_<<std::endl;

  // 状态机 (不考虑follow车会卡自车的情况)
  double automated_vehicle_rear_x = automated_vehicle_odom.pose.pose.position.x;
  double oncoming_vehcle_rear_x = nearest_oncoming.pose.pose.position.x;
  double automated_vehicle_rear_v = automated_vehicle_odom.twist.twist.linear.x * cos(tf::getYaw(automated_vehicle_odom.pose.pose.orientation));
  double oncoming_vehicle_rear_v = nearest_oncoming.twist.twist.linear.x * cos(tf::getYaw(nearest_oncoming.pose.pose.orientation));
  double overal_time = 0;

  static std::string decision_state;   // 状态
  static std::string distribution;    // gap distribution
  static std::string game_res;         // game result
  static potentialGap final_gap;    // gap
  static std::string ref_path_type = "";

  // 判断是否完成了一次会车 is_oncoming_veh_changed
  bool is_oncoming_veh_changed = false;
  if (last_oncoming_vehs_num!=oncoming_vehs_num_)
    is_oncoming_veh_changed = true;
  last_oncoming_vehs_num = oncoming_vehs_num_;
  // 判断是否到达局部目标点
  bool is_reach_local_goal = false;
  if (!selected_path.empty() && hypot(selected_path.back()[0]-automated_vehicle_odom.pose.pose.position.x, selected_path.back()[1]-automated_vehicle_odom.pose.pose.position.y)<0.02)
    is_reach_local_goal = true;
  // 判断是否在backUp中 或 cutIn中
  bool is_in_backUp = false;
  if (final_gap.is_init && selected_type=="backUp" && automated_vehicle_rear_x<gaps_[final_gap.gap_index].x_back_switching+0.05)
    is_in_backUp = true;
  bool is_in_cutIn = false;
  if (final_gap.is_init && selected_type=="cutIn" && automated_vehicle_rear_x>gaps_[final_gap.gap_index].x_cut_switching-0.15)
    is_in_cutIn = true;

  // 是否处于 危险情况的处理中
  bool is_in_processing = false;
  std::vector<double> normal_considering_dynamic_veh;

  std::cout<<("---------------------")<<std::endl;
  // 状态机：
  // 1. 如果没有oncoming vehicle，av则正常行驶，直接找局部目标点就好
  if (!is_exist_oncoming_veh){
    decision_state = "normal";
    ROS_INFO("Normally forward as no oncoming vehicle");
  }
  // 2. 如果有oncoming vehicles: 
  else if (is_exist_oncoming_veh){
    // 2.1 oncoming vehicle 倒车
    if (oncoming_vehicle_rear_v>0){
      decision_state = "normal";
      geometry_msgs::Point p;
      p.x = nearest_oncoming.pose.pose.position.x;
      p.y = nearest_oncoming.pose.pose.position.y;
      p.z = tf::getYaw(nearest_oncoming.pose.pose.orientation);
      normal_considering_dynamic_veh = env_.addDynamicVehToBorerlineAndGainMiddlePath(p, env_.y_positive_footprint_, oncoming_vehcle_rear_x-footprint_supplement_front);
      if (normal_considering_dynamic_veh.size()==env_.down_borderline_.size())
        ROS_INFO("Normally forward as oncoming vehicle is backing, can detour");
      else
        ROS_INFO("Normally forward as oncoming vehicle is backing, parking before the oncoming veh");
    }
    // 2.2 oncoming vehicle 静止，且停在路边 (todo 这里的“”停在路边“”只是近似考虑了不在主路 0.15，需要更深的思考)
    else if (oncoming_vehicle_rear_v==0 && abs(nearest_oncoming.pose.pose.position.y)>0.15){
      decision_state = "normal";
      geometry_msgs::Point p;
      p.x = nearest_oncoming.pose.pose.position.x;
      p.y = nearest_oncoming.pose.pose.position.y;
      p.z = tf::getYaw(nearest_oncoming.pose.pose.orientation);
      normal_considering_dynamic_veh = env_.addDynamicVehToBorerlineAndGainMiddlePath(p, env_.y_positive_footprint_, oncoming_vehcle_rear_x-footprint_supplement_front);
      ROS_INFO("Normally forward as oncoming vehicle parking near the road");
    }
    // 2.3 oncoming vehicle 静止，且oncoming veh在自车的目标点之外
    else if (oncoming_vehicle_rear_v==0 && automated_vehicle_goal_.pose.position.x < nearest_oncoming.pose.pose.position.x-footprint_supplement_front){
      decision_state = "normal";
      ROS_INFO("Normally forward as oncoming vehicle parking out of the goal");
    }
    // 2.4 oncoming_vehicle_rear_v<0 或 oncoming_veh停在路中间
    else{
      // 危险情况的处理中：在会车还没完成 且 oncoming车辆处于v<=0的情况下: av和oncomign veh离的很近，或av速度<0, 或当av到达局部目标点，或处于backUp的倒车阶段，或处于cutIn的插入阶段，则当前是in_processing中，不会改变gap
      if (!is_oncoming_veh_changed && oncoming_vehicle_rear_v<=0){
        if (oncoming_vehcle_rear_x <= automated_vehicle_rear_x+footprint_supplement_front && oncoming_vehcle_rear_x >= automated_vehicle_rear_x-footprint_supplement_rear)
          is_in_processing = true;
        if (automated_vehicle_rear_v<0)  // 这里如果automated_vehicle_rear_v<0 要么是处于back中，要么是处于here的back中，要么是forward或者here时抽风中，只需要保持原决策就好
          is_in_processing = true;
        if (is_reach_local_goal)
          is_in_processing = true;
        if (selected_type=="backUp" && (automated_vehicle_rear_v<=0 || is_in_backUp))
          is_in_processing = true;
        if (selected_type=="cutIn" && (automated_vehicle_rear_v<=0 || is_in_cutIn))
          is_in_processing = true;
      }
      // 2.4.1 在危险的处理中，保持原decision，不要轻举妄动
      if (is_in_processing){
        ROS_INFO("Continue last decision state: %s,  %s,  %s", decision_state.c_str(), game_res.c_str(), distribution.c_str());
      }
      // 2.4.2 重新选择gap来完成会车
      else{
        double gap_x;
        std::vector<potentialGap> potential_gaps;
        // automated_vehicle_rear_v = 0.1;
        overal_time = (oncoming_vehcle_rear_x-automated_vehicle_rear_x) / (automated_vehicle_rear_v-oncoming_vehicle_rear_v);
        assert(oncoming_vehcle_rear_x-automated_vehicle_rear_x>0);
        // 相遇位置的x坐标，如果oncoming停在路中间，则自车顶到离它最近的gap去；否则按速度计算相对gap
        if (oncoming_vehicle_rear_v==0)
          gap_x = oncoming_vehcle_rear_x - footprint_supplement_front;
        else
          gap_x = automated_vehicle_rear_x + automated_vehicle_rear_v*overal_time;      
        potential_gaps = findNearestGap(gap_x);
        // std::cout<<"== "<<potential_gaps.size() << "; "<< automated_vehicle_odom.pose.pose.position.x <<"; ";
        // std::cout<<gaps_[potential_gaps.front().gap_index].x_start<<", "<<gaps_[potential_gaps.front().gap_index].x_end<<std::endl;
        // std::cout<<gaps_[potential_gaps.back().gap_index].x_start<<", "<<gaps_[potential_gaps.back().gap_index].x_end<<std::endl;
        // 只有一个gap，只有三种情况
        if (potential_gaps.size()==1){
          // 仅在gap的index发生改变时，ref_path_type才会被重置
          if (final_gap.is_init && potential_gaps.front().gap_index != final_gap.gap_index)
            ref_path_type = "";
          // 设x<自车为A, x在自车和对面车之间为B，x>对面车为C, x在自车附近为D
          final_gap = potential_gaps.front();
          std::string gap_distribution;
          if (automated_vehicle_odom.pose.pose.position.x > gaps_[final_gap.gap_index].x_start && automated_vehicle_odom.pose.pose.position.x < gaps_[final_gap.gap_index].x_end)
            gap_distribution = "D";
          else if (gaps_[final_gap.gap_index].x_end < automated_vehicle_odom.pose.pose.position.x)
            gap_distribution = "A";
          else if (gaps_[final_gap.gap_index].x_start > oncoming_vehcle_rear_x)
            gap_distribution = "C";
          else
            gap_distribution = "B";
          // 根据gap分布决定decision
          if (gap_distribution=="A")
            decision_state = "back";
          else if (gap_distribution=="B")
            decision_state = "forward";
          else if (gap_distribution=="D")
            decision_state = "here";
          else if (gap_distribution=="C"){
            std::cout<<"== "<<automated_vehicle_rear_x<<", "<<automated_vehicle_rear_v<<", "<<overal_time<<", "<<gap_x<<std::endl;
            assert("error in determinate"=="error");
          }
        }
        // 有两个gap 共六中情况
        else if (potential_gaps.size()==2){
          // 设x<自车为A, x在自车和对面车之间为B，x>对面车为C
          std::string gap1_distribution, gap2_distribution;
          if (gaps_[potential_gaps.front().gap_index].x_end < automated_vehicle_odom.pose.pose.position.x)
            gap1_distribution = "A";
          else if (gaps_[potential_gaps.front().gap_index].x_end > oncoming_vehcle_rear_x)
            gap1_distribution = "C";
          else
            gap1_distribution = "B";

          if (gaps_[potential_gaps.back().gap_index].x_end < automated_vehicle_odom.pose.pose.position.x)
            gap2_distribution = "A";
          else if (gaps_[potential_gaps.back().gap_index].x_end > oncoming_vehcle_rear_x)
            gap2_distribution = "C";
          else
            gap2_distribution = "B";

          // 1. 两个gap都在{自车后} AA，path_type为backup
          if (gap1_distribution=="A" && gap2_distribution=="A"){
            decision_state = "back";
            game_res = "aggressive";
            ref_path_type = "backUp";
          }
          // 2. 一个gap在{自车后}，一个gap在{自车，对面车} AB
          else if (gap1_distribution=="A" && gap2_distribution=="B"){
            // 有两个以上的gap 根据distance选择：当抢行所需的distance很小时，抢行； 或者当oncoming vehicle需要10s以上才能到达gap时，抢行
            if (potential_gaps.front().distance > 1.2 * potential_gaps.back().distance || (oncoming_vehcle_rear_x-gaps_[potential_gaps.back().gap_index].x_end)/(std::max(0.0001,abs(oncoming_vehcle_rear_x)))>10 ){
              decision_state = "forward";
              game_res = "aggressive";
              ref_path_type = "cutIn";
            }
            else{
              decision_state = "back";
              game_res = "compromise";
              ref_path_type = "backUp";
            }
          }
          // 3. 一个gap在{自车后}，一个gap在{对面车后} AC
          else if (gap1_distribution=="A" && gap2_distribution=="C"){
            decision_state = "back";
            game_res = "compromise";
            ref_path_type = "backUp";
          }
          // 4. 两个gap都在{自车，对面车}，则启动game BB
          else if (gap1_distribution=="B" && gap2_distribution=="B"){
            // 面前还有两个以上的gap 根据distance选择：当抢行所需的distance很小时，抢行； 或者当oncoming vehicle需要10s以上才能到达gap时，抢行
            if (potential_gaps.front().distance > 1.5 * potential_gaps.back().distance || (oncoming_vehcle_rear_x-gaps_[potential_gaps.back().gap_index].x_end)/(std::max(0.0001,abs(automated_vehicle_rear_v))>10) ){
              decision_state = "forward";
              game_res = "compromise";
              ref_path_type = "backUp";
            }
            else {
              decision_state = "forward";
              game_res = "aggressive";
              ref_path_type = "cutIn";
            }
          }
          // 5. 一个gap都在{自车，对面车}，一个在{对面车后} BC
          else if (gap1_distribution=="B" && gap2_distribution=="C"){
            decision_state = "forward";
            game_res = "compromise";
            ref_path_type = "";
          }
          // 6. 两个gap都在{对面车后} CC
          else if (gap1_distribution=="C" && gap2_distribution=="C"){
            assert("error in game"=="error");
          }


          if (game_res == "compromise")
            final_gap = potential_gaps.front();
          else if (game_res == "aggressive")
            final_gap = potential_gaps.back();
          distribution = gap1_distribution + gap2_distribution;
        }
          // decision_state = "stop";
        ROS_INFO("New decision state: %s,  %s,  %s", decision_state.c_str(), game_res.c_str(), distribution.c_str());
      }
    }
  }

  static std::vector<std::vector<std::vector<double>>> potential_paths;
  static std::vector<std::string> paths_types;
  static std::vector<std::vector<std::vector<double>>> potential_paths_ends;

  // ROS_INFO("is_in_processing: %d, is_reach_goal: %d", is_in_processing, is_reach_local_goal);
  if(!is_in_processing){
    potential_paths.clear(); paths_types.clear(), potential_paths_ends.clear();
    generateTEBPaths(decision_state, automated_vehicle_odom, final_gap, normal_considering_dynamic_veh, potential_paths, paths_types, potential_paths_ends);
    selectPathForTEB(potential_paths, paths_types, gaps_[final_gap.gap_index], ref_path_type, oncoming_vehs_num_-1>followed_vehs_num_ || selected_type=="turnLeft", selected_type!="cutIn"&&(1.2*abs(automated_vehicle_rear_x-gaps_[final_gap.gap_index].x_back_switching)<abs(oncoming_vehcle_rear_x-gaps_[final_gap.gap_index].x_back_switching) || automated_vehicle_rear_x>gaps_[final_gap.gap_index].x_back_switching), selected_path, selected_type);
    pubPotentialEnds(potential_paths_ends);
    std::cout<<decision_state<<", "<<selected_type<<std::endl;
  }

  // std::cout<<"+-+-+-   "<<potential_paths.size()<<std::endl;
  // std::cout<<"pose and velocity: "<<automated_vehicle_rear_x<<", "<<oncoming_vehcle_rear_x<<"; "<<automated_vehicle_rear_v<<", "<<oncoming_vehicle_rear_v<<", "<<overal_time<<std::endl;
  // std::cout<<"gap_x: "<<gap_x<<std::endl;
  // std::cout<<"paths size: "<<potential_paths.size()<<", "<<paths_types.size()<<std::endl;
  pubCurrentGap(final_gap);
  if (decision_state == "normal"){
    pubPathsForTEB(potential_paths, paths_types, selected_path, selected_type);
    ROS_INFO("path_type: %s", selected_type.c_str());
  }
  else if (is_reach_local_goal){
    pubPathsForTEB(potential_paths, paths_types, selected_path, "stop");
    ROS_INFO("path_type: %s, and stop at the local goal ", selected_type.c_str());
  }
  else{
    pubPathsForTEB(potential_paths, paths_types, selected_path, selected_type);
    ROS_INFO("path_type: %s ", selected_type.c_str());
  }
}
void ChickenGame::selectPathForTEB(std::vector<std::vector<std::vector<double>>> paths, std::vector<std::string> paths_types, Gap coordinated_gap, std::string ref_path_type, bool is_avoid_by_left, bool is_back_up, std::vector<std::vector<double>>& selected_path, std::string& selected_type){
  assert(paths.size()==paths_types.size());
  // 1. 如果有normal 那肯定是normal
  // 2. 如果gap在车道左侧，那么需要判断双方的车数量，如果对面车多且自己只有一辆车，那只能采取turn left (back)
  // 3. 剩下的情况都是右侧会车了，不考虑turn left了
  //  3.1. 如果处于game情况，当存在ref_path_type时，对应执行
  //  3.2. 如果不处于game或者game无ref_path_type，则说明turn right是可行的，优先考虑back up，然后是turn right，最后是cut in

  // case 1
  for (int i=0; i<paths_types.size(); i++){
    if (paths_types[i]=="normal"){
      selected_path = paths[i];
      selected_type = paths_types[i];
      return;
    }
  }

  int start_index = env_.xPos2Index(coordinated_gap.x_start);
  int end_index = env_.xPos2Index(coordinated_gap.x_end);
  int middle_index = (start_index + end_index)/2;
  double start_y = (env_.down_borderline_[start_index] + env_.up_borderline_[start_index])/2;
  double end_y = (env_.down_borderline_[end_index] + env_.up_borderline_[end_index])/2;
  double middle_y = (env_.down_borderline_[middle_index] + env_.up_borderline_[middle_index])/2;
  double average_y = (start_y+middle_y+end_y)/3;
  
  double start_delta = env_.up_borderline_[start_index] - env_.down_borderline_[start_index];
  double end_delta = env_.up_borderline_[end_index] - env_.down_borderline_[end_index];
  double middle_delta = env_.up_borderline_[middle_index] - env_.down_borderline_[middle_index];
  double max_delta = 0; max_delta=std::max(max_delta, start_delta); max_delta=std::max(max_delta, middle_delta); max_delta=std::max(max_delta, end_delta);
  
  // case 2
  // std::cout<<coordinated_gap.has_left<<", "<<(average_y > (env_.yMax_+0)*0.1)<<", "<<is_avoid_by_left<<std::endl;
  if (coordinated_gap.has_left && average_y > (env_.yMax_+0)*0.1 && is_avoid_by_left){
    for (int i=0; i<paths_types.size(); i++){
      if (paths_types[i]=="turnLeft"){
        selected_path = paths[i];
        selected_type = paths_types[i];
        return;
      }
    }
  }

  // case 3.1
  if (ref_path_type!=""){
    for (int i=0; i<paths_types.size(); i++){
      if (paths_types[i]==ref_path_type){
        selected_path = paths[i];
        selected_type = paths_types[i];
        return;
      }
    }    
  }

  // case 3.2 todo这里最后再优化的话，也不能根据车的相对状态改变动作，因为车的相对x和v变得会很快。得根据gap的性质改【尤其是选择backup还是cutin！】。
  // 如果只有turnRight了，那就turnRight；如果道路比较宽或比较长，也可以turnRIght
  if ( (!coordinated_gap.has_cut && !coordinated_gap.has_back) || max_delta > 1.5*(0.5*env_.y_negative_vehwidth_ + 0.5*env_.y_positive_vehwidth_) || coordinated_gap.x_normal_end-coordinated_gap.x_normal_start>1.0){
    for (int i=0; i<paths_types.size(); i++){
      if (paths_types[i]=="turnRight"){
        selected_path = paths[i];
        selected_type = paths_types[i];
        return;
      }
    }
  }
  // 如果只有backup了，或者自车离backup的switch点更近
  std::cout<<is_back_up<<std::endl;
  if (is_back_up || !coordinated_gap.has_cut){    
    for (int i=0; i<paths_types.size(); i++){
      if (paths_types[i]=="backUp"){
        selected_path = paths[i];
        selected_type = paths_types[i];
        return;
      }
    }
  }

  for (int i=0; i<paths_types.size(); i++){
    if (paths_types[i]=="cutIn"){
      selected_path = paths[i];
      selected_type = paths_types[i];
      return;
    }
  }

}

void ChickenGame::generateTEBPaths(const std::string& decision_state, const nav_msgs::Odometry& automated_vehicle_odom, potentialGap pG, const std::vector<double>& normal_considering_dynamic_veh, std::vector<std::vector<std::vector<double>>>& potential_paths, std::vector<std::string>& paths_types, std::vector<std::vector<std::vector<double>>>& potential_paths_ends){
  int forward_delta = 50;
  int back_delta = 10;
  int sign_flag = convertSign(automated_vehicle_goal_.pose.position.x-automated_vehicle_odom.pose.pose.position.x);

  if (decision_state == "normal"){
    // 从0.2米前出发，目标在2米前
    double current_x = automated_vehicle_odom.pose.pose.position.x;
    double target_x;
    if (sign_flag>=0){
      // target_x = std::min(current_x+2*sign_flag, automated_vehicle_goal_.pose.position.x);
      // target_x = std::min(target_x, env_.xMax_);
      target_x = std::min(env_.xMax_, automated_vehicle_goal_.pose.position.x);
    }
    else{
      // target_x = std::max(current_x+2*sign_flag, automated_vehicle_goal_.pose.position.x);
      // target_x = std::max(target_x, env_.xMin_);
      target_x = std::max(env_.xMin_, automated_vehicle_goal_.pose.position.x);
    }
    std::vector<std::vector<double>> normal_path;
    int current_index = env_.xPos2Index(current_x);
    int target_index = env_.xPos2Index(target_x);

    tf::Quaternion quat;
    tf::quaternionMsgToTF(automated_vehicle_odom.pose.pose.orientation, quat);
    double roll,pitch,yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (yaw>M_PI_2) yaw-=M_PI;
    if (yaw<-M_PI_2) yaw+=M_PI;

    normal_path.push_back({automated_vehicle_odom.pose.pose.position.x, automated_vehicle_odom.pose.pose.position.y, yaw});
    // 往前走
    if (sign_flag==1){
      // 1. 正常走 不需要考虑oncoming vehicle
      if (normal_considering_dynamic_veh.empty()){
        for (int i=current_index; i<target_index; i++){
          if ((i-current_index+1)%forward_delta!=0) continue;
          double yaw = atan2((env_.up_borderline_[i+1]+env_.down_borderline_[i+1])*0.5-(env_.up_borderline_[i]+env_.down_borderline_[i])*0.5, env_.index2x_list_[i+1]-env_.index2x_list_[i]);
          if (yaw>M_PI_2) yaw-=M_PI;
          if (yaw<-M_PI_2) yaw+=M_PI;        
          normal_path.push_back({env_.index2x_list_[i], (env_.up_borderline_[i]+env_.down_borderline_[i])*0.5, yaw});
        }
      }
      // 2. oncoming vehicle parks at the road center，且不可以绕行，av停在oncoming vehicle前
      else if (normal_considering_dynamic_veh.size()!=env_.down_borderline_.size()){
        for (int i=current_index; i<normal_considering_dynamic_veh.size()-1; i++){
          if ((i-current_index+1)%forward_delta!=0) continue;
          double yaw = atan2((env_.up_borderline_[i+1]+env_.down_borderline_[i+1])*0.5-(env_.up_borderline_[i]+env_.down_borderline_[i])*0.5, env_.index2x_list_[i+1]-env_.index2x_list_[i]);
          if (yaw>M_PI_2) yaw-=M_PI;
          if (yaw<-M_PI_2) yaw+=M_PI;        
          normal_path.push_back({env_.index2x_list_[i], (env_.up_borderline_[i]+env_.down_borderline_[i])*0.5, yaw});
        }        
      }
      // 3. oncoming vehicle 虽然park at the road center，但可以绕行过去
      else{
        for (int i=current_index; i<target_index; i++){
          if ((i-current_index+1)%forward_delta!=0) continue;
          double yaw = atan2(normal_considering_dynamic_veh[i+1]-normal_considering_dynamic_veh[i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
          if (yaw>M_PI_2) yaw-=M_PI;
          if (yaw<-M_PI_2) yaw+=M_PI;        
          normal_path.push_back({env_.index2x_list_[i], normal_considering_dynamic_veh[i], yaw});
        }        
      }
    }
    // 往后走
    else if (sign_flag==-1){
      for (int i=current_index; i>target_index; i--){
        if ((i-current_index+1)%forward_delta!=0) continue;
        double yaw = atan2((env_.up_borderline_[i+1]+env_.down_borderline_[i+1])*0.5-(env_.up_borderline_[i]+env_.down_borderline_[i])*0.5, env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;        
        normal_path.push_back({env_.index2x_list_[i], (env_.up_borderline_[i]+env_.down_borderline_[i])*0.5, yaw});
      }
    }
    potential_paths.push_back(normal_path);
    paths_types.push_back("normal");
    potential_paths_ends.push_back({});
  }

  else if (decision_state == "forward" || decision_state =="here"){
    auto gap = gaps_[pG.gap_index];
    // 普通右转
    if (gap.has_right){
      std::cout<<"turn right"<<std::endl;
      // double current_x = automated_vehicle_odom.pose.pose.position.x;
      double current_x = automated_vehicle_odom.pose.pose.position.x + 0.05*sign_flag;
      int index_fake_x_start = env_.xPos2Index(gap.x_end-0.1), index_x_end = env_.xPos2Index(gap.x_end);
      float max_dis = -1;
      int index_target = -1;
      for(int j=std::min(index_fake_x_start,index_x_end-1); j<index_x_end; j++){
        if (env_.up_borderline_[j]-env_.down_borderline_[j] > max_dis){
          max_dis = env_.up_borderline_[j]-env_.down_borderline_[j];
          index_target = j;
        }
      }
      double target_x = std::min(env_.Index2xPos(index_target), automated_vehicle_goal_.pose.position.x);
      target_x = std::min(target_x, env_.xMax_);
      std::vector<std::vector<double>> normal_path;
      int current_index = env_.xPos2Index(current_x);

      tf::Quaternion quat;
      tf::quaternionMsgToTF(automated_vehicle_odom.pose.pose.orientation, quat);
      double roll,pitch,yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      if (yaw>M_PI_2) yaw-=M_PI;
      if (yaw<-M_PI_2) yaw+=M_PI;

      normal_path.push_back({automated_vehicle_odom.pose.pose.position.x, automated_vehicle_odom.pose.pose.position.y, yaw});
      // normal_path.push_back({current_x, env_.down_borderline_[current_index]});
      for (int i=current_index; i<env_.xPos2Index(target_x); i++){
        if ((i-current_index+1)%forward_delta!=0) continue;
        double yaw = atan2(env_.down_borderline_[i+1]-env_.down_borderline_[i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;        
        normal_path.push_back({env_.index2x_list_[i], env_.down_borderline_[i], yaw});
      }
      potential_paths.push_back(normal_path);
      paths_types.push_back("turnRight");
      potential_paths_ends.push_back({});
    }

    // back up
    if (gap.has_back){
      std::cout<<"back up"<<std::endl;
      // double current_x = automated_vehicle_odom.pose.pose.position.x;
      double current_x = automated_vehicle_odom.pose.pose.position.x + 0.05*sign_flag;
      int current_index = env_.xPos2Index(current_x);
      double switch_x = gap.x_back_switching;
      double target_x = gap.x_back_constraint_ending;
      std::vector<std::vector<double>> back_path;

      tf::Quaternion quat;
      tf::quaternionMsgToTF(automated_vehicle_odom.pose.pose.orientation, quat);
      double roll,pitch,yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      if (yaw>M_PI_2) yaw-=M_PI;
      if (yaw<-M_PI_2) yaw+=M_PI;
      
      back_path.push_back({automated_vehicle_odom.pose.pose.position.x, automated_vehicle_odom.pose.pose.position.y, yaw});
      // back_path.push_back({current_x, env_.down_borderline_[current_index]});
      for (int i=current_index; i<env_.xPos2Index(switch_x); i++){
        if ((i-current_index+1)%forward_delta!=0) continue;
        double yaw = atan2(env_.down_borderline_[i+1]-env_.down_borderline_[i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;        
        back_path.push_back({env_.index2x_list_[i], env_.down_borderline_[i], yaw});
      }
      // 最接近路边的点是下面的方法获得的      
      for (int i=env_.xPos2Index(switch_x); i>env_.xPos2Index(target_x); i--){
        if ((i-env_.xPos2Index(switch_x))%back_delta!=0) continue;
        // if (env_.down_borderline_individual_[gap.individual_right_index][i] == env_.yMin_ ) break;
        double yaw = atan2(env_.down_borderline_individual_[gap.individual_right_index][i+1]-env_.down_borderline_individual_[gap.individual_right_index][i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;         
        back_path.push_back({env_.index2x_list_[i], env_.down_borderline_individual_[gap.individual_right_index][i], yaw});
      }
      potential_paths.push_back(back_path);
      paths_types.push_back("backUp");
      auto path_ends = findPotentialEndPoints("backUp", {gap.x_back_switching, env_.down_borderline_[env_.xPos2Index(gap.x_back_switching)]}, {gap.x_back_constraint_meeting, env_.down_borderline_[env_.xPos2Index(gap.x_back_constraint_meeting)]}, {gap.x_back_constraint_ending, env_.down_borderline_[env_.xPos2Index(gap.x_back_constraint_ending)]});
      potential_paths_ends.push_back(path_ends);
    }    

    // cut in
    if (gap.has_cut){
      std::cout<<"cut in"<<std::endl;
      // double current_x = automated_vehicle_odom.pose.pose.position.x;
      double current_x = automated_vehicle_odom.pose.pose.position.x + 0.05*sign_flag;
      int current_index = env_.xPos2Index(current_x);
      double switch_x = gap.x_cut_switching;
      double target_x = gap.x_cut_constraint_ending;
      std::vector<std::vector<double>> cut_path;

      tf::Quaternion quat;
      tf::quaternionMsgToTF(automated_vehicle_odom.pose.pose.orientation, quat);
      double roll,pitch,yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      if (yaw>M_PI_2) yaw-=M_PI;
      if (yaw<-M_PI_2) yaw+=M_PI;

      cut_path.push_back({automated_vehicle_odom.pose.pose.position.x, automated_vehicle_odom.pose.pose.position.y, yaw});
      // cut_path.push_back({current_x, env_.down_borderline_[current_index]});
      for (int i=current_index; i<env_.xPos2Index(switch_x); i++){
        if ((i-current_index+1)%forward_delta!=0) continue;
        double yaw = atan2(env_.down_borderline_[i+1]-env_.down_borderline_[i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;        
        cut_path.push_back({env_.index2x_list_[i], env_.down_borderline_[i], yaw});
      }
      // 最接近路边的点是下面的方法获得的
      for (int i=env_.xPos2Index(switch_x); i<env_.xPos2Index(target_x); i++){
        // if (env_.down_borderline_individual_[gap.individual_right_index-1][i] == env_.yMin_) break;
        if ((i-env_.xPos2Index(switch_x))%back_delta!=0) continue;
        double yaw = atan2(env_.down_borderline_individual_[gap.individual_right_index-1][i+1]-env_.down_borderline_individual_[gap.individual_right_index-1][i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;          
        cut_path.push_back({env_.index2x_list_[i], env_.down_borderline_individual_[gap.individual_right_index-1][i], yaw});   // 这里的individual_index-1很重要
      }
      potential_paths.push_back(cut_path);
      paths_types.push_back("cutIn");
      auto path_ends = findPotentialEndPoints("cutIn", {gap.x_cut_switching, env_.down_borderline_[env_.xPos2Index(gap.x_cut_switching)]}, {gap.x_cut_constraint_meeting, env_.down_borderline_[env_.xPos2Index(gap.x_cut_constraint_meeting)]}, {gap.x_cut_constraint_ending, env_.down_borderline_[env_.xPos2Index(gap.x_cut_constraint_ending)]});
      potential_paths_ends.push_back(path_ends);      
    }

    // 左转
    if (gap.has_left){
      std::cout<<"turn left"<<std::endl;
      // double current_x = automated_vehicle_odom.pose.pose.position.x;
      double current_x = automated_vehicle_odom.pose.pose.position.x + 0.05*sign_flag;
      int current_index = env_.xPos2Index(current_x);
      double switch_x = gap.x_left_switching;
      double target_x = gap.x_left_constraint_ending;
      std::vector<std::vector<double>> left_path;

      tf::Quaternion quat;
      tf::quaternionMsgToTF(automated_vehicle_odom.pose.pose.orientation, quat);
      double roll,pitch,yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      if (yaw>M_PI_2) yaw-=M_PI;
      if (yaw<-M_PI_2) yaw+=M_PI;

      left_path.push_back({automated_vehicle_odom.pose.pose.position.x, automated_vehicle_odom.pose.pose.position.y, yaw});
      // left_path.push_back({current_x, env_.up_borderline_[current_index]});
      for (int i=current_index; i<env_.xPos2Index(switch_x); i++){
        if ((i-current_index+1)%forward_delta!=0) continue;
        double yaw = atan2(env_.up_borderline_[i+1]-env_.up_borderline_[i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;        
        left_path.push_back({env_.index2x_list_[i], env_.up_borderline_[i], yaw});
      }
      for (int i=env_.xPos2Index(switch_x); i>=env_.xPos2Index(target_x); i--){
        if ((i-env_.xPos2Index(switch_x))%back_delta!=0) continue;
        // if (env_.up_borderline_individual_[gap.individual_left_index][i] == env_.yMax_ ) break;
        double yaw = atan2(env_.up_borderline_individual_for_left_backup_[gap.individual_left_index][i+1]-env_.up_borderline_individual_for_left_backup_[gap.individual_left_index][i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;         
        left_path.push_back({env_.index2x_list_[i], env_.up_borderline_individual_for_left_backup_[gap.individual_left_index][i], yaw});
      }

      potential_paths.push_back(left_path);
      paths_types.push_back("turnLeft");
      auto path_ends = findPotentialEndPoints("turnLeft", {gap.x_left_switching, env_.down_borderline_[env_.xPos2Index(gap.x_left_switching)]}, {gap.x_left_constraint_meeting, env_.down_borderline_[env_.xPos2Index(gap.x_left_constraint_meeting)]}, {gap.x_left_constraint_ending, env_.down_borderline_[env_.xPos2Index(gap.x_left_constraint_ending)]});
      potential_paths_ends.push_back(path_ends);       
    }
  }

  else if (decision_state == "back"){
    auto gap = gaps_[pG.gap_index];

    // back up
    if (gap.has_back){
      std::cout<<"back up"<<std::endl;
      double current_x = automated_vehicle_odom.pose.pose.position.x;
      // int current_index = env_.xPos2Index(current_x);
      int current_index = env_.xPos2Index(current_x) + 0.05*sign_flag;
      double switch_x = gap.x_back_switching;
      double target_x = gap.x_back_constraint_ending;
      std::vector<std::vector<double>> back_path;

      tf::Quaternion quat;
      tf::quaternionMsgToTF(automated_vehicle_odom.pose.pose.orientation, quat);
      double roll,pitch,yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      if (yaw>M_PI_2) yaw-=M_PI;
      if (yaw<-M_PI_2) yaw+=M_PI;

      back_path.push_back({automated_vehicle_odom.pose.pose.position.x, automated_vehicle_odom.pose.pose.position.y, yaw});
      // back_path.push_back({current_x, env_.down_borderline_[current_index]});
      for (int i=current_index; i>env_.xPos2Index(switch_x); i--){
        if ((i-current_index+1)%forward_delta!=0) continue;
        double yaw = atan2(env_.down_borderline_[i+1]-env_.down_borderline_[i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;          
        back_path.push_back({env_.index2x_list_[i], env_.down_borderline_[i], yaw});
      }
      for (int i=env_.xPos2Index(switch_x); i>=env_.xPos2Index(target_x); i--){
        if ((i-env_.xPos2Index(switch_x))%back_delta!=0) continue;
        // if (env_.down_borderline_individual_[gap.individual_right_index][i] == env_.yMin_ ) break;
        double yaw = atan2(env_.down_borderline_individual_[gap.individual_right_index][i+1]-env_.down_borderline_individual_[gap.individual_right_index][i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;         
        back_path.push_back({env_.index2x_list_[i], env_.down_borderline_individual_[gap.individual_right_index][i], yaw});
      }
      potential_paths.push_back(back_path);
      paths_types.push_back("backUp");
      auto path_ends = findPotentialEndPoints("backUp", {gap.x_back_switching, env_.down_borderline_[env_.xPos2Index(gap.x_back_switching)]}, {gap.x_back_constraint_meeting, env_.down_borderline_[env_.xPos2Index(gap.x_back_constraint_meeting)]}, {gap.x_back_constraint_ending, env_.down_borderline_[env_.xPos2Index(gap.x_back_constraint_ending)]});
      potential_paths_ends.push_back(path_ends);      
    }

    // 左转
    if (gap.has_left){
      std::cout<<"turn left"<<std::endl;
      // double current_x = automated_vehicle_odom.pose.pose.position.x;
      double current_x = automated_vehicle_odom.pose.pose.position.x + 0.05*sign_flag;
      int current_index = env_.xPos2Index(current_x);
      double switch_x = gap.x_left_switching;
      double target_x = gap.x_left_constraint_ending;
      std::vector<std::vector<double>> left_path;

      tf::Quaternion quat;
      tf::quaternionMsgToTF(automated_vehicle_odom.pose.pose.orientation, quat);
      double roll,pitch,yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      if (yaw>M_PI_2) yaw-=M_PI;
      if (yaw<-M_PI_2) yaw+=M_PI;

      left_path.push_back({automated_vehicle_odom.pose.pose.position.x, automated_vehicle_odom.pose.pose.position.y, yaw});
      // left_path.push_back({current_x, env_.up_borderline_[current_index]});
      for (int i=current_index; i>env_.xPos2Index(switch_x); i--){
        if ((i-current_index+1)%forward_delta!=0) continue;
        double yaw = atan2(env_.up_borderline_[i+1]-env_.up_borderline_[i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;        
        left_path.push_back({env_.index2x_list_[i], env_.up_borderline_[i], yaw});
      }
      for (int i=env_.xPos2Index(switch_x); i>=env_.xPos2Index(target_x); i--){
        if ((i-env_.xPos2Index(switch_x))%back_delta!=0) continue;
        // if (env_.up_borderline_individual_[gap.individual_left_index][i] == env_.yMax_ ) break;
        double yaw = atan2(env_.up_borderline_individual_for_left_backup_[gap.individual_left_index][i+1]-env_.up_borderline_individual_for_left_backup_[gap.individual_left_index][i], env_.index2x_list_[i+1]-env_.index2x_list_[i]);
        if (yaw>M_PI_2) yaw-=M_PI;
        if (yaw<-M_PI_2) yaw+=M_PI;         
        left_path.push_back({env_.index2x_list_[i], env_.up_borderline_individual_for_left_backup_[gap.individual_left_index][i], yaw});
      }

      potential_paths.push_back(left_path);
      paths_types.push_back("turnLeft");
      auto path_ends = findPotentialEndPoints("turnLeft", {gap.x_left_switching, env_.down_borderline_[env_.xPos2Index(gap.x_left_switching)]}, {gap.x_left_constraint_meeting, env_.down_borderline_[env_.xPos2Index(gap.x_left_constraint_meeting)]}, {gap.x_left_constraint_ending, env_.down_borderline_[env_.xPos2Index(gap.x_left_constraint_ending)]});
      potential_paths_ends.push_back(path_ends);       
    }

  }
}

/**
 * @input: gap的x坐标
 * @output: 潜在的gaps
 * @description: 如果gaps的长度为一，那么说明会车点在gap内或自车和对面车之间只剩下一个gap了； 如果长度为二，说明会车区域有争议，需要开始博弈了。
 * **/
std::vector<potentialGap> ChickenGame::findNearestGap(const double& gap_x){
  std::vector<potentialGap> pGs_x_smaller;  // x值更小
  std::vector<potentialGap> pGs_x_larger; // x值更大
  for (int i=0; i<gaps_.size(); i++){
    double length = gaps_[i].x_end - gaps_[i].x_start;
    // 会车位置在gap内，直接返回i
    if (gaps_[i].x_start<gap_x && gap_x<gaps_[i].x_end){
      return {potentialGap(i, 0, length)};
    }
    // 会车位置在gap外，找到前面和后面的gaps
    else{
      double dis = std::min(abs(gap_x-gaps_[i].x_start), abs(gap_x-gaps_[i].x_end));
      if (gaps_[i].x_start+gaps_[i].x_end < gap_x*2)
        pGs_x_smaller.push_back(potentialGap(i, dis, length));
      else 
        pGs_x_larger.push_back(potentialGap(i, dis, length));
    }
  }
  // 找到离会车位置最近的 前面一个gap和后面一个gap
  sort(pGs_x_smaller.begin(), pGs_x_smaller.end(), comparePotentialGap);
  sort(pGs_x_larger.begin(), pGs_x_larger.end(), comparePotentialGap);
  
  std::vector<potentialGap> pGs;
  if (!pGs_x_smaller.empty())
    pGs.push_back(pGs_x_smaller.front());
  if (!pGs_x_larger.empty())
    pGs.push_back(pGs_x_larger.front());

  return pGs;
}


void ChickenGame::odomCB(const nav_msgs::Odometry::ConstPtr& msg){
  automated_vehicle_odom_ = *msg;
  isAvInitialized_ = true;
}

void ChickenGame::goalCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
  automated_vehicle_goal_ = *msg;
  isAVGoalRecivied_ = true;
}


void ChickenGame::extractGaps(){
  // 1. 正常平滑右转形成的gap
  // 遍历对所有点，标记该x是否可供会车
  std::vector<int> state_list_right;
  for(int i=0; i<env_.index2x_list_.size(); i++){
    double dis = env_.up_borderline_extended_right_[i] - env_.down_borderline_extended_right_[i];
    // -1表示无法会车，1表示可以会车
    if (dis<=0)
      state_list_right.push_back(-1);
    else
      state_list_right.push_back(1);
  }
  if (!state_list_right.empty()){
    int current_state = state_list_right.front();
    segmented_states_right_.push_back({env_.index2x_list_[0], current_state});
    for (int i=1; i<state_list_right.size(); i++){
      if (state_list_right[i]!=current_state){
        current_state = state_list_right[i];
        segmented_states_right_.push_back({env_.index2x_list_[i], current_state});
      }
    }
    // 每个点：(x坐标，是否可会车)，每个点都是一个switch点
    segmented_states_right_.push_back({env_.index2x_list_.back(),current_state});  //complete
  }
  // pure的转化
  for (auto p:segmented_states_right_){
    int index = env_.index_from_extended_dbr_to_dbr_[env_.xPos2Index(p.first)];
    // 每个点：(x坐标，是否可会车)，每个点都是一个switch点
    segmented_states_right_pure_.push_back({env_.index2x_list_[index] , p.second});
  }


  // 2. 平滑左转形成的gap
  std::vector<int> state_list_left;
  for(int i=0; i<env_.index2x_list_.size(); i++){
    double dis = env_.up_borderline_extended_left_[i] - env_.down_borderline_extended_left_[i];
    // -1表示无法会车，1表示可以会车
    if (dis<=0)
      state_list_left.push_back(-1);
    else
      state_list_left.push_back(1);
  }
  // 提取left turning形成的gap，用于可视化
  state_list_left_ = state_list_left;
  if (!state_list_left.empty()){
    int current_state = state_list_left.front();
    segmented_states_left_.push_back({env_.index2x_list_[0], current_state});
    for (int i=1; i<state_list_left.size(); i++){
      if (state_list_left[i]!=current_state){
        current_state = state_list_left[i];
        segmented_states_left_.push_back({env_.index2x_list_[i], current_state});
      }
    }
    segmented_states_left_.push_back({env_.index2x_list_.back(),current_state});  //complete
  }

  const int delta_step = 1;
  int index_individual = -1;
  for (auto individual:env_.up_borderline_individual_for_left_backup_){
    ++index_individual;
    int index_back=-1;
    geometry_msgs::Point false_pos; false_pos.y=1.0;
    std::vector<double> temp_up_borderline_for_back;
    for (int i=0; i<env_.index2x_list_.size(); i++){
      temp_up_borderline_for_back.push_back(env_.yMax_);
    }
    // 考虑back的动作，来赋值temp_up_borderline_for_back
    // 倒出的端点 (x_back, y_back, theta_back)
    double x_back, y_back, theta_back;
    bool has_solution = false;
    int index_back_ending=-1;
    for (int i=0; i<env_.xPos2Index(env_.veh_x_borders_for_up_borderline_individual_[index_individual].first)-delta_step; i++){
      // 每一个individual都是一个半圆
      if (individual[i]==env_.yMax_ || individual[i]==env_.yMin_)
        continue;
      if (index_back_ending==-1)
        index_back_ending=i;
      // back
      index_back = i;
      x_back = env_.index2x_list_[i];
      y_back = individual[i];
      theta_back = atan2(individual[i+delta_step]-individual[i], env_.index2x_list_[i+delta_step]-env_.index2x_list_[i]);

      // 检测碰撞
      for (int j=1; j<=env_.y_negative_footprint_.size(); j++){
        geometry_msgs::Point p1; p1.x=env_.y_negative_footprint_[j-1].first; p1.y=env_.y_negative_footprint_[j-1].second;
        geometry_msgs::Point p1_rotated;
        p1_rotated.x = x_back + cos(theta_back)*p1.x + -sin(theta_back)*p1.y;
        p1_rotated.y = y_back + sin(theta_back)*p1.x + cos(theta_back)*p1.y;
        geometry_msgs::Point p2; p2.x=env_.y_negative_footprint_[j%env_.y_negative_footprint_.size()].first; p2.y=env_.y_negative_footprint_[j%env_.y_negative_footprint_.size()].second;
        geometry_msgs::Point p2_rotated;
        p2_rotated.x = x_back + cos(theta_back)*p2.x + -sin(theta_back)*p2.y;
        p2_rotated.y = y_back + sin(theta_back)*p2.x + cos(theta_back)*p2.y;
        env_.addBorderPointsToSpecialVec(temp_up_borderline_for_back, p1_rotated, p2_rotated, false_pos);
      }
      bool flag_back_is_collision = false;
      for (int k=0; k<temp_up_borderline_for_back.size(); k++){
        if (temp_up_borderline_for_back[k] <= env_.down_borderline_extended_left_[k]){
          flag_back_is_collision = true;
          break;
        }
      }
      // 如果有碰撞 就直接break掉 停在这里
      if (flag_back_is_collision)
        break;
      has_solution = true;
    }
    // 如果无collision，则{车辆的index，点在车轮廓上的index, 点在车后轮中心点处ending时的index}
    if (has_solution)
      state_list_self_back_left_.push_back({index_individual, index_back-1, index_back_ending+1});
    // 如果有collision，则{车辆的index，-1标志无效}
    else
      state_list_self_back_left_.push_back({index_individual, index_back+1, index_back+1});
  }


  // 3. 扎入 和 倒入 形成的gap
  index_individual = -1;
  for (auto individual:env_.down_borderline_individual_){
    ++index_individual;
    int index_back=-1, index_cut=-1;
    geometry_msgs::Point false_pos; false_pos.y=-1.0;
    std::vector<double> temp_down_borderline_for_back;
    std::vector<double> temp_down_borderline_for_cut;
    for (int i=0; i<env_.index2x_list_.size(); i++){
      temp_down_borderline_for_back.push_back(env_.yMin_);
      temp_down_borderline_for_cut.push_back(env_.yMin_);
    }
    
    // 考虑back的动作
    // 倒出的端点 (x_back, y_back, theta_back)
    double x_back, y_back, theta_back;
    bool has_solution = false;
    int index_back_ending=-1;
    for (int i=0; i<env_.xPos2Index(env_.veh_x_borders_for_down_borderline_individual_[index_individual].first)-delta_step; i++){
      // 每一个individual都是一个半圆
      if (individual[i]==env_.yMax_ || individual[i]==env_.yMin_)
        continue;
      if (index_back_ending==-1)
        index_back_ending=i;        
      // back
      index_back = i;
      x_back = env_.index2x_list_[i];
      y_back = individual[i];
      theta_back = atan2(individual[i+delta_step]-individual[i], env_.index2x_list_[i+delta_step]-env_.index2x_list_[i]);

      // 检测碰撞
      for (int j=1; j<=env_.y_negative_footprint_.size(); j++){
        geometry_msgs::Point p1; p1.x=env_.y_negative_footprint_[j-1].first; p1.y=env_.y_negative_footprint_[j-1].second;
        geometry_msgs::Point p1_rotated;
        p1_rotated.x = x_back + cos(theta_back)*p1.x + -sin(theta_back)*p1.y;
        p1_rotated.y = y_back + sin(theta_back)*p1.x + cos(theta_back)*p1.y;
        geometry_msgs::Point p2; p2.x=env_.y_negative_footprint_[j%env_.y_negative_footprint_.size()].first; p2.y=env_.y_negative_footprint_[j%env_.y_negative_footprint_.size()].second;
        geometry_msgs::Point p2_rotated;
        p2_rotated.x = x_back + cos(theta_back)*p2.x + -sin(theta_back)*p2.y;
        p2_rotated.y = y_back + sin(theta_back)*p2.x + cos(theta_back)*p2.y;
        env_.addBorderPointsToSpecialVec(temp_down_borderline_for_back, p1_rotated, p2_rotated, false_pos);
      }
      bool flag_back_is_collision = false;
      for (int i=0; i<temp_down_borderline_for_back.size(); i++)
        if (temp_down_borderline_for_back[i] >= env_.up_borderline_extended_right_[i]){
          flag_back_is_collision = true;
        }
      // 如果有碰撞 就直接break掉 停在这里
      if (flag_back_is_collision)
        break;
      has_solution = true;
    }
    // 如果无collision，则{车辆的index，点在车轮廓上的index, 点在车后轮中心点处ending时的index}
    if (has_solution)
      state_list_self_back_right_.push_back({index_individual, index_back-1, index_back_ending+1});
    // 如果有collision，则{车辆的index，-1标志无效}
    else
      state_list_self_back_right_.push_back({index_individual, -1});


    // 考虑cut in的动作
    // 扎入的端点(x_cut, y_cut, theta_cut)
    double x_cut, y_cut, theta_cut;
    has_solution = false;
    int index_cut_ending=-1;
    for (int i=individual.size()-delta_step; i>env_.xPos2Index(env_.veh_x_borders_for_down_borderline_individual_[index_individual].second); i--){
      // 每一个individual都是一个半圆
      if (individual[i]==env_.yMax_ || individual[i]==env_.yMin_)
        continue;
      if (index_cut_ending==-1)
        index_cut_ending=i;
      // cut
      index_cut = i;
      x_cut = env_.index2x_list_[i];
      y_cut = individual[i];
      theta_cut = atan2(individual[i+delta_step]-individual[i], env_.index2x_list_[i+delta_step]-env_.index2x_list_[i]);

      for (int j=1; j<=env_.y_negative_footprint_.size(); j++){
        geometry_msgs::Point p1; p1.x=env_.y_negative_footprint_[j-1].first; p1.y=env_.y_negative_footprint_[j-1].second;
        geometry_msgs::Point p1_rotated;
        p1_rotated.x = x_cut + cos(theta_cut)*p1.x + -sin(theta_cut)*p1.y;
        p1_rotated.y = y_cut + sin(theta_cut)*p1.x + cos(theta_cut)*p1.y;
        geometry_msgs::Point p2; p2.x=env_.y_negative_footprint_[j%env_.y_negative_footprint_.size()].first; p2.y=env_.y_negative_footprint_[j%env_.y_negative_footprint_.size()].second;
        geometry_msgs::Point p2_rotated;
        p2_rotated.x = x_cut + cos(theta_cut)*p2.x + -sin(theta_cut)*p2.y;
        p2_rotated.y = y_cut + sin(theta_cut)*p2.x + cos(theta_cut)*p2.y;
        env_.addBorderPointsToSpecialVec(temp_down_borderline_for_cut, p1_rotated, p2_rotated, false_pos);
      }
      bool flag_cut_is_collision = false;
      for (int i=0; i<temp_down_borderline_for_cut.size(); i++)
        if (temp_down_borderline_for_cut[i] >= env_.up_borderline_extended_right_[i])
          flag_cut_is_collision = true;

      if (flag_cut_is_collision)
        break;
      has_solution = true;
    }
    // 如果无collision，则{车辆的index，点在车轮廓上meeting时的index，点在车后轮中心点处ending时的index}
    if (has_solution)
      state_list_self_cut_right_.push_back({index_individual, index_cut+1, index_cut_ending});
    // 如果有collision，则{车辆的index，-1标志无效}
    else
      state_list_self_cut_right_.push_back({index_individual, -1});

  }

}


void ChickenGame::combineGaps(){
  // 靠右走的可行gaps （pure）: {x1, x2}
  std::vector<std::pair<double, double>> gaps_normal_in_world; 
  for (int i=0; i<segmented_states_right_pure_.size()-1; i++){
    if (segmented_states_right_pure_[i].second == 1){
      gaps_normal_in_world.push_back({segmented_states_right_pure_[i].first,segmented_states_right_pure_[i+1].first});
    }
  }

  // back 和 cut组成的gaps： {x_cut, x_back}
  std::vector<std::vector<double>> gaps_constraint_meeting_cut2back_in_world;
  std::vector<std::vector<double>> gaps_constraint_ending_cut2back_in_world;
  std::vector<std::pair<double, double>> switching_cut2back_in_world;
  for (int i=1; i<state_list_self_back_right_.size(); i++){
    // 忽略掉不能成对出现的cut in和back
    if (state_list_self_cut_right_[i-1][1]==-1 || state_list_self_back_right_[i][1]==-1) continue;
    gaps_constraint_meeting_cut2back_in_world.push_back( {env_.index2x_list_[state_list_self_cut_right_[i-1][1]], env_.index2x_list_[state_list_self_back_right_[i][1]], 1.0*state_list_self_back_right_[i][0]} );
    gaps_constraint_ending_cut2back_in_world.push_back( {env_.index2x_list_[state_list_self_cut_right_[i-1][2]], env_.index2x_list_[state_list_self_back_right_[i][2]] } );
    // 当没有cutin或back 或者他们并不成对出现时，用meeting点补足switching点 （实际上就是 此时少了个动作）
    if (env_.veh_x_switching_for_down_borderline_individual_[state_list_self_cut_right_[i-1][0]].second == 999)
      env_.veh_x_switching_for_down_borderline_individual_[state_list_self_cut_right_[i-1][0]].second = env_.index2x_list_[state_list_self_cut_right_[i-1][1]];
    if (env_.veh_x_switching_for_down_borderline_individual_[state_list_self_back_right_[i][0]].first == -999)
      env_.veh_x_switching_for_down_borderline_individual_[state_list_self_back_right_[i][0]].first = env_.index2x_list_[state_list_self_back_right_[i][1]];
    switching_cut2back_in_world.push_back( {env_.veh_x_switching_for_down_borderline_individual_[state_list_self_cut_right_[i-1][0]].second, env_.veh_x_switching_for_down_borderline_individual_[state_list_self_back_right_[i][0]].first} );
  }

  // 完整的gaps_ 类型为std::vector<Gap>
  for (int index_normal=0; index_normal<gaps_normal_in_world.size(); index_normal++){
    // normal turning
    auto temp_gap = Gap(gaps_normal_in_world[index_normal].first, gaps_normal_in_world[index_normal].second);

    // left turning
    double x_left_limit = -999; // left gap的左右边界
    double x_right_limit = 999;
    for (int i= env_.xPos2Index(gaps_normal_in_world[index_normal].second); i>env_.xPos2Index(gaps_normal_in_world[index_normal].first); i--){
      if (state_list_left_[i]==1){
        // left和right的区间 不能和up的障碍物车重叠
        if (x_right_limit == 999 && env_.up_borderline_with_static_vehicles_footprints_[i]==env_.yMax_)
          x_right_limit = env_.index2x_list_[i];
        if (x_right_limit!=999 && env_.up_borderline_with_static_vehicles_footprints_[i]==env_.yMax_)
          x_left_limit = env_.index2x_list_[i];
      }
      // if (state_list_left_[i]==-1 && x_right_limit!=999 && x_left_limit!=-999)
      // if (state_list_left_[1]==-1 && x_right_limit!=999 && x_left_limit!=-999)
      if (state_list_left_[i]==-1)
        break;
    }
    // 在veh_ordered_up_中，找到离x_left_target最近的车的index
    if (x_left_limit!=-999 && x_right_limit!=999){
      // std::cout<<"==== "<<gaps_normal_in_world[index_normal].first<<", "<<gaps_normal_in_world[index_normal].second<<"; "<<x_left_limit<<", "<<x_right_limit<<std::endl;
      int index_veh_ordered_up = -1;
      float min_dis = 99999;
      for (int index=0; index<env_.veh_ordered_up_.size(); index++){
        float x_pos=0;
        auto veh = env_.veh_ordered_up_[index];
        for (int i=0; i<veh.points.size()-1; i++){
          x_pos += veh.points[i].x;
        }
        x_pos /= (veh.points.size()-1);
        if (x_right_limit <= x_pos && std::abs(x_pos-x_right_limit)<min_dis){
          min_dis = std::abs(x_pos-x_right_limit);
          index_veh_ordered_up = index;
        }
        // std::cout<<"+++ "<<x_pos<<", "<<index<<", "<<index_veh_ordered_up<<std::endl;
      }
      // 赋值left
      int index_state = -1;
      for (auto p:state_list_self_back_left_){
        if (p.front()==index_veh_ordered_up)
          index_state = index_veh_ordered_up;
      }
      if(index_state!=-1){
        // 当不存在switch点时 默认switch==meeting点
        if (env_.veh_x_switching_for_up_borderline_individual_[index_state] == -999)
          env_.veh_x_switching_for_up_borderline_individual_[index_state] = env_.index2x_list_[state_list_self_back_left_[index_state][1]];
        temp_gap.setLeftConstrains(env_.index2x_list_[state_list_self_back_left_[index_state][1]], env_.index2x_list_[state_list_self_back_left_[index_state][2]], env_.veh_x_switching_for_up_borderline_individual_[index_state]);
        // std::cout<<"+++------ "<<temp_gap.x_left_constraint_meeting<<", "<<temp_gap.x_left_constraint_ending<<", "<<temp_gap.x_left_switching<<std::endl;
        // std::cout<<"----- "<<index_state<<std::endl;
        temp_gap.setIndividualLeftIndex(index_state);
      }

    }
    gaps_.push_back(temp_gap);
    
  }

  // cut和back组成的gap
  for (int j=0; j<gaps_constraint_meeting_cut2back_in_world.size(); j++){
    auto cut2back_meeting = gaps_constraint_meeting_cut2back_in_world[j];
    auto cut2back_ending = gaps_constraint_ending_cut2back_in_world[j];
    auto switching = switching_cut2back_in_world[j];
    bool is_added = false;
    for (int i=0; i<gaps_.size(); i++){
      if (gaps_[i].x_start<=cut2back_meeting.front() && cut2back_meeting.front()<gaps_[i].x_end){
        gaps_[i].addXCut(cut2back_meeting.front(), cut2back_ending.front(), switching.first);
        gaps_[i].addXBack(cut2back_meeting[1], cut2back_ending.back(), switching.second);
        gaps_[i].setIndividualRightIndex(int(cut2back_meeting.back())); // back的index
        is_added = true;
        // 如果某个gap 没有xcut这个动作 仅有back，则把cut设置成false
        if (gaps_[i].x_cut_constraint_meeting == gaps_[i].x_cut_switching)
          gaps_[i].has_cut = false;
        break;
      }
    }

    if (!is_added){
      Gap temp;
      temp.initializeWithCutAndBack(cut2back_meeting.front(), cut2back_meeting[1], cut2back_ending.front(), cut2back_ending.back(), switching.first, switching.second);
      temp.setIndividualRightIndex(cut2back_meeting.back());
      gaps_.push_back(temp);
    }
  }

  // 按照x_start升序排列gaps
  sort(gaps_.begin(), gaps_.end(), compareGap);

  // 第一个back和最后一个cut 没有对应的另一半
  double start_back_meeting = env_.index2x_list_[state_list_self_back_right_.front()[1]];
  double start_back_ending = env_.index2x_list_[state_list_self_back_right_.front()[2]];
  double end_cut_meeting = env_.index2x_list_[state_list_self_cut_right_.back()[1]];
  double end_cut_ending = env_.index2x_list_[state_list_self_cut_right_.back()[2]];
  if (gaps_.size()>=2){
      if (env_.veh_x_switching_for_down_borderline_individual_[state_list_self_back_right_.front()[0]].first == -999)
        env_.veh_x_switching_for_down_borderline_individual_[state_list_self_back_right_.front()[0]].first = start_back_meeting;
      gaps_[0].addXBack(start_back_meeting, start_back_ending, env_.veh_x_switching_for_down_borderline_individual_[state_list_self_back_right_.front()[0]].first);
      gaps_[0].setIndividualRightIndex(state_list_self_back_right_.front()[0]); // back的index
      // 对最右边的车 没有cut
      if (env_.veh_x_switching_for_down_borderline_individual_[state_list_self_cut_right_.back()[0]].second == 999){
        env_.veh_x_switching_for_down_borderline_individual_[state_list_self_cut_right_.back()[0]].second = end_cut_meeting;
        gaps_[gaps_.size()-1].addXCut(end_cut_meeting, end_cut_ending, env_.veh_x_switching_for_down_borderline_individual_[state_list_self_cut_right_.back()[0]].second);
        gaps_[gaps_.size()-1].setIndividualRightIndex(state_list_self_cut_right_.back()[0]+1); // back的index
        gaps_[gaps_.size()-1].has_cut = false;
      }
      else{
        gaps_[gaps_.size()-1].addXCut(end_cut_meeting, end_cut_ending, env_.veh_x_switching_for_down_borderline_individual_[state_list_self_cut_right_.back()[0]].second);
        gaps_[gaps_.size()-1].setIndividualRightIndex(state_list_self_cut_right_.back()[0]+1); // back的index
      }
  }

  std::cout<<"-----  "<<gaps_.size()<<std::endl;
  for (auto gap:gaps_){
    std::cout<<gap.individual_right_index<<";  ";
    std::cout<<"("<<gap.x_start<<", "<<gap.x_end<<"):  ";
    std::cout<<"("<<gap.x_normal_start<<", "<<gap.x_normal_end<<");  ";
    std::cout<<"("<<gap.x_cut_constraint_meeting<<", "<<gap.x_cut_switching<<")  ";
    std::cout<<"("<<gap.x_back_constraint_meeting<<", "<<gap.x_back_switching<<")";
    std::cout<<std::endl;
  }
}

std::vector<std::vector<double>> ChickenGame::findPotentialEndPoints(std::string path_type, std::pair<float,float> switch_point, std::pair<float,float> constrain_point1, std::pair<float,float> constrain_point2){
    std::vector<std::vector<double>> temp;
    return temp;

    // 获取圆心
    float a1, b1, c1, d1;
    float a2, b2, c2, d2;
    float a3, b3, c3, d3;

    float x1 = switch_point.first, y1 = switch_point.second, z1 = 0;
    float x2 = constrain_point1.first, y2 = constrain_point1.second, z2 = 0;
    float x3 = constrain_point2.first, y3 = constrain_point2.second, z3 = 0;

    a1 = (y1*z2 - y2*z1 - y1*z3 + y3*z1 + y2*z3 - y3*z2);
    b1 = -(x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2);
    c1 = (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
    d1 = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1);

    a2 = 2 * (x2 - x1);
    b2 = 2 * (y2 - y1);
    c2 = 2 * (z2 - z1);
    d2 = x1 * x1 + y1 * y1 + z1 * z1 - x2 * x2 - y2 * y2 - z2 * z2;

    a3 = 2 * (x3 - x1);
    b3 = 2 * (y3 - y1);
    c3 = 2 * (z3 - z1);
    d3 = x1 * x1 + y1 * y1 + z1 * z1 - x3 * x3 - y3 * y3 - z3 * z3;

    std::pair<double, double> centerpoint;
    centerpoint.first = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1) / (a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    centerpoint.second = (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1) / (a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);

    return generateEndPoints(path_type, centerpoint, switch_point, constrain_point1,constrain_point2);
}


std::vector<std::vector<double>> ChickenGame::generateEndPoints(std::string type, std::pair<float,float> center_point, std::pair<float,float>switch_point, std::pair<float,float> constrain_point1, std::pair<float,float> constrain_point2){
    float center_x = center_point.first;
    float center_y = center_point.second;
    std::pair<float,float> real_constrain_point;
    // 更靠近边界的那个点
    if (abs(constrain_point1.second)>abs(constrain_point2.second))
        real_constrain_point = constrain_point1;
    else
        real_constrain_point = constrain_point2;
    
    assert(std::hypot(switch_point.second-center_y, switch_point.first-center_x)>0);
    float init_curve = 1 / std::hypot(switch_point.second-center_y, switch_point.first-center_x);
    float init_angle = std::atan2(switch_point.second-center_y, switch_point.first-center_x);
    // 其他候选end point的弧长一定大于基础弧长
    float init_arc_length = std::hypot(switch_point.second-real_constrain_point.second, switch_point.first-real_constrain_point.first);

    float step_arc_length = 0.05;
    float step_curve = 0.05;
    std::vector<std::vector<double>> potential_end_points;   // { {x,y,thta},  }
    if (type=="cutIn"){
        potential_end_points.push_back({real_constrain_point.first, real_constrain_point.second, init_angle-float(M_PI_2)});
        for (float cur=init_curve-step_curve; cur>0; cur-= step_curve){
            float x_o = switch_point.first - 1/init_curve * cos(init_angle);
            float y_o = switch_point.second - 1/init_curve * sin(init_angle);
            std::vector<double> current_end_point;
            for (float arc_length=init_arc_length; ; arc_length+=step_arc_length){
                float alpha = arc_length*cur;
                float x_end = x_o + 1/init_curve * cos(init_angle-alpha);
                float y_end = y_o + 1/init_curve * sin(init_angle-alpha);
                float theta_end = init_angle -float(M_PI_2) - alpha;
                // check collision
                geometry_msgs::Point p; p.x = x_end; p.y=y_end; p.z=theta_end;
                bool is_collision = env_.isVehCollisionBorderOrStaticVeh(p, env_.y_negative_footprint_);
                if (!is_collision)
                    current_end_point = {x_end,y_end,theta_end};
                else
                    break;
            }
            if (!current_end_point.empty())
                potential_end_points.push_back(current_end_point);
        }
    }

    else if (type=="backUp"){
        potential_end_points.push_back({real_constrain_point.first, real_constrain_point.second, init_angle-float(M_PI_2)});
        for (float cur=init_curve-step_curve; cur>0; cur-= step_curve){
            float x_o = switch_point.first - 1/init_curve * cos(init_angle);
            float y_o = switch_point.second - 1/init_curve * sin(init_angle);
            std::vector<double> current_end_point;
            for (float arc_length=init_arc_length; ; arc_length+=step_arc_length){
                float alpha = arc_length*cur;
                float x_end = x_o + 1/init_curve * cos(init_angle+alpha);
                float y_end = y_o + 1/init_curve * sin(init_angle+alpha);
                float theta_end = init_angle -float(M_PI_2) + alpha;
                // check collision
                geometry_msgs::Point p; p.x = x_end; p.y=y_end; p.z=theta_end;
                bool is_collision = env_.isVehCollisionBorderOrStaticVeh(p, env_.y_negative_footprint_);
                if (!is_collision)
                    current_end_point = {x_end,y_end,theta_end};
                else
                    break;
            }
            if (!current_end_point.empty())
                potential_end_points.push_back(current_end_point);
        }
    }

    else if (type=="turnLeft"){
        potential_end_points.push_back({real_constrain_point.first, real_constrain_point.second, init_angle+float(M_PI_2)});
        for (float cur=init_curve-step_curve; cur>0; cur-= step_curve){
            float x_o = switch_point.first - 1/init_curve * cos(init_angle);
            float y_o = switch_point.second - 1/init_curve * sin(init_angle);
            std::vector<double> current_end_point;
            for (float arc_length=init_arc_length; ; arc_length+=step_arc_length){
                float alpha = arc_length*cur;
                float x_end = x_o + 1/init_curve * cos(init_angle-alpha);
                float y_end = y_o + 1/init_curve * sin(init_angle-alpha);
                float theta_end = init_angle +float(M_PI_2) - alpha;
                // check collision
                geometry_msgs::Point p; p.x = x_end; p.y=y_end; p.z=theta_end;
                bool is_collision = env_.isVehCollisionBorderOrStaticVeh(p, env_.y_negative_footprint_);
                if (!is_collision)
                    current_end_point = {x_end,y_end,theta_end};
                else
                    break;
            }
            if (!current_end_point.empty())
                potential_end_points.push_back(current_end_point);
        }
    }

    return potential_end_points;
}

void ChickenGame::pubPathsForTEB(std::vector<std::vector<std::vector<double>>> potential_paths, std::vector<std::string> paths_types, std::vector<std::vector<double>> path, std::string path_type){

  assert(potential_paths.size()==paths_types.size());
  visualization_msgs::MarkerArray paths_marker_array_for_show;
  int idx = 0;
  double height = 0.0;
  for (auto path:potential_paths){
    height += 0.05;
    visualization_msgs::Marker path_marker_for_show;
    path_marker_for_show.header.stamp = ros::Time::now();
    path_marker_for_show.header.frame_id = frame_id_;
    path_marker_for_show.ns = paths_types[idx];
    path_marker_for_show.id = idx++;
    path_marker_for_show.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker_for_show.action = visualization_msgs::Marker::ADD;
    path_marker_for_show.lifetime = ros::Duration(1.0);
    path_marker_for_show.pose.orientation.w = 1.0;
    path_marker_for_show.scale.x = 0.01;
    path_marker_for_show.scale.y = 0.01;
    path_marker_for_show.color.a = 1.0;
    path_marker_for_show.color.r = 1.0;
    path_marker_for_show.color.g = 0.0;
    path_marker_for_show.color.b = 0.0;
    for (auto poi:path){
      geometry_msgs::Point p; p.x=poi[0], p.y=poi[1]; p.z = height;
      path_marker_for_show.points.push_back(p);
    }
    paths_marker_array_for_show.markers.push_back(path_marker_for_show);
  }
  pub_paths_for_show_.publish(paths_marker_array_for_show);  



  visualization_msgs::Marker path_marker_for_teb;
  path_marker_for_teb.header.stamp = ros::Time::now();
  path_marker_for_teb.header.frame_id = frame_id_;
  path_marker_for_teb.ns = path_type;
  path_marker_for_teb.id = 0;
  path_marker_for_teb.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker_for_teb.action = visualization_msgs::Marker::ADD;
  path_marker_for_teb.lifetime = ros::Duration(1.0);
  path_marker_for_teb.pose.orientation.w = 1.0;
  path_marker_for_teb.scale.x = 0.01;
  path_marker_for_teb.scale.y = 0.01;
  path_marker_for_teb.color.a = 1.0;
  path_marker_for_teb.color.r = 1.0;
  path_marker_for_teb.color.g = 0.0;
  path_marker_for_teb.color.b = 0.0;

  if(path_type == "stop"){
    geometry_msgs::Point p; p.x=automated_vehicle_odom_.pose.pose.position.x; p.y=automated_vehicle_odom_.pose.pose.position.y; p.z = tf::getYaw(automated_vehicle_odom_.pose.pose.orientation);; // z轴上保存yaw角
    path_marker_for_teb.points.push_back(p); 
    path_marker_for_teb.points.push_back(p); 
    path_marker_for_teb.points.push_back(p); 
  }
  else{
    double shift = 0;
    if (path_type=="normal") shift = 0;
    else if (path_type=="turnLeft") shift = -0.05;
    else shift = 0.05;
      
    for (auto poi:path){
      geometry_msgs::Point p; p.x=poi[0], p.y=poi[1]+shift; p.z = poi[2]; // z轴上保存yaw角
      path_marker_for_teb.points.push_back(p);
    }
  }
  pub_path_for_teb_.publish(path_marker_for_teb);
}

void ChickenGame::pubPotentialEnds(std::vector<std::vector<std::vector<double>>> path_ends){
  visualization_msgs::MarkerArray paths_ends_markers;
  int idx = 0;
  double height = 0.3;
  for (auto ends:path_ends){
    height += 0.05;
    visualization_msgs::Marker ends_marker;
    ends_marker.header.stamp = ros::Time::now();
    ends_marker.header.frame_id = frame_id_;
    ends_marker.id = idx++;
    ends_marker.type = visualization_msgs::Marker::POINTS;
    ends_marker.action = visualization_msgs::Marker::ADD;
    ends_marker.lifetime = ros::Duration(1.0);
    ends_marker.pose.orientation.w = 1.0;
    ends_marker.scale.x = 0.02;
    ends_marker.scale.y = 0.02;
    ends_marker.color.a = 1.0;
    ends_marker.color.r = 1.0;
    ends_marker.color.g = 0.0;
    ends_marker.color.b = 0.0;
    for (auto point:ends){
      geometry_msgs::Point p;
      p.x = point[0];
      p.y = point[1];
      p.z = height;
      ends_marker.points.push_back(p);
    }
    paths_ends_markers.markers.push_back(ends_marker);
  }
  pub_potential_ends_.publish(paths_ends_markers);
}

// 发布分割后的区域：可会车区域、不可会车区域
void ChickenGame::pubSegmentArea(){

  int idx = 0;
  // segmented_states_right_
  for (int i=1; i<segmented_states_right_.size(); i++){    
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.id = idx++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // marker.lifetime = ros::Duration(2.0);
    geometry_msgs::Point start;
    start.x = segmented_states_right_[i-1].first;
    start.y = 0;
    start.z = 0.2;
    marker.points.push_back(start);
    geometry_msgs::Point end;
    end.x = segmented_states_right_[i].first;
    end.y = 0;
    end.z = 0.2;
    marker.points.push_back(end);

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.color.a = 1.0;
    if (segmented_states_right_[i-1].second==1){
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    else if (segmented_states_right_[i-1].second==-1){
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    }
    pub_states_right_.publish( marker );
  }

  // segmented_states_right_pure_
  for (int i=1; i<segmented_states_right_pure_.size(); i++){    
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.id = idx++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // marker.lifetime = ros::Duration(2.0);
    geometry_msgs::Point start;
    start.x = segmented_states_right_pure_[i-1].first;
    start.y = 0;
    start.z = 0.3;
    marker.points.push_back(start);
    geometry_msgs::Point end;
    end.x = segmented_states_right_pure_[i].first;
    end.y = 0;
    end.z = 0.3;
    marker.points.push_back(end);

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.color.a = 1.0;
    if (segmented_states_right_pure_[i-1].second==1){
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    else if (segmented_states_right_pure_[i-1].second==-1){
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    }
    pub_states_right_pure_.publish( marker );
  }

  // segmented_states_left_
  for (int i=1; i<segmented_states_left_.size(); i++){    
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.id = idx++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // marker.lifetime = ros::Duration(2.0);
    geometry_msgs::Point start;
    start.x = segmented_states_left_[i-1].first;
    start.y = 0;
    start.z = 0.4;
    marker.points.push_back(start);
    geometry_msgs::Point end;
    end.x = segmented_states_left_[i].first;
    end.y = 0;
    end.z = 0.4;
    marker.points.push_back(end);

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.color.a = 1.0;
    if (segmented_states_left_[i-1].second==1){
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    else if (segmented_states_left_[i-1].second==-1){
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    }
    pub_states_left_.publish( marker );
  }

  // segmented_states_back_
  for (int i=0; i<state_list_self_back_right_.size(); i++){
    if (state_list_self_back_right_[i][1]==-1)
      continue;  
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.id = idx++;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // marker.lifetime = ros::Duration(2.0);
    geometry_msgs::Point p;
    p.x = env_.index2x_list_[state_list_self_back_right_[i][1]];
    p.y = env_.down_borderline_individual_[state_list_self_back_right_[i][0]][state_list_self_back_right_[i][1]];
    p.z = 0.1*state_list_self_cut_right_[i][0];
    marker.points.push_back(p);
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    pub_states_back_.publish( marker );
  }

  // segmented_states_cut_
  for (int i=0; i<state_list_self_cut_right_.size(); i++){
    if (state_list_self_cut_right_[i][1]==-1)
      continue;
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.id = idx++;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // marker.lifetime = ros::Duration(2.0);
    geometry_msgs::Point p;
    p.x = env_.index2x_list_[state_list_self_cut_right_[i][1]];
    p.y = env_.down_borderline_individual_[state_list_self_cut_right_[i][0]][state_list_self_cut_right_[i][1]];
    p.z = 0.1*(state_list_self_cut_right_[i][0]+1);
    marker.points.push_back(p);
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    pub_states_cut_.publish( marker );
  }

  // pub_potential_gaps_cuts_and_backs_
  for (auto gap:gaps_){
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.a = 1.0;
    // marker.lifetime = ros::Duration(2.0);
    geometry_msgs::Point p;
    if (gap.has_back){
      marker.id = idx++;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;      
      p.x = gap.x_back_constraint_meeting;
      p.y = env_.down_borderline_individual_[gap.individual_right_index][env_.xPos2Index(p.x)];
      // p.z = gap.individual_right_index*0.1;
      p.z = 0.02;
      marker.points = {p};

      p.x = gap.x_back_constraint_ending;
      p.y = env_.down_borderline_individual_[gap.individual_right_index][env_.xPos2Index(p.x)];
      // p.z = gap.individual_right_index*0.1;
      p.z = 0.02;
      marker.points.push_back(p);
      marker.ns="constraints";
      pub_potential_gaps_cuts_and_backs_.publish( marker );

      p.x = gap.x_back_switching;
      p.y = env_.down_borderline_individual_[gap.individual_right_index][env_.xPos2Index(p.x)];
      // p.z = gap.individual_right_index*0.1;
      p.z = 0.02;
      marker.points = {p};
      marker.ns="switching";
      pub_potential_gaps_cuts_and_backs_.publish( marker );
    }
    if (gap.has_cut){
      marker.id = idx++;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      p.x = gap.x_cut_constraint_meeting;
      p.y = env_.down_borderline_individual_[gap.individual_right_index-1][env_.xPos2Index(p.x)];
      // p.z = gap.individual_right_index*0.1;
      p.z = 0.02;
      marker.points = {p};

      p.x = gap.x_cut_constraint_ending;
      p.y = env_.down_borderline_individual_[gap.individual_right_index-1][env_.xPos2Index(p.x)];
      // p.z = gap.individual_right_index*0.1;
      p.z = 0.02;
      marker.points.push_back(p);
      marker.ns="constraints";
      pub_potential_gaps_cuts_and_backs_.publish( marker );

      p.x = gap.x_cut_switching;
      p.y = env_.down_borderline_individual_[gap.individual_right_index-1][env_.xPos2Index(p.x)];
      // p.z = gap.individual_right_index*0.1;
      p.z = 0.02;
      marker.points={p};
      marker.ns="switching";
      pub_potential_gaps_cuts_and_backs_.publish( marker );
    }
    if (gap.has_left){
      marker.id = idx++;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      p.x = gap.x_left_constraint_meeting;
      p.y = env_.up_borderline_individual_for_left_backup_[gap.individual_left_index][env_.xPos2Index(p.x)];
      // p.z = gap.individual_left_index*0.1;
      p.z = 0.02;
      marker.points = {p};

      p.x = gap.x_left_constraint_ending;
      p.y = env_.up_borderline_individual_for_left_backup_[gap.individual_left_index][env_.xPos2Index(p.x)];
      // p.z = gap.individual_left_index*0.1;
      p.z = 0.02;
      marker.points.push_back(p);
      marker.ns="constraints";
      pub_potential_gaps_cuts_and_backs_.publish( marker );

      p.x = gap.x_left_switching;
      p.y = env_.up_borderline_individual_for_left_backup_[gap.individual_left_index][env_.xPos2Index(p.x)];
      // p.z = gap.individual_left_index*0.1;
      p.z = 0.02;
      marker.points={p};
      marker.ns="switching";
      pub_potential_gaps_cuts_and_backs_.publish( marker );
    }    
  }

  // gaps_
  for (int i=0; i<gaps_.size(); i++){    
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.id = idx++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // marker.lifetime = ros::Duration(2.0);
    geometry_msgs::Point start;
    start.x = gaps_[i].x_start;
    start.y = 0;
    start.z = 0.05;
    marker.points.push_back(start);
    geometry_msgs::Point end;
    end.x = gaps_[i].x_end;
    end.y = 0;
    end.z = 0.05;
    marker.points.push_back(end);

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    pub_gaps_.publish( marker );
  }

}

void ChickenGame::pubCurrentGap(potentialGap current_gap){
  if (!current_gap.is_init)
    return;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id_;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = ros::Duration(10.0);
  geometry_msgs::Point start;
  start.x = gaps_[current_gap.gap_index].x_start;
  start.y = 0;
  start.z = 0.1;
  marker.points.push_back(start);
  geometry_msgs::Point end;
  end.x = gaps_[current_gap.gap_index].x_end;
  end.y = 0;
  end.z = 0.1;
  marker.points.push_back(end);

  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  pub_meeting_gap_.publish( marker );
}
