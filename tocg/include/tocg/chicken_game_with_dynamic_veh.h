#ifndef CHICKEN_GAME_WITH_DYNAMIC_VEH_H
#define CHICKEN_GAME_WITH_DYNAMIC_VEH_H
#include <tocg/extract_static_env.h>

struct Gap{
  Gap(){has_right=false; has_left=false; has_cut=false; has_back=false; }
  Gap(double start_input, double end_input){
    x_start=start_input; x_end = end_input; 
    x_normal_start=start_input; x_normal_end=end_input;
    has_right = true;
    has_left = false;
    has_cut = false;
    has_back = false;
  }
  void setIndividualRightIndex(double individual_right_index_input){
    individual_right_index = individual_right_index_input;
  }
  void setIndividualLeftIndex(double individual_left_index_input){
    individual_left_index = individual_left_index_input;
  }
  void setLeftConstrains(double x_left_constraint_meeting_input, double x_left_constraint_ending_input, double x_left_switching_input){
    x_left_switching = x_left_switching_input;
    x_left_constraint_meeting = x_left_constraint_meeting_input;
    x_left_constraint_ending = x_left_constraint_ending_input;
    has_left = true;
  }

  void initializeWithCutAndBack(double x_cut_constraint_meeting_input, double x_back_constraint_meeting_input, double x_cut_constraint_ending_input, double x_back_constraint_ending_input, double x_cut_switching_input, double x_back_switching_input){
    x_start = x_cut_constraint_meeting_input;
    x_end = x_back_constraint_meeting_input;
    x_normal_start = -999;
    x_normal_end = -999;
    x_cut_constraint_meeting = x_cut_constraint_meeting_input;
    x_cut_constraint_ending = x_cut_constraint_ending_input;
    x_cut_switching = x_cut_switching_input;
    x_back_constraint_meeting = x_back_constraint_meeting_input;
    x_back_constraint_ending = x_back_constraint_ending_input;
    x_back_switching = x_back_switching_input;
    has_cut = true;
    has_back = true;
  }
  void addXCut(double x_cut_constraint_meeting_input, double x_cut_constraint_ending_input, double x_cut_switching_input){
    x_cut_constraint_meeting = x_cut_constraint_meeting_input;
    x_cut_constraint_ending = x_cut_constraint_ending_input;
    x_cut_switching = x_cut_switching_input;
    x_start = std::min(x_start,x_cut_constraint_meeting);
    has_cut = true;
  }
  void addXBack(double x_back_constraint_meeting_input, double x_back_constraint_ending_input, double x_back_switching_input){
    x_back_constraint_meeting = x_back_constraint_meeting_input;
    x_back_constraint_ending = x_back_constraint_ending_input;
    x_back_switching = x_back_switching_input;
    x_end = std::max(x_back_constraint_meeting,x_end);
    has_back = true;
  }

  double x_start, x_end;
  double x_normal_start, x_normal_end;
  double x_left_switching, x_left_constraint_meeting, x_left_constraint_ending;
  double x_cut_switching, x_cut_constraint_meeting, x_cut_constraint_ending;
  double x_back_switching, x_back_constraint_meeting, x_back_constraint_ending;
  bool has_right, has_left, has_cut, has_back;
  double individual_right_index=-1; // back的index
  double individual_left_index=-1; // back的index
};

// {gap的index，离边界的距离，gap的长度}
struct potentialGap{
  potentialGap(){is_init=false;};
  potentialGap(int i, double d, double l):gap_index(i),distance(d),length(l),is_init(true){};
  bool is_init;
  int gap_index;
  double distance;
  double length;
};

class ChickenGame{
  public:
    ChickenGame(){};
    ~ChickenGame(){};
    void initialize(const ros::NodeHandle& publicHandle, const ros::NodeHandle& privateHandle, const ExtractEnv env);
    void dynamicVehCB(const vehicle_simulator::SocialVehicles::ConstPtr& msg);
    void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    // 提取所有可行的gap
    void extractGaps();
    void combineGaps();
    /**
     * @input: gap的x坐标
     * @output: 潜在的gaps
     * @description: 如果gaps的长度为一，那么说明会车点在gap内或自车和对面车之间只剩下一个gap了； 如果长度为二，说明会车区域有争议，需要开始博弈了。
     * **/
    std::vector<potentialGap> findNearestGap(const double& gap_x);
    void generateTEBPaths(const std::string& decision_state, const nav_msgs::Odometry& automated_vehicle_odom, potentialGap pG, const std::vector<double>& normal_considering_dynamic_veh, std::vector<std::vector<std::vector<double>>>& potential_paths, std::vector<std::string>& paths_types, std::vector<std::vector<std::vector<double>>>& potential_paths_ends);

    /**
     * find potential end points for a path
     * **/
    std::vector<std::vector<double>> findPotentialEndPoints(std::string path_type, std::pair<float,float> switch_point, std::pair<float,float> constrain_point1, std::pair<float,float> constrain_point2);
    std::vector<std::vector<double>> generateEndPoints(std::string type, std::pair<float,float> center_point, std::pair<float,float>switch_point, std::pair<float,float> constrain_point1, std::pair<float,float> constrain_point2);

    void selectPathForTEB(std::vector<std::vector<std::vector<double>>> paths, std::vector<std::string> paths_types, Gap final_gap, std::string ref_path_type, bool is_avoid_by_left, bool is_back_up, std::vector<std::vector<double>>& selected_path, std::string& selected_type);

    // 发布分割后的区域：可会车区域、不可会车区域
    void pubSegmentArea();
    void pubPathsForTEB(std::vector<std::vector<std::vector<double>>> potential_paths, std::vector<std::string> paths_types, std::vector<std::vector<double>> path, std::string paths_type);
    void pubPotentialEnds(std::vector<std::vector<std::vector<double>>> path_ends);
    void pubCurrentGap(potentialGap current_gap);
    
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle privatenh_;
    std::string frame_id_;
    ExtractEnv env_;
    std::vector<std::pair<double, double>> segmented_states_right_; // 两车均向右走形成的gap(extended by footprint)
    std::vector<std::pair<double, double>> segmented_states_right_pure_; // 两车均向右走形成的gap(no extend)
    std::vector<std::pair<double, double>> segmented_states_left_;  // 两车均向左走形成的gap
    std::vector<int> state_list_left_;
    std::vector<std::vector<int>> state_list_self_back_left_;          // 自车向左侧back所形成的额外的gap {individual在up_borderline_individual_对应的index, individual内具体的index位置, ending的index}
    std::vector<std::vector<int>> state_list_self_back_right_;          // 自车向右侧back所形成的额外的gap {individual在down_borderline_individual_对应的index, individual内具体的index位置, ending的index}
    std::vector<std::vector<int>> state_list_self_cut_right_;           // 自车cut所形成的额外的gap
    std::vector<Gap> gaps_;
    
    nav_msgs::Odometry automated_vehicle_odom_;
    geometry_msgs::PoseStamped automated_vehicle_goal_;

    bool isAvInitialized_;
    bool isAVGoalRecivied_;

    ros::Subscriber sub_dynamic_social_vehicles_;
    ros::Subscriber sub_automated_vehicle_odom_;
    ros::Subscriber sub_automated_vehicle_goal_;
    ros::Publisher pub_paths_for_show_;
    ros::Publisher pub_path_for_teb_;
    ros::Publisher pub_states_right_;
    ros::Publisher pub_states_right_pure_;
    ros::Publisher pub_states_left_;
    ros::Publisher pub_states_back_;
    ros::Publisher pub_states_cut_;
    ros::Publisher pub_potential_gaps_cuts_and_backs_;
    ros::Publisher pub_gaps_;
    ros::Publisher pub_meeting_gap_;
    ros::Publisher pub_potential_ends_;
};


#endif
