#include<tocg/extract_static_env.h>

double WHEELBASE = 0.144;
// double VEHWIDTH = 0.186;
double VEHWIDTH = 0.206;
double MIN_TURNING_RADIUS = 0.350;
// std::vector<std::pair<double,double>> FOOTPRINT_CENTER ={{-0.101, -0.093}, {-0.101, 0.093}, {0.139, 0.093}, {0.139, -0.093}};
// 比真正的footprint扩大一圈 (0.01)
// std::vector<std::pair<double,double>> FOOTPRINT_REAR ={{-0.04, -0.093}, {-0.04, 0.093}, {0.22, 0.093}, {0.22, -0.093}};
std::vector<std::pair<double,double>> FOOTPRINT_REAR ={{-0.05, -0.103}, {-0.05, 0.103}, {0.23, 0.103}, {0.23, -0.103}};

int getSign(const double& input){
  if (input>0) return 1;
  else if (input<0) return -1;
  return 0;
}


/** 
 * @input: vector<Point>
 * @description: 升序排列
 * **/
bool comparePoints(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return p1.x<=p2.x;
}

/** 
 * @input: vector<visualization_msgs::Marker>
 * @description: 按x升序排列
 * **/
bool compareMarkers(visualization_msgs::Marker veh1, visualization_msgs::Marker veh2)
{
  geometry_msgs::Point center1;
  for (int i=0; i<veh1.points.size()-1; i++)
    center1.x += veh1.points[i].x;
  center1.x /= (veh1.points.size()-1);
  geometry_msgs::Point center2;
  for (int i=0; i<veh2.points.size()-1; i++)
    center2.x += veh2.points[i].x;
  center2.x /= (veh2.points.size()-1);
  return center1.x < center2.x;
}

/** 
 * @input: vector< vector<pair> >
 * @description: 升序排列
 * **/
bool compareList(std::vector<std::pair<double,double>> p1_list, std::vector<std::pair<double,double>> p2_list)
{
  auto center1_x = p1_list.back().first;
  auto center2_x = p2_list.back().first;
  return center1_x<center2_x;
}

ExtractEnv::~ExtractEnv(){
  sub_borderline_.shutdown();
  sub_static_vehs_.shutdown();
}

void ExtractEnv::initializeExt(){
  is_borderline_initialized_ = false;
  is_static_veh_projected_ = false;
  step_size_ = 0.001;
  theta_step_ = 0.001;
  draw_step_ = 0;
  xMin_=999; xMax_=-999; yMin_=999; yMax_=-999;
  nh_.param<std::string>("frame_id", frame_id_, "odom");

  // 下边界
  y_negative_vehwidth_ = VEHWIDTH;
  y_negative_min_turning_radius_ = MIN_TURNING_RADIUS;
  y_negative_footprint_ = FOOTPRINT_REAR;
  // 上边界
  y_positive_vehwidth_ = VEHWIDTH;
  y_positive_min_turning_radius_ = MIN_TURNING_RADIUS;
  y_positive_footprint_ = FOOTPRINT_REAR;

  sub_borderline_ = nh_.subscribe<visualization_msgs::Marker>("/simulate_environment/walls", 3, boost::bind(&ExtractEnv::borderlineCB, this ,_1));
  sub_static_vehs_ = nh_.subscribe<visualization_msgs::MarkerArray>("/simulate_social_car/static_sv_polys", 3, boost::bind(&ExtractEnv::staticVehCB, this ,_1));
  // sub_static_vehs_ = nh_.subscribe<visualization_msgs::MarkerArray>("/simulate_social_car/static_sv_extended_polys", 3, boost::bind(&ExtractEnv::staticVehCB, this ,_1));

  pub_up_borderline_ = privatenh_.advertise<visualization_msgs::Marker>("up_borderline",3);
  pub_up_borderline_extended_right_ = privatenh_.advertise<visualization_msgs::Marker>("up_borderline_extend_right",3);
  pub_up_borderline_extended_left_ = privatenh_.advertise<visualization_msgs::Marker>("up_borderline_extend_left",3);
  pub_down_borderline_ = privatenh_.advertise<visualization_msgs::Marker>("down_borderline",3);
  pub_down_borderline_extended_right_ = privatenh_.advertise<visualization_msgs::Marker>("down_borderline_extend_right",3);
  pub_down_borderline_extended_left_ = privatenh_.advertise<visualization_msgs::Marker>("down_borderline_extend_left",3);

  pub_up_borderline_individual_ = privatenh_.advertise<visualization_msgs::Marker>("up_borderline_individual",3);
  pub_up_borderline_individual_extended_ = privatenh_.advertise<visualization_msgs::Marker>("up_borderline_individual_extend",3);
  pub_down_borderline_individual_ = privatenh_.advertise<visualization_msgs::Marker>("down_borderline_individual",3);
  pub_down_borderline_individual_extended_ = privatenh_.advertise<visualization_msgs::Marker>("down_borderline_individual_extend",3);
  pub_borderline_with_static_vehicles_footprints_ = privatenh_.advertise<visualization_msgs::Marker>("borderlines_with_static_vehicles_footprints",3);

  pub_borderline_messsages_for_teb_ = privatenh_.advertise<teb_local_planner::borderConstraintsMsg>("borderline_messsages_for_teb",3);
  pub_stripe_borderline_ = privatenh_.advertise<visualization_msgs::Marker>("stripe_borderline",3);
  pub_footprints_ = privatenh_.advertise<visualization_msgs::Marker>("intermediate_footprints",3);
}

// 世界坐标系 转化到 数组的index
int ExtractEnv::xPos2Index(const double& xPos){
    return int((xPos-xMin_)/step_size_);
}

// 数组的index 转化到 世界坐标系
double ExtractEnv::Index2xPos(const int& index){
    return index2x_list_[index];
}

/**
 * @input: 根据车当前位置旋转机器人的footprint的一条边p1p2
 * @descrption: 离散化p1p2，将点加入到intermediate_footprints_; 此函数被迭代调用
 * **/
void ExtractEnv::addFootprintPoints(geometry_msgs::Point center, double theta, std::vector<std::pair<double, double>> footprint){
  std::vector<geometry_msgs::Point> temp;
  for (int j=0; j<footprint.size(); j++){
    geometry_msgs::Point p; p.x=footprint[j].first; p.y=footprint[j].second;
    geometry_msgs::Point p_rotated;
    p_rotated.x = center.x + cos(theta)*p.x + -sin(theta)*p.y;
    p_rotated.y = center.y + sin(theta)*p.x + cos(theta)*p.y;
    p_rotated.z = 0.05;
    temp.push_back(p_rotated);
  }
  intermediate_footprints_.push_back(temp);
}

/**
 * @input: 一个borderline的copy版 temp_vec; 两个点footprint的边线的端点p1、p2; 一个自车的位置点center
 * @description: 离散化p1p2，将点加入到temp_vec里，如果center在中心往下，则对temp_vec去max；否则取min
 * */
void ExtractEnv::addBorderPointsToSpecialVec(std::vector<double>& temp_vec, const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, const geometry_msgs::Point& center){
  double x_start = point1.x;
  double y_start = point1.y;
  double x_end = point2.x;
  double y_end = point2.y;
  double x_step = step_size_;
  if (x_start>x_end){
    double temp = x_start;
    x_start = x_end;
    x_end = temp;
    temp = y_start;
    y_start = y_end;
    y_end = temp;
  }
  double y_step = (y_end-y_start)/abs(x_end-x_start)*abs(x_step);

  for (double x=x_start, y=y_start; x<=x_end; x+=x_step, y+=y_step){
    if (x>=xMin_ && x<=xMax_){
      int index = xPos2Index(x);
      if (center.y>0) temp_vec[index] = std::min(y, temp_vec[index]);
      else if (center.y<0) temp_vec[index] = std::max(y, temp_vec[index]);
    }
  }
}

/**
 * @input: 一个borderline的copy版 temp_vec; 两个点footprint的边线的端点p1、p2; 一个自车的位置点center
 * @description: 离散化p1p2，将点加入到temp_vec里，如果center在中心往下，则对temp_vec去max；否则取min
 * */
void ExtractEnv::addBorderPointsToSpecialVecAndRecordProjections(std::vector<double>& temp_vec, const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, const geometry_msgs::Point& center){
  double x_start = point1.x;
  double y_start = point1.y;
  double x_end = point2.x;
  double y_end = point2.y;
  double x_step = step_size_;
  if (x_start>x_end){
    double temp = x_start;
    x_start = x_end;
    x_end = temp;
    temp = y_start;
    y_start = y_end;
    y_end = temp;
  }
  double y_step = (y_end-y_start)/abs(x_end-x_start)*abs(x_step);

  for (double x=x_start, y=y_start; x<=x_end; x+=x_step, y+=y_step){
    if (x>=xMin_ && x<=xMax_){
      int index = xPos2Index(x);
      if (center.y>0) temp_vec[index] = std::min(y, temp_vec[index]);
      else if (center.y<0){
        if (y<temp_vec[index]){
          int index2 = xPos2Index(center.x);;
          if (index2>=0 && index2<index2x_list_.size()){
            index_from_extended_dbr_to_dbr_[index] = std::min(index_from_extended_dbr_to_dbr_[index], index2);
          }
        }
        temp_vec[index] = std::max(y, temp_vec[index]);
      }
    }
  }
}

/**
 * @input: 任意两个点p1、p2，自车位置center
 * @description: 离散化p1p2，加入到up_borderline或down_borderline
 *               将离散点加入到stripe_borderline用作可视化
 * **/
void ExtractEnv::addBorderPoints(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, const geometry_msgs::Point& center, double height=0, bool flag=true){
  if(center.y>0 || (!flag)){
    double x_start = point1.x;
    double y_start = -point1.y;
    double x_end = point2.x;
    double y_end = -point2.y;
    double x_step = step_size_;
    if (x_start>x_end){
      double temp = x_start;
      x_start = x_end;
      x_end = temp;
      temp = y_start;
      y_start = y_end;
      y_end = temp;
    }
    double y_step = (y_end-y_start)/abs(x_end-x_start)*abs(x_step);
    for (double x=x_start, y=y_start; x<=x_end; x+=x_step, y+=y_step){
      int index = xPos2Index(x);
      up_borderline_[index] = std::min(y, up_borderline_[index]);
      stripe_borderline_.push_back({x, y, height});
    }
  }

  else if (center.y<0){
    double x_start = point1.x;
    double y_start = point1.y;
    double x_end = point2.x;
    double y_end = point2.y;
    double x_step = step_size_;
    if (x_start>x_end){
      double temp = x_start;
      x_start = x_end;
      x_end = temp;
      temp = y_start;
      y_start = y_end;
      y_end = temp;
    }
    double y_step = (y_end-y_start)/abs(x_end-x_start)*abs(x_step);
    for (double x=x_start, y=y_start; x<=x_end; x+=x_step, y+=y_step){
      int index = xPos2Index(x);
      down_borderline_[index] = std::max(y, down_borderline_[index]);
      stripe_borderline_.push_back({x, y, height});
    }
  }
}

void ExtractEnv::addAbsBorderPoints(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, const geometry_msgs::Point& center, double height=0){
  double x_start = point1.x;
  double y_start = point1.y;
  double x_end = point2.x;
  double y_end = point2.y;
  double x_step = step_size_;
  if (x_start>x_end){
    double temp = x_start;
    x_start = x_end;
    x_end = temp;
    temp = y_start;
    y_start = y_end;
    y_end = temp;
  }
  double y_step = (y_end-y_start)/abs(x_end-x_start)*abs(x_step);
  
  if(center.y>0){
    for (double x=x_start, y=y_start; x<=x_end; x+=x_step, y+=y_step){
      int index = xPos2Index(x);
      up_borderline_[index] = std::min(y, up_borderline_[index]);
      stripe_borderline_.push_back({x, y, height});
    }
  }
  else if (center.y<0){
    for (double x=x_start, y=y_start; x<=x_end; x+=x_step, y+=y_step){
      int index = xPos2Index(x);
      down_borderline_[index] = std::max(y, down_borderline_[index]);
      stripe_borderline_.push_back({x, y, height});
    }
  }
}

/**
 * @input: 任意点p、自车位置center
 * @description: 将p加入到up_borderline或down_borderline
 *               将p加入到stripe_borderline用作可视化
 * **/
void ExtractEnv::addBorderPoints(const geometry_msgs::Point& point, const geometry_msgs::Point& center, double height=0, bool flag=true){
  int index = xPos2Index(point.x);
  if(center.y>0 || (!flag)) {
    up_borderline_[index] = std::min(-point.y, up_borderline_[index]);
    stripe_borderline_.push_back({point.x, -point.y, height});
  }
  else if(center.y<0){
    down_borderline_[index] = std::max(point.y, down_borderline_[index]);
    stripe_borderline_.push_back({point.x, point.y, height});
  }
}

/**
 * @input: 一个borderline的copy版 temp_vec; 
 *         一个点p; 
 *         一个自车的位置点center
 * @description: 将p点加入到temp_vec里，如果center在中心往下，则对temp_vec去max；否则取min
 * **/
void ExtractEnv::addPointToSpecialVec(std::vector<std::pair<double,double>>& temp_vec, const geometry_msgs::Point& point, const geometry_msgs::Point& center, bool flag=true){
  if(center.y>0 || (!flag)) {
    temp_vec.push_back({point.x,-point.y});
  }
  else if(center.y<0){
    temp_vec.push_back({point.x,point.y});
  }
}

std::vector<double> ExtractEnv::addDynamicVehToBorerlineAndGainMiddlePath(geometry_msgs::Point p, std::vector<std::pair<double, double>> footprint, double normal_limited_goal){
  double min_dis_2_up = 999;
  double min_dis_2_down = 999;
  std::vector<double> res;
  for (int j=1; j<=footprint.size(); j++){
    // 每个边离散成三个点，算每个点到up和down的最小距离
    geometry_msgs::Point p1; p1.x=footprint[j-1].first; p1.y=footprint[j-1].second;
    geometry_msgs::Point p1_rotated;
    p1_rotated.x = p.x + cos(p.z)*p1.x + -sin(p.z)*p1.y;
    p1_rotated.y = p.y + sin(p.z)*p1.x + cos(p.z)*p1.y;

    geometry_msgs::Point p2; p2.x=footprint[j%footprint.size()].first; p2.y=footprint[j%footprint.size()].second;
    geometry_msgs::Point p2_rotated;
    p2_rotated.x = p.x + cos(p.z)*p2.x + -sin(p.z)*p2.y;
    p2_rotated.y = p.y + sin(p.z)*p2.x + cos(p.z)*p2.y;

    geometry_msgs::Point p3_rotated; 
    p3_rotated.x = (p1_rotated.x+p2_rotated.x)/2.0;
    p3_rotated.y = (p1_rotated.y+p2_rotated.y)/2.0;

    int index1 = xPos2Index(p1_rotated.x);
    int index2 = xPos2Index(p2_rotated.x);
    int index3 = xPos2Index(p3_rotated.x);
    min_dis_2_down = std::min(min_dis_2_down, p1_rotated.y - down_borderline_extended_right_[index1]);
    min_dis_2_down = std::min(min_dis_2_down, p2_rotated.y - down_borderline_extended_right_[index2]);
    min_dis_2_down = std::min(min_dis_2_down, p3_rotated.y - down_borderline_extended_right_[index3]);

    min_dis_2_up = std::min(min_dis_2_up, up_borderline_extended_left_[index1] - p1_rotated.y);
    min_dis_2_up = std::min(min_dis_2_up, up_borderline_extended_left_[index2] - p2_rotated.y);
    min_dis_2_up = std::min(min_dis_2_up, up_borderline_extended_left_[index3] - p3_rotated.y);
  }

  // 1. 距离都很小，dynamic vehicle堵在窄路中间，路被截止在dynamic vehicle前面
  double temp_fake_extend_margin = 0.1;
  double detour_margin = 0.2;
  if (min_dis_2_up<temp_fake_extend_margin && min_dis_2_down<temp_fake_extend_margin){
    int temp_index = xPos2Index(normal_limited_goal);
    for (int i=0; i<temp_index; i++){
      res.push_back(up_borderline_[i]*0.5+down_borderline_[i]*0.5);
    }
  }
  // 2. 离up更近，合并该dynamic vehicle到up borderline
  else if (min_dis_2_up<min_dis_2_down){
    // std::cout<<2<<std::endl;
    std::vector<double> up_borderline_copy = up_borderline_;
    for (int j=1; j<=footprint.size(); j++){
      geometry_msgs::Point p1; p1.x=footprint[j-1].first+getSign(footprint[j-1].first)*detour_margin; p1.y=footprint[j-1].second;
      geometry_msgs::Point p1_rotated;
      p1_rotated.x = p.x + cos(p.z)*p1.x + -sin(p.z)*p1.y;
      p1_rotated.y = p.y + sin(p.z)*p1.x + cos(p.z)*p1.y;

      geometry_msgs::Point p2; p2.x=footprint[j%footprint.size()].first+getSign(footprint[j%footprint.size()].first)*detour_margin; p2.y=footprint[j%footprint.size()].second;
      geometry_msgs::Point p2_rotated;
      p2_rotated.x = p.x + cos(p.z)*p2.x + -sin(p.z)*p2.y;
      p2_rotated.y = p.y + sin(p.z)*p2.x + cos(p.z)*p2.y;    
      
      geometry_msgs::Point center; center.y=1;
      addBorderPointsToSpecialVec(up_borderline_copy, p1_rotated, p2_rotated, center);
    }
    for (int i=0; i<up_borderline_copy.size(); i++)
      res.push_back(up_borderline_copy[i]*0.5 + down_borderline_[i]*0.5);
  }
  // 3. 离down更近，合并该dynamic vehicle到down borderline
  else{
    // std::cout<<3<<std::endl;
    std::vector<double> down_borderline_copy = down_borderline_;
    for (int j=1; j<=footprint.size(); j++){
      geometry_msgs::Point p1; p1.x=footprint[j-1].first+getSign(footprint[j-1].first)*detour_margin; p1.y=footprint[j-1].second;
      geometry_msgs::Point p1_rotated;
      p1_rotated.x = p.x + cos(p.z)*p1.x + -sin(p.z)*p1.y;
      p1_rotated.y = p.y + sin(p.z)*p1.x + cos(p.z)*p1.y;

      geometry_msgs::Point p2; p2.x=footprint[j%footprint.size()].first+getSign(footprint[j%footprint.size()].first)*detour_margin; p2.y=footprint[j%footprint.size()].second;
      geometry_msgs::Point p2_rotated;
      p2_rotated.x = p.x + cos(p.z)*p2.x + -sin(p.z)*p2.y;
      p2_rotated.y = p.y + sin(p.z)*p2.x + cos(p.z)*p2.y;    
      
      geometry_msgs::Point center; center.y=-1;
      addBorderPointsToSpecialVec(down_borderline_copy, p1_rotated, p2_rotated, center);
    }
    for (int i=0; i<down_borderline_copy.size(); i++)
      res.push_back(up_borderline_[i]*0.5 + down_borderline_copy[i]*0.5);
  }
  return res;
}

std::vector<double> ExtractEnv::combineTwoBorderlines(const std::vector<double>& vec1, const std::vector<double>& vec2, const geometry_msgs::Point& center){
  assert(vec1.size()==vec2.size());
  std::vector<double> vec;
  if (center.y>0)
    for (int i=0; i<vec1.size(); i++)
      vec.push_back(std::min(vec1[i],vec2[i]));

  else if (center.y<0)
    for (int i=0; i<vec1.size(); i++)
      vec.push_back(std::max(vec1[i],vec2[i]));

  return vec;
}


bool ExtractEnv::isExtendedVehCollisionBorderOrStaticVeh(const geometry_msgs::Point& p, std::vector<std::pair<double, double>> footprint){
  for (int j=1; j<=footprint.size(); j++){
    geometry_msgs::Point p1; p1.x=footprint[j-1].first; p1.y=footprint[j-1].second;
    geometry_msgs::Point p1_rotated;
    p1_rotated.x = p.x + cos(p.z)*p1.x + -sin(p.z)*p1.y;
    p1_rotated.y = p.y + sin(p.z)*p1.x + cos(p.z)*p1.y;

    geometry_msgs::Point p2; p2.x=footprint[j%footprint.size()].first; p2.y=footprint[j%footprint.size()].second;
    geometry_msgs::Point p2_rotated;
    p2_rotated.x = p.x + cos(p.z)*p2.x + -sin(p.z)*p2.y;
    p2_rotated.y = p.y + sin(p.z)*p2.x + cos(p.z)*p2.y;

    geometry_msgs::Point p3_rotated; 
    p3_rotated.x = (p1_rotated.x+p2_rotated.x)/2.0;
    p3_rotated.y = (p1_rotated.y+p2_rotated.y)/2.0;

    int index = xPos2Index(p1_rotated.x);
    if (p.y>0 && p1_rotated.y>=up_borderline_[index])
      return true;
    if (p.y<=0 && p1_rotated.y<=down_borderline_[index])
      return true;

    index = xPos2Index(p2_rotated.x);
    if (p.y>0 && p2_rotated.y>=up_borderline_[index])
      return true;
    if (p.y<=0 && p2_rotated.y<=down_borderline_[index])
      return true;

    index = xPos2Index(p3_rotated.x);
    if (p.y>0 && p3_rotated.y>=up_borderline_[index])
      return true;
    if (p.y<=0 && p3_rotated.y<=down_borderline_[index])
      return true;
  }

  return false;
}


bool ExtractEnv::isVehCollisionBorderOrStaticVeh(const geometry_msgs::Point& p, std::vector<std::pair<double, double>> footprint){
  for (int j=1; j<=footprint.size(); j++){
    geometry_msgs::Point p1; p1.x=footprint[j-1].first; p1.y=footprint[j-1].second;
    geometry_msgs::Point p1_rotated;
    p1_rotated.x = p.x + cos(p.z)*p1.x + -sin(p.z)*p1.y;
    p1_rotated.y = p.y + sin(p.z)*p1.x + cos(p.z)*p1.y;

    geometry_msgs::Point p2; p2.x=footprint[j%footprint.size()].first; p2.y=footprint[j%footprint.size()].second;
    geometry_msgs::Point p2_rotated;
    p2_rotated.x = p.x + cos(p.z)*p2.x + -sin(p.z)*p2.y;
    p2_rotated.y = p.y + sin(p.z)*p2.x + cos(p.z)*p2.y;

    geometry_msgs::Point p3_rotated; 
    p3_rotated.x = (p1_rotated.x+p2_rotated.x)/2.0;
    p3_rotated.y = (p1_rotated.y+p2_rotated.y)/2.0;

    int index = xPos2Index(p1_rotated.x);
    if (p.y>0 && p1_rotated.y>=up_borderline_with_static_vehicles_footprints_[index])
      return true;
    if (p.y<=0 && p1_rotated.y<=down_borderline_with_static_vehicles_footprints_[index])
      return true;

    index = xPos2Index(p2_rotated.x);
    if (p.y>0 && p2_rotated.y>=up_borderline_with_static_vehicles_footprints_[index])
      return true;
    if (p.y<=0 && p2_rotated.y<=down_borderline_with_static_vehicles_footprints_[index])
      return true;

    index = xPos2Index(p3_rotated.x);
    if (p.y>0 && p3_rotated.y>=up_borderline_with_static_vehicles_footprints_[index])
      return true;
    if (p.y<=0 && p3_rotated.y<=down_borderline_with_static_vehicles_footprints_[index])
      return true;
  }

  return false;
}

/**
 * @input: 地图信息
 * @description: 初始化index2x_list_, up_borderline_, down_borderline_等
 * **/
void ExtractEnv::borderlineCB(const visualization_msgs::Marker::ConstPtr& msg){
  // publish
  // if(is_borderline_initialized_ && is_static_veh_projected_){
  //     pubBorderlines();
  //     pubIntermediateFootprints();
  // }

  // 只处理一次
  if (is_borderline_initialized_)
    return;

  for (auto p:msg->points){
      xMin_ = std::min(xMin_, p.x);
      xMax_ = std::max(xMax_, p.x);
      yMin_ = std::min(yMin_, p.y);
      yMax_ = std::max(yMax_, p.y);
  }

  road_width_ = yMax_-yMin_;

  for (float x=xMin_; x<=xMax_; x+=step_size_){
      index2x_list_.push_back(x);
      up_borderline_.push_back(yMax_);
      up_borderline_extended_right_.push_back(yMax_);
      up_borderline_extended_left_.push_back(yMax_);
      down_borderline_.push_back(yMin_);
      down_borderline_extended_right_.push_back(yMin_);
      down_borderline_extended_left_.push_back(yMin_);
  }
  for(int i=0; i<index2x_list_.size(); i++)
    index_from_extended_dbr_to_dbr_.push_back(index2x_list_.size()+1);

  is_borderline_initialized_ = true;
}

/**
 * @input: 静态车辆信息
 * @description: 调用doubleWinInteract，处理正常向右会车、正常向左会车
 *               调用zeroSumInteract，处理扎入 和 倒入
 * **/
void ExtractEnv::staticVehCB(const visualization_msgs::MarkerArray::ConstPtr& msg){
  // 先处理borderline
  if (!is_borderline_initialized_)
    return;
  // 只处理一次
  if (is_static_veh_projected_)
    return;
  // 重新排列车辆，y<0的在前面，x最小的在前面
  veh_ordered_.clear();
  veh_ordered_down_.clear();
  veh_ordered_up_.clear();
  for (auto veh:msg->markers){
    geometry_msgs::Point center;
    for (int i=0; i<veh.points.size()-1; i++){
      center.y += veh.points[i].y;
    }
    center.y /= (veh.points.size()-1);
    if (center.y < 0)
      veh_ordered_down_.push_back(veh);
    else
      veh_ordered_up_.push_back(veh);
  }
  sort(veh_ordered_down_.begin(), veh_ordered_down_.end(), compareMarkers);
  sort(veh_ordered_up_.begin(), veh_ordered_up_.end(), compareMarkers);
  veh_ordered_ = veh_ordered_down_;
  veh_ordered_.insert(veh_ordered_.end(), veh_ordered_up_.begin(), veh_ordered_up_.end());
  zeroSumInteract(veh_ordered_);
  doubleWinInteract(veh_ordered_);

  // 赋值veh_x_switching_for_down_borderline_individual_
  int temp_index = -1;
  for (auto temp_down_borderline:down_borderline_individual_){
    auto veh = veh_ordered_down_[++temp_index];
    float pos_x = 0;
    for (int i=0; i<veh.points.size()-1; i++)
      pos_x += veh.points[i].x;
    pos_x /= (veh.points.size()-1);
    double temp_switch_back_x=-999, temp_switch_cut_x=999;
    for(int i=0; i<temp_down_borderline.size(); i++){
      if (temp_down_borderline[i]==yMin_)
        continue;
      if ( abs(temp_down_borderline[i]-down_borderline_[i])>0.001 ){
        if (index2x_list_[i]<pos_x)
          temp_switch_back_x = std::max(temp_switch_back_x, index2x_list_[i]);
        if (index2x_list_[i]>pos_x)
          temp_switch_cut_x = std::min(temp_switch_cut_x, index2x_list_[i]);
      }
    }
    veh_x_switching_for_down_borderline_individual_.push_back({temp_switch_back_x,temp_switch_cut_x});
    // std::cout<<"=== "<<temp_switch_back_x<<"; "<<temp_switch_cut_x<<", "<<std::endl;
  }

  // 赋值veh_x_switching_for_up_borderline_individual_
  temp_index = -1;
  for (auto temp_up_borderline:up_borderline_individual_for_left_backup_){
    auto veh = veh_ordered_up_[++temp_index];
    float pos_x = 0;
    for (int i=0; i<veh.points.size()-1; i++)
      pos_x += veh.points[i].x;
    pos_x /= (veh.points.size()-1);    
    double temp_switch_back_x=-999;
    for(int i=0; i<temp_up_borderline.size(); i++){
      if (temp_up_borderline[i]==yMax_)
        continue;
      if ( abs(temp_up_borderline[i]-up_borderline_[i])>0.001 ){
        if (index2x_list_[i] < pos_x)
          temp_switch_back_x = std::max(temp_switch_back_x, index2x_list_[i]);
      }
    }
    veh_x_switching_for_up_borderline_individual_.push_back(temp_switch_back_x);
  }
  
  is_static_veh_projected_ = true;
}

/**
 * @input: 静态车辆信息
 * @description: 0.按车辆宽度扩展边界
 *               1.把每个车最多离散成3个点，假设自车后轮贴着这些点划弧，做投影 （划弧只考虑中间弧 不考虑两侧弧）
 *               2.同一个车的离散点间的平滑连接、车辆向右转 按照自车footprint膨胀borderline
 * **/ 
void ExtractEnv::zeroSumInteract(const std::vector<visualization_msgs::Marker>& msg){

  /** 0.假设两个动态车辆靠着边界走，用车辆宽度扩展边界 **/
  float draw_index = 0;

  // 投影静态车辆到边界
  for (auto veh:msg){
    // 找车的中心点
    geometry_msgs::Point center;
    for (int i=0; i<veh.points.size()-1; i++){
      center.x += veh.points[i].x;
      center.y += veh.points[i].y;
    }
    center.x /= (veh.points.size()-1);
    center.y /= (veh.points.size()-1);

    for (int i=1; i<veh.points.size(); i++){
      addAbsBorderPoints(veh.points[i-1], veh.points[i], center);
    }
  }
  up_borderline_with_static_vehicles_footprints_ = up_borderline_;
  down_borderline_with_static_vehicles_footprints_ = down_borderline_;

  /** 遍历所有静态车辆 **/
  for (auto veh:msg){
    // 找车的中心点
    geometry_msgs::Point center;
    for (int i=0; i<veh.points.size()-1; i++){
      center.x += veh.points[i].x;
      center.y += veh.points[i].y;
    }
    center.x /= (veh.points.size()-1);
    center.y /= (veh.points.size()-1);
    // 车有四个角点，找到离中心线最近的三个点，顺序为：|p1.y|>=|p2.y|>=|p3.y|
    geometry_msgs::Point near_center_p1; near_center_p1.y = 999;
    geometry_msgs::Point near_center_p2; near_center_p2.y = 999;
    geometry_msgs::Point near_center_p3; near_center_p3.y = 999;
    for (int i=0; i<veh.points.size()-1; i++){
      const double& x = veh.points[i].x;
      const double& y = veh.points[i].y;
      if (abs(y)<=abs(near_center_p1.y)){
        near_center_p3 = near_center_p2;
        near_center_p2 = near_center_p1;
        near_center_p1.x = x;
        near_center_p1.y = y;
      }
      else if (abs(y)<=abs(near_center_p2.y)){
        near_center_p3 = near_center_p2;
        near_center_p2.x = x;
        near_center_p2.y = y;
      }
      else if (abs(y)<=abs(near_center_p3.y)){
        near_center_p3.x = x;
        near_center_p3.y = y;
      }
    }

    /** 考虑沿着down_borderline方向运动的车，考虑动态车辆的 运动学约束和半径vehwidth，投影到borderline。 
     * 共有两次投影，第一次是（至多）三个near_center_p各自形成的左、中、右三个弧的投影； 第二次是为了使每两个near_center_p的弧 是被平滑连接的，两个部分的中弧需要被连接。
     * **/
    {
      if(center.y<0){
        // 0. 剔除所有顶点near_center_p_list中 无法被执行的顶点
        std::vector<geometry_msgs::Point> near_center_p_list = {near_center_p1, near_center_p2, near_center_p3};
        std::vector<geometry_msgs::Point>::iterator iter = near_center_p_list.begin();
        while(iter!=near_center_p_list.end()){
          const double& x3 = iter->x;
          const double& y3 = iter->y;
          // 0.1 当前点 比 车沿着边沿走 还低，直接剔除掉
          if (y3<=yMin_ + y_negative_vehwidth_/2){
            iter = near_center_p_list.erase(iter);
            continue;
          };
          double beta = atan2(near_center_p1.y-y3, near_center_p1.x-x3);   //atanh() \in [-pi, pi]
          if (beta>M_PI_2) beta-=M_PI;  //cluseter beta into [-pi/2, pi/2]
          double max_beta_for_y3 = acos((y_negative_min_turning_radius_- (y3-(yMin_+y_negative_vehwidth_/2)))/y_negative_min_turning_radius_);
          // 0.2 弯太大 拐不过来，直接剔除掉
          if (abs(beta)>abs(max_beta_for_y3)){
            iter = near_center_p_list.erase(iter);
            continue;
          }
          iter++;
        }
        
        // 按照x从小到大，对顶点进行排序
        sort(near_center_p_list.begin(), near_center_p_list.end(), comparePoints);
        std::vector<std::vector<double>> states_backup;  // {(x1_minus, x_plus, x3, y3, beta),...} 存过程中的的某些当前状态，用来计算两个near_center_p生成的弧之间的连接
        std::vector<double> temp_down_borderline;
        for (float x=xMin_; x<=xMax_; x+=step_size_){
          temp_down_borderline.push_back(yMin_);
        }
        std::vector<double> temp_down_borderline1 = temp_down_borderline; 
        std::vector<double> temp_down_borderline2 = temp_down_borderline; 

        // 1. 遍历所有可执行顶点near_center_p，产生左、中、右三个弧，project到borderline
        for (auto near_center_p:near_center_p_list){
          const double y1 = yMin_ + y_negative_vehwidth_/2;
          const double& x3 = near_center_p.x;
          const double& y3 = near_center_p.y;
          // 定义beta点的正方向为逆时针，即在p_top(near_center_p1)左侧的点的beta是正的，p_top(near_center_p1)右侧的点的beta是负的
          double beta = atan2(near_center_p1.y-near_center_p.y, near_center_p1.x-near_center_p.x);  //atanh() \in [-pi, pi]
          if (beta>M_PI_2) beta-=M_PI;  //cluseter beta into [-pi/2, pi/2]

          // 1.1 计算near_center_p的中心点(x3,y3)左右两侧的弧角点(x_minus,y,theta_minus)和(x_plus,y,theta_plus) 以及最左侧和最右侧的(x1_minus,y1)和(x1_plus,y1)
          // theta_minus指的是x_minus在x3左侧，但其实 theta_minus是正的；对应的，x_plu在右侧，theta_plus是负的
          double x_minus, x_plus, y, theta_minus, theta_plus, x1_minus, x1_plus;
          y = (y1+y_negative_min_turning_radius_+y3-(y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta))/2;
          theta_minus = acos((y1+y_negative_min_turning_radius_-y3+(y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta))/(2*y_negative_min_turning_radius_));
          theta_plus = -theta_minus;  //acos() \in [0,pi]
          x_minus = x3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta) - sqrt( pow(y_negative_min_turning_radius_,2) - pow( (y1+y_negative_min_turning_radius_-y3+(y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta))/2 ,2) );
          x_plus = x3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta) + sqrt( pow(y_negative_min_turning_radius_,2) - pow( (y1+y_negative_min_turning_radius_-y3+(y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta))/2 ,2) );
          x1_minus = x_minus - y_negative_min_turning_radius_*sin(theta_minus);
          x1_plus = x_plus - y_negative_min_turning_radius_*sin(theta_plus);
          draw_index += draw_step_;

          states_backup.push_back({x1_minus, x1_plus, x_minus, x_plus, x3, y3, beta});

          // 1.2.1 检测当前点near_center_p是否是最贴近centerline的点，如果是的话，则全部投影，否则，仅投影从beta开始的半侧          
          if (beta==0){
            draw_index += draw_step_;
            // 中间的长弧
            const double y5 = y3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta);
            for (double the= M_PI; the>=-M_PI; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_negative_min_turning_radius_*sin(the); p.y=y5+y_negative_min_turning_radius_*cos(the); p.z=the;
              if (isExtendedVehCollisionBorderOrStaticVeh(p, y_negative_footprint_))
                continue;
              int index = xPos2Index(p.x);
              temp_down_borderline[index] = std::max(p.y, temp_down_borderline[index]);
            }
          }

          // 1.2.2 near_center_p是车的最左侧的顶点
          else if (beta > 0){   
            draw_index += draw_step_;
            // 中间的长弧 从beta到theta
            const double y5 = y3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta);
            for (double the= M_PI; the>=-M_PI; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_negative_min_turning_radius_*sin(the); p.y=y5+y_negative_min_turning_radius_*cos(the); p.z=the;
              if (isExtendedVehCollisionBorderOrStaticVeh(p, y_negative_footprint_))
                continue;
              int index = xPos2Index(p.x);
              temp_down_borderline1[index] = std::max(p.y, temp_down_borderline1[index]);
            }
          }

          // 1.2.3 near_center_p是车的最右侧的顶点
          else if (beta < 0){   // 右侧的点
            draw_index += draw_step_;
            // 中间的长弧
            const double y5 = y3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta);
            for (double the= M_PI; the>=-M_PI; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_negative_min_turning_radius_*sin(the); p.y=y5+y_negative_min_turning_radius_*cos(the); p.z=the;
              if (isExtendedVehCollisionBorderOrStaticVeh(p, y_negative_footprint_))
                continue;
              int index = xPos2Index(p.x);
              temp_down_borderline2[index] = std::max(p.y, temp_down_borderline2[index]);
            }
          }
        }

        // 2. 一个车内部点间的连接，使每两个near_center_p的弧 是被平滑连接的【因为任意两个点的左弧（或右弧）是平行相等的，且每个点的中弧 相对于侧弧 斜率是逐渐下降的，所以不同点的弧线的交点一定发生在两个不同点的中弧的交点上，否则就是一个点涵盖了另一个点。】【是否涵盖可以通过比较x1_minus或x1_plus的相对位置】
        // 2.1 只有一个near_center_p点，无需操作
        if (states_backup.size()==1) {
          // veh_x_switching_for_down_borderline_individual_.push_back({states_backup.front()[2], states_backup.front()[3]});
        }
        else {
          // 2.2 考虑左侧点和中间点的弧线间的连接
          std::vector<double> left = states_backup.front();
          std::vector<double> top;
          std::vector<double> right = states_backup.back();
          for (auto p:states_backup)
            if (p.back()==0)
              top = p;
          // 判断left点和top点的x1_minus，如果left的x1_minus更小，则说明left和top的中弧需要被以gamma为斜率、起点为(x_left,y_left)、终点为(x_top,y_top)的线段连接
          if (left[0] < top[0]){
            const double& x_left3 = left[4], y_left3 = left[5], beta_left = left[6];
            const double& x_top3 = top[4], y_top3 = top[5], beta_top = top[6];
            // gamma = numerator / denominator
            double numerator = ( y_left3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_left) ) - ( y_top3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_top) );
            double denominator = ( x_left3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_left) ) - ( x_top3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_top) );
            double gamma = numerator / denominator;
            double x_top = x_top3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_top) - y_negative_min_turning_radius_*sin(gamma);
            double y_top = y_top3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_top) + y_negative_min_turning_radius_*cos(gamma);
            double x_left = x_left3 + ((y_negative_min_turning_radius_-y_negative_vehwidth_/2))*sin(beta_left) - y_negative_min_turning_radius_*sin(gamma);
            double y_left = y_left3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_left) + y_negative_min_turning_radius_*cos(gamma);
            // 对线段(x_left,y_left)-->(x_top,y_top)进行离散，并投影到borderline
            geometry_msgs::Point p1; p1.x=x_left; p1.y=y_left;
            geometry_msgs::Point p2; p2.x=x_top; p2.y=y_top;
            temp_down_borderline = combineTwoBorderlines(temp_down_borderline,temp_down_borderline1,center);
            addBorderPointsToSpecialVec(temp_down_borderline, p1, p2, center);
            // veh_x_switching_for_down_borderline_individual_.push_back({left[2], top[3]});
          }
          // 判断top点和right点的x1_plus，如果right的x1_plus更小，则说明top和right的中弧需要被以gamma为斜率、起点为(x_top,y_top)、终点为(x_right,y_right)的线段连接
          if (right[1] > top[1]){
            const double& x_right3 = right[4], y_right3 = right[5], beta_right = right[6];
            const double& x_top3 = top[4], y_top3 = top[5], beta_top = top[6];
            // gamma = numerator / denominator
            double numerator = ( y_right3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_right) ) - ( y_top3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_top) );
            double denominator = ( x_right3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_right) ) - ( x_top3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_top) );
            double gamma = numerator / denominator;
            double x_top = x_top3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_top) - y_negative_min_turning_radius_*sin(gamma);
            double y_top = y_top3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_top) + y_negative_min_turning_radius_*cos(gamma);
            double x_right = x_right3 + ((y_negative_min_turning_radius_-y_negative_vehwidth_/2))*sin(beta_right) - y_negative_min_turning_radius_*sin(gamma);
            double y_right = y_right3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_right) + y_negative_min_turning_radius_*cos(gamma);
            // 对线段(x_top,y_top) --> (x_right,y_right)进行离散，并投影到borderline
            geometry_msgs::Point p1; p1.x=x_right; p1.y=y_right;
            geometry_msgs::Point p2; p2.x=x_top; p2.y=y_top;
            temp_down_borderline = combineTwoBorderlines(temp_down_borderline,temp_down_borderline2,center);
            addBorderPointsToSpecialVec(temp_down_borderline, p1, p2, center);
            // veh_x_switching_for_down_borderline_individual_.push_back({top[2], right[3]});
          }
        }
        down_borderline_individual_.push_back(temp_down_borderline);
        // 赋值veh_x_borders_for_down_borderline_individual_
        double temp_min_x=999, temp_max_x=-999;
        for (auto p:veh.points){
          temp_min_x = std::min(temp_min_x, p.x);
          temp_max_x = std::max(temp_max_x, p.x);
        }
        veh_x_borders_for_down_borderline_individual_.push_back({temp_min_x,temp_max_x});

        /** 4.1 按照车模型 膨胀down boderline所有点 **/
        geometry_msgs::Point false_pos; false_pos.y=-1.0;
        auto temp_down_borderline_copy = temp_down_borderline;
        // 太密集的话 画出来的borderline会很多参差/毛刺
        int delta_step = 20;
        for (int i=delta_step; i<temp_down_borderline.size();i++){
          geometry_msgs::Point center;
          center.x = index2x_list_[i-delta_step];
          center.y = temp_down_borderline[i-delta_step];
          if (center.y==yMin_ || center.y==yMax_)
            continue;          
          double theta = atan2(temp_down_borderline[i]-temp_down_borderline[i-delta_step], index2x_list_[i]-index2x_list_[i-delta_step]);
          // double theta = 0;
          for (int j=1; j<y_negative_footprint_.size(); j++){
            geometry_msgs::Point p1; p1.x=y_negative_footprint_[j-1].first; p1.y=y_negative_footprint_[j-1].second;
            geometry_msgs::Point p1_rotated;
            p1_rotated.x = center.x + cos(theta)*p1.x + -sin(theta)*p1.y;
            p1_rotated.y = center.y + sin(theta)*p1.x + cos(theta)*p1.y;
            geometry_msgs::Point p2; p2.x=y_negative_footprint_[j].first; p2.y=y_negative_footprint_[j].second;
            geometry_msgs::Point p2_rotated;
            p2_rotated.x = center.x + cos(theta)*p2.x + -sin(theta)*p2.y;
            p2_rotated.y = center.y + sin(theta)*p2.x + cos(theta)*p2.y;
            addBorderPointsToSpecialVec(temp_down_borderline_copy, p1_rotated, p2_rotated, false_pos);
          }
        }
        down_borderline_individual_extended_.push_back(temp_down_borderline_copy);
      }
    }

    /** 考虑沿着up_borderline方向运动的车 **/
    {
      if(center.y>=0){
        // proj: 将点反置，防止过量计算
        near_center_p1.y *= -1;
        near_center_p2.y *= -1;
        near_center_p3.y *= -1;

        // 0. 剔除所有顶点near_center_p_list中 无法被执行的顶点
        std::vector<geometry_msgs::Point> near_center_p_list = {near_center_p1, near_center_p2, near_center_p3};
        std::vector<geometry_msgs::Point>::iterator iter = near_center_p_list.begin();
        while(iter!=near_center_p_list.end()){
          const double& x3 = iter->x;
          const double& y3 = iter->y;
          // 0.1 当前点 比 车沿着边沿走 还低，直接剔除掉
          if (y3<=yMin_ + y_positive_vehwidth_/2){
            iter = near_center_p_list.erase(iter);
            continue;
          };
          double beta = atan2(near_center_p1.y-y3, near_center_p1.x-x3);   //atanh() \in [-pi, pi]
          if (beta>M_PI_2) beta-=M_PI;  //cluseter beta into [-pi/2, pi/2]
          double max_beta_for_y3 = acos((y_positive_min_turning_radius_- (y3-(yMin_+y_positive_vehwidth_/2)))/y_positive_min_turning_radius_);
          // 0.2 弯太大 拐不过来，直接剔除掉
          if (abs(beta)>abs(max_beta_for_y3)){
            iter = near_center_p_list.erase(iter);
            continue;
          }
          iter++;
        }
        
        // 按照x从小到大，对顶点进行排序
        sort(near_center_p_list.begin(), near_center_p_list.end(), comparePoints);
        std::vector<std::vector<double>> states_backup;  // {(x1_minus, x_plus, x3, y3, beta),...} 存过程中的的某些当前状态，用来计算两个near_center_p生成的弧之间的连接
        std::vector<double> temp_up_borderline_oncoming;
        for (float x=xMin_; x<=xMax_; x+=step_size_){
          temp_up_borderline_oncoming.push_back(yMax_);
        }
        std::vector<double> temp_up_borderline1_oncoming = temp_up_borderline_oncoming; 
        std::vector<double> temp_up_borderline2_oncoming = temp_up_borderline_oncoming; 
        
        std::vector<double> temp_up_borderline_av = temp_up_borderline_oncoming;
        std::vector<double> temp_up_borderline1_av = temp_up_borderline1_oncoming; 
        std::vector<double> temp_up_borderline2_av = temp_up_borderline2_oncoming;
        // 1. 遍历所有可执行顶点near_center_p，产生左、中、右三个弧，project到borderline
        for (auto near_center_p:near_center_p_list){
          const double y1 = yMin_ + y_positive_vehwidth_/2;
          const double& x3 = near_center_p.x;
          const double& y3 = near_center_p.y;
          // 定义beta点的正方向为逆时针，即在p_top(near_center_p1)左侧的点的beta是正的，p_top(near_center_p1)右侧的点的beta是负的
          double beta = atan2(near_center_p1.y-near_center_p.y, near_center_p1.x-near_center_p.x);  //atanh() \in [-pi, pi]
          if (beta>M_PI_2) beta-=M_PI;  //cluseter beta into [-pi/2, pi/2]

          // 1.1 计算near_center_p的中心点(x3,y3)左右两侧的弧角点(x_minus,y,theta_minus)和(x_plus,y,theta_plus) 以及最左侧和最右侧的(x1_minus,y1)和(x1_plus,y1)
          // theta_minus指的是x_minus在x3左侧，但其实 theta_minus是正的；对应的，x_plu在右侧，theta_plus是负的
          double x_minus, x_plus, y, theta_minus, theta_plus, x1_minus, x1_plus;
          y = (y1+y_positive_min_turning_radius_+y3-(y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta))/2;
          theta_minus = acos((y1+y_positive_min_turning_radius_-y3+(y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta))/(2*y_positive_min_turning_radius_));
          theta_plus = -theta_minus;  //acos() \in [0,pi]
          x_minus = x3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta) - sqrt( pow(y_positive_min_turning_radius_,2) - pow( (y1+y_positive_min_turning_radius_-y3+(y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta))/2 ,2) );
          x_plus = x3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta) + sqrt( pow(y_positive_min_turning_radius_,2) - pow( (y1+y_positive_min_turning_radius_-y3+(y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta))/2 ,2) );
          x1_minus = x_minus - y_positive_min_turning_radius_*sin(theta_minus);
          x1_plus = x_plus - y_positive_min_turning_radius_*sin(theta_plus);
          draw_index += draw_step_;

          states_backup.push_back({x1_minus, x1_plus, x_minus, x_plus, x3, y3, beta});

          // 1.2.1 检测当前点near_center_p是否是最贴近centerline的点，如果是的话，则全部投影，否则，仅投影从beta开始的半侧          
          if (beta==0){
            draw_index += draw_step_;
            // 中间的长弧
            const double y5 = y3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta);
            for (double the= M_PI; the>=-M_PI; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_positive_min_turning_radius_*sin(the); p.y=y5+y_positive_min_turning_radius_*cos(the); p.z = the;
              geometry_msgs::Point temp_p; temp_p.x=p.x; temp_p.y=-p.y; temp_p.z=-p.z+M_PI;
              if (!isExtendedVehCollisionBorderOrStaticVeh(temp_p, y_positive_footprint_)){
                int index = xPos2Index(temp_p.x);
                temp_up_borderline_oncoming[index] = std::min(temp_p.y, temp_up_borderline_oncoming[index]);
              }
              temp_p.z -= M_PI;
              if (!isExtendedVehCollisionBorderOrStaticVeh(temp_p, y_negative_footprint_)){
                int index = xPos2Index(temp_p.x);
                temp_up_borderline_av[index] = std::min(temp_p.y, temp_up_borderline_av[index]);
              }
            }
          }

          // 1.2.2 near_center_p是车的最左侧的顶点
          else if (beta > 0){   
            draw_index += draw_step_;
            // 中间的长弧 从beta到theta
            const double y5 = y3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta);
            for (double the= M_PI; the>=-M_PI; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_positive_min_turning_radius_*sin(the); p.y=y5+y_positive_min_turning_radius_*cos(the); p.z = the;
              geometry_msgs::Point temp_p; temp_p.x=p.x; temp_p.y=-p.y; temp_p.z=-p.z+M_PI;
              if (!isExtendedVehCollisionBorderOrStaticVeh(temp_p, y_positive_footprint_)){
                int index = xPos2Index(temp_p.x);
                temp_up_borderline1_oncoming[index] = std::min(temp_p.y, temp_up_borderline1_oncoming[index]);
              }
              temp_p.z -= M_PI;
              if (!isExtendedVehCollisionBorderOrStaticVeh(temp_p, y_negative_footprint_)){
                int index = xPos2Index(temp_p.x);
                temp_up_borderline1_av[index] = std::min(temp_p.y, temp_up_borderline1_av[index]);
              }              
            }
          }

          // 1.2.3 near_center_p是车的最右侧的顶点
          else if (beta < 0){   // 右侧的点
            draw_index += draw_step_;
            // 中间的长弧
            const double y5 = y3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta);
            for (double the= M_PI; the>=-M_PI; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_positive_min_turning_radius_*sin(the); p.y=y5+y_positive_min_turning_radius_*cos(the); p.z = the;
              geometry_msgs::Point temp_p; temp_p.x=p.x; temp_p.y=-p.y; temp_p.z=-p.z+M_PI;
              if (!isExtendedVehCollisionBorderOrStaticVeh(temp_p, y_positive_footprint_)){
                int index = xPos2Index(temp_p.x);
                temp_up_borderline2_oncoming[index] = std::min(temp_p.y, temp_up_borderline2_oncoming[index]);
              }
              temp_p.z -= M_PI;
              if (!isExtendedVehCollisionBorderOrStaticVeh(temp_p, y_negative_footprint_)){
                int index = xPos2Index(temp_p.x);
                temp_up_borderline2_av[index] = std::min(temp_p.y, temp_up_borderline2_av[index]);
              }              
            }
          }
        }
        temp_up_borderline_oncoming = combineTwoBorderlines(temp_up_borderline_oncoming,temp_up_borderline1_oncoming,center);
        temp_up_borderline_oncoming = combineTwoBorderlines(temp_up_borderline_oncoming,temp_up_borderline2_oncoming,center);
        temp_up_borderline_av = combineTwoBorderlines(temp_up_borderline_av,temp_up_borderline1_av,center);
        temp_up_borderline_av = combineTwoBorderlines(temp_up_borderline_av,temp_up_borderline2_av,center);

        // 2. 一个车内部点间的连接，使每两个near_center_p的弧 是被平滑连接的【因为任意两个点的左弧（或右弧）是平行相等的，且每个点的中弧 相对于侧弧 斜率是逐渐下降的，所以不同点的弧线的交点一定发生在两个不同点的中弧的交点上，否则就是一个点涵盖了另一个点。】【是否涵盖可以通过比较x1_minus或x1_plus的相对位置】
        // 2.1 只有一个near_center_p点，无需操作
        if (states_backup.size()==1) {
          // veh_x_switching_for_up_borderline_individual_.push_back(states_backup.front()[2]);
        }
        else {
          // 2.2 考虑左侧点和中间点的弧线间的连接
          std::vector<double> left = states_backup.front();
          std::vector<double> top;
          std::vector<double> right = states_backup.back();
          for (auto p:states_backup)
            if (p.back()==0)
              top = p;
          // 判断left点和top点的x1_minus，如果left的x1_minus更小，则说明left和top的中弧需要被以gamma为斜率、起点为(x_left,y_left)、终点为(x_top,y_top)的线段连接
          if (left[0] < top[0]){
            const double& x_left3 = left[4], y_left3 = left[5], beta_left = left[6];
            const double& x_top3 = top[4], y_top3 = top[5], beta_top = top[6];
            // gamma = numerator / denominator
            double numerator = ( y_left3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_left) ) - ( y_top3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_top) );
            double denominator = ( x_left3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_left) ) - ( x_top3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_top) );
            double gamma = numerator / denominator;
            double x_top = x_top3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_top) - y_positive_min_turning_radius_*sin(gamma);
            double y_top = y_top3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_top) + y_positive_min_turning_radius_*cos(gamma);
            double x_left = x_left3 + ((y_positive_min_turning_radius_-y_positive_vehwidth_/2))*sin(beta_left) - y_positive_min_turning_radius_*sin(gamma);
            double y_left = y_left3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_left) + y_positive_min_turning_radius_*cos(gamma);
            // 对线段(x_left,y_left)-->(x_top,y_top)进行离散，并投影到borderline
            geometry_msgs::Point p1; p1.x=x_left; p1.y=y_left;
            geometry_msgs::Point p2; p2.x=x_top; p2.y=y_top;
            temp_up_borderline_oncoming = combineTwoBorderlines(temp_up_borderline_oncoming,temp_up_borderline1_oncoming,center);
            temp_up_borderline_av = combineTwoBorderlines(temp_up_borderline_av,temp_up_borderline1_av,center);
            geometry_msgs::Point temp_p1=p1; temp_p1.y *= -1;
            geometry_msgs::Point temp_p2=p2; temp_p2.y *= -1;
            addBorderPointsToSpecialVec(temp_up_borderline_oncoming, temp_p1, temp_p2, center);
            addBorderPointsToSpecialVec(temp_up_borderline_av, temp_p1, temp_p2, center);
            // veh_x_switching_for_up_borderline_individual_.push_back(left[2]);
          }
          // 判断top点和right点的x1_plus，如果right的x1_plus更小，则说明top和right的中弧需要被以gamma为斜率、起点为(x_top,y_top)、终点为(x_right,y_right)的线段连接
          if (right[1] > top[1]){
            const double& x_right3 = right[4], y_right3 = right[5], beta_right = right[6];
            const double& x_top3 = top[4], y_top3 = top[5], beta_top = top[6];
            // gamma = numerator / denominator
            double numerator = ( y_right3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_right) ) - ( y_top3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_top) );
            double denominator = ( x_right3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_right) ) - ( x_top3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_top) );
            double gamma = numerator / denominator;
            double x_top = x_top3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_top) - y_positive_min_turning_radius_*sin(gamma);
            double y_top = y_top3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_top) + y_positive_min_turning_radius_*cos(gamma);
            double x_right = x_right3 + ((y_positive_min_turning_radius_-y_positive_vehwidth_/2))*sin(beta_right) - y_positive_min_turning_radius_*sin(gamma);
            double y_right = y_right3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_right) + y_positive_min_turning_radius_*cos(gamma);
            // 对线段(x_top,y_top) --> (x_right,y_right)进行离散，并投影到borderline
            geometry_msgs::Point p1; p1.x=x_right; p1.y=y_right;
            geometry_msgs::Point p2; p2.x=x_top; p2.y=y_top;
            temp_up_borderline_oncoming = combineTwoBorderlines(temp_up_borderline_oncoming,temp_up_borderline2_oncoming,center);
            temp_up_borderline_av = combineTwoBorderlines(temp_up_borderline_av,temp_up_borderline2_av,center);
            geometry_msgs::Point temp_p1=p1; temp_p1.y *= -1;
            geometry_msgs::Point temp_p2=p2; temp_p2.y *= -1;
            addBorderPointsToSpecialVec(temp_up_borderline_oncoming, temp_p1, temp_p2, center);
            addBorderPointsToSpecialVec(temp_up_borderline_av, temp_p1, temp_p2, center);
            // veh_x_switching_for_up_borderline_individual_.push_back(top[2]);
          }
        }        
        up_borderline_individual_.push_back(temp_up_borderline_oncoming);
        up_borderline_individual_for_left_backup_.push_back(temp_up_borderline_av);
        double temp_min_x=999, temp_max_x=-999;
        for (auto p:veh.points){
          temp_min_x = std::min(temp_min_x, p.x);
          temp_max_x = std::max(temp_max_x, p.x);
        }
        veh_x_borders_for_up_borderline_individual_.push_back({temp_min_x,temp_max_x});

        /** 4.2 按照车模型 膨胀up boderline所有点 **/
        geometry_msgs::Point false_pos; false_pos.y=1.0;
        auto temp_up_borderline_copy = temp_up_borderline_oncoming;
        // 太密集的话 画出来的borderline会很多参差/毛刺
        int delta_step = 20;
        for (int i=temp_up_borderline_oncoming.size()-1; i>=delta_step; i--){
          geometry_msgs::Point center;
          center.x = index2x_list_[i];
          center.y = temp_up_borderline_oncoming[i];
          if (center.y==yMin_ || center.y==yMax_)
            continue;
          double theta = atan2(temp_up_borderline_oncoming[i-delta_step]-temp_up_borderline_oncoming[i], index2x_list_[i-delta_step]-index2x_list_[i]);
          // double theta = 0;
          for (int j=1; j<y_positive_footprint_.size(); j++){
            geometry_msgs::Point p1; p1.x=y_positive_footprint_[j-1].first; p1.y=y_positive_footprint_[j-1].second;
            geometry_msgs::Point p1_rotated;
            p1_rotated.x = center.x + cos(theta)*p1.x + -sin(theta)*p1.y;
            p1_rotated.y = center.y + sin(theta)*p1.x + cos(theta)*p1.y;
            geometry_msgs::Point p2; p2.x=y_positive_footprint_[j].first; p2.y=y_positive_footprint_[j].second;
            geometry_msgs::Point p2_rotated;
            p2_rotated.x = center.x + cos(theta)*p2.x + -sin(theta)*p2.y;
            p2_rotated.y = center.y + sin(theta)*p2.x + cos(theta)*p2.y;
            addBorderPointsToSpecialVec(temp_up_borderline_copy, p1_rotated, p2_rotated, false_pos);
          }
        }
        up_borderline_individual_extended_.push_back(temp_up_borderline_copy);
      }
    }  
  } // end for
}


/**
 * @input: 静态车辆信息
 * @description: 0.按车辆宽度扩展边界
 *               1.把每个车最多离散成3个点，假设自车后轮贴着这些点划弧，做投影
 *               2.同一个车的离散点间的平滑连接
 *               3.相邻两个车间的平滑连接
 *               4.车辆向右转 按照自车footprint膨胀borderline
 *               5.车辆向左转 按照自车footprint膨胀borderline
 * **/  
void ExtractEnv::doubleWinInteract(const std::vector<visualization_msgs::Marker>& msg){
  /** 0.假设两个动态车辆靠着边界走，用车辆宽度扩展边界 **/
  // 下边界
  double y_negative_vehwidth_ = VEHWIDTH;
  double y_negative_min_turning_radius_ = MIN_TURNING_RADIUS;
  for (int i=0; i<down_borderline_.size(); i++)
    down_borderline_[i] += y_negative_vehwidth_/2;
  std::vector<std::pair<double, double>> y_negative_footprint_ = FOOTPRINT_REAR;
  // 上边界
  double y_positive_vehwidth_ = VEHWIDTH;
  double y_positive_min_turning_radius_ = MIN_TURNING_RADIUS;
  for (int i=0; i<up_borderline_.size(); i++)
    up_borderline_[i] -= y_positive_vehwidth_/2;
  std::vector<std::pair<double, double>> y_positive_footprint_ = FOOTPRINT_REAR;


  /** 遍历所有静态车辆 **/
  float draw_index = 0;
  std::vector<std::vector<std::pair<double,double>>> veh_down_borderline_all_x5_y5;  // 对每个车，保存其每个有效near_center_p的{x5,y5}及center点： { {(x_{5,1},y_{5,1}), ..., (x_{5,n},y_{5,n}), {center.x,center.y}}, ... }
  std::vector<std::vector<std::pair<double,double>>> veh_up_borderline_all_x5_y5;
  for (auto veh:msg){
    // 找车的中心点
    geometry_msgs::Point center;
    for (int i=0; i<veh.points.size()-1; i++){
      center.x += veh.points[i].x;
      center.y += veh.points[i].y;
    }
    center.x /= (veh.points.size()-1);
    center.y /= (veh.points.size()-1);
    // 车有四个角点，找到离中心线最近的三个点，顺序为：|p1.y|>=|p2.y|>=|p3.y|
    geometry_msgs::Point near_center_p1; near_center_p1.y = 999;
    geometry_msgs::Point near_center_p2; near_center_p2.y = 999;
    geometry_msgs::Point near_center_p3; near_center_p3.y = 999;
    for (int i=0; i<veh.points.size()-1; i++){
      const double& x = veh.points[i].x;
      const double& y = veh.points[i].y;
      if (abs(y)<=abs(near_center_p1.y)){
        near_center_p3 = near_center_p2;
        near_center_p2 = near_center_p1;
        near_center_p1.x = x;
        near_center_p1.y = y;
      }
      else if (abs(y)<=abs(near_center_p2.y)){
        near_center_p3 = near_center_p2;
        near_center_p2.x = x;
        near_center_p2.y = y;
      }
      else if (abs(y)<=abs(near_center_p3.y)){
        near_center_p3.x = x;
        near_center_p3.y = y;
      }
    }

    /** 考虑沿着down_borderline方向运动的车，考虑动态车辆的 运动学约束和半径vehwidth，投影到borderline。 
     * 共有两次投影，第一次是（至多）三个near_center_p各自形成的左、中、右三个弧的投影； 第二次是为了使每两个near_center_p的弧 是被平滑连接的，两个部分的中弧需要被连接。
     * **/
    {
      if(center.y<0){
        // 0. 剔除所有顶点near_center_p_list中 无法被执行的顶点
        std::vector<geometry_msgs::Point> near_center_p_list = {near_center_p1, near_center_p2, near_center_p3};
        std::vector<geometry_msgs::Point>::iterator iter = near_center_p_list.begin();
        while(iter!=near_center_p_list.end()){
          const double& x3 = iter->x;
          const double& y3 = iter->y;
          // 0.1 当前点 比 车沿着边沿走 还低，直接剔除掉
          if (y3<=yMin_ + y_negative_vehwidth_/2){
            iter = near_center_p_list.erase(iter);
            continue;
          };
          double beta = atan2(near_center_p1.y-y3, near_center_p1.x-x3);   //atanh() \in [-pi, pi]
          if (beta>M_PI_2) beta-=M_PI;  //cluseter beta into [-pi/2, pi/2]
          double max_beta_for_y3 = acos((y_negative_min_turning_radius_- (y3-(yMin_+y_negative_vehwidth_/2)))/y_negative_min_turning_radius_);
          // 0.2 弯太大 拐不过来，直接剔除掉
          if (abs(beta)>abs(max_beta_for_y3)){
            iter = near_center_p_list.erase(iter);
            continue;
          }
          iter++;
        }
        
        // 按照x从小到大，对顶点进行排序
        sort(near_center_p_list.begin(), near_center_p_list.end(), comparePoints);
        std::vector<std::vector<double>> states_backup;  // {(x1_minus, x_plus, x3, y3, beta),...} 存过程中的的某些当前状态，用来计算两个near_center_p生成的弧之间的连接
        
        // 1. 遍历所有可执行顶点near_center_p，产生左、中、右三个弧，project到borderline
        for (auto near_center_p:near_center_p_list){
          const double y1 = yMin_ + y_negative_vehwidth_/2;
          const double& x3 = near_center_p.x;
          const double& y3 = near_center_p.y;
          // 定义beta点的正方向为逆时针，即在p_top(near_center_p1)左侧的点的beta是正的，p_top(near_center_p1)右侧的点的beta是负的
          double beta = atan2(near_center_p1.y-near_center_p.y, near_center_p1.x-near_center_p.x);  //atanh() \in [-pi, pi]
          if (beta>M_PI_2) beta-=M_PI;  //cluseter beta into [-pi/2, pi/2]

          // 1.1 计算near_center_p的中心点(x3,y3)左右两侧的弧角点(x_minus,y,theta_minus)和(x_plus,y,theta_plus) 以及最左侧和最右侧的(x1_minus,y1)和(x1_plus,y1)
          // theta_minus指的是x_minus在x3左侧，但其实 theta_minus是正的；对应的，x_plu在右侧，theta_plus是负的
          double x_minus, x_plus, y, theta_minus, theta_plus, x1_minus, x1_plus;
          y = (y1+y_negative_min_turning_radius_+y3-(y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta))/2;
          theta_minus = acos((y1+y_negative_min_turning_radius_-y3+(y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta))/(2*y_negative_min_turning_radius_));
          theta_plus = -theta_minus;  //acos() \in [0,pi]
          x_minus = x3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta) - sqrt( pow(y_negative_min_turning_radius_,2) - pow( (y1+y_negative_min_turning_radius_-y3+(y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta))/2 ,2) );
          x_plus = x3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta) + sqrt( pow(y_negative_min_turning_radius_,2) - pow( (y1+y_negative_min_turning_radius_-y3+(y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta))/2 ,2) );
          x1_minus = x_minus - y_negative_min_turning_radius_*sin(theta_minus);
          x1_plus = x_plus - y_negative_min_turning_radius_*sin(theta_plus);
          draw_index += draw_step_;

          states_backup.push_back({x1_minus, x1_plus, x_minus, x_plus, x3, y3, beta});

          // 1.2.1 检测当前点near_center_p是否是最贴近centerline的点，如果是的话，则全部投影，否则，仅投影从beta开始的半侧          
          if (beta==0){
            // 左侧弧的投影
            for (double the=0; the<=theta_minus; the+=theta_step_){
              geometry_msgs::Point p; p.x=x1_minus+y_negative_min_turning_radius_*sin(the); p.y=y1+y_negative_min_turning_radius_-y_negative_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index);
            }
            draw_index += draw_step_;
            // 右侧弧的投影
            for (double the=0; the>=theta_plus; the-=theta_step_){
              geometry_msgs::Point p; p.x=x1_plus+y_negative_min_turning_radius_*sin(the); p.y=y1+y_negative_min_turning_radius_-y_negative_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index);
            }
            draw_index += draw_step_;
            // 中间的长弧
            const double y5 = y3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta);
            for (double the= theta_minus; the>=theta_plus; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_negative_min_turning_radius_*sin(the); p.y=y5+y_negative_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index);
            }
          }

          // 1.2.2 near_center_p是车的最左侧的顶点
          else if (beta > 0){   
            //左侧弧的投影
            for (double the=0; the<=theta_minus; the+=theta_step_){
              geometry_msgs::Point p; p.x=x1_minus+y_negative_min_turning_radius_*sin(the); p.y=y1+y_negative_min_turning_radius_-y_negative_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index);
            }
            draw_index += draw_step_;
            // 中间的长弧 从beta到theta
            const double y5 = y3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta);
            for (double the= theta_minus; the>=beta; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_negative_min_turning_radius_*sin(the); p.y=y5+y_negative_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index);
            }
          }

          // 1.2.3 near_center_p是车的最右侧的顶点
          else if (beta < 0){   // 右侧的点
            // 右侧弧的投影
            for (double the=0; the>=theta_plus; the-=theta_step_){
              geometry_msgs::Point p; p.x=x1_plus+y_negative_min_turning_radius_*sin(the); p.y=y1+y_negative_min_turning_radius_-y_negative_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index);
            }
            draw_index += draw_step_;
            // 中间的长弧
            const double y5 = y3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta);
            for (double the= beta; the>=theta_plus; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_negative_min_turning_radius_*sin(the); p.y=y5+y_negative_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index);
            }
          }
        }

        // 2. 一个车内部点间的连接，使每两个near_center_p的弧 是被平滑连接的【因为任意两个点的左弧（或右弧）是平行相等的，且每个点的中弧 相对于侧弧 斜率是逐渐下降的，所以不同点的弧线的交点一定发生在两个不同点的中弧的交点上，否则就是一个点涵盖了另一个点。】【是否涵盖可以通过比较x1_minus或x1_plus的相对位置】
        // 2.1 只有一个near_center_p点，无需操作
        if (states_backup.size()==1) {}
        else {
          // 2.2 考虑左侧点和中间点的弧线间的连接
          std::vector<double> left = states_backup.front();
          std::vector<double> top;
          std::vector<double> right = states_backup.back();
          for (auto p:states_backup)
            if (p.back()==0)
              top = p;
          // 判断left点和top点的x1_minus，如果left的x1_minus更小，则说明left和top的中弧需要被以gamma为斜率、起点为(x_left,y_left)、终点为(x_top,y_top)的线段连接
          if (left[0] < top[0]){
            const double& x_left3 = left[4], y_left3 = left[5], beta_left = left[6];
            const double& x_top3 = top[4], y_top3 = top[5], beta_top = top[6];
            // gamma = numerator / denominator
            double numerator = ( y_left3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_left) ) - ( y_top3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_top) );
            double denominator = ( x_left3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_left) ) - ( x_top3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_top) );
            double gamma = numerator / denominator;
            double x_top = x_top3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_top) - y_negative_min_turning_radius_*sin(gamma);
            double y_top = y_top3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_top) + y_negative_min_turning_radius_*cos(gamma);
            double x_left = x_left3 + ((y_negative_min_turning_radius_-y_negative_vehwidth_/2))*sin(beta_left) - y_negative_min_turning_radius_*sin(gamma);
            double y_left = y_left3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_left) + y_negative_min_turning_radius_*cos(gamma);
            // 对线段(x_left,y_left)-->(x_top,y_top)进行离散，并投影到borderline
            geometry_msgs::Point p1; p1.x=x_left; p1.y=y_left;
            geometry_msgs::Point p2; p2.x=x_top; p2.y=y_top;
            draw_index += draw_step_;
            addBorderPoints(p1, p2, center, draw_index);
          }
          // 判断top点和right点的x1_plus，如果right的x1_plus更小，则说明top和right的中弧需要被以gamma为斜率、起点为(x_top,y_top)、终点为(x_right,y_right)的线段连接
          if (right[1] > top[1]){
            const double& x_right3 = right[4], y_right3 = right[5], beta_right = right[6];
            const double& x_top3 = top[4], y_top3 = top[5], beta_top = top[6];
            // gamma = numerator / denominator
            double numerator = ( y_right3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_right) ) - ( y_top3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_top) );
            double denominator = ( x_right3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_right) ) - ( x_top3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_top) );
            double gamma = numerator / denominator;
            double x_top = x_top3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta_top) - y_negative_min_turning_radius_*sin(gamma);
            double y_top = y_top3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_top) + y_negative_min_turning_radius_*cos(gamma);
            double x_right = x_right3 + ((y_negative_min_turning_radius_-y_negative_vehwidth_/2))*sin(beta_right) - y_negative_min_turning_radius_*sin(gamma);
            double y_right = y_right3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta_right) + y_negative_min_turning_radius_*cos(gamma);
            // 对线段(x_top,y_top) --> (x_right,y_right)进行离散，并投影到borderline
            geometry_msgs::Point p1; p1.x=x_right; p1.y=y_right;
            geometry_msgs::Point p2; p2.x=x_top; p2.y=y_top;
            draw_index += draw_step_;
            addBorderPoints(p1, p2, center, draw_index);
          }
        }
        // 3.1 保存所有有效点的(x5,y5)
        std::vector<std::pair<double,double>> down_borderline_x5_y5_list;
        for (auto near_center_p:near_center_p_list){
          const double y1 = yMin_ + y_negative_vehwidth_/2;
          const double& x3 = near_center_p.x;
          const double& y3 = near_center_p.y;
          // 定义beta点的正方向为逆时针，即在p_top(near_center_p1)左侧的点的beta是正的，p_top(near_center_p1)右侧的点的beta是负的
          double beta = atan2(near_center_p1.y-near_center_p.y, near_center_p1.x-near_center_p.x);  //atanh() \in [-pi, pi]
          if (beta>M_PI_2) beta-=M_PI;  //cluseter beta into [-pi/2, pi/2]
          const double y5 = y3 - (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*cos(beta);
          const double x5 = x3 + (y_negative_min_turning_radius_-y_negative_vehwidth_/2)*sin(beta);
          down_borderline_x5_y5_list.push_back({x5,y5});
        }
        down_borderline_x5_y5_list.push_back({center.x, center.y});
        veh_down_borderline_all_x5_y5.push_back(down_borderline_x5_y5_list);
      }
    }

    /** 考虑沿着up_borderline方向运动的车 **/
    {
      if(center.y>=0){
        // proj: 将点反置，防止过量计算
        near_center_p1.y *= -1;
        near_center_p2.y *= -1;
        near_center_p3.y *= -1;

        // 0. 剔除所有顶点near_center_p_list中 无法被执行的顶点
        std::vector<geometry_msgs::Point> near_center_p_list = {near_center_p1, near_center_p2, near_center_p3};
        std::vector<geometry_msgs::Point>::iterator iter = near_center_p_list.begin();
        while(iter!=near_center_p_list.end()){
          const double& x3 = iter->x;
          const double& y3 = iter->y;
          // 0.1 当前点 比 车沿着边沿走 还低，直接剔除掉
          if (y3<=yMin_ + y_positive_vehwidth_/2){
            iter = near_center_p_list.erase(iter);
            continue;
          };
          double beta = atan2(near_center_p1.y-y3, near_center_p1.x-x3);   //atanh() \in [-pi, pi]
          if (beta>M_PI_2) beta-=M_PI;  //cluseter beta into [-pi/2, pi/2]
          double max_beta_for_y3 = acos((y_positive_min_turning_radius_- (y3-(yMin_+y_positive_vehwidth_/2)))/y_positive_min_turning_radius_);
          // 0.2 弯太大 拐不过来，直接剔除掉
          if (abs(beta)>abs(max_beta_for_y3)){
            iter = near_center_p_list.erase(iter);
            continue;
          }
          iter++;
        }
        
        // 按照x从小到大，对顶点进行排序
        sort(near_center_p_list.begin(), near_center_p_list.end(), comparePoints);
        std::vector<std::vector<double>> states_backup;  // {(x1_minus, x_plus, x3, y3, beta),...} 存过程中的的某些当前状态，用来计算两个near_center_p生成的弧之间的连接
        
        // 1. 遍历所有可执行顶点near_center_p，产生左、中、右三个弧，project到borderline
        for (auto near_center_p:near_center_p_list){
          const double y1 = yMin_ + y_positive_vehwidth_/2;
          const double& x3 = near_center_p.x;
          const double& y3 = near_center_p.y;
          // 定义beta点的正方向为逆时针，即在p_top(near_center_p1)左侧的点的beta是正的，p_top(near_center_p1)右侧的点的beta是负的
          double beta = atan2(near_center_p1.y-near_center_p.y, near_center_p1.x-near_center_p.x);  //atanh() \in [-pi, pi]
          if (beta>M_PI_2) beta-=M_PI;  //cluseter beta into [-pi/2, pi/2]

          // 1.1 计算near_center_p的中心点(x3,y3)左右两侧的弧角点(x_minus,y,theta_minus)和(x_plus,y,theta_plus) 以及最左侧和最右侧的(x1_minus,y1)和(x1_plus,y1)
          // theta_minus指的是x_minus在x3左侧，但其实 theta_minus是正的；对应的，x_plu在右侧，theta_plus是负的
          double x_minus, x_plus, y, theta_minus, theta_plus, x1_minus, x1_plus;
          y = (y1+y_positive_min_turning_radius_+y3-(y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta))/2;
          theta_minus = acos((y1+y_positive_min_turning_radius_-y3+(y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta))/(2*y_positive_min_turning_radius_));
          theta_plus = -theta_minus;  //acos() \in [0,pi]
          x_minus = x3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta) - sqrt( pow(y_positive_min_turning_radius_,2) - pow( (y1+y_positive_min_turning_radius_-y3+(y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta))/2 ,2) );
          x_plus = x3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta) + sqrt( pow(y_positive_min_turning_radius_,2) - pow( (y1+y_positive_min_turning_radius_-y3+(y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta))/2 ,2) );
          x1_minus = x_minus - y_positive_min_turning_radius_*sin(theta_minus);
          x1_plus = x_plus - y_positive_min_turning_radius_*sin(theta_plus);
          draw_index += draw_step_;

          states_backup.push_back({x1_minus, x1_plus, x_minus, x_plus, x3, y3, beta});

          // 1.2.1 检测当前点near_center_p是否是最贴近centerline的点，如果是的话，则全部投影，否则，仅投影从beta开始的半侧          
          if (beta==0){
            // 左侧弧的投影
            for (double the=0; the<=theta_minus; the+=theta_step_){
              geometry_msgs::Point p; p.x=x1_minus+y_positive_min_turning_radius_*sin(the); p.y=y1+y_positive_min_turning_radius_-y_positive_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index, false);  // false指示取负
            }
            draw_index += draw_step_;
            // 右侧弧的投影
            for (double the=0; the>=theta_plus; the-=theta_step_){
              geometry_msgs::Point p; p.x=x1_plus+y_positive_min_turning_radius_*sin(the); p.y=y1+y_positive_min_turning_radius_-y_positive_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index, false);
            }
            draw_index += draw_step_;
            // 中间的长弧
            const double y5 = y3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta);
            for (double the= theta_minus; the>=theta_plus; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_positive_min_turning_radius_*sin(the); p.y=y5+y_positive_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index, false);
            }
          }

          // 1.2.2 near_center_p是车的最左侧的顶点
          else if (beta > 0){   
            //左侧弧的投影
            for (double the=0; the<=theta_minus; the+=theta_step_){
              geometry_msgs::Point p; p.x=x1_minus+y_positive_min_turning_radius_*sin(the); p.y=y1+y_positive_min_turning_radius_-y_positive_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index, false);
            }
            draw_index += draw_step_;
            // 中间的长弧 从beta到theta
            const double y5 = y3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta);
            for (double the= theta_minus; the>=beta; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_positive_min_turning_radius_*sin(the); p.y=y5+y_positive_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index, false);
            }
          }

          // 1.2.3 near_center_p是车的最右侧的顶点
          else if (beta < 0){   // 右侧的点
            // 右侧弧的投影
            for (double the=0; the>=theta_plus; the-=theta_step_){
              geometry_msgs::Point p; p.x=x1_plus+y_positive_min_turning_radius_*sin(the); p.y=y1+y_positive_min_turning_radius_-y_positive_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index, false);
            }
            draw_index += draw_step_;
            // 中间的长弧
            const double y5 = y3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta);
            const double x5 = x3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta);
            for (double the= beta; the>=theta_plus; the-=theta_step_){
              geometry_msgs::Point p; p.x=x5-y_positive_min_turning_radius_*sin(the); p.y=y5+y_positive_min_turning_radius_*cos(the);
              addBorderPoints(p, center, draw_index, false);
            }
          }
        }

        // 2. 一个车内部点间的连接，使每两个near_center_p的弧 是被平滑连接的【因为任意两个点的左弧（或右弧）是平行相等的，且每个点的中弧 相对于侧弧 斜率是逐渐下降的，所以不同点的弧线的交点一定发生在两个不同点的中弧的交点上，否则就是一个点涵盖了另一个点。】【是否涵盖可以通过比较x1_minus或x1_plus的相对位置】
        // 2.1 只有一个near_center_p点，无需操作
        if (states_backup.size()==1) {}
        else {
          // 2.2 考虑左侧点和中间点的弧线间的连接
          std::vector<double> left = states_backup.front();
          std::vector<double> top;
          std::vector<double> right = states_backup.back();
          for (auto p:states_backup)
            if (p.back()==0)
              top = p;
          // 判断left点和top点的x1_minus，如果left的x1_minus更小，则说明left和top的中弧需要被以gamma为斜率、起点为(x_left,y_left)、终点为(x_top,y_top)的线段连接
          if (left[0] < top[0]){
            const double& x_left3 = left[4], y_left3 = left[5], beta_left = left[6];
            const double& x_top3 = top[4], y_top3 = top[5], beta_top = top[6];
            // gamma = numerator / denominator
            double numerator = ( y_left3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_left) ) - ( y_top3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_top) );
            double denominator = ( x_left3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_left) ) - ( x_top3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_top) );
            double gamma = numerator / denominator;
            double x_top = x_top3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_top) - y_positive_min_turning_radius_*sin(gamma);
            double y_top = y_top3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_top) + y_positive_min_turning_radius_*cos(gamma);
            double x_left = x_left3 + ((y_positive_min_turning_radius_-y_positive_vehwidth_/2))*sin(beta_left) - y_positive_min_turning_radius_*sin(gamma);
            double y_left = y_left3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_left) + y_positive_min_turning_radius_*cos(gamma);
            // 对线段(x_left,y_left)-->(x_top,y_top)进行离散，并投影到borderline
            geometry_msgs::Point p1; p1.x=x_left; p1.y=y_left;
            geometry_msgs::Point p2; p2.x=x_top; p2.y=y_top;
            draw_index += draw_step_;
            addBorderPoints(p1, p2, center, draw_index, false);
          }
          // 判断top点和right点的x1_plus，如果right的x1_plus更小，则说明top和right的中弧需要被以gamma为斜率、起点为(x_top,y_top)、终点为(x_right,y_right)的线段连接
          if (right[1] > top[1]){
            const double& x_right3 = right[4], y_right3 = right[5], beta_right = right[6];
            const double& x_top3 = top[4], y_top3 = top[5], beta_top = top[6];
            // gamma = numerator / denominator
            double numerator = ( y_right3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_right) ) - ( y_top3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_top) );
            double denominator = ( x_right3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_right) ) - ( x_top3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_top) );
            double gamma = numerator / denominator;
            double x_top = x_top3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta_top) - y_positive_min_turning_radius_*sin(gamma);
            double y_top = y_top3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_top) + y_positive_min_turning_radius_*cos(gamma);
            double x_right = x_right3 + ((y_positive_min_turning_radius_-y_positive_vehwidth_/2))*sin(beta_right) - y_positive_min_turning_radius_*sin(gamma);
            double y_right = y_right3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta_right) + y_positive_min_turning_radius_*cos(gamma);
            // 对线段(x_top,y_top) --> (x_right,y_right)进行离散，并投影到borderline
            geometry_msgs::Point p1; p1.x=x_right; p1.y=y_right;
            geometry_msgs::Point p2; p2.x=x_top; p2.y=y_top;
            draw_index += draw_step_;
            addBorderPoints(p1, p2, center, draw_index, false);
          }
        }
        // 3.1 保存所有有效点的(x5,y5)
        std::vector<std::pair<double,double>> up_borderline_x5_y5_list;
        for (auto near_center_p:near_center_p_list){
          const double y1 = yMin_ + y_positive_vehwidth_/2;
          const double& x3 = near_center_p.x;
          const double& y3 = near_center_p.y;
          // 定义beta点的正方向为逆时针，即在p_top(near_center_p1)左侧的点的beta是正的，p_top(near_center_p1)右侧的点的beta是负的
          double beta = atan2(near_center_p1.y-near_center_p.y, near_center_p1.x-near_center_p.x);  //atanh() \in [-pi, pi]
          if (beta>M_PI_2) beta-=M_PI;  //cluseter beta into [-pi/2, pi/2]
          const double y5 = y3 - (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*cos(beta);
          const double x5 = x3 + (y_positive_min_turning_radius_-y_positive_vehwidth_/2)*sin(beta);
          up_borderline_x5_y5_list.push_back({x5,y5});
        }
        up_borderline_x5_y5_list.push_back({center.x, center.y});
        veh_up_borderline_all_x5_y5.push_back(up_borderline_x5_y5_list);
      }
    }
  } // end for

  // *** down_borderline ***
  // 3.2 计算相邻车辆间的交点，做填充并投影到borderline：
  sort(veh_down_borderline_all_x5_y5.begin(), veh_down_borderline_all_x5_y5.end(), compareList);
  for (int i=1; i<veh_down_borderline_all_x5_y5.size(); i++){
    auto left_veh_x5_y5_s = veh_down_borderline_all_x5_y5[i-1];
    left_veh_x5_y5_s.pop_back();  //去掉center点
    auto right_veh_x5_y5_s = veh_down_borderline_all_x5_y5[i];
    right_veh_x5_y5_s.pop_back();  //去掉center点
    std::vector<double> state;  //(x_{left,5,i}, y_{left,5,i}, x_{right,5,j}, y_{right,5,j}, x_o, y_o)
    double y_o_max = yMin_ + y_negative_vehwidth_/2;
    for (auto p1:left_veh_x5_y5_s){
      for (auto p2:right_veh_x5_y5_s){
        const double& x_left_5 = p1.first;
        const double& y_left_5 = p1.second;
        const double& x_right_5 = p2.first;
        const double& y_right_5 = p2.second;
        double a = x_left_5-x_right_5;
        double b = y_left_5-y_right_5;
        // double in_sqrt_value = a*a* (a*a+b*b-pow((a*a+b*b)/(4*y_negative_min_turning_radius_),2));
        double in_sqrt_value = 16*y_negative_min_turning_radius_*y_negative_min_turning_radius_/(a*a+b*b)-1;
        // 两圆心的距离大于4R
        if (in_sqrt_value<0) continue;
        double sin_value;
        // 根据图形学可知，如果左车的y大，那么lambda应该取cos值小的
        sin_value = -b/(4*y_negative_min_turning_radius_) + -a / (4*y_negative_min_turning_radius_) * sqrt(in_sqrt_value);
        // if (y_left_5<=y_right_5) cos_value = -a/(4*y_negative_min_turning_radius_) - sqrt(in_sqrt_value)/(a*a+b*b) ;
        // else if (y_left_5>y_right_5) cos_value = -a/(4*y_negative_min_turning_radius_) + sqrt(in_sqrt_value)/(a*a+b*b) ;
        double alpha = asin(sin_value);
        // double alpha = asin(-b/(4*y_negative_min_turning_radius_) + sqrt(a*a* pow(a*a+b*b-(a*a+b*b)/(4*y_negative_min_turning_radius_),2) )/(a*a+b*b) );   // asin \in [-pi/2, pi/2]
        // (x_o, y_o)是拟合出来的虚拟大弧的圆心点
        double x_o = x_left_5 + 2*y_negative_min_turning_radius_*cos(alpha);
        double y_o = y_left_5 + 2*y_negative_min_turning_radius_*sin(alpha);
        // 如果y_o-y_negative_min_turning_radius_比沿着边缘走还低，那说明根本这两个点形成的两个弧线之间没有交点
        if (y_o-y_negative_min_turning_radius_<yMin_+y_negative_vehwidth_/2) continue;
        // 如果y_o不必y_left5和
        // 如果x_o在x_left和x_right之间，且 y_o比y_o_max大，则当前圆心是要被保留的
        if (x_o>=x_left_5 && x_o<=x_right_5 && y_o>y_o_max){
          y_o_max = y_o;
          state = {x_left_5,y_left_5,x_right_5,y_right_5,x_o,y_o};
        }
      }
    }
    if (!state.empty()){
      // 利用state 填充两个车间的borderline
      double lambda1 = atan2(state[1]-state[5], state[0]-state[4]); // atan2 \in [-pi,pi]
      double lambda2 = atan2(state[3]-state[5], state[2]-state[4]);
      draw_index += draw_step_;
      for (float lambda=lambda1; lambda<=lambda2; lambda+=theta_step_){
        geometry_msgs::Point p; p.x=state[4]+cos(lambda)*y_negative_min_turning_radius_; p.y=state[5]+sin(lambda)*y_negative_min_turning_radius_;
        geometry_msgs::Point center; center.y=(state[1]+state[3])/2;
        addBorderPoints(p, center, draw_index);
      }
    }
  }

  // *** up_borderline ***
  // 3.2 计算相邻车辆间的交点，做填充并投影到borderline：
  sort(veh_up_borderline_all_x5_y5.begin(), veh_up_borderline_all_x5_y5.end(), compareList);
  for (int i=1; i<veh_up_borderline_all_x5_y5.size(); i++){
    auto left_veh_x5_y5_s = veh_up_borderline_all_x5_y5[i-1];
    left_veh_x5_y5_s.pop_back();  //去掉center点
    auto right_veh_x5_y5_s = veh_up_borderline_all_x5_y5[i];
    right_veh_x5_y5_s.pop_back();  //去掉center点
    std::vector<double> state;  //(x_{left,5,i}, y_{left,5,i}, x_{right,5,j}, y_{right,5,j}, x_o, y_o)
    double y_o_max = yMin_ + y_positive_vehwidth_/2;
    for (auto p1:left_veh_x5_y5_s){
      for (auto p2:right_veh_x5_y5_s){
        const double& x_left_5 = p1.first;
        const double& y_left_5 = p1.second;
        const double& x_right_5 = p2.first;
        const double& y_right_5 = p2.second;
        double a = x_left_5-x_right_5;
        double b = y_left_5-y_right_5;
        // double in_sqrt_value = a*a* (a*a+b*b-pow((a*a+b*b)/(4*y_negative_min_turning_radius_),2));
        double in_sqrt_value = 16*y_positive_min_turning_radius_*y_positive_min_turning_radius_/(a*a+b*b)-1;
        // 两圆心的距离大于4R
        if (in_sqrt_value<0) continue;
        double sin_value;
        // 根据图形学可知，如果左车的y大，那么lambda应该取cos值小的
        sin_value = -b/(4*y_positive_min_turning_radius_) + -a / (4*y_positive_min_turning_radius_) * sqrt(in_sqrt_value);
        // if (y_left_5<=y_right_5) cos_value = -a/(4*y_negative_min_turning_radius_) - sqrt(in_sqrt_value)/(a*a+b*b) ;
        // else if (y_left_5>y_right_5) cos_value = -a/(4*y_negative_min_turning_radius_) + sqrt(in_sqrt_value)/(a*a+b*b) ;
        double alpha = asin(sin_value);
        // double alpha = asin(-b/(4*y_negative_min_turning_radius_) + sqrt(a*a* pow(a*a+b*b-(a*a+b*b)/(4*y_negative_min_turning_radius_),2) )/(a*a+b*b) );   // asin \in [-pi/2, pi/2]
        // (x_o, y_o)是拟合出来的虚拟大弧的圆心点
        double x_o = x_left_5 + 2*y_positive_min_turning_radius_*cos(alpha);
        double y_o = y_left_5 + 2*y_positive_min_turning_radius_*sin(alpha);
        // 如果y_o-y_negative_min_turning_radius_比沿着边缘走还低，那说明根本这两个点形成的两个弧线之间没有交点
        if (y_o-y_positive_min_turning_radius_<yMin_+y_positive_vehwidth_/2) continue;
        // 如果y_o不必y_left5和
        // 如果x_o在x_left和x_right之间，且 y_o比y_o_max大，则当前圆心是要被保留的
        if (x_o>=x_left_5 && x_o<=x_right_5 && y_o>y_o_max){
          y_o_max = y_o;
          state = {x_left_5,y_left_5,x_right_5,y_right_5,x_o,y_o};
        }
      }
    }
    if (!state.empty()){
      // 利用state 填充两个车间的borderline
      double lambda1 = atan2(state[1]-state[5], state[0]-state[4]); // atan2 \in [-pi,pi]
      double lambda2 = atan2(state[3]-state[5], state[2]-state[4]);
      draw_index += draw_step_;
      for (float lambda=lambda1; lambda<=lambda2; lambda+=theta_step_){
        geometry_msgs::Point p; p.x=state[4]+cos(lambda)*y_positive_min_turning_radius_; p.y=state[5]+sin(lambda)*y_positive_min_turning_radius_;
        geometry_msgs::Point center; center.y=(state[1]+state[3])/2;
        addBorderPoints(p, center, draw_index,false);
      }
    }
  }

  auto down_borderline_copy = down_borderline_;
  auto up_borderline_copy = up_borderline_;

  // normal situation: 车辆靠右行走
  /** 4.1 按照车模型 膨胀down boderline所有点 **/
  {
    geometry_msgs::Point false_pos; false_pos.y=-1.0;
    auto temp_borderline = down_borderline_;
    // 太密集的话 画出来的borderline会很多参差/毛刺
    int delta_step = 20;
    for (int i=delta_step; i<down_borderline_.size();i++){
      geometry_msgs::Point center;
      center.x = index2x_list_[i-delta_step];
      center.y = down_borderline_[i-delta_step];
      false_pos.x = center.x;
      double theta = atan2(down_borderline_[i]-down_borderline_[i-delta_step], index2x_list_[i]-index2x_list_[i-delta_step]);
      // double theta = 0;
      for (int j=1; j<=y_negative_footprint_.size(); j++){
        geometry_msgs::Point p1; p1.x=y_negative_footprint_[j-1].first; p1.y=y_negative_footprint_[j-1].second;
        geometry_msgs::Point p1_rotated;
        p1_rotated.x = center.x + cos(theta)*p1.x + -sin(theta)*p1.y;
        p1_rotated.y = center.y + sin(theta)*p1.x + cos(theta)*p1.y;
        geometry_msgs::Point p2; p2.x=y_negative_footprint_[j%y_negative_footprint_.size()].first; p2.y=y_negative_footprint_[j%y_negative_footprint_.size()].second;
        geometry_msgs::Point p2_rotated;
        p2_rotated.x = center.x + cos(theta)*p2.x + -sin(theta)*p2.y;
        p2_rotated.y = center.y + sin(theta)*p2.x + cos(theta)*p2.y;
        addBorderPointsToSpecialVecAndRecordProjections(temp_borderline, p1_rotated, p2_rotated, false_pos);
      }
      addFootprintPoints(center, theta, y_negative_footprint_);
    }
    
    // visualize
    draw_index += draw_step_;
    for (int i=0; i<temp_borderline.size(); i++){
      down_borderline_extended_right_[i] = std::max(temp_borderline[i], down_borderline_[i]);
      stripe_borderline_.push_back({index2x_list_[i], temp_borderline[i], draw_index});
    }
  }
  
  /** 4.2 按照车模型 膨胀up boderline所有点 **/
  {
    geometry_msgs::Point false_pos; false_pos.y=1.0;
    auto temp_borderline = up_borderline_;
    // 太密集的话 画出来的borderline会很多参差/毛刺
    int delta_step = 20;
    for (int i=up_borderline_.size()-1; i>=delta_step; i--){
      geometry_msgs::Point center;
      center.x = index2x_list_[i];
      center.y = up_borderline_[i];
      double theta = atan2(up_borderline_[i-delta_step]-up_borderline_[i], index2x_list_[i-delta_step]-index2x_list_[i]);
      // double theta = 0;
      for (int j=1; j<=y_positive_footprint_.size(); j++){
        geometry_msgs::Point p1; p1.x=y_positive_footprint_[j-1].first; p1.y=y_positive_footprint_[j-1].second;
        geometry_msgs::Point p1_rotated;
        p1_rotated.x = center.x + cos(theta)*p1.x + -sin(theta)*p1.y;
        p1_rotated.y = center.y + sin(theta)*p1.x + cos(theta)*p1.y;
        geometry_msgs::Point p2; p2.x=y_positive_footprint_[j%y_positive_footprint_.size()].first; p2.y=y_positive_footprint_[j%y_positive_footprint_.size()].second;
        geometry_msgs::Point p2_rotated;
        p2_rotated.x = center.x + cos(theta)*p2.x + -sin(theta)*p2.y;
        p2_rotated.y = center.y + sin(theta)*p2.x + cos(theta)*p2.y;
        addBorderPointsToSpecialVec(temp_borderline, p1_rotated, p2_rotated, false_pos);
      }
      addFootprintPoints(center, theta, y_positive_footprint_);
    }
    // visualize
    draw_index += draw_step_;
    for (int i=0; i<temp_borderline.size(); i++){
      up_borderline_extended_right_[i] = std::min(temp_borderline[i], up_borderline_[i]);
      stripe_borderline_.push_back({index2x_list_[i], temp_borderline[i], draw_index});
    }
  }
  
  // special situation: 车辆靠左行走
  /** 5.1 按照车模型 膨胀down boderline所有点 **/
  {
    geometry_msgs::Point false_pos; false_pos.y=-1.0;
    auto temp_borderline = down_borderline_copy;
    // 太密集的话 画出来的borderline会很多参差/毛刺
    int delta_step = 20;
    for (int i=down_borderline_copy.size()-1; i>=delta_step; i--){
      geometry_msgs::Point center;
      center.x = index2x_list_[i-delta_step];
      center.y = down_borderline_copy[i-delta_step];
      double theta = atan2(down_borderline_copy[i-delta_step]-down_borderline_copy[i], index2x_list_[i-delta_step]-index2x_list_[i]);
      // double theta = 0;
      for (int j=1; j<=y_positive_footprint_.size(); j++){
        geometry_msgs::Point p1; p1.x=y_positive_footprint_[j-1].first; p1.y=y_positive_footprint_[j-1].second;
        geometry_msgs::Point p1_rotated;
        p1_rotated.x = center.x + cos(theta)*p1.x + -sin(theta)*p1.y;
        p1_rotated.y = center.y + sin(theta)*p1.x + cos(theta)*p1.y;
        geometry_msgs::Point p2; p2.x=y_positive_footprint_[j%y_positive_footprint_.size()].first; p2.y=y_positive_footprint_[j%y_positive_footprint_.size()].second;
        geometry_msgs::Point p2_rotated;
        p2_rotated.x = center.x + cos(theta)*p2.x + -sin(theta)*p2.y;
        p2_rotated.y = center.y + sin(theta)*p2.x + cos(theta)*p2.y;
        addBorderPointsToSpecialVec(temp_borderline, p1_rotated, p2_rotated, false_pos);
      }
    }
    // visualize
    draw_index += draw_step_;
    for (int i=0; i<temp_borderline.size(); i++){
      down_borderline_extended_left_[i] = std::max(temp_borderline[i], down_borderline_copy[i]);
      stripe_borderline_.push_back({index2x_list_[i], temp_borderline[i], draw_index});
    }
  }

  /** 5.2 按照车模型 膨胀up boderline所有点 **/
  {
    geometry_msgs::Point false_pos; false_pos.y=1.0;
    auto temp_borderline = up_borderline_copy;
    // 太密集的话 画出来的borderline会很多参差/毛刺
    int delta_step = 20;
    for (int i=delta_step; i<up_borderline_copy.size();i++){
      geometry_msgs::Point center;
      center.x = index2x_list_[i];
      center.y = up_borderline_copy[i];
      double theta = atan2(up_borderline_copy[i]-up_borderline_copy[i-delta_step], index2x_list_[i]-index2x_list_[i-delta_step]);
      // double theta = 0;
      for (int j=1; j<=y_negative_footprint_.size(); j++){
        geometry_msgs::Point p1; p1.x=y_negative_footprint_[j-1].first; p1.y=y_negative_footprint_[j-1].second;
        geometry_msgs::Point p1_rotated;
        p1_rotated.x = center.x + cos(theta)*p1.x + -sin(theta)*p1.y;
        p1_rotated.y = center.y + sin(theta)*p1.x + cos(theta)*p1.y;
        geometry_msgs::Point p2; p2.x=y_negative_footprint_[j%y_negative_footprint_.size()].first; p2.y=y_negative_footprint_[j%y_negative_footprint_.size()].second;
        geometry_msgs::Point p2_rotated;
        p2_rotated.x = center.x + cos(theta)*p2.x + -sin(theta)*p2.y;
        p2_rotated.y = center.y + sin(theta)*p2.x + cos(theta)*p2.y;
        addBorderPointsToSpecialVec(temp_borderline, p1_rotated, p2_rotated, false_pos);
      }
    }
    // visualize
    draw_index += draw_step_;
    for (int i=0; i<temp_borderline.size(); i++){
      up_borderline_extended_left_[i] = std::min(temp_borderline[i], up_borderline_copy[i]);
      stripe_borderline_.push_back({index2x_list_[i], temp_borderline[i], draw_index});
    }
  }
}

// 发布up_borderline、down_borderline、stripe_borderline
void ExtractEnv::pubBorderlines(){
  // 画图时降低分辨率
  int draw_delta = 10;
  // up_borderline
  visualization_msgs::Marker up_borderline_marker;
  std_msgs::Header msg_header1;
  msg_header1.stamp = ros::Time::now();
  msg_header1.frame_id = frame_id_;
  up_borderline_marker.header = msg_header1;
  up_borderline_marker.id = 10000;
  up_borderline_marker.color.r = 0.1;
  up_borderline_marker.color.g = 0.5;
  up_borderline_marker.color.b = 0.1;
  up_borderline_marker.color.a = 1.0;
  up_borderline_marker.scale.x = 0.01; 
  up_borderline_marker.pose.orientation.w = 1.0;
  up_borderline_marker.type = visualization_msgs::Marker::LINE_STRIP;
  for (int i=0; i<index2x_list_.size(); i++){
    if (i%draw_delta!=0)
      continue;
    geometry_msgs::Point p;
    p.x = index2x_list_[i];
    p.y = up_borderline_[i];
    up_borderline_marker.points.push_back(p);
  }
  pub_up_borderline_.publish(up_borderline_marker);

  // up_borderline_extend_right
  draw_delta = 10;
  visualization_msgs::Marker up_borderline_marker_extend_right;
  msg_header1.stamp = ros::Time::now();
  msg_header1.frame_id = frame_id_;
  up_borderline_marker_extend_right.header = msg_header1;
  up_borderline_marker_extend_right.id = 10000;
  up_borderline_marker_extend_right.color.r = 0.1;
  up_borderline_marker_extend_right.color.g = 0.5;
  up_borderline_marker_extend_right.color.b = 0.1;
  up_borderline_marker_extend_right.color.a = 1.0;
  up_borderline_marker_extend_right.scale.x = 0.005; 
  up_borderline_marker_extend_right.pose.orientation.w = 1.0;
  up_borderline_marker_extend_right.type = visualization_msgs::Marker::LINE_STRIP;
  for (int i=0; i<index2x_list_.size(); i++){
    if (i%draw_delta!=0)
      continue;    
    geometry_msgs::Point p;
    p.x = index2x_list_[i];
    p.y = up_borderline_extended_right_[i];
    up_borderline_marker_extend_right.points.push_back(p);
  }
  pub_up_borderline_extended_right_.publish(up_borderline_marker_extend_right);

  // up_borderline_extend_left
  draw_delta = 10;
  visualization_msgs::Marker up_borderline_marker_extend_left;
  msg_header1.stamp = ros::Time::now();
  msg_header1.frame_id = frame_id_;
  up_borderline_marker_extend_left.header = msg_header1;
  up_borderline_marker_extend_left.id = 10000;
  up_borderline_marker_extend_left.color.r = 0.1;
  up_borderline_marker_extend_left.color.g = 0.5;
  up_borderline_marker_extend_left.color.b = 0.1;
  up_borderline_marker_extend_left.color.a = 1.0;
  up_borderline_marker_extend_left.scale.x = 0.005; 
  up_borderline_marker_extend_left.pose.orientation.w = 1.0;
  up_borderline_marker_extend_left.type = visualization_msgs::Marker::LINE_STRIP;
  for (int i=0; i<index2x_list_.size(); i++){
    if (i%draw_delta!=0)
      continue;
    geometry_msgs::Point p;
    p.x = index2x_list_[i];
    p.y = up_borderline_extended_left_[i];
    up_borderline_marker_extend_left.points.push_back(p);
  }
  pub_up_borderline_extended_left_.publish(up_borderline_marker_extend_left);

  // up_borderline_individual
  draw_delta = 1;
  visualization_msgs::Marker up_borderline_marker_individual;
  msg_header1.stamp = ros::Time::now();
  msg_header1.frame_id = frame_id_;
  up_borderline_marker_individual.header = msg_header1;
  up_borderline_marker_individual.id = 10000;
  up_borderline_marker_individual.ns = "oncoming";
  up_borderline_marker_individual.color.r = 0.1;
  up_borderline_marker_individual.color.g = 0.5;
  up_borderline_marker_individual.color.b = 0.1;
  up_borderline_marker_individual.color.a = 1.0;
  up_borderline_marker_individual.scale.x = 0.005;
  up_borderline_marker_individual.scale.y = 0.005;
  up_borderline_marker_individual.scale.z = 0.005;  
  up_borderline_marker_individual.pose.orientation.w = 1.0;
  up_borderline_marker_individual.type = visualization_msgs::Marker::POINTS;
  for (auto vec:up_borderline_individual_){
    for (int i=0; i<index2x_list_.size(); i++){
      if (i%draw_delta!=0)
        continue;      
      geometry_msgs::Point p;
      p.x = index2x_list_[i];
      p.y = vec[i];
      if (p.y==yMin_ || p.y==yMax_)
        continue;      
      up_borderline_marker_individual.points.push_back(p);
    }
  }  
  pub_up_borderline_individual_.publish(up_borderline_marker_individual);
  visualization_msgs::Marker up_borderline_marker_individual_for_av;
  msg_header1.stamp = ros::Time::now();
  msg_header1.frame_id = frame_id_;
  up_borderline_marker_individual_for_av.header = msg_header1;
  up_borderline_marker_individual_for_av.id = 10001;
  up_borderline_marker_individual_for_av.ns = "autonomous vehicle";
  up_borderline_marker_individual_for_av.color.r = 0.1;
  up_borderline_marker_individual_for_av.color.g = 0.5;
  up_borderline_marker_individual_for_av.color.b = 0.1;
  up_borderline_marker_individual_for_av.color.a = 1.0;
  up_borderline_marker_individual_for_av.scale.x = 0.005;
  up_borderline_marker_individual_for_av.scale.y = 0.005;
  up_borderline_marker_individual_for_av.scale.z = 0.005;  
  up_borderline_marker_individual_for_av.pose.orientation.w = 1.0;
  up_borderline_marker_individual_for_av.type = visualization_msgs::Marker::POINTS;
  for (auto vec:up_borderline_individual_for_left_backup_){
    for (int i=0; i<index2x_list_.size(); i++){
      if (i%draw_delta!=0)
        continue;      
      geometry_msgs::Point p;
      p.x = index2x_list_[i];
      p.y = vec[i];
      if (p.y==yMin_ || p.y==yMax_)
        continue;      
      up_borderline_marker_individual_for_av.points.push_back(p);
    }
  }  
  pub_up_borderline_individual_.publish(up_borderline_marker_individual_for_av);



  // up_borderline_individual_extend
  draw_delta = 1;
  visualization_msgs::Marker up_borderline_marker_individual_extend;
  msg_header1.stamp = ros::Time::now();
  msg_header1.frame_id = frame_id_;
  up_borderline_marker_individual_extend.header = msg_header1;
  up_borderline_marker_individual_extend.id = 10000;
  up_borderline_marker_individual_extend.color.r = 0.1;
  up_borderline_marker_individual_extend.color.g = 0.5;
  up_borderline_marker_individual_extend.color.b = 0.1;
  up_borderline_marker_individual_extend.color.a = 1.0;
  up_borderline_marker_individual_extend.scale.x = 0.005;
  up_borderline_marker_individual_extend.scale.y = 0.005;
  up_borderline_marker_individual_extend.scale.z = 0.005;  
  up_borderline_marker_individual_extend.pose.orientation.w = 1.0;
  up_borderline_marker_individual_extend.type = visualization_msgs::Marker::POINTS;
  for (auto vec:up_borderline_individual_extended_){
    for (int i=0; i<index2x_list_.size(); i++){
      if (i%draw_delta!=0)
        continue;      
      geometry_msgs::Point p;
      p.x = index2x_list_[i];
      p.y = vec[i];
      if (p.y==yMin_ || p.y==yMax_)
        continue;      
      up_borderline_marker_individual_extend.points.push_back(p);
    }
  }  
  pub_up_borderline_individual_extended_.publish(up_borderline_marker_individual_extend);

  // down_borderline
  draw_delta = 10;
  visualization_msgs::Marker down_borderline_marker;
  std_msgs::Header msg_header2;
  msg_header2.stamp = ros::Time::now();
  msg_header2.frame_id = frame_id_;
  down_borderline_marker.header = msg_header2;
  down_borderline_marker.id = 10000;
  down_borderline_marker.color.r = 0.5;
  down_borderline_marker.color.g = 0.1;
  down_borderline_marker.color.b = 0.1;
  down_borderline_marker.color.a = 1.0;
  down_borderline_marker.scale.x = 0.01; 
  down_borderline_marker.pose.orientation.w = 1.0;
  down_borderline_marker.type = visualization_msgs::Marker::LINE_STRIP;
  for (int i=0; i<index2x_list_.size(); i++){
    if (i%draw_delta!=0)
      continue;    
    geometry_msgs::Point p;
    p.x = index2x_list_[i];
    p.y = down_borderline_[i];
    down_borderline_marker.points.push_back(p);
  }
  pub_down_borderline_.publish(down_borderline_marker);

  // down_borderline_extend_right
  draw_delta = 10;
  visualization_msgs::Marker down_borderline_marker_extend_right;
  msg_header2.stamp = ros::Time::now();
  msg_header2.frame_id = frame_id_;
  down_borderline_marker_extend_right.header = msg_header2;
  down_borderline_marker_extend_right.id = 10000;
  down_borderline_marker_extend_right.color.r = 0.5;
  down_borderline_marker_extend_right.color.g = 0.1;
  down_borderline_marker_extend_right.color.b = 0.1;
  down_borderline_marker_extend_right.color.a = 1.0;
  down_borderline_marker_extend_right.scale.x = 0.005; 
  down_borderline_marker_extend_right.pose.orientation.w = 1.0;
  down_borderline_marker_extend_right.type = visualization_msgs::Marker::LINE_STRIP;
  for (int i=0; i<index2x_list_.size(); i++){
    if (i%draw_delta!=0)
      continue;    
    geometry_msgs::Point p;
    p.x = index2x_list_[i];
    p.y = down_borderline_extended_right_[i];
    down_borderline_marker_extend_right.points.push_back(p);
  }
  pub_down_borderline_extended_right_.publish(down_borderline_marker_extend_right);

  // down_borderline_extend_left
  draw_delta = 10;
  visualization_msgs::Marker down_borderline_marker_extend_left;
  msg_header2.stamp = ros::Time::now();
  msg_header2.frame_id = frame_id_;
  down_borderline_marker_extend_left.header = msg_header2;
  down_borderline_marker_extend_left.id = 10000;
  down_borderline_marker_extend_left.color.r = 0.5;
  down_borderline_marker_extend_left.color.g = 0.1;
  down_borderline_marker_extend_left.color.b = 0.1;
  down_borderline_marker_extend_left.color.a = 1.0;
  down_borderline_marker_extend_left.scale.x = 0.005; 
  down_borderline_marker_extend_left.pose.orientation.w = 1.0;
  down_borderline_marker_extend_left.type = visualization_msgs::Marker::LINE_STRIP;
  for (int i=0; i<index2x_list_.size(); i++){
    if (i%draw_delta!=0)
      continue;    
    geometry_msgs::Point p;
    p.x = index2x_list_[i];
    p.y = down_borderline_extended_left_[i];
    down_borderline_marker_extend_left.points.push_back(p);
  }
  pub_down_borderline_extended_left_.publish(down_borderline_marker_extend_left);

  // down_borderline_individual
  draw_delta = 1;
  visualization_msgs::Marker down_borderline_marker_individual;
  msg_header2.stamp = ros::Time::now();
  msg_header2.frame_id = frame_id_;
  down_borderline_marker_individual.header = msg_header2;
  down_borderline_marker_individual.id = 10000;
  down_borderline_marker_individual.color.r = 0.5;
  down_borderline_marker_individual.color.g = 0.1;
  down_borderline_marker_individual.color.b = 0.1;
  down_borderline_marker_individual.color.a = 1.0;
  down_borderline_marker_individual.scale.x = 0.005;
  down_borderline_marker_individual.scale.y = 0.005;
  down_borderline_marker_individual.scale.z = 0.005;    
  down_borderline_marker_individual.pose.orientation.w = 1.0;
  down_borderline_marker_individual.type = visualization_msgs::Marker::POINTS;
  for (auto vec:down_borderline_individual_){
    for (int i=0; i<index2x_list_.size(); i++){
      if (i%draw_delta!=0)
        continue;      
      geometry_msgs::Point p;
      p.x = index2x_list_[i];
      p.y = vec[i];
      if (p.y==yMin_ || p.y==yMax_)
        continue;      
      down_borderline_marker_individual.points.push_back(p);
    }
  }
  pub_down_borderline_individual_.publish(down_borderline_marker_individual);

  // down_borderline_individual_extend
  draw_delta = 1;
  visualization_msgs::Marker down_borderline_marker_individual_extend;
  msg_header2.stamp = ros::Time::now();
  msg_header2.frame_id = frame_id_;
  down_borderline_marker_individual_extend.header = msg_header2;
  down_borderline_marker_individual_extend.id = 10000;
  down_borderline_marker_individual_extend.color.r = 0.5;
  down_borderline_marker_individual_extend.color.g = 0.1;
  down_borderline_marker_individual_extend.color.b = 0.1;
  down_borderline_marker_individual_extend.color.a = 1.0;
  down_borderline_marker_individual_extend.scale.x = 0.005;
  down_borderline_marker_individual_extend.scale.y = 0.005;
  down_borderline_marker_individual_extend.scale.z = 0.005;    
  down_borderline_marker_individual_extend.pose.orientation.w = 1.0;
  down_borderline_marker_individual_extend.type = visualization_msgs::Marker::POINTS;
  for (auto vec:down_borderline_individual_extended_){
    for (int i=0; i<index2x_list_.size(); i++){
      if (i%draw_delta!=0)
        continue;      
      geometry_msgs::Point p;
      p.x = index2x_list_[i];
      p.y = vec[i];
      if (p.y==yMin_ || p.y==yMax_)
        continue;      
      down_borderline_marker_individual_extend.points.push_back(p);
    }
  }
  pub_down_borderline_individual_extended_.publish(down_borderline_marker_individual_extend);

  // borderline_with_static_vehicles_footprints_
  draw_delta = 10;
  visualization_msgs::Marker up_borderline_with_static_vehicles_footprints;
  msg_header2.stamp = ros::Time::now();
  msg_header2.frame_id = frame_id_;
  up_borderline_with_static_vehicles_footprints.header = msg_header2;
  up_borderline_with_static_vehicles_footprints.id = 10000;
  up_borderline_with_static_vehicles_footprints.ns = "up";
  up_borderline_with_static_vehicles_footprints.color.r = 0.5;
  up_borderline_with_static_vehicles_footprints.color.g = 0.1;
  up_borderline_with_static_vehicles_footprints.color.b = 0.1;
  up_borderline_with_static_vehicles_footprints.color.a = 1.0;
  up_borderline_with_static_vehicles_footprints.scale.x = 0.005;
  up_borderline_with_static_vehicles_footprints.scale.y = 0.005;
  up_borderline_with_static_vehicles_footprints.scale.z = 0.005;    
  up_borderline_with_static_vehicles_footprints.pose.orientation.w = 1.0;
  up_borderline_with_static_vehicles_footprints.type = visualization_msgs::Marker::LINE_STRIP;
  for (int i=0; i<up_borderline_with_static_vehicles_footprints_.size(); i++){
    if (i%draw_delta!=0)
      continue;       
    geometry_msgs::Point p;
    p.x = index2x_list_[i];
    p.y = up_borderline_with_static_vehicles_footprints_[i];    
    up_borderline_with_static_vehicles_footprints.points.push_back(p);
  }
  pub_borderline_with_static_vehicles_footprints_.publish(up_borderline_with_static_vehicles_footprints);

  draw_delta = 10;
  visualization_msgs::Marker down_borderline_with_static_vehicles_footprints;
  msg_header2.stamp = ros::Time::now();
  msg_header2.frame_id = frame_id_;
  down_borderline_with_static_vehicles_footprints.header = msg_header2;
  down_borderline_with_static_vehicles_footprints.id = 10000;
  down_borderline_with_static_vehicles_footprints.color.r = 0.5;
  down_borderline_with_static_vehicles_footprints.color.g = 0.1;
  down_borderline_with_static_vehicles_footprints.color.b = 0.1;
  down_borderline_with_static_vehicles_footprints.color.a = 1.0;
  down_borderline_with_static_vehicles_footprints.scale.x = 0.005;
  down_borderline_with_static_vehicles_footprints.scale.y = 0.005;
  down_borderline_with_static_vehicles_footprints.scale.z = 0.005;    
  down_borderline_with_static_vehicles_footprints.pose.orientation.w = 1.0;
  down_borderline_with_static_vehicles_footprints.type = visualization_msgs::Marker::LINE_STRIP;
  for (int i=0; i<down_borderline_with_static_vehicles_footprints_.size(); i++){
    if (i%draw_delta!=0)
      continue;       
    geometry_msgs::Point p;
    p.x = index2x_list_[i];
    p.y = down_borderline_with_static_vehicles_footprints_[i];    
    down_borderline_with_static_vehicles_footprints.points.push_back(p);
  }
  pub_borderline_with_static_vehicles_footprints_.publish(down_borderline_with_static_vehicles_footprints);

  // stripe_borderline
  draw_delta = 10;
  visualization_msgs::Marker stripe_borderline_marker;
  std_msgs::Header msg_header3;
  msg_header3.stamp = ros::Time::now();
  msg_header3.frame_id = frame_id_;
  stripe_borderline_marker.header = msg_header3;
  stripe_borderline_marker.id = 10000;
  stripe_borderline_marker.color.r = 0.1;
  stripe_borderline_marker.color.g = 0.5;
  stripe_borderline_marker.color.b = 0.1;
  stripe_borderline_marker.color.a = 1.0;
  stripe_borderline_marker.scale.x = 0.002;
  stripe_borderline_marker.scale.y = 0.002;
  stripe_borderline_marker.scale.z = 0.002;
  stripe_borderline_marker.pose.orientation.w = 1.0;
  stripe_borderline_marker.type = visualization_msgs::Marker::POINTS;
  for (int i=0; i<stripe_borderline_.size(); i++){
    if (i%draw_delta!=0)
      continue;    
    auto point = stripe_borderline_[i];
    geometry_msgs::Point p;
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    stripe_borderline_marker.points.push_back(p);
  }
  pub_stripe_borderline_.publish(stripe_borderline_marker);
}

// 利用pub_footprints_ 发布两侧的车贴边走的整个过程中的footprints
void ExtractEnv::pubIntermediateFootprints(){
  int idx = 0;
  int draw_delta = 50;
  for (int j=0; j<intermediate_footprints_.size(); j++){
    // 每draw_delta个footprint 显示一次
    if (j%draw_delta!=0)
      continue;
    auto footprint = intermediate_footprints_[j];
    for (int i=1; i<=footprint.size(); i++){
      visualization_msgs::Marker intermediate_footprints_marker;
      std_msgs::Header msg_header;
      msg_header.stamp = ros::Time::now();
      msg_header.frame_id = frame_id_;
      intermediate_footprints_marker.header = msg_header;
      intermediate_footprints_marker.id = idx++;
      intermediate_footprints_marker.color.r = 0.5;
      intermediate_footprints_marker.color.g = 0.1;
      intermediate_footprints_marker.color.b = 0.1;
      intermediate_footprints_marker.color.a = 1.0;
      intermediate_footprints_marker.scale.x = 0.002;
      intermediate_footprints_marker.scale.y = 0.002;
      intermediate_footprints_marker.pose.orientation.w = 1.0;
      intermediate_footprints_marker.type = visualization_msgs::Marker::LINE_STRIP;
      intermediate_footprints_marker.action = visualization_msgs::Marker::ADD;

      intermediate_footprints_marker.points.push_back(footprint[i-1]);
      intermediate_footprints_marker.points.push_back(footprint[i%footprint.size()]);   
    
      pub_footprints_.publish(intermediate_footprints_marker);
    }
  }
}

void ExtractEnv::pubBorderInformationForTEB(){
  teb_local_planner::borderConstraintsMsg msg;
  int delta = 10;
  std::vector<float> temp_up;
  for (int i=0; i<up_borderline_.size(); i++)
    if (i%delta==0)
      temp_up.push_back(up_borderline_[i]);
  std::vector<float> temp_down;
  for (int i=0; i<down_borderline_.size(); i++)
    if (i%delta==0)
      temp_down.push_back(down_borderline_[i]);
  
  msg.xMin = xMin_;
  msg.xMax = xMax_;
  msg.stepSize = step_size_*delta;
  msg.upBorderline = temp_up;
  msg.downBorderline = temp_down;

  pub_borderline_messsages_for_teb_.publish(msg);
}
