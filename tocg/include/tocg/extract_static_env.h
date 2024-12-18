#ifndef EXTRACT_STATIC_ENV_H
#define EXTRACT_STATIC_ENV_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <teb_local_planner/borderConstraintsMsg.h>
#include <vehicle_simulator/SocialVehicles.h>
#include <tf/tf.h>


class ExtractEnv{
  public:
    ExtractEnv(){};
    ExtractEnv(const ros::NodeHandle& publicHandle, const ros::NodeHandle& privateHandle): nh_(publicHandle), privatenh_(privateHandle){};
    ~ExtractEnv();
    
    // 初始化参数及publisher、subscriber
    void initializeExt();

    /**
     * @input: 地图信息
     * @description: 初始化index2x_list_, up_borderline_, down_borderline_等
     * **/
    void borderlineCB(const visualization_msgs::Marker::ConstPtr& msg);

    /**
     * @input: 静态车辆信息
     * @description: 调用doubleWinInteract，处理正常向右会车、正常向左会车
     *               调用zeroSumInteract，处理扎入 和 倒入
     * **/
    void staticVehCB(const visualization_msgs::MarkerArray::ConstPtr& msg);

    /**
     * @input: 静态车辆信息
     * @description: 0.按车辆宽度扩展边界
     *               1.把每个车最多离散成3个点，假设自车后轮贴着这些点划弧，做投影
     *               2.同一个车的离散点间的平滑连接
     *               3.相邻两个车间的平滑连接
     *               4.车辆向右转 按照自车footprint膨胀borderline
     *               5.车辆向左转 按照自车footprint膨胀borderline
     * **/    
    void doubleWinInteract(const std::vector<visualization_msgs::Marker>& msg);

    /**
     * @input: 静态车辆信息
     * @description: 0.按车辆宽度扩展边界
     *               1.把每个车最多离散成3个点，假设自车后轮贴着这些点划弧，做投影 （划弧只考虑中间弧 不考虑两侧弧）
     *               2.同一个车的离散点间的平滑连接、车辆向右转 按照自车footprint膨胀borderline
     * **/    
    void zeroSumInteract(const std::vector<visualization_msgs::Marker>& msg);

    /**
     * @input: 一个borderline的copy版 temp_vec; 
     *         两个点footprint的边线的端点p1、p2; 
     *         一个自车的位置点center
     * @description: 离散化p1p2，将点加入到temp_vec里，如果center在中心往下，则对temp_vec去max；否则取min
     * **/
    void addBorderPointsToSpecialVec(std::vector<double>& temp_vec, const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, const geometry_msgs::Point& center);

    /**
     * @input: 一个空向量index_from_extended_dbr_to_dbr； 一个borderline的copy版 temp_vec; 两个点footprint的边线的端点p1、p2; 一个自车的位置点center
     * @description: 离散化p1p2，将点加入到temp_vec里，如果center在中心往下，则对temp_vec去max；否则取min
     *               保存映射index_from_extended_dbr_to_dbr
     * */
    void addBorderPointsToSpecialVecAndRecordProjections(std::vector<double>& temp_vec, const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, const geometry_msgs::Point& center);

    /**
     * @input: 一个borderline的copy版 temp_vec; 
     *         一个点p; 
     *         一个自车的位置点center
     * @description: 将p点加入到temp_vec里，如果center在中心往下，则对temp_vec去max；否则取min
     * **/
    void addPointToSpecialVec(std::vector<std::pair<double,double>>& temp_vec, const geometry_msgs::Point& point, const geometry_msgs::Point& center, bool flag);
    
    /**
     * @input: 任意两个点p1、p2，自车位置center
     * @description: 离散化p1p2，加入到up_borderline或down_borderline
     *               将离散点加入到stripe_borderline用作可视化
     * **/
    void addBorderPoints(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, const geometry_msgs::Point& center, double height, bool flag);
    void addAbsBorderPoints(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, const geometry_msgs::Point& center, double height);

    /**
     * @input: 任意点p、自车位置center
     * @description: 将p加入到up_borderline或down_borderline
     *               将p加入到stripe_borderline用作可视化
     * **/
    void addBorderPoints(const geometry_msgs::Point& point, const geometry_msgs::Point& center, double height, bool flag);

    /**
     * @input: 根据车当前位置旋转机器人的footprint的一条边线的端点p1、p2
     * @descrption: 离散化p1p2，将点加入到intermediate_footprints_; 此函数被迭代调用
     * **/
    void addFootprintPoints(geometry_msgs::Point center, double theta, std::vector<std::pair<double, double>> footprint);

    std::vector<double> addDynamicVehToBorerlineAndGainMiddlePath(geometry_msgs::Point point, std::vector<std::pair<double, double>> footprint, double normal_limited_goal);
    std::vector<double> combineTwoBorderlines(const std::vector<double>& vec1, const std::vector<double>& vec2, const geometry_msgs::Point& center);

    // 世界坐标系 转化到 数组的index
    int xPos2Index(const double& xPos);

    // 数组的index 转化到 世界坐标系
    double Index2xPos(const int& index);

    // 检测p点是否超越了border或撞击到了静态车
    bool isExtendedVehCollisionBorderOrStaticVeh(const geometry_msgs::Point& p, std::vector<std::pair<double, double>> footprint);

    bool isVehCollisionBorderOrStaticVeh(const geometry_msgs::Point& p, std::vector<std::pair<double, double>> footprint);

    // 检测静态环境是否被处理完成了
    bool isCompleted(){return is_borderline_initialized_ && is_static_veh_projected_; };
    // 发布up_borderline、down_borderline、stripe_borderline
    void pubBorderlines();

    // 利用pub_footprints_ 发布两侧的车贴边走的整个过程中的footprints
    void pubIntermediateFootprints();

    // 把borderline信息发给teb
    void pubBorderInformationForTEB();
  

  public:
    ros::NodeHandle nh_;
    ros::NodeHandle privatenh_;
    ros::Publisher pub_up_borderline_;
    ros::Publisher pub_down_borderline_;
    ros::Publisher pub_borderline_messsages_for_teb_;

    ros::Publisher pub_up_borderline_extended_right_;
    ros::Publisher pub_down_borderline_extended_right_;
    ros::Publisher pub_up_borderline_extended_left_;
    ros::Publisher pub_down_borderline_extended_left_;

    ros::Publisher pub_up_borderline_individual_;
    ros::Publisher pub_down_borderline_individual_;
    ros::Publisher pub_up_borderline_individual_extended_;
    ros::Publisher pub_down_borderline_individual_extended_;
    ros::Publisher pub_borderline_with_static_vehicles_footprints_;

    ros::Publisher pub_stripe_borderline_;
    ros::Publisher pub_footprints_;   // 两侧的车贴边走的整个过程中的footprints
    ros::Subscriber sub_borderline_;  // 处理borderline 
    ros::Subscriber sub_static_vehs_; // 处理static vehicles

    bool is_borderline_initialized_;  // 仅处理一次borderline；等borderline的投影和static vehicle的投影都被处理完之后，才会去publish
    bool is_static_veh_projected_;    // 仅处理一次static vehicle；等borderline的投影和static vehicle的投影都被处理完之后，才会去publish
    float step_size_, theta_step_, draw_step_;  // 离散borderline时的步长、离散角度的步长、可视化的高度增量步长
    double xMin_, xMax_, yMin_, yMax_;  // borderline的边界
    std::string frame_id_;            // publish时的frame_id，默认为odom
    double road_width_;               // yMax_ - yMin_

    double y_negative_vehwidth_;
    double y_negative_min_turning_radius_;
    std::vector<std::pair<double, double>> y_negative_footprint_;
    double y_positive_vehwidth_;
    double y_positive_min_turning_radius_;
    std::vector<std::pair<double, double>> y_positive_footprint_;

    std::vector<double> index2x_list_;    // 数组的index 转化到 世界坐标系
    std::vector<double> up_borderline_;   // 道路上边界的borderline  y>0
    std::vector<double> down_borderline_; // 道路下边界的borderline  y<0
    std::vector<double> up_borderline_extended_right_;    // 按车footprint投影后的up_borderline
    std::vector<double> down_borderline_extended_right_;  // 按车footprint投影后的down_borderline
    std::vector<int> index_from_extended_dbr_to_dbr_;  // 输入down_borderline_extended_right_的index 返回该点在down_borderline的index
    std::vector<double> up_borderline_extended_left_;    // 按车footprint投影后的up_borderline
    std::vector<double> down_borderline_extended_left_;  // 按车footprint投影后的down_borderline
    
    std::vector<std::vector<double>> up_borderline_individual_;   // 不考虑车间连接的borderline  y>0
    std::vector<std::vector<double>> down_borderline_individual_; // 不考虑车间连接的borderline  y<0
    std::vector<std::vector<double>> up_borderline_individual_for_left_backup_;   // 不考虑车间连接的borderline  y>0 仅用于left backup
    std::vector<std::pair<double,double>> veh_x_borders_for_down_borderline_individual_;  // down_borderline_individual_对应的veh的footprint的最左侧和最右侧的x坐标
    std::vector<std::pair<double,double>> veh_x_borders_for_up_borderline_individual_;    // up_borderline_individual_对应的veh的footprint的最左侧和最右侧的x坐标
    std::vector<std::pair<double,double>> veh_x_switching_for_down_borderline_individual_;  // down_borderline_individual_对应的从normal转到换back和cut的switch点
    std::vector<double> veh_x_switching_for_up_borderline_individual_;  // up_borderline_individual_对应的从normal转到换back的switch点
    std::vector<double> up_borderline_with_static_vehicles_footprints_;  // 仅投影static vehicle的footprint到up borderline
    std::vector<double> down_borderline_with_static_vehicles_footprints_;  // 仅投影static vehicle的footprint到down borderline
    std::vector<std::vector<double>> up_borderline_individual_extended_;   // 不考虑车间连接的borderline_extend  y>0
    std::vector<std::vector<double>> down_borderline_individual_extended_; // 不考虑车间连接的borderline_extend  y<0

    std::vector<visualization_msgs::Marker> veh_ordered_;
    std::vector<visualization_msgs::Marker> veh_ordered_down_;
    std::vector<visualization_msgs::Marker> veh_ordered_up_;
    
    std::vector<std::vector<double>> stripe_borderline_;    // 整个投影过程中的所有离散点
    std::vector<std::vector<geometry_msgs::Point>> intermediate_footprints_;  // 两侧的车贴边走的整个过程中的footprints
};

#endif