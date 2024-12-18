#ifndef TOTF_INFO_H_
#define TOTF_INFO_H_
#include <teb_local_planner/borderConstraintsMsg.h>

class TotgProcessor{
public:
    TotgProcessor(){is_initialized_=false;};
    TotgProcessor(teb_local_planner::borderConstraintsMsg msg);
    int getIndexValue(float x);
    float getUpValue(float x);
    float getDownValue(float x);
    float getCenterLine(float x);
    bool is_initialized_;

private:
    std::vector<float> up_borderline_;
    std::vector<float> down_borderline_;
    std::vector<float> center_line_;
    float x_min_, x_max_, step_size_;
};

#endif