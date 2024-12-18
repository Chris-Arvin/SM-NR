#include<teb_local_planner/totg_info_processor.h>

TotgProcessor::TotgProcessor(teb_local_planner::borderConstraintsMsg msg){
    is_initialized_ = true;
    up_borderline_ = msg.upBorderline;
    down_borderline_ = msg.downBorderline;
    x_min_ = msg.xMin;
    x_max_ = msg.xMax;
    step_size_ = msg.stepSize;
    for (int i=0; i<up_borderline_.size(); i++)
        center_line_.push_back((up_borderline_[i]+down_borderline_[i])*0.5);
}

int TotgProcessor::getIndexValue(float x){
    return (x-x_min_)/step_size_;
}

float TotgProcessor::getUpValue(float x){
    up_borderline_[getIndexValue(x)];
}

float TotgProcessor::getDownValue(float x){
    down_borderline_[getIndexValue(x)];
}

float TotgProcessor::getCenterLine(float x){
    center_line_[getIndexValue(x)];
}