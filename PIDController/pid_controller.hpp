#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <vector>

class PIDControl
{
    public:
    struct Bound
    {
        double upper;
        double lower;
    };
    struct Parameter
    {
        double kp;
        double ki;
        double kd;
        Bound bound;
        Bound threshold;
    };
  
    PIDControl(const Parameter& param) : param_(param)
    {
        err_.resize(3,0);
        control_input_.resize(2,0);
    };
    ~PIDControl() = default;

    const double& calculate(const double& ref, const double& cur);

    const Parameter& parameter(void) const { return param_; };
    void parameter(const Parameter& param) { param_ = param; };
    
    double calculate(double ref, double cur, double time_step)
    {        
        err_[2] = ( (ref - cur)-err_[0] ) / time_step;
        err_[0] = ref - cur;
        
        // calculate control input
        control_input_[0] = param_.kp*err_[0] + param_.ki*err_[1] + param_.kd*err_[2];
        
        // limit of control input
        control_input_[0] = min(control_input_[0], param_.bound.upper);
        control_input_[0] = max(control_input_[0], param_.bound.lower);
        
        // update
        control_input_[1] = control_input_[0];
        err_[1] += err_[0] * time_step;
        
        // limit of i gain
        err_[1] = min(err_[1], param_.bound.upper);
        err_[1] = max(err_[1], param_.bound.lower);
        
        if( (ref - cur < param_.threshold.upper) && (ref - cur > param_.threshold.lower) )
            return 0;
        
        return control_input_[0];
    }
  
    private:
    Parameter param_;

    std::vector<double> err_;   //[current_error, error_integral, error_diff]
    std::vector<double> control_input_;
};

#endif