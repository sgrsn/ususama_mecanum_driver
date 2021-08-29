#ifndef WAYPOINT_GENERATOR_HPP
#define WAYPOINT_GENERATOR_HPP

#include <list>

template <class PoseClass>
class WaypointGenerator
{
    public:
    WaypointGenerator(PoseClass start, PoseClass goal, double duration_time)
    {    
        start_ = start;
        goal_ = goal;
        duration_time_ = duration_time;
    }
    
    PoseClass constraint(PoseClass start, PoseClass goal, PoseClass way)
    {
        way.ConstraintUpper(goal, goal - start);
        way.ConstraintLower(start, goal - start);
        return way;
    }
    
    PoseClass GetPose(float t)
    {
        PoseClass way = start_ + (goal_ - start_) * (t/duration_time_);
        way = constraint(start_, goal_, way);
        return way;
    }

    double get_duration_time()
    {
        return duration_time_;
    }
    
    private:
    int way_n_;
    PoseClass start_;
    PoseClass goal_;
    double duration_time_;
};

#endif

