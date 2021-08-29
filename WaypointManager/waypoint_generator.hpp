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

/*#ifndef WAYPOINT_GENERATOR_HPP
#define WAYPOINT_GENERATOR_HPP

#include <list>

template <class PoseClass>
class WaypointGenerator
{
    public:
    WaypointGenerator(PoseClass start, PoseClass goal, double duration_time)
    {    
        //start_ = start;
        //goal_ = goal;
        start.time_stamp = 0;
        goal.time_stamp = duration_time;
        waypoint_list.push_back(start);
        waypoint_list.push_back(goal);
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
        PoseClass start = waypoint_list[0];
        PoseClass goal;

        for(PoseClass way : waypoint_list)
        {
            if(t < way.time_stamp)
            {
                goal = way;
                break;
            }
            else 
            {
                start = way;
            }
        }
        PoseClass way = start + (goal-start) * ((t-start.time_stamp)/(goal.time_stamp - start.time_stamp));
        way = constraint(start, goal, way);
        return way;
    }

    double get_duration_time()
    {
        return duration_time_;
    }

    void operator +=(WaypointGenerator<PoseClass> new_waypoint)
    {
        for(int i=0;i < new_waypoint.waypoint_list.size(); i++)
        {
            new_waypoint.waypoint_list[i].time_stamp += duration_time_;
        }
        duration_time_ += new_waypoint.duration_time_;
        waypoint_list.insert(waypoint_list.end(), new_waypoint.waypoint_list.begin(), new_waypoint.waypoint_list.end());
    }
    
    //private:
    int way_n_;
    std::vector<PoseClass> waypoint_list;
    double duration_time_;
};


#endif

*/