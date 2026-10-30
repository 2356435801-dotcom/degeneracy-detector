#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

#include "point_processor.h"
#include "degeneracy_engine.h"
#include "degeneracy_config.h"


class ScanExperiment
{
public:

    ScanExperiment()
    {
        output_.open("/home/sj/scan_experiment.csv");

        if (!output_.is_open())
        {
            ROS_ERROR("Cannot open output CSV.");
            return;
        }
	
	output_ << std::fixed << std::setprecision(9);
	
        output_
            << "frame,"
            << "time,"
            << "point_num,"
            << "lambda_max,"
            << "lambda_min,"
            << "score,"
            << "translation_weight,"
            << "rotation_weight,"
            << "direction_x,"
            << "direction_y,"
            << "degenerate"
            << std::endl;

        sub_ = nh_.subscribe(
            "/scan",
            10,
            &ScanExperiment::scanCallback,
            this);

        ROS_INFO(
            "Scan experiment started.");
    }

    ~ScanExperiment()
    {
        if (output_.is_open())
            output_.close();
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber sub_;

    PointProcessor point_processor_;

    DegeneracyEngine engine_;

    std::ofstream output_;

    int frame_id_ = 0;


    void scanCallback(
        const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        frame_id_++;

        ScanFrame current_frame;

        if (!point_processor_.convertLaserScan(
                *scan,
                current_frame))
        {
            return;
        }

        ScanFrame merged_frame;

        CovarianceResult covariance_result;

        DegeneracyResult degeneracy_result;

        WeightResult weight_result;

        bool ok =
            engine_.update(
                current_frame.points,
                current_frame.stamp,
                merged_frame,
                covariance_result,
                degeneracy_result,
                weight_result);

        if (!ok)
            return;


        output_
            << frame_id_ << ","
            << scan->header.stamp.toSec() << ","
            << merged_frame.points.size() << ","
            << covariance_result.eigen_values(1) << ","
            << covariance_result.eigen_values(0) << ","
            << degeneracy_result.score << ","
            << weight_result.translation_weight << ","
            << weight_result.rotation_weight << ","
            << degeneracy_result.direction.x() << ","
            << degeneracy_result.direction.y() << ","
            << (degeneracy_result.degenerate ? 1 : 0)
            << std::endl;


        //if (frame_id_ % 30 == 0)
        //{
            //std::cout
                //<< "Frame = "
                //<< frame_id_
                //<< "  Score = "
                //<< degeneracy_result.score
              //  << "  Degenerate = "
            //    << degeneracy_result.degenerate
          //      << std::endl;
        //}
        if (frame_id_ % 30 == 0)
{
    std::cout
        << std::fixed
        << std::setprecision(4)
        << "Frame = "
        << frame_id_
        << "  Time = "
        << scan->header.stamp.toSec()
        << "  Points = "
        << merged_frame.points.size()
        << "  LambdaMax = "
        << covariance_result.eigen_values(1)
        << "  LambdaMin = "
        << covariance_result.eigen_values(0)
        << "  Score = "
        << degeneracy_result.score
        << "  Degenerate = "
        << degeneracy_result.degenerate
        << std::endl;
}
    }
};


int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "scan_experiment");

    ScanExperiment experiment;

    ros::spin();

    return 0;
}
