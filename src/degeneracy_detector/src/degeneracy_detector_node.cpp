#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include "degeneracy_detector/Degeneracy.h"

#include "point_processor.h"
#include "degeneracy_engine.h"
#include "evaluation.h"

class DegeneracyDetector
{
public:

    DegeneracyDetector()
    {
        scan_sub_ =
                nh_.subscribe(
                        "/scan",
                        1,
                        &DegeneracyDetector::scanCallback,
                        this);

        degeneracy_pub_ =
                nh_.advertise<
                    degeneracy_detector::Degeneracy>(
                        "/degeneracy",
                        1);

        /*
         * 当前实验数据保存位置。
         */
        if(!evaluation_.open(
                "/home/sj/degeneracy.csv"))
        {
            ROS_ERROR(
                "Failed to open "
                "/home/sj/degeneracy.csv");
        }

        ROS_INFO(
            "Degeneracy Detector Started.");
    }

private:

    int frame_id_ = 0;

    ros::NodeHandle nh_;

    ros::Subscriber scan_sub_;

    ros::Publisher degeneracy_pub_;

    PointProcessor point_processor_;

    DegeneracyEngine engine_;

    Evaluation evaluation_;

    void scanCallback(
            const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        frame_id_++;

        //----------------------------------
        // Step 1
        // LaserScan -> ScanFrame
        //----------------------------------

        ScanFrame current_frame;

        if(!point_processor_.convertLaserScan(
                *scan,
                current_frame))
        {
            return;
        }

        //----------------------------------
        // Step 2
        // 提取二维点
        //----------------------------------

        const std::vector<Eigen::Vector2d>&
                points =
                current_frame.points;

        //----------------------------------
        // Step 3
        // Engine
        //----------------------------------

        ScanFrame merged_frame;

        CovarianceResult covariance_result;

        DegeneracyResult degeneracy_result;

        WeightResult weight_result;

        if(!engine_.update(
                points,
                current_frame.stamp,
                merged_frame,
                covariance_result,
                degeneracy_result,
                weight_result))
        {
            return;
        }

        //----------------------------------
        // Step 4
        // 发布ROS退化消息
        //----------------------------------

        degeneracy_detector::Degeneracy msg;

        msg.header =
                scan->header;

        msg.degenerate =
                degeneracy_result.degenerate;

        msg.score =
                degeneracy_result.score;

        msg.lambda_max =
                degeneracy_result.lambda_max;

        msg.lambda_min =
                degeneracy_result.lambda_min;

        msg.direction.x =
                degeneracy_result.direction.x();

        msg.direction.y =
                degeneracy_result.direction.y();

        msg.direction.z = 0.0;

        degeneracy_pub_.publish(msg);

        //----------------------------------
        // Step 5
        // 保存实验数据
        //----------------------------------

        evaluation_.save(
                frame_id_,
                merged_frame.points.size(),
                merged_frame,
                degeneracy_result,
                covariance_result,
                weight_result);

        //----------------------------------
        // Step 6
        // 调试输出
        //----------------------------------

        if(frame_id_ % 30 == 0)
        {
            ROS_INFO_STREAM(
                "==============================");

            ROS_INFO_STREAM(
                "Frame : "
                << frame_id_);

            ROS_INFO_STREAM(
                "Current Point : "
                << points.size());

            ROS_INFO_STREAM(
                "Merged Point : "
                << merged_frame.points.size());

            ROS_INFO_STREAM(
                "Window Frame : "
                << engine_.frameCount());

            ROS_INFO_STREAM(
                "Score : "
                << degeneracy_result.score);

            ROS_INFO_STREAM(
                "Lambda Max : "
                << degeneracy_result.lambda_max);

            ROS_INFO_STREAM(
                "Lambda Min : "
                << degeneracy_result.lambda_min);

            ROS_INFO_STREAM(
                "Degenerate : "
                << degeneracy_result.degenerate);

            ROS_INFO_STREAM(
                "Direction : "
                << degeneracy_result.direction.transpose());

            ROS_INFO_STREAM(
                "Translation Weight : "
                << weight_result.translation_weight);

            ROS_INFO_STREAM(
                "Rotation Weight : "
                << weight_result.rotation_weight);
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
        "degeneracy_detector");

    DegeneracyDetector detector;

    ros::spin();

    return 0;
}
