#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Core>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>

#include "degeneracy_engine.h"


class VelodyneExperiment
{
public:
    VelodyneExperiment()
        : nh_(),
          pnh_("~"),
          engine_()
    {
        // =========================
        // ROS 参数
        // =========================

        pnh_.param<std::string>(
            "input_topic",
            input_topic_,
            "/velodyne_points");

        pnh_.param<double>(
            "z_min",
            z_min_,
            -0.15);

        pnh_.param<double>(
            "z_max",
            z_max_,
            0.15);

        pnh_.param<double>(
            "min_range",
            min_range_,
            0.3);

        pnh_.param<double>(
            "max_range",
            max_range_,
            30.0);

        pnh_.param<int>(
            "minimum_point_count",
            minimum_point_count_,
            100);

        pnh_.param<int>(
            "print_every",
            print_every_,
            10);

        pnh_.param<std::string>(
            "output_csv",
            output_csv_,
            "/home/sj/velodyne_experiment.csv");


        // =========================
        // CSV
        // =========================

        csv_.open(output_csv_.c_str());

        if (!csv_.is_open())
        {
            ROS_ERROR_STREAM(
                "Cannot open CSV file: "
                << output_csv_);

            ros::shutdown();
            return;
        }

        csv_ << std::setprecision(12);

        csv_
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
            << "\n";


        // =========================
        // Subscriber
        // =========================

        sub_ = nh_.subscribe(
            input_topic_,
            10,
            &VelodyneExperiment::cloudCallback,
            this);

        ROS_INFO_STREAM(
            "Velodyne experiment node started.");

        ROS_INFO_STREAM(
            "Input topic: "
            << input_topic_);

        ROS_INFO_STREAM(
            "Z range: ["
            << z_min_
            << ", "
            << z_max_
            << "]");

        ROS_INFO_STREAM(
            "XY range: ["
            << min_range_
            << ", "
            << max_range_
            << "]");

        ROS_INFO_STREAM(
            "Minimum point count: "
            << minimum_point_count_);

        ROS_INFO_STREAM(
            "Output CSV: "
            << output_csv_);
    }


    ~VelodyneExperiment()
    {
        if (csv_.is_open())
        {
            csv_.close();
        }
    }


private:

    void cloudCallback(
        const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        std::vector<Eigen::Vector2d> points;

        points.reserve(cloud->width * cloud->height);


        // ============================================
        // PointCloud2 迭代器
        //
        // M2UD 数据已经确认存在：
        // x / y / z / intensity / ring / time
        // ============================================

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(
            *cloud,
            "x");

        sensor_msgs::PointCloud2ConstIterator<float> iter_y(
            *cloud,
            "y");

        sensor_msgs::PointCloud2ConstIterator<float> iter_z(
            *cloud,
            "z");


        const size_t total_points =
            static_cast<size_t>(cloud->width) *
            static_cast<size_t>(cloud->height);


        for (size_t i = 0;
             i < total_points;
             ++i,
             ++iter_x,
             ++iter_y,
             ++iter_z)
        {
            const double x = static_cast<double>(*iter_x);
            const double y = static_cast<double>(*iter_y);
            const double z = static_cast<double>(*iter_z);


            // -----------------------------
            // 1. 排除 NaN / Inf
            // -----------------------------

            if (!std::isfinite(x) ||
                !std::isfinite(y) ||
                !std::isfinite(z))
            {
                continue;
            }


            // -----------------------------
            // 2. 高度筛选
            //
            // 当前只保留接近水平面的点
            // -----------------------------

            if (z < z_min_ ||
                z > z_max_)
            {
                continue;
            }


            // -----------------------------
            // 3. XY 平面距离筛选
            // -----------------------------

            const double range =
                std::sqrt(x * x + y * y);

            if (range < min_range_ ||
                range > max_range_)
            {
                continue;
            }


            // -----------------------------
            // 4. 三维点 → 二维点
            // -----------------------------

            points.emplace_back(x, y);
        }


        // ============================================
        // 点数检查
        // ============================================

        if (static_cast<int>(points.size()) <
            minimum_point_count_)
        {
            ROS_WARN_THROTTLE(
                2.0,
                "Too few valid 2D points: %zu",
                points.size());

            return;
        }


        // ============================================
        // 调用现有 DegeneracyEngine
        // ============================================

        ScanFrame merged_frame;

        CovarianceResult covariance_result;

        DegeneracyResult degeneracy_result;

        AdaptiveWeightResult weight_result;


        const bool success =
            engine_.update(
                points,
                cloud->header.stamp,
                merged_frame,
                covariance_result,
                degeneracy_result,
                weight_result);


        if (!success)
        {
            ROS_WARN_THROTTLE(
                2.0,
                "DegeneracyEngine update failed.");

            return;
        }


        // ============================================
        // CSV
        // ============================================

        ++frame_count_;


        csv_
            << frame_count_
            << ","
            << cloud->header.stamp.toSec()
            << ","
            << points.size()
            << ","
            << covariance_result.lambda_max
            << ","
            << covariance_result.lambda_min
            << ","
            << degeneracy_result.score
            << ","
            << weight_result.translation_weight
            << ","
            << weight_result.rotation_weight
            << ","
            << degeneracy_result.direction.x()
            << ","
            << degeneracy_result.direction.y()
            << ","
            << (degeneracy_result.degenerate ? 1 : 0)
            << "\n";


        csv_.flush();


        // ============================================
        // ROS 日志
        // ============================================

        if (frame_count_ % print_every_ == 0)
        {
            ROS_INFO_STREAM(
                "Frame "
                << frame_count_
                << " | points="
                << points.size()
                << " | lambda_max="
                << covariance_result.lambda_max
                << " | lambda_min="
                << covariance_result.lambda_min
                << " | score="
                << degeneracy_result.score
                << " | degenerate="
                << (degeneracy_result.degenerate
                        ? "YES"
                        : "NO"));
        }
    }


private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber sub_;

    DegeneracyEngine engine_;

    std::ofstream csv_;

    std::string input_topic_;
    std::string output_csv_;

    double z_min_;
    double z_max_;

    double min_range_;
    double max_range_;

    int minimum_point_count_;
    int print_every_;

    size_t frame_count_ = 0;
};


int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "velodyne_experiment");

    VelodyneExperiment node;

    ros::spin();

    return 0;
}
