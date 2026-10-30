#include <gtest/gtest.h>

#include <cmath>
#include <vector>
#include <limits>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "data_type.h"
#include "covariance.h"
#include "degeneracy_analyzer.h"
#include "degeneracy_config.h"
#include "adaptive_weight.h"
#include "sliding_window.h"
#include "degeneracy_engine.h"
#include "point_processor.h"


// ============================================================
// 工具函数
// ============================================================

/**
 * @brief 生成二维矩形点云
 *
 * 用于测试非退化环境。
 */
static std::vector<Eigen::Vector2d>
CreateNormalPointCloud()
{
    std::vector<Eigen::Vector2d> points;

    // 一个近似均匀分布的矩形
    for(int x = -5; x <= 5; ++x)
    {
        for(int y = -5; y <= 5; ++y)
        {
            points.emplace_back(
                static_cast<double>(x),
                static_cast<double>(y));
        }
    }

    return points;
}


/**
 * @brief 生成细长点云
 *
 * x方向长度很大，
 * y方向变化很小。
 *
 * 因此：
 *
 * lambda_max >> lambda_min
 *
 * 应该得到明显较大的 score。
 */
static std::vector<Eigen::Vector2d>
CreateDegeneratePointCloud()
{
    std::vector<Eigen::Vector2d> points;

    for(int x = -50; x <= 50; ++x)
    {
        // 两侧增加少量厚度，
        // 避免 lambda_min 完全等于0。
        points.emplace_back(
            static_cast<double>(x) * 0.1,
            0.01);

        points.emplace_back(
            static_cast<double>(x) * 0.1,
            -0.01);
    }

    return points;
}


/**
 * @brief 生成用于测试 LaserScan 的消息
 */
static sensor_msgs::LaserScan
CreateLaserScan()
{
    sensor_msgs::LaserScan scan;

    scan.header.stamp = ros::Time(1.0);
    scan.header.frame_id = "laser";

    scan.angle_min = -M_PI / 2.0;
    scan.angle_max = M_PI / 2.0;

    scan.ranges.resize(21);

    scan.angle_increment =
        (scan.angle_max - scan.angle_min)
        / static_cast<double>(scan.ranges.size() - 1);

    scan.range_min = 0.1;
    scan.range_max = 30.0;

    for(size_t i = 0; i < scan.ranges.size(); ++i)
    {
        scan.ranges[i] = 5.0;
    }

    return scan;
}


// ============================================================
// 1. CovarianceCalculator 测试
// ============================================================

TEST(CovarianceTest, NormalPointCloud)
{
    CovarianceCalculator calculator;

    ScanFrame frame;

    frame.points = CreateNormalPointCloud();

    CovarianceResult result;

    ASSERT_TRUE(
        calculator.compute(frame, result));

    EXPECT_GT(
        result.eigen_values(0),
        0.0);

    EXPECT_GT(
        result.eigen_values(1),
        0.0);

    // 矩形点云在x/y方向分布接近一致，
    // 两个特征值应该比较接近。
    double ratio =
        result.eigen_values(1)
        /
        result.eigen_values(0);

    EXPECT_LT(
        ratio,
        1.2);
}


TEST(CovarianceTest, DegeneratePointCloud)
{
    CovarianceCalculator calculator;

    ScanFrame frame;

    frame.points =
        CreateDegeneratePointCloud();

    CovarianceResult result;

    ASSERT_TRUE(
        calculator.compute(frame, result));

    double lambda_min =
        result.eigen_values(0);

    double lambda_max =
        result.eigen_values(1);

    EXPECT_GT(
        lambda_max,
        lambda_min);

    double ratio =
        lambda_max /
        lambda_min;

    // 细长点云应该产生明显的特征值差异。
    EXPECT_GT(
        ratio,
        100.0);
}


TEST(CovarianceTest, TooFewPoints)
{
    CovarianceCalculator calculator;

    ScanFrame frame;

    frame.points.push_back(
        Eigen::Vector2d(0.0, 0.0));

    CovarianceResult result;

    EXPECT_FALSE(
        calculator.compute(frame, result));
}


// ============================================================
// 2. DegeneracyAnalyzer 测试
// ============================================================

TEST(DegeneracyAnalyzerTest, NormalEnvironment)
{
    DegeneracyConfig config;

    config.degeneracy_threshold = 100.0;

    DegeneracyAnalyzer analyzer(config);

    ScanFrame frame;

    frame.points =
        CreateNormalPointCloud();

    CovarianceCalculator calculator;

    CovarianceResult covariance;

    ASSERT_TRUE(
        calculator.compute(
            frame,
            covariance));

    DegeneracyResult result;

    ASSERT_TRUE(
        analyzer.analyze(
            covariance,
            result));

    EXPECT_LT(
        result.score,
        100.0);

    EXPECT_FALSE(
        result.degenerate);
}


TEST(DegeneracyAnalyzerTest, DegenerateEnvironment)
{
    DegeneracyConfig config;

    config.degeneracy_threshold = 100.0;

    DegeneracyAnalyzer analyzer(config);

    ScanFrame frame;

    frame.points =
        CreateDegeneratePointCloud();

    CovarianceCalculator calculator;

    CovarianceResult covariance;

    ASSERT_TRUE(
        calculator.compute(
            frame,
            covariance));

    DegeneracyResult result;

    ASSERT_TRUE(
        analyzer.analyze(
            covariance,
            result));

    EXPECT_GT(
        result.score,
        100.0);

    EXPECT_TRUE(
        result.degenerate);
}


TEST(DegeneracyAnalyzerTest, Threshold)
{
    DegeneracyConfig config;

    config.degeneracy_threshold = 10.0;

    DegeneracyAnalyzer analyzer(config);

    EXPECT_DOUBLE_EQ(
        analyzer.threshold(),
        10.0);

    analyzer.setThreshold(50.0);

    EXPECT_DOUBLE_EQ(
        analyzer.threshold(),
        50.0);
}


// ============================================================
// 3. SlidingWindow 测试
// ============================================================

TEST(SlidingWindowTest, WindowSize)
{
    SlidingWindow window(3);

    EXPECT_EQ(
        window.windowSize(),
        3u);

    EXPECT_EQ(
        window.frameCount(),
        0u);
}


TEST(SlidingWindowTest, AddFrames)
{
    SlidingWindow window(3);

    for(int i = 0; i < 3; ++i)
    {
        ScanFrame frame;

        frame.stamp =
            ros::Time(i);

        frame.points.push_back(
            Eigen::Vector2d(
                static_cast<double>(i),
                0.0));

        window.addFrame(frame);
    }

    EXPECT_EQ(
        window.frameCount(),
        3u);

    EXPECT_EQ(
        window.mergedFrame().points.size(),
        3u);
}


TEST(SlidingWindowTest, WindowSlides)
{
    SlidingWindow window(3);

    for(int i = 0; i < 5; ++i)
    {
        ScanFrame frame;

        frame.stamp =
            ros::Time(i);

        frame.points.push_back(
            Eigen::Vector2d(
                static_cast<double>(i),
                0.0));

        window.addFrame(frame);
    }

    // 窗口最大只能保存3帧
    EXPECT_EQ(
        window.frameCount(),
        3u);

    // 每帧1个点，因此融合后3个点
    EXPECT_EQ(
        window.mergedFrame().points.size(),
        3u);

    // 最新帧时间戳应该是4
    EXPECT_EQ(
        window.mergedFrame().stamp,
        ros::Time(4));
}


// ============================================================
// 4. AdaptiveWeightCalculator 测试
// ============================================================

TEST(AdaptiveWeightTest, ZeroScore)
{
    DegeneracyConfig config;

    config.translation_max = 10.0;
    config.rotation_weight = 40.0;
    config.weight_gain = 0.03;

    AdaptiveWeightCalculator calculator(config);

    WeightResult result =
        calculator.calculate(0.0);

    EXPECT_DOUBLE_EQ(
        result.translation_weight,
        10.0);

    EXPECT_DOUBLE_EQ(
        result.rotation_weight,
        40.0);
}


TEST(AdaptiveWeightTest, WeightDecreasesWithScore)
{
    DegeneracyConfig config;

    config.translation_max = 10.0;
    config.rotation_weight = 40.0;
    config.weight_gain = 0.03;

    AdaptiveWeightCalculator calculator(config);

    WeightResult normal =
        calculator.calculate(1.0);

    WeightResult degenerate =
        calculator.calculate(100.0);

    EXPECT_GT(
        normal.translation_weight,
        degenerate.translation_weight);

    EXPECT_DOUBLE_EQ(
        normal.rotation_weight,
        40.0);

    EXPECT_DOUBLE_EQ(
        degenerate.rotation_weight,
        40.0);
}


TEST(AdaptiveWeightTest, Formula)
{
    DegeneracyConfig config;

    config.translation_max = 10.0;
    config.weight_gain = 0.03;

    AdaptiveWeightCalculator calculator(config);

    double score = 10.0;

    WeightResult result =
        calculator.calculate(score);

    double expected =
        10.0 /
        (1.0 + 0.03 * score);

    EXPECT_NEAR(
        result.translation_weight,
        expected,
        1e-10);
}


// ============================================================
// 5. PointProcessor 测试
// ============================================================

TEST(PointProcessorTest, ConvertLaserScan)
{
    PointProcessor processor;

    ScanFrame frame;

    sensor_msgs::LaserScan scan =
        CreateLaserScan();

    ASSERT_TRUE(
        processor.convertLaserScan(
            scan,
            frame));

    EXPECT_GT(
        frame.points.size(),
        10u);

    EXPECT_EQ(
        frame.stamp,
        scan.header.stamp);
}


TEST(PointProcessorTest, RemoveInvalidRanges)
{
    PointProcessor processor;

    ScanFrame frame;

    sensor_msgs::LaserScan scan =
        CreateLaserScan();

    // 人为加入无效数据
    scan.ranges[0] =
        std::numeric_limits<float>::quiet_NaN();

    scan.ranges[1] =
        std::numeric_limits<float>::infinity();

    scan.ranges[2] =
        0.01;

    ASSERT_TRUE(
        processor.convertLaserScan(
            scan,
            frame));

    // 剩余有效点仍然应该足够
    EXPECT_GT(
        frame.points.size(),
        10u);
}


// ============================================================
// 6. DegeneracyEngine 整体测试
// ============================================================

TEST(DegeneracyEngineTest, NormalEnvironment)
{
    DegeneracyConfig config;

    config.window_size = 20;

    config.degeneracy_threshold = 100.0;

    config.minimum_point_count = 10;

    DegeneracyEngine engine(config);

    std::vector<Eigen::Vector2d>
        points =
            CreateNormalPointCloud();

    ScanFrame merged_frame;

    CovarianceResult covariance;

    DegeneracyResult degeneracy;

    WeightResult weight;

    ASSERT_TRUE(
        engine.update(
            points,
            ros::Time(1.0),
            merged_frame,
            covariance,
            degeneracy,
            weight));

    EXPECT_FALSE(
        degeneracy.degenerate);

    EXPECT_LT(
        degeneracy.score,
        100.0);

    EXPECT_EQ(
        engine.frameCount(),
        1u);
}


TEST(DegeneracyEngineTest, DegenerateEnvironment)
{
    DegeneracyConfig config;

    config.window_size = 20;

    config.degeneracy_threshold = 100.0;

    config.minimum_point_count = 10;

    DegeneracyEngine engine(config);

    std::vector<Eigen::Vector2d>
        points =
            CreateDegeneratePointCloud();

    ScanFrame merged_frame;

    CovarianceResult covariance;

    DegeneracyResult degeneracy;

    WeightResult weight;

    ASSERT_TRUE(
        engine.update(
            points,
            ros::Time(1.0),
            merged_frame,
            covariance,
            degeneracy,
            weight));

    EXPECT_TRUE(
        degeneracy.degenerate);

    EXPECT_GT(
        degeneracy.score,
        100.0);

    EXPECT_LT(
        weight.translation_weight,
        config.translation_max);
}


// ============================================================
// 7. Eigenvalue / direction 测试
// ============================================================

TEST(DegeneracyAnalyzerTest, PrincipalDirection)
{
    DegeneracyConfig config;

    config.degeneracy_threshold = 10.0;

    DegeneracyAnalyzer analyzer(config);

    ScanFrame frame;

    // 构造一个沿X方向延伸的点云
    for(int i = -50; i <= 50; ++i)
    {
        frame.points.emplace_back(
            static_cast<double>(i),
            0.05);

        frame.points.emplace_back(
            static_cast<double>(i),
            -0.05);
    }

    CovarianceCalculator calculator;

    CovarianceResult covariance;

    ASSERT_TRUE(
        calculator.compute(
            frame,
            covariance));

    DegeneracyResult result;

    ASSERT_TRUE(
        analyzer.analyze(
            covariance,
            result));

    EXPECT_TRUE(
        result.degenerate);

    // 主方向应该接近X轴
    EXPECT_GT(
        std::abs(result.direction.x()),
        0.9);

    EXPECT_LT(
        std::abs(result.direction.y()),
        0.2);
}


// ============================================================
// main
// ============================================================

int main(
        int argc,
        char** argv)
{
    ros::init(
        argc,
        argv,
        "degeneracy_algorithm_test");

    testing::InitGoogleTest(
        &argc,
        argv);

    return RUN_ALL_TESTS();
}
