#include "degeneracy_engine.h"

DegeneracyEngine::
DegeneracyEngine()
    : config_(),
      sliding_window_(config_.window_size),
      analyzer_(config_),
      weight_calculator_(config_)
{
}

DegeneracyEngine::
DegeneracyEngine(
        const DegeneracyConfig& config)
    : config_(config),
      sliding_window_(config.window_size),
      analyzer_(config),
      weight_calculator_(config)
{
}

void DegeneracyEngine::
setConfig(
        const DegeneracyConfig& config)
{
    config_ = config;

    /*
     * 注意：
     * SlidingWindow的window_size在构造时确定。
     *
     * 当前阶段不支持运行过程中动态改变窗口大小。
     *
     * 如果后续需要动态调参，再增加reset/configure接口。
     */

    analyzer_.setConfig(config_);

    weight_calculator_.setConfig(config_);
}

const DegeneracyConfig&
DegeneracyEngine::
config() const
{
    return config_;
}

bool DegeneracyEngine::
update(
        const std::vector<Eigen::Vector2d>& points,
        const ros::Time& stamp,
        ScanFrame& merged_frame,
        CovarianceResult& covariance_result,
        DegeneracyResult& degeneracy_result,
        WeightResult& weight_result)
{
    //----------------------------------
    // Step 0
    // 检查当前帧点数
    //----------------------------------

    if(points.size() <
       static_cast<size_t>(
           config_.minimum_point_count))
    {
        return false;
    }

    //----------------------------------
    // Step 1
    // vector<Eigen::Vector2d>
    // ->
    // ScanFrame
    //----------------------------------

    current_frame_.points =
            points;

    current_frame_.stamp =
            stamp;

    //----------------------------------
    // Step 2
    // 添加到滑动窗口
    //----------------------------------

    sliding_window_.addFrame(
            current_frame_);

    //----------------------------------
    // Step 3
    // 获取窗口融合点云
    //----------------------------------

    merged_frame =
            sliding_window_.mergedFrame();

    //----------------------------------
    // Step 4
    // 计算协方差 + PCA
    //----------------------------------

    if(!covariance_calculator_.compute(
            merged_frame,
            covariance_result))
    {
        return false;
    }

    //----------------------------------
    // Step 5
    // 退化分析
    //----------------------------------

    if(!analyzer_.analyze(
            covariance_result,
            degeneracy_result))
    {
        return false;
    }

    //----------------------------------
    // Step 6
    // 自适应权重
    //----------------------------------

    weight_result =
            weight_calculator_.calculate(
                    degeneracy_result.score);

    return true;
}

size_t DegeneracyEngine::
frameCount() const
{
    return sliding_window_.frameCount();
}

size_t DegeneracyEngine::
windowSize() const
{
    return sliding_window_.windowSize();
}
