#ifndef DEGENERACY_CONFIG_H
#define DEGENERACY_CONFIG_H

/**
 * @brief 退化检测模块统一配置参数
 *
 * 所有算法相关参数统一由该结构体管理。
 *
 * 这样做的目的：
 * 1. 避免参数散落在不同cpp中；
 * 2. 方便论文实验进行参数消融；
 * 3. 方便后续Cartographer集成；
 * 4. 保持算法核心与ROS解耦。
 */
struct DegeneracyConfig
{
    /**
     * @brief 滑动窗口大小
     *
     * 当前系统默认使用20帧。
     */
    int window_size = 20;

    /**
     * @brief 退化判定阈值
     *
     * score = lambda_max / lambda_min
     *
     * 当：
     *
     * score > degeneracy_threshold
     *
     * 时认为当前环境存在退化。
     */
    double degeneracy_threshold = 100.0;

    /**
     * @brief 最小特征值保护阈值
     *
     * 防止：
     *
     * lambda_min -> 0
     *
     * 时产生除零或者数值爆炸。
     */
    double minimum_eigenvalue = 1e-8;

    /**
     * @brief 最大平移权重
     *
     * 非退化情况下对应的基础平移权重。
     */
    double translation_max = 10.0;

    /**
     * @brief 旋转权重
     */
    double rotation_weight = 40.0;

    /**
     * @brief 自适应权重增益
     *
     * 当前采用：
     *
     * translation_weight =
     * translation_max /
     * (1 + gain * score)
     */
    double weight_gain = 0.03;

    /**
     * @brief LaserScan最小有效距离
     */
    double min_range = 0.10;

    /**
     * @brief LaserScan最大有效距离
     */
    double max_range = 30.0;

    /**
     * @brief 最小有效点数
     */
    int minimum_point_count = 10;
};

#endif
