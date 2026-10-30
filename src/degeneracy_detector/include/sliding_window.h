#ifndef SLIDING_WINDOW_H
#define SLIDING_WINDOW_H

#include <deque>

#include "data_type.h"

/**
 * @brief 固定长度滑动窗口
 *
 * 功能：
 * 1. 保存最近N帧ScanFrame
 * 2. 自动维护融合点云
 * 3. 零拷贝获取MergedFrame
 */
class SlidingWindow
{
public:

    explicit SlidingWindow(size_t window_size = 20);

    /**
     * @brief 添加一帧数据
     */
    void addFrame(const ScanFrame& frame);

    /**
     * @brief 获取融合后的窗口点云
     */
    const ScanFrame& mergedFrame() const;

    /**
     * @brief 当前窗口帧数
     */
    size_t frameCount() const;

    /**
     * @brief 窗口大小
     */
    size_t windowSize() const;

private:

    size_t window_size_;

    std::deque<ScanFrame> window_;

    /// 融合后的窗口点云
    ScanFrame merged_frame_;

    /// 重建融合点云
    void rebuildMergedFrame();
};

#endif
