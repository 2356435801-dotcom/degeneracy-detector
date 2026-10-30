#include "sliding_window.h"

SlidingWindow::SlidingWindow(size_t window_size)
    : window_size_(window_size)
{
}

void SlidingWindow::addFrame(const ScanFrame& frame)
{
    window_.push_back(frame);

    while(window_.size() > window_size_)
    {
        window_.pop_front();
    }

    rebuildMergedFrame();
}

void SlidingWindow::rebuildMergedFrame()
{
    merged_frame_.points.clear();

    if(window_.empty())
        return;

    // 使用最新帧时间戳
    merged_frame_.stamp = window_.back().stamp;

    // 统计总点数，提前分配内存
    size_t total_points = 0;

    for(const auto& frame : window_)
    {
        total_points += frame.points.size();
    }

    merged_frame_.points.reserve(total_points);

    // 合并点云
    for(const auto& frame : window_)
    {
        merged_frame_.points.insert(
                merged_frame_.points.end(),
                frame.points.begin(),
                frame.points.end());
    }
}

const ScanFrame& SlidingWindow::mergedFrame() const
{
    return merged_frame_;
}

size_t SlidingWindow::frameCount() const
{
    return window_.size();
}

size_t SlidingWindow::windowSize() const
{
    return window_size_;
}
