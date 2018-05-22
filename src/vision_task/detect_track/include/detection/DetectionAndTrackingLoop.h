//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_DETECTIONANDTRACKINGLOOP_H
#define UAV_WS_DETECTIONANDTRACKINGLOOP_H
class DetectionAndTrackingLoop
{
public:
    DetectionAndTrackingLoop(DetectionBase *detector, cv::Tracker * tracker);
    ~DetectionAndTrackingLoop();
    cv::Rect2f detectFrame(cv::Mat &frame);
private:
    enum State{wait, detection, tracking};
    State state;
    cv::Rect2f roi;
    cv::Mat frame;
    DetectionBase *detector;
    cv::Tracker *tracker;
};
#endif //UAV_WS_DETECTIONANDTRACKINGLOOP_H
