#pragma once

#include <iostream>

namespace holodeck {

  class BaseFeatureMatcher
  {
  public:
    // This method receives an image and it is expected that it returns
    // feature correspondences (i.e., matched features) across the previous
    // image and the currently passed in image.
    virtual void find_correspoinding_features(const cv::Mat& img, const cv::Mat& prev_image, std::vector<cv::Point2f>& prev_matched, std::vector<cv::Point2f>& matched_features, const cv::Mat& mask) = 0;

    // virtual void set_max_features(int max_points) = 0;

  protected:
    bool first_image_ = true;

  };

}