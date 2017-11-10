#ifndef PANARAMIC_STITCHING_H_
#define PANARAMIC_STITCHING_H_

#include <opencv2\core\core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\features2d.hpp>
#include <opencv2\calib3d.hpp>
#include <opencv2\nonfree\nonfree.hpp>
#include <Eigen/Dense>
#include <vector>
#include <utility>

namespace panSti {
/* Use 3 different ways to extract feature points and match */
enum featureMethod {
    SIFT,
    SURF,
    HARRIS,
};

/*
 * the function for eigen to do the optimization
 */
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor {
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const {
        return m_inputs;
    }
    int values() const {
        return m_values;
    }

    // you should define that in the subclass :
//  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};

struct geometricError : Functor<double> {
    // input the left point matrix and right point matrix for optimization
    geometricError(cv::Mat left_points, cv::Mat right_points);
    // use the existing homography and left point and right point to get the error
    int operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;
    // get the gradient matrix
    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const;
  private:
    cv::Mat left_points, right_points;
};

/* use ransac homography to combine several images taken at one position to one big image.
 * Main step:
 * 1. Use some feature detector to match different images
 * 2. Use ransac to find homography between every 2 images
 * 3. Combine different images to one big panorama
 */
class panoramicStitching {
  public:
    panoramicStitching(int base, const std::vector<std::string>& name,
                       std::vector<std::pair<int, int>>& image_pairs, featureMethod method = featureMethod::SIFT);
    /*
     * Use the homography to stitch the images
     * My strategy is to set one middle image as the base.
     * Transform other images to blend with this image.
     * return the result colorful image
     */
    cv::Mat stitch_image();
  private:
    /* extract match for every pair of images with sift
     * return true if extract is success
     */
    bool siftMatchExtract();
    /* Use ransac to calculate the homography for the m_image_pairs[pair_index]
     * 1. calculate the model with most inlines
     * 2. use the inlines to calculate the homography with DLT
     * 3. refine the DLT with LM iteration
     */
    bool calculateHomography();
    /* use the ransac algorithm to calculate the inlines
     * l_point, r_point: keypoints of 2 images
     * match: good_matches
     * reproj_threshold: the threshold for the distance between inlines and the model
     * return the mask of the inline
     */
    cv::Mat runRansac(const std::vector<cv::KeyPoint>& l_point,
                      const std::vector<cv::KeyPoint>& r_point,
                      const std::vector<cv::DMatch>& match, double reproj_threshold, double confidence,
                      int maxIter, bool& succeed, int& max_inlier_number);
    /* return true if the points are colinear
     */
    bool isColinear(cv::Mat points, int number);
    /* use the left points matrix and right points matrix to calculate the homography with DLT
    */
    cv::Mat runKernel(cv::Mat l_points, cv::Mat r_points);
    /* use the left points coordinate, right coordinate and homography to find the mask of inliers
     * and the number of inliers
     * return the mask Mat. 1 if inlier and 0 if outlier
     */
    cv::Mat findInliers(cv::Mat l_points, cv::Mat r_points, cv::Mat homography,
                        const double threshold, int& inlier_number);
    /* Every time when the new max inlier number is found, use it to update the max iteration
     * maxIter = log(1 - confidence) / log(1 - inlier_rate ^ 4))
     */
    int updateMaxIter(int inlier_number, int total_number, double confidence);
    /* input the inlier left_points, corresponding right_points and initial homography
     * use LM optimization to calculate the refined homography
     * return the refined homography
     */
    cv::Mat refine(cv::Mat left_points, cv::Mat right_points, cv::Mat ini_homo);

    featureMethod m_method; // method of feature detection
    std::vector<std::vector<cv::DMatch>> m_matches; // match for every pair
    std::vector<std::vector<cv::KeyPoint>> m_keyPoints; // keyPoints for every image
    std::vector<cv::Mat> m_images; // colorful images
    std::vector<cv::Mat> m_gray_images; // gray images
    std::vector<std::pair<int, int>> m_image_pairs; // pair of image index for image match
    std::vector<cv::Mat> m_homographies;
    int m_base_index;
};
}

#endif
