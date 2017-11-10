#include "panoramic_stitching.h"

#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <unordered_set>
#include <unsupported/Eigen/NonLinearOptimization>
#include <Eigen/src/Core/util/DisableStupidWarnings.h>

namespace panSti {
panoramicStitching::panoramicStitching(int base, const std::vector<std::string>& name,
                                       std::vector<std::pair<int, int>>& image_pairs, featureMethod method): m_image_pairs(image_pairs),
    m_base_index(base)	{
    for(int i = 0; i < name.size(); i++) {
        m_images.push_back(cv::imread(name[i]));
        if (!m_images.back().data) {
            std::cout << "cannot read image" << std::endl;
            return;
        }
        cv::Mat gray_image;
        cv::cvtColor(m_images.back(), gray_image, CV_RGB2GRAY);
        m_gray_images.push_back(gray_image);
    }
    if (method == featureMethod::SIFT) {
        bool result = siftMatchExtract();
        if (!result) {
            std::cout << "sift extract fails" << std::endl;
			return;
        }
    } else {
		std::cout << "At present only SIFT provided" << std::endl;
	}
}
cv::Mat panoramicStitching::stitch_image() {
    calculateHomography();
    //-- step 1: calculate all the homography with the base
    std::unordered_map<int, int> hash_map; // record the
    int pair_index = 0;
    std::vector<std::pair<std::pair<int, int>, int>> unsolved_pairs;
    for (std::pair<int, int>& image_pair : m_image_pairs) {
        if (image_pair.first == m_base_index) {
            if (hash_map.find(image_pair.second) != hash_map.end()) {
                pair_index++;
                continue;
            }
            hash_map.insert({image_pair.second, pair_index});
            pair_index++;
        } else if (image_pair.second == m_base_index) {
            if (hash_map.find(image_pair.first) != hash_map.end()) {
                pair_index++;
                continue;
            }
            hash_map.insert({image_pair.first, pair_index});
            // invert the homography
            m_homographies[pair_index] = m_homographies[pair_index].inv();
            image_pair.second = image_pair.first;
            image_pair.first = m_base_index;
            pair_index++;
        } else {
            if (hash_map.find(image_pair.first) != hash_map.end()) {
                if (hash_map.find(image_pair.second) != hash_map.end()) {
                    pair_index++;
                    continue;
                }
                m_homographies[pair_index] = m_homographies[hash_map.at(image_pair.first)] * 
					m_homographies[pair_index];
                //image_pair.second = image_pair.first;
                image_pair.first = m_base_index;
                hash_map.insert({image_pair.second, pair_index});
                pair_index++;
            } else if (hash_map.find(image_pair.second) != hash_map.end()) {
                if (hash_map.find(image_pair.first) != hash_map.end()) {
                    pair_index++;
                    continue;
                }
                m_homographies[pair_index] = m_homographies[hash_map.at(image_pair.second)] * 
					m_homographies[pair_index].inv();
				image_pair.second = image_pair.first;
                image_pair.first = m_base_index;
                hash_map.insert({image_pair.second, pair_index});
                pair_index++;
            } else {
                // two images are all not in the
                unsolved_pairs.push_back({image_pair, pair_index});
                pair_index++;
            }
        }
    }
    int max_iter = 3;
    while (!unsolved_pairs.empty() && max_iter >= 0) {
        for (auto i = unsolved_pairs.begin(); i != unsolved_pairs.end();) {
            if (hash_map.find(i->first.first) != hash_map.end() ||
                    hash_map.find(i->first.second) != hash_map.end()) {
                if (hash_map.find(i->first.first) != hash_map.end() &&
                        hash_map.find(i->first.second) == hash_map.end()) {
                    m_homographies[i->second] = m_homographies[i->second] *
                                                m_homographies[hash_map.at(i->first.first)];
                    hash_map.insert({i->first.second, i->second});
                } else if (hash_map.find(i->first.first) == hash_map.end() &&
                           hash_map.find(i->first.second) != hash_map.end()) {
                    m_homographies[i->second] = m_homographies[i->second].inv() *
                                                m_homographies[hash_map.at(i->first.second)];
                    hash_map.insert({i->first.first, i->second});
                }
                unsolved_pairs.erase(i++); // erase this element and let i plus one
            } else {
                i++;
            }
        }
        max_iter--;
    }
    if (max_iter < 0) {
        std::cout << "there is no enough link between images" << std::endl;
        cv::Mat tmp;
        return tmp;
    }
    //-- step 2: calculate four edge points of the source image to find the size of the image
    // first of all, calculate the size of the big image
    int left_most = 0;
    int right_most = m_gray_images[m_base_index].cols - 1;
    int down_most = m_gray_images[m_base_index].rows - 1;
    int up_most = 0;
    // the interval of every_image before adding the left_most and up_most
    std::unordered_map<int, std::pair<int, int>> x_interval;
    std::unordered_map<int, std::pair<int, int>> y_interval;
    x_interval.insert({m_base_index, {left_most, right_most}});
    y_interval.insert({m_base_index, {up_most, down_most}});
    for (auto i = hash_map.begin(); i != hash_map.end(); i++) {
        int image_index = i->first;
        int homo_index = i->second;
        m_homographies[homo_index] = m_homographies[homo_index] /
                                     m_homographies[homo_index].at<double>(2, 2);
        int x[4] = {0, 0,
                    m_gray_images[image_index].cols - 1, m_gray_images[image_index]. cols - 1
                   };
        int y[4] = {0, m_gray_images[image_index].rows - 1,
                    0, m_gray_images[image_index]. rows - 1
                   };
        int this_left = INT_MAX;
        int this_right = INT_MIN;
        int this_up = INT_MAX;
        int this_down = INT_MIN;
        for (int j = 0; j < 4; j++) {
            cv::Mat coor = m_homographies[homo_index].col(0) * x[j] +
                           m_homographies[homo_index].col(1) * y[j] +
                           m_homographies[homo_index].col(2);
            double x = coor.at<double>(0, 0) / coor.at<double>(2, 0);
            double y = coor.at<double>(1, 0) / coor.at<double>(2, 0);
            left_most = std::min(left_most, int(std::floor(x)));
            right_most = std::max(right_most, int(std::ceil(x)));
            up_most = std::min(up_most, int(std::floor(y)));
            down_most = std::max(down_most, int(std::ceil(y)));
            this_left = std::min(this_left, int(std::floor(x)));
            this_right = std::max(this_right, int(std::ceil(x)));
            this_up = std::min(this_up, int(std::floor(y)));
            this_down = std::max(this_down, int(std::ceil(y)));
        }
        x_interval.insert({image_index, {this_left, this_right}});
        y_interval.insert({image_index, {this_up, this_down}});
    }
    // all images should be translated with (fabs(left_most), fabs(up_most))
    // and then get the warped_images
    cv::Mat base_homo = (cv::Mat_<double>(3, 3) <<
                         1, 0, std::abs(left_most),
                         0, 1, std::abs(up_most),
                         0, 0, 1);
    std::vector<cv::Mat> warped_images;
    int new_width = right_most - left_most + 1;
    int new_height = down_most - up_most + 1;
    for (int i = 0; i < m_gray_images.size(); i++) {
        if (i != m_base_index) {
            int homo_index = hash_map.at(i);
            m_homographies[homo_index] = base_homo * m_homographies[homo_index];
            cv::Mat warped_image;
            cv::warpPerspective(m_images[i], warped_image,
                                m_homographies[homo_index], cv::Size2i(new_width, new_height));
            warped_images.push_back(warped_image);
        } else {
            cv::Mat warped_image;
            cv::warpPerspective(m_images[m_base_index], warped_image,
                                base_homo, cv::Size2i(new_width, new_height));
            warped_images.push_back(warped_image);
        }
    }
    // secondly, combine the corresponding warpped image
    // for every pixel, set the pixel-value of image whose center is nearest the the pixel
    // calculate every center coordinate first of all
    std::unordered_map<int, std::pair<int, int>> center_coor; // the coordinate is (y, x)
    center_coor.insert({ m_base_index, {m_gray_images[m_base_index].rows / 2 + std::abs(up_most),
                                        m_gray_images[m_base_index].cols / 2 + std::abs(left_most)
                                       }
                       });
    for (int i = 0; i < m_images.size(); i++) {
        if (i == m_base_index)
            continue;
        cv::Mat tmp_H = m_homographies[hash_map.at(i)];
        int mid_col = m_gray_images[i].cols >> 1;
        int mid_row = m_gray_images[i].rows >> 1;
        int mid_x = tmp_H.at<double>(0, 0) * mid_col + tmp_H.at<double>(0, 1) * mid_row + tmp_H.at<double>(0, 2);
        int mid_y = tmp_H.at<double>(1, 0) * mid_col + tmp_H.at<double>(1, 1) * mid_row + tmp_H.at<double>(1, 2);
        double ww = tmp_H.at<double>(2, 0) * mid_col + tmp_H.at<double>(2, 1) * mid_row + tmp_H.at<double>(2, 2);
        mid_x /= ww;
        mid_y /= ww;
        center_coor.insert({ i, {mid_y, mid_x} });
    }
    // set pixel value for new image
    cv::Mat result = warped_images[m_base_index];
    for (int i = 0; i < new_height; i++) {
        for (int j = 0; j < new_width; j++) {
            int nearest_value[3] = { -1, -1, -1 };
            int nearest_dst = INT_MAX;
            for (int k = 0; k < m_images.size(); k++) {
                if (j < x_interval.at(k).second + std::abs(left_most) && j > x_interval.at(k).first + std::abs(left_most) &&
                        i < y_interval.at(k).second + std::abs(up_most)&& i > y_interval.at(k).first + std::abs(up_most)) {
                    if (warped_images[k].at<cv::Vec3b>(i, j)[0] == 0 &&
                            warped_images[k].at<cv::Vec3b>(i, j)[1] == 0 &&
                            warped_images[k].at<cv::Vec3b>(i, j)[2] == 0)
                        continue;
                    else {
                        int dist = std::fabs(i - center_coor.at(k).first) + std::fabs(j - center_coor.at(k).second);
                        if (dist < nearest_dst) {
                            nearest_dst = dist;
                            nearest_value[0] = warped_images[k].at<cv::Vec3b>(i, j)[0];
                            nearest_value[1] = warped_images[k].at<cv::Vec3b>(i, j)[1];
                            nearest_value[2] = warped_images[k].at<cv::Vec3b>(i, j)[2];
                        }
                    }
                }
            }
            if (nearest_value[0] == -1)
                continue;
            else {
                result.at<cv::Vec3b>(i, j)[0] = nearest_value[0];
                result.at<cv::Vec3b>(i, j)[1] = nearest_value[1];
                result.at<cv::Vec3b>(i, j)[2] = nearest_value[2];
            }
        }
    }
    return result;
}

bool panoramicStitching::siftMatchExtract() {
    cv::SiftFeatureDetector detector;
    cv::SiftDescriptorExtractor extractor;
    std::vector<cv::Mat> descriptor_vec;
    for (const cv::Mat& image : m_gray_images) {
        if (!image.data) {
            std::cout << "image data wrong" << std::endl;
            return false;
        }
        //-- Step 1: detect the feature points with the Sift Detector
        std::vector<cv::KeyPoint> keypoints;
        detector.detect(image, keypoints);
        m_keyPoints.push_back(keypoints);
        //-- Step 2: calculate the descriptor
        cv::Mat descriptor;
        extractor.compute(image, keypoints, descriptor);
        descriptor_vec.push_back(descriptor);
    }
    double good_match_threshold = 0.02;
    for (const std::pair<int, int>& pair_index : m_image_pairs) {
        //-- Step 3: Match the descriptor vectors using FLANN matcher
        cv::Mat l_descriptor = descriptor_vec[pair_index.first];
        cv::Mat r_descriptor = descriptor_vec[pair_index.second];
        cv::FlannBasedMatcher matcher;
        std::vector<cv::DMatch> matches;
        matcher.match(l_descriptor, r_descriptor, matches);
        //-- Calculate good matches whose distance is less than 2*min_dist
        //-- or a small arbitary value (0.02) when min_dist is very small
        double min_dist = 100, max_dist = 0;
        for (const cv::DMatch& match : matches) {
            min_dist = std::min(min_dist, double(match.distance));
            max_dist = std::max(max_dist, double(match.distance));
        }
        std::vector<cv::DMatch> good_matches;
        for (const cv::DMatch& match : matches) {
            if (match.distance <= std::max(2 * min_dist, good_match_threshold))
                good_matches.push_back(match);
        }
        m_matches.push_back(good_matches);
    }
    return true;
}

bool panoramicStitching::calculateHomography() {
    const double confidence = 0.95;
    const int maxIter = 2000;
    const double ransac_threshold = 3;
    bool result = false;
    for (int index = 0; index < m_matches.size(); index++) {
        int left_index = m_image_pairs[index].first;
        int right_index = m_image_pairs[index].second;
        int inlier_number = 0;
        cv::Mat inlier_mask = runRansac(m_keyPoints[left_index], m_keyPoints[right_index], m_matches[index],
                                        ransac_threshold, confidence, maxIter, result, inlier_number);
        if (!result) {
            std::cout << "ransac fails" << std::endl;
            return false;
        } else {
            std::cout << "ransac succeeds" << std::endl;
        }
        cv::Mat left_points(2, inlier_number, CV_64F);
        cv::Mat right_points(2, inlier_number, CV_64F);
        double *p2lx = left_points.ptr<double>(0);
        double *p2ly = left_points.ptr<double>(1);
        double *p2rx = right_points.ptr<double>(0);
        double *p2ry = right_points.ptr<double>(1);
        int *p2mask = inlier_mask.ptr<int>(0);
        int tmp_index = 0; // the tmp index of the point matrix
        for (int i = 0; i < inlier_mask.cols; i++) {
            if (*(p2mask + i) == 1) {
                *(p2lx + tmp_index) = m_keyPoints[left_index][m_matches[index][i].queryIdx].pt.x;
                *(p2ly + tmp_index) = m_keyPoints[left_index][m_matches[index][i].queryIdx].pt.y;
                *(p2rx + tmp_index) = m_keyPoints[right_index][m_matches[index][i].trainIdx].pt.x;
                *(p2ry + tmp_index) = m_keyPoints[right_index][m_matches[index][i].trainIdx].pt.y;
                tmp_index++;
            }
        }
        // use the inlier mask to get the initial homography with DLT
        cv::Mat homo = runKernel(left_points, right_points);
        // use the iteration to refine the homography
        homo = refine(left_points, right_points, homo);
        m_homographies.push_back(homo);
    }
    return true;
}

cv::Mat panoramicStitching::runRansac(const std::vector<cv::KeyPoint>& l_point,
                                      const std::vector<cv::KeyPoint>& r_point,
                                      const std::vector<cv::DMatch>& match,
                                      double reproj_threshold, double confidence,
                                      int maxIter, bool & succeed, int &max_inlier_number) {
    const int least_point = 4; // the least number points needed to calculate the homography
    const int n_points = match.size();
    cv::Mat l_pointsM(3, n_points, CV_64F);
    cv::Mat r_pointsM(3, n_points, CV_64F);
    // get the coordinate matrix for all points
    for (int i = 0 ; i < n_points; i++) {
        l_pointsM.at<double>(0, i) = l_point[match[i].queryIdx].pt.x;
        l_pointsM.at<double>(1, i) = l_point[match[i].queryIdx].pt.y;
        l_pointsM.at<double>(2, i) = 1;
        r_pointsM.at<double>(0, i) = r_point[match[i].trainIdx].pt.x;
        r_pointsM.at<double>(1, i) = r_point[match[i].trainIdx].pt.y;
        r_pointsM.at<double>(2, i) = 1;
    }
    int iter_times = 0;
    max_inlier_number = 0;
    cv::Mat most_inlier_mask;
    while (iter_times <= maxIter) {
        std::unordered_set<int> hash_set; // use hash_set to ensure no duplicate points in the example points
        cv::Mat l_pointSample(3, least_point, CV_64F);
        cv::Mat r_pointSample(3, least_point, CV_64F);
        int exist_points = 0;
        //-- step 1:get 4 points randomly
        while (exist_points < least_point) {
            int next_index = rand() % n_points;
            if (hash_set.find(next_index) != hash_set.end())
                continue;
            hash_set.insert(next_index);
            l_pointsM.col(next_index).copyTo(l_pointSample.col(exist_points));
            r_pointsM.col(next_index).copyTo(r_pointSample.col(exist_points));
            exist_points++;
            if (exist_points >= 3 &&
                    (isColinear(l_pointSample, exist_points) ||
                     isColinear(r_pointSample, exist_points)))
                exist_points--;
        }
        //-- step 2: use 4 points to get one homography
        cv::Mat model = runKernel(l_pointSample, r_pointSample);
        //-- step 3: find the inlines
        int tmp_inlier_number = 0;
        cv::Mat tmp_mask = findInliers(l_pointsM, r_pointsM, model,
                                       reproj_threshold, tmp_inlier_number);
        //-- change the iteration times according to the confidence and the inlier number
        if (tmp_inlier_number > max_inlier_number) {
            most_inlier_mask = tmp_mask;
            max_inlier_number = tmp_inlier_number;
            maxIter = std::min(maxIter,
                               updateMaxIter(max_inlier_number, n_points, confidence));
        }
        iter_times++;
    }
    if (max_inlier_number > least_point) {
        succeed = true;
        return most_inlier_mask;
    } else {
        succeed = false;
        return most_inlier_mask;
    }
}

// In order to increase the speed, we use the knowledge that everytime we will
// only add one point, and the previous point is guaranted to be not colinear
bool panoramicStitching::isColinear(cv::Mat points, int number) {
    if (number < 3) return false;
    // choose every possible 2 points
    int k = number - 1;
    for (int i = 0; i < number - 2; i++) {
        for (int j = i + 1; j < number - 1; j++) {
            double x1 = points.at<double>(0, i);
            double y1 = points.at<double>(1, i);
            double x2 = points.at<double>(0, j);
            double y2 = points.at<double>(1, j);
            double x3 = points.at<double>(0, k);
            double y3 = points.at<double>(1, k);
            double dx1 = x1 - x2;
            double dy1 = y1 - y2;
            double dx2 = x1 - x3;
            double dy2 = y1 - y3;
            // DBL_EPSILON: is used to compare double precision with 0
            // the equation is derived from the distance from one point to one line
            if (fabs(dx1 * dy2 - dx2 * dy1) <
                    DBL_EPSILON * (fabs(dx1) + fabs(dy1) + fabs(dx2) + fabs(dy2)))
                return true;
        }
    }
    return false;
}

cv::Mat panoramicStitching::runKernel(cv::Mat l_points, cv::Mat r_points) {
    //-- Step 1: Normalize, center is 0, and average distance to center is sqrt(2)
    // the detailed algorithm explain can be seen in Multiview Geometry in Computer Vision P109
    int data_number = l_points.cols;
    double *p2lx = l_points.ptr<double>(0);
    double *p2ly = l_points.ptr<double>(1);
    double *p2rx = r_points.ptr<double>(0);
    double *p2ry = r_points.ptr<double>(1);
    double l_center[2] = {0, 0};
    double r_center[2] = {0, 0};
    for (int i = 0; i < data_number; i++) {
        l_center[0] += *(p2lx + i);
        l_center[1] += *(p2ly + i);
        r_center[0] += *(p2rx + i);
        r_center[1] += *(p2ry + i);
    }
    l_center[0] /= data_number;
    l_center[1] /= data_number;
    r_center[0] /= data_number;
    r_center[1] /= data_number;
    double sum_l[2] = {0, 0};
    double sum_r[2] = {0, 0};
    // use the sum of absolute value to approximate the distance between points and center
    for (int i = 0; i < data_number; i++) {
        sum_l[0] += fabs(*(p2lx + i) - l_center[0]);
        sum_l[1] += fabs(*(p2ly + i) - l_center[1]);
        sum_r[0] += fabs(*(p2rx + i) - r_center[0]);
        sum_r[1] += fabs(*(p2ry + i) - r_center[1]);
    }
    if (fabs(sum_l[0]) < DBL_EPSILON || fabs(sum_l[1]) < DBL_EPSILON ||
            fabs(sum_r[0]) < DBL_EPSILON || fabs(sum_r[1]) < DBL_EPSILON) {
        std::cout << "the inlier points have something wrong" << std::endl;
        return cv::Mat(0, 0, CV_64F);
    }
    sum_l[0] = data_number / sum_l[0];
    sum_l[1] = data_number / sum_l[1];
    sum_r[0] = data_number / sum_r[0];
    sum_r[1] = data_number / sum_r[1];
    // final homography is T'^(-1) * H_normalize * T
    // Be careful that the Homography is used to transfer the right to the left
    cv::Mat T1_inv = (cv::Mat_<double>(3, 3) <<
                      1 / sum_l[0], 0, l_center[0],
                      0, 1 / sum_l[1], l_center[1],
                      0, 0, 1);
    cv::Mat T2 = (cv::Mat_<double>(3, 3) <<
                  sum_r[0], 0, -sum_r[0] * r_center[0],
                  0, sum_r[1], -sum_r[1] * r_center[1],
                  0, 0, 1);
    //--Step 2: use the normalized coordinate to calculate DLT
    cv::Mat L(cv::Mat::zeros(2 * data_number, 9, CV_64F));
    for (int i = 0; i < data_number; i++) {
        double lx = (*(p2lx + i) - l_center[0]) * sum_l[0];
        double ly = (*(p2ly + i) - l_center[1]) * sum_l[1];
        double rx = (*(p2rx + i) - r_center[0]) * sum_r[0];
        double ry = (*(p2ry + i) - r_center[1]) * sum_r[1];
        double *p2up = L.ptr<double>(i * 2);
        double *p2down = L.ptr<double>(i * 2 + 1);
        *(p2up + 3) = -rx;
        *(p2up + 4) = -ry;
        *(p2up + 5) = -1;
        *(p2up + 6) = ly * rx;
        *(p2up + 7) = ly * rx;
        *(p2up + 8) = ly;
        *p2down = rx;
        *(p2down + 1) = ry;
        *(p2down + 2) = 1;
        *(p2down + 6) = -lx * rx;
        *(p2down + 7) = -lx * ry;
        *(p2down + 8) = -lx;
    }
    cv::Mat LtL = L.t() * L;
    cv::Mat eigenValue, eigenVector;
    cv::eigen(LtL, eigenValue, eigenVector);
    cv::Mat result = eigenVector.row(8);
    // we need to clone the result here because the reshape needs that matrix data is continuous
    result = T1_inv * result.clone().reshape(0, 3) * T2;
    result = result / result.at<double>(2, 2);
    // return the 3*3 H matrix
    return result;
}

cv::Mat panoramicStitching::findInliers(cv::Mat l_points, cv::Mat r_points,
                                        cv::Mat homography,
                                        const double threshold, int& inlier_number) {
    int n = l_points.cols; // number of points
    cv::Mat result_mask(1, n, CV_32S);
    // calculate the geometry error
    double h[9];
    for (int i = 0; i < 9; i++) {
        h[i] = homography.at<double>(i / 3, i - 3 * (i / 3));
    }
    double* p2rx = r_points.ptr<double>(0);
    double* p2ry = r_points.ptr<double>(1);
    double* p2rw = r_points.ptr<double>(2);
    double* p2lx = l_points.ptr<double>(0);
    double* p2ly = l_points.ptr<double>(1);
    double* p2lw = l_points.ptr<double>(2);
    double threshold_square = threshold * threshold;
    int *p2result = result_mask.ptr<int>(0);
    int count = 0;
    for (int i = 0; i < n; i++) {
        double ww = 1.0 / (h[6] * (*(p2rx + i)) + h[7] * (*(p2ry + i)) + h[8] * (*(p2rw + i)));
        double xx = ww * (h[0] * (*(p2rx + i)) + h[1] * (*(p2ry + i)) + h[2] * (*(p2rw + i)))
                    - (*(p2lx + i)) / (*(p2lw + i));
        double yy = ww * (h[3] * (*(p2rx + i)) + h[4] * (*(p2ry + i)) + h[5] * (*(p2rw + i)))
                    - (*(p2ly + i)) / (*(p2lw + i));
        if ((xx * xx + yy * yy) <= threshold_square) {
            *(p2result + i) = 1;
            count++;
        } else {
            *(p2result + i) = 0;
        }
    }
    inlier_number = count;
    return result_mask;
}

int panoramicStitching::updateMaxIter(int inlier_number, int total_number, double confidence) {
    int model_points = 4; // we need 4 points to calculate one homography
    double e_p = 1 - double(inlier_number) / total_number;
    if (e_p > 1) {
        std::cout << "the error rate is greater than 1" << std::endl;
        return -1;
    }
    double num = std::max(1.0 - confidence, DBL_MIN); // avoid inf
    double denom = 1.0 - pow(1.0 - e_p, model_points);
    if (denom < DBL_MIN) return 0; // all points are inliers
    num = std::log(num) / std::log(denom);
	// prevent the overflow
	if (num > 10000)
		return 10000;
    return std::round(num);
}

cv::Mat panoramicStitching::refine(cv::Mat left_points, cv::Mat right_points, cv::Mat ini_homo) {
    Eigen::VectorXd x(8);
    double h8 = ini_homo.at<double>(2, 2);
    for (int i = 0; i <= 7; i++) {
        x[i] = ini_homo.at<double>(i / 3, i - 3 * (i / 3)) / h8;
    }
    //Eigen::VectorXd x_previous = x.replicate(1, 1);
    geometricError functor(left_points, right_points);
    Eigen::NumericalDiff<geometricError> num_diff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<geometricError>> lm(num_diff);
    int ret = lm.minimize(x);
    //std::cout << x - x_previous << std::endl;
    for (int i = 0; i <= 7; i++) {
        ini_homo.at<double>(i / 3, i - 3 * (i / 3)) = x[i];
    }
    ini_homo.at<double>(2, 2) = 1;
    return ini_homo;
}

geometricError::geometricError(cv::Mat left_points, cv::Mat right_points):
    Functor<double>(8, 2 * left_points.cols),
    left_points(left_points),
    right_points(right_points) {
}

int geometricError::operator()(const Eigen::VectorXd & x, Eigen::VectorXd & fvec) const {
    double* p2lx = const_cast<double*>(left_points.ptr<double>(0));
    double* p2ly = const_cast<double*>(left_points.ptr<double>(1));
    double* p2rx = const_cast<double*>(right_points.ptr<double>(0));
    double* p2ry = const_cast<double*>(right_points.ptr<double>(1));
    for (int i = 0; i < values() / 2; i++) {
        double lx = *(p2lx + i);
        double ly = *(p2ly + i);
        double rx = *(p2rx + i);
        double ry = *(p2ry + i);
        double ww = x[6] * rx + x[7] * ry + 1;
        ww = std::fabs(ww) > DBL_EPSILON ? 1. / ww : 0;
        double lx_est = ww * (x[0] * rx + x[1] * ry + x[2]);
        double ly_est = ww * (x[3] * rx + x[4] * ry + x[5]);
        fvec[i * 2] = lx - lx_est;
        fvec[i * 2 + 1] = ly - ly_est;
    }
    return 0;
}

int geometricError::df(const Eigen::VectorXd & x, Eigen::MatrixXd & fjac) const {
    double* p2lx = const_cast<double*>(left_points.ptr<double>(0));
    double* p2ly = const_cast<double*>(left_points.ptr<double>(1));
    double* p2rx = const_cast<double*>(right_points.ptr<double>(0));
    double* p2ry = const_cast<double*>(right_points.ptr<double>(1));
    for (int i = 0; i < values() >> 1; i++) {
        double rx = *(p2rx + i);
        double ry = *(p2ry + i);
        double ww = x[6] * rx + x[7] * ry + 1;
        ww = std::fabs(ww) > DBL_EPSILON ? 1. / ww : 0;
        double lx_est = ww * (x[0] * rx + x[1] * ry + x[2]);
        double ly_est = ww * (x[3] * rx + x[4] * ry + x[3]);
        fjac(i * 2, 0) = rx * ww;
        fjac(i * 2, 1) = ry * ww;
        fjac(i * 2, 2) = ww;
        fjac(i * 2, 3) = 0;
        fjac(i * 2, 4) = 0;
        fjac(i * 2, 5) = 0;
        fjac(i * 2, 6) = -rx * lx_est * ww;
        fjac(i * 2, 7) = -ry * lx_est * ww;
        fjac(i * 2 + 1, 0) = 0;
        fjac(i * 2 + 1, 1) = 0;
        fjac(i * 2 + 1, 2) = 0;
        fjac(i * 2 + 1, 3) = rx * ww;
        fjac(i * 2 + 1, 4) = ry * ww;
        fjac(i * 2 + 1, 5) = ww;
        fjac(i * 2 + 1, 6) = -rx * ly_est * ww;
        fjac(i * 2 + 1, 7) = -ry * ly_est * ww;
    }
    return 0;
}

}
