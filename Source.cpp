#include "panoramic_stitching.h"

/*
void panorama_stitch_data1() {
	std::string path_name = "C:\\xinyuan Gui\\pcvAssignment2\\data1\\";
	std::vector<std::string> names = { "112_1298_small.JPG", "112_1299_small.JPG", "112_1300_small.JPG", "113_1301.JPG",
		"113_1302_small.JPG", "113_1303_small.JPG" };
	for (std::string& name : names) {
		name = path_name + name;
	}
	std::vector<std::pair<int, int>> image_pairs = { { 3, 0 },{ 3, 1 },{ 3, 2 },{ 3, 4 },{ 3, 5 } };
	int base_index = 3;
	panSti::panoramicStitching pan(base_index, names, image_pairs);
}

void test_panorama_stitch_data1() {
	std::string path_name = "C:\\xinyuan Gui\\pcvAssignment2\\data1\\";
	std::vector<std::string> names = {"112_1298_small.JPG", "112_1299_small.JPG", "112_1300_small.JPG", "113_1301_small.JPG", 
	"113_1302_small.JPG", "113_1303_small.JPG"};
	for (std::string& name : names) {
		name = path_name + name;
	}
	std::vector<std::pair<int, int>> image_pairs = { { 3, 0 }, {3, 1}, {3, 2}, {3, 4}, {3, 5} };
	int base_index = 3;
	panSti::panoramicStitching pan(base_index, names, image_pairs);
	cv::Mat result = pan.stitch_image();
	std::string result_name = path_name + "small_result.JPG";
	cv::imwrite(result_name, result);
}
void test_panorama_stitch_iteration() {
	std::string path_name = "C:\\xinyuan Gui\\pcvAssignment2\\data1\\";
	std::vector<std::string> names = { "112_1298_small.JPG", "113_1301_small.JPG"};
	for (std::string& name : names) {
		name = path_name + name;
	}
	std::vector<std::pair<int, int>> image_pairs = { { 1, 0 }};
	int base_index = 1;
	panSti::panoramicStitching pan(base_index, names, image_pairs);
	cv::Mat result = pan.stitch_image();
	cv::imshow("result_image", result);
	cv::waitKey(0);
}
void test_data2() {
	std::string path_name = "C:\\xinyuan Gui\\pcvAssignment2\\data2\\";
	std::vector<std::string> names = { "IMG_0488.JPG", "IMG_0489.JPG", "IMG_0490.JPG",
		"IMG_0491.JPG" };
	for (std::string& name : names) {
		name = path_name + name;
	}
	std::vector<std::pair<int, int>> image_pairs = { { 2, 0 }, {2, 1}, {2, 3} };
	int base_index = 2;
	panSti::panoramicStitching pan(base_index, names, image_pairs);
	cv::Mat result = pan.stitch_image();
	cv::imshow("result_image", result);
	cv::waitKey(0);
}
*/
void panorama_stitch_sift_data1() {
	std::string path_name = "C:\\xinyuan Gui\\pcvAssignment2\\data1\\";
	std::vector<std::string> names = { "112_1298.JPG", "112_1299.JPG", "112_1300.JPG", "113_1301.JPG",
		"113_1302.JPG", "113_1303.JPG" };
	for (std::string& name : names) {
		name = path_name + name;
	}
	std::vector<std::pair<int, int>> image_pairs = { { 3, 0 },{ 3, 1 },{ 3, 2 },{ 3, 4 },{ 3, 5 } };
	int base_index = 3;
	panSti::panoramicStitching pan(base_index, names, image_pairs);
	cv::Mat result = pan.stitch_image();
	std::string result_name = path_name + "sift_result.JPG";
	cv::imwrite(result_name, result);
}
void panorama_stitch_sift_data2() {
	std::string path_name = "C:\\xinyuan Gui\\pcvAssignment2\\data2\\";
	std::vector<std::string> names = { "IMG_0488.JPG", "IMG_0489.JPG", "IMG_0490.JPG",
		"IMG_0491.JPG" };
	for (std::string& name : names) {
		name = path_name + name;
	}
	std::vector<std::pair<int, int>> image_pairs = { { 2, 0 },{ 2, 1 },{ 2, 3 } };
	int base_index = 2;
	panSti::panoramicStitching pan(base_index, names, image_pairs);
	cv::Mat result = pan.stitch_image();
	cv::string result_name = path_name + "sift_result.JPG";
	cv::imwrite(result_name, result);
}
void panorama_stitch_sift_data3() {
	std::string path_name = "C:\\xinyuan Gui\\pcvAssignment2\\data3\\";
	std::vector<std::string> names = { "IMG_0675.JPG", "IMG_0676.JPG", "IMG_0677.JPG"};
	for (std::string& name : names) {
		name = path_name + name;
	}
	std::vector<std::pair<int, int>> image_pairs = { { 1, 0 },{ 1, 2 } };
	int base_index = 1;
	panSti::panoramicStitching pan(base_index, names, image_pairs);
	cv::Mat result = pan.stitch_image();
	cv::string result_name = path_name + "sift_result.JPG";
	cv::imwrite(result_name, result);
}
void panorama_stitch_sift_data4() {
	std::string path_name = "C:\\xinyuan Gui\\pcvAssignment2\\data4\\";
	std::vector<std::string> names = { "IMG_7355.JPG", "IMG_7356.JPG", "IMG_7357.JPG", "IMG_7358.JPG" };
	for (std::string& name : names) {
		name = path_name + name;
	}
	std::vector<std::pair<int, int>> image_pairs = { { 1, 0 }, { 1, 2 }, {2, 3} };
	int base_index = 1;
	panSti::panoramicStitching pan(base_index, names, image_pairs);
	cv::Mat result = pan.stitch_image();
	cv::string result_name = path_name + "sift_result.JPG";
	cv::imwrite(result_name, result);
}
void test_panorama_stitch_sift_data4() {
	std::string path_name = "C:\\xinyuan Gui\\pcvAssignment2\\data4\\";
	std::vector<std::string> names = { "IMG_7357.JPG", "IMG_7358.JPG" };
	for (std::string& name : names) {
		name = path_name + name;
	}
	std::vector<std::pair<int, int>> image_pairs = { { 0, 1 } };
	int base_index = 0;
	panSti::panoramicStitching pan(base_index, names, image_pairs);
	cv::Mat result = pan.stitch_image();
	cv::string result_name = path_name + "sift_result.JPG";
	cv::imwrite(result_name, result);
}
int main() {
	panorama_stitch_sift_data1();
	panorama_stitch_sift_data2();
	panorama_stitch_sift_data3();
	//panorama_stitch_sift_data4();
}