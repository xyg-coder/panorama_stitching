Panoramic Stitching
======

This algorithm mainly uses several pictures taken by one camera in one fixed position from different angles. These pictures can be stitched together with the Homography matrix.<br>

## Construct the panoramicStitching class
cpp`
std::vector<std::pair<int, int>> image_pairs = { { 3, 0 },{ 3, 1 },{ 3, 2 } };
int base_index = 3;
panSti::panoramicStitching pan(base_index, names, image_pairs);
cv::Mat result = pan.stitch_image();
`
* First of all, I indicate the base image of the panorama. The base image means all other images will project to the coordinate of this image. We should choose the image at the center as the base image to prevent huge distortion.<br>
* Then,
