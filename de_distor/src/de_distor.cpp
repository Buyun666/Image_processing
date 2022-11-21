#include <opencv2/opencv.hpp>
#include <string>

using namespace std;
using namespace cv;

//string image_file = "./distorted.png";

int main(int argc, char **argv){
       
//image_file = string(argv[1]);
		//定义畸变系数
        double k1 =-0.546125, k2 = 0.279072, p1 = -0.003153, p2 = 0.00463;
        //相机内参
        double fx = 1908, fy = 1910, cx = 918, cy = 495;

cv::Mat cv_camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
cv_camera_matrix.at<double>(0,0) = fx; 
cv_camera_matrix.at<double>(1,1) = fy; //* incoming_image_ptr_mono8->image.rows;
cv_camera_matrix.at<double>(0,2) = cx; //* incoming_image_ptr_mono8->image.cols;
cv_camera_matrix.at<double>(1,2) = cy; //* incoming_image_ptr_mono8->image.rows;
cv_camera_matrix.at<double>(2,2) = 1;

//filling up of an array with the generated distortion parameters of the calibration algorithm (they are correct)

 cv::Mat distortion_coefficients = cv::Mat::zeros(1, 4, CV_64F);
distortion_coefficients.at<double>(0,0) = k1;
distortion_coefficients.at<double>(0,1) = k2;
distortion_coefficients.at<double>(0,2) = p1;
distortion_coefficients.at<double>(0,3) = p2;

		
		//读入图像，灰度图
        Mat image = imread(argv[1], 0);
        Mat image3 = imread(argv[1]);  
        Mat image_undistort = Mat(image.rows, image.cols, CV_8UC1);
        Mat output_image = Mat(image.rows, image.cols, CV_8UC1);
        //undistort(image3, output_image,cv_camera_matrix, cv::getDefaultNewCameraMatrix(cv_camera_matrix,image.size(),true),distortion_coefficients);
        undistort(image3, output_image,cv_camera_matrix,distortion_coefficients);

        //遍历每个像素，计算后去畸变
        for (int v = 0; v < image.rows; v++){
                for (int u = 0; u < image.cols; u++){
                		//根据公式计算去畸变图像上点(u, v)对应在畸变图像的坐标(u_distorted, v(distorted))，建立对应关系
                        double x = (u - cx) / fx;
                        double y = (v - cy) / fx;
                        double r = sqrt(x * x + y * y);
                        double x_distorted = x*(1+k1*r*r+k2*r*r*r*r)+2*p1*x*y+p2*(r*r+2*x*x);
                        double y_distorted = y*(1+k1*r*r+k2*r*r*r*r)+2*p2*x*y+p1*(r*r+2*x*x);
                        double u_distorted = fx * x_distorted + cx;
                        double v_distorted = fy * y_distorted + cy;
						
						//将畸变图像上点的坐标，赋值到去畸变图像中（最近邻插值）
                        if (u_distorted >= 0 && v_distorted >=0 && u_distorted < image.cols && v_distorted < image.rows){
                                image_undistort.at<uchar>(v, u) = image.at<uchar>((int)v_distorted, (int)u_distorted);
                        }else{
                                image_undistort.at<uchar>(v, u) = 0;
                        }
                }
        }
        imshow("Distorted Image", image);
        imshow("Undistorted Image", image_undistort);
        imshow("CVUndistorted Image", output_image);
        waitKey();
        return 0;
}

