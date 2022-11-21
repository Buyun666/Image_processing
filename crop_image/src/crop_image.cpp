// Include Libraries
#include<opencv2/opencv.hpp>
#include<iostream>
#include<string>
#include<vector>

// Namespace nullifies the use of cv::function();
using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
    // Get image path
    string filepath = "./crop_image/picture/";
    string source_path = "./crop_image/picture/*.png";
    string target_path = "./crop_image/picture_cropped/";
    int filepath_size = filepath.size();

    //Read the image pathname from the folder
    vector<string> result;
    glob(source_path, result, false); 


    for (int i = 0; i < result.size(); i++) {


        string tmp = result[i].c_str();
        // cout<<tmp<<endl;

        string picture_name = tmp.erase(0,filepath_size);
        picture_name.erase(picture_name.end()-4, picture_name.end());

        cout<<"filename: "<<picture_name<<endl;

        string  leftback_picture_path = target_path +  picture_name + "_leftback" + ".png";
        string  rightback_picture_path = target_path + picture_name +  "_rightback" + ".png";
        string  leftfront_picture_path = target_path + picture_name + "_leftfront" + ".png";
        string  rightfront_picture_path = target_path + picture_name + "_rightfront" + ".png";

        Mat img = imread(result[i], -1);//Read the picture from the path in an unchanging format

        // Crop image
	    Mat cropped_image_leftfront = img(Range(0,1080), Range(0,1920));
        Mat cropped_image_rightfront = img(Range(0,1080), Range(1920,3840));
        Mat cropped_image_leftback = img(Range(1080,2160), Range(0,1920));
        Mat cropped_image_rightback = img(Range(1080,2160), Range(1920,3840));

        //write image
        imwrite(leftback_picture_path,  cropped_image_leftback);
        imwrite(rightback_picture_path,  cropped_image_rightback);
        imwrite(leftfront_picture_path,  cropped_image_leftfront);
        imwrite(rightfront_picture_path,  cropped_image_rightfront);
    }

}
