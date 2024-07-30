#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

void printProgress(float percentage) {
    int barWidth = 50;
    cout << "[";
    int pos = barWidth * percentage;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos)
            cout << "#";
        else
            cout << " ";
    }
    cout << "] " << int(percentage * 100.0) << " %\r";
    cout.flush();
}

int main() {
    // Checkerboard dimensions
    Size checkerboard_size(10, 6); // 11x7 inner corners
    float square_size = 0.08f;     // 8 cm square size

    // Prepare object points based on the real-world dimensions of the checkerboard
    vector<Point3f> objp;
    for (int i = 0; i < checkerboard_size.height; i++) {
        for (int j = 0; j < checkerboard_size.width; j++) {
            objp.push_back(Point3f(j * square_size, i * square_size, 0));
        }
    }

    // Arrays to store object points and image points from all the images
    vector<vector<Point3f>> objpoints;  // 3D points in real world space
    vector<vector<Point2f>> imgpoints;  // 2D points in image plane

    // Read images from the specified directory
    string image_directory = "/home/cthalia/EndToEnd/DATA/OUTPUT/*.jpg";
    vector<String> image_files;
    glob(image_directory, image_files);

    if (image_files.empty()) {
        cerr << "No images found in the specified directory." << endl;
        return -1;
    }

    cout << "Found " << image_files.size() << " images for calibration." << endl;

    Mat gray;  // Declare gray outside the loop to use its size later
    bool first_image_processed = false;  // Flag to check if at least one image has been processed

    for (size_t i = 0; i < image_files.size(); ++i) {
        Mat img = imread(image_files[i]);
        if (img.empty()) {
            cerr << "Could not open or find the image: " << image_files[i] << endl;
            continue;
        }

        cout << "Processing image " << i + 1 << " of " << image_files.size() << ": " << image_files[i] << endl;

        cvtColor(img, gray, COLOR_BGR2GRAY);

        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, checkerboard_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

        if (found) {
            objpoints.push_back(objp);
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001));
            imgpoints.push_back(corners);

            drawChessboardCorners(img, checkerboard_size, corners, found);
            imshow("Checkerboard", img);
            waitKey(500);

            first_image_processed = true;  // Set flag to true after processing the first image
        }

        // Print progress bar
        printProgress(static_cast<float>(i + 1) / image_files.size());
    }
    cout << endl; // Move to the next line after the progress bar

    if (!first_image_processed) {
        cerr << "No valid images for calibration." << endl;
        return -1;
    }

    destroyAllWindows();

    // Camera calibration
    Mat camera_matrix = Mat::eye(3, 3, CV_64F);
    Mat dist_coeffs = Mat::zeros(8, 1, CV_64F);
    vector<Mat> rvecs, tvecs;

    // Use the size of the last processed image for calibration
    Size image_size = gray.size();
    double rms = calibrateCamera(objpoints, imgpoints, image_size, camera_matrix, dist_coeffs, rvecs, tvecs);

    cout << "Camera matrix:\n" << camera_matrix << endl;
    cout << "Distortion coefficients:\n" << dist_coeffs << endl;
    cout << "RMS error: " << rms << endl;

    // Save the calibration results
    FileStorage fs("camera_calibration.yml", FileStorage::WRITE);
    fs << "camera_matrix" << camera_matrix;
    fs << "dist_coeffs" << dist_coeffs;
    fs.release();

    return 0;
}
