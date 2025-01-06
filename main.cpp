#include <chrono>
#include "depthai/depthai.hpp"
#include <iostream>
#include <unistd.h>
#include <pigpio.h>
#include <wiringPi.h>
#include "rotary_encoder.hpp"
#include <stdlib.h>
#include <cmath>
#include "t34.cpp"

#define cone_amount 5

void callback(int wayR){
    posR += wayR;
}

void callback2(int wayL){
    posL += wayL;
}

static const std::vector<std::string> labelMap = {
    "cone"};

static std::atomic<bool> syncNN{true};



int main(int argc, char **argv){


    using namespace std;
    using namespace std::chrono;
    std::string nnPath(BLOB_PATH);

    // If path to blob specified, use that
    if (argc > 1)
    {
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());
//
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto detectionNetwork = pipeline.create<dai::node::YoloDetectionNetwork>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("rgb");
    nnOut->setStreamName("detections");

    // Properties
    camRgb->setPreviewSize(416, 416);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setFps(15);

    // Network specific settings
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->setNumClasses(1);
    detectionNetwork->setCoordinateSize(4);
    detectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    detectionNetwork->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
    detectionNetwork->setIouThreshold(0.5f);
    detectionNetwork->setBlobPath(nnPath);
    detectionNetwork->setNumInferenceThreads(2);
    detectionNetwork->input.setBlocking(false);

    // Linking
    camRgb->preview.link(detectionNetwork->input);
    if (syncNN)
    {
        detectionNetwork->passthrough.link(xoutRgb->input);
    }
    else
    {
        camRgb->preview.link(xoutRgb->input);
    }

    detectionNetwork->out.link(nnOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the rgb frames and nn data from the outputs defined above
    auto qRgb = device.getOutputQueue("rgb", 4, false);
    auto qDet = device.getOutputQueue("detections", 4, false);

    cv::Mat frame;
    std::vector<dai::ImgDetection> detections;
    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color2 = cv::Scalar(255, 255, 255);

    // Add bounding boxes and text to the frame and show it to the user
    auto displayFrame = [](std::string name, cv::Mat frame, std::vector<dai::ImgDetection> &detections,int& largestXCenter)
    {
        auto color = cv::Scalar(255, 0, 0);

        int maxArea = 0;
        largestXCenter = 0;
        int largestYCenter = 0;

        // nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
        for (auto &detection : detections)
        {
            int x1 = detection.xmin * frame.cols;
            int y1 = detection.ymin * frame.rows;
            int x2 = detection.xmax * frame.cols;
            int y2 = detection.ymax * frame.rows;
            int x_center = x1 + (x2 - x1) / 2;
            int y_center = y1 + (y2 - y1) / 2;
            int area = (x2 - x1) * (y2 - y1);

            if (area > maxArea)
            {
                maxArea = area;
                largestXCenter = x_center;
                largestYCenter = y_center;
            }

            // std::string positionText1 = "Area:"  + std::to_string(area);
            std::string positionText = "Pos center: (" + std::to_string(x_center) + ", " + std::to_string(y_center) + ")";
            // cv::putText(frame, positionText, cv::Point(x1, y1 + 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            // cv::putText(frame, positionText1, cv::Point(x1, y1 - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            // std::stringstream confStr;
            // confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            // cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        // Display the largest detection center and its bounding box
        if (maxArea > 0)
        {
            std::string largestCenterText = "Largest center: (" + std::to_string(largestXCenter) + ", " + std::to_string(largestYCenter) + ")";
            cv::putText(frame, largestCenterText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::rectangle(frame, cv::Rect(cv::Point(largestXCenter - 30, largestYCenter - 30), cv::Point(largestXCenter + 30, largestYCenter + 30)), cv::Scalar(0, 255, 0), 4);
        }

        // Show the frame
        cv::imshow(name, frame);
    };

//część luxonis na górze

    if (gpioInitialise() < 0) return 1;
    gpioDelay(50000); 
    int done =0;
    re_decoder dec(5, 6, callback);
    re_decoder dec2(9, 11, callback2);
    double elapsed_time = 0;
    fd = i2C_init();
    setup_PWM(pin_R);
    setup_PWM(pin_L);
    uint32_t start_time1 =0;
    uint32_t start_time2 = gpioTick(); 



//gpioDelay(1000000);
//test kwadratu
//   square_test2();



// wykrywanie pachołków 
    for(int i=0; i<cone_amount; i++){
    static bool flag = false;
    static float angle1;
    static float angle2;

    if(i%2==0){
        angle1 = 315.0;
        angle2 = 45.0;
    }else{
        angle1 = 45.0;
        angle2 = 315.0;
    }


    while (done!=1){   
        std::shared_ptr<dai::ImgFrame> inRgb;
        std::shared_ptr<dai::ImgDetections> inDet;

        if (syncNN)
        {
            inRgb = qRgb->get<dai::ImgFrame>();
            inDet = qDet->get<dai::ImgDetections>();
        }
        else
        {
            inRgb = qRgb->tryGet<dai::ImgFrame>();
            inDet = qDet->tryGet<dai::ImgDetections>();
        }

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if (elapsed > seconds(1))
        {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        if (inRgb)
        {
            frame = inRgb->getCvFrame();
            std::stringstream fpsStr;
            fpsStr << "NN fps: " << std::fixed << std::setprecision(2) << fps;
            cv::putText(frame, fpsStr.str(), cv::Point(2, inRgb->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color2);
        }

        if (inDet)
        {
            detections = inDet->detections;
        }

        if (!frame.empty())
        {
            displayFrame("rgb", frame, detections,posX);
        }

        done = camera_turning(posX,centerX,countx,flag,last_time);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            return 0;
        }
    }
    
    
    done = 0;
    flag = true;

    while (distance1>30) {
        sensors_measure(0.1,distance1, distance2, distance3, distance4, rpmR, rpmL, start_time2);
        drive_straight(40,rpmL,rpmR,0,1);
    }
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    turning_better(angle1, fd,dt1);
    reset_error_integral(prevErrorR,integralR,prevErrorL,integralL,prevErrorX,integralX,prevErrorO,integralO,prevErrorC,integralC,prevErrordiff,integraldiff); 
    gpioDelay(500000);
    start_time1 = gpioTick();
    elapsed_time = 0; 
    while (elapsed_time<6) {
        uint32_t current_time1 = gpioTick();
        elapsed_time = (current_time1 - start_time1) / 1e6; 
            std::cout << "Elapsed: " << elapsed_time << std::endl;
        sensors_measure(0.1,distance1, distance2, distance3, distance4, rpmR, rpmL, start_time2);
        drive_straight(50,rpmL,rpmR,0,1); 
    }
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    gpioDelay(500000); 
    turning_better(angle2, fd,dt1);
    reset_error_integral(prevErrorR,integralR,prevErrorL,integralL,prevErrorX,integralX,prevErrorO,integralO,prevErrorC,integralC,prevErrordiff,integraldiff); 
    start_time1 = gpioTick();
    elapsed_time = 0; 
    while (elapsed_time<6) {
        uint32_t current_time1 = gpioTick();
        elapsed_time = (current_time1 - start_time1) / 1e6; 
            std::cout << "Elapsed: " << elapsed_time << std::endl;
        sensors_measure(0.1,distance1, distance2, distance3, distance4, rpmR, rpmL, start_time2);
        drive_straight(50,rpmL,rpmR,0,1); 
    }
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    gpioDelay(500000); 
    turning_better(0, fd,dt1);
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    reset_error_integral(prevErrorR,integralR,prevErrorL,integralL,prevErrorX,integralX,prevErrorO,integralO,prevErrorC,integralC,prevErrordiff,integraldiff); 
}


    return 0;
}
