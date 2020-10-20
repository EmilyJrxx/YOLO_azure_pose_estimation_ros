// YOLODetection.h
// Description: this header file contains functions for YOLODetection,
// which are mainly: Detection, Outputing Boundingboxes
// Input:
//      rgb images,
//      depth images
// Output:
//      bounding boxes
//      (corresponding) confidences
//      (corresponding) classids
// # include <ros/ros.h>

# include <iostream>
# include <fstream>
# include <opencv4/opencv2/dnn.hpp>
# include <opencv4/opencv2/imgproc.hpp>
# include <opencv4/opencv2/highgui.hpp>

// # include <pcl/features/normal_3d_omp.h>
// # include <pcl/kdtree/kdtree_flann.h>
// # include <pcl/surface/convex_hull.h>
// # include <pcl/filters/crop_hull.h>

using namespace std;
using namespace cv;
using namespace dnn;

namespace yolo
{
    class YOLODetector
    {
        private:
            Mat rgb_img;
            Mat depth_img;
            vector<float> confidences;
            vector<Rect> boxes;
            vector<int> classIds;
            vector<int> indices;

            vector<string> class_labels;
            bool if_detected;
            bool if_postprocessed;
            Net net;

        public:
            YOLODetector(const string modelConfiguration, const string modelWeights){
                net = readNetFromDarknet(modelConfiguration, modelWeights);
                net.setPreferableBackend(DNN_BACKEND_OPENCV);
                net.setPreferableTarget(DNN_TARGET_CPU);
                cout << "YOLO network configured. " << endl; 
                boxes.resize(1);
                classIds.resize(1);
                indices.resize(1); 
                confidences.resize(1);
                if_detected = false;
                if_postprocessed = false;
            }
            YOLODetector(const string modelConfiguration, const string modelWeights, 
                Mat rgb_input, Mat depth_input){
                net = readNetFromDarknet(modelConfiguration, modelWeights);
                net.setPreferableBackend(DNN_BACKEND_OPENCV);
                net.setPreferableTarget(DNN_TARGET_CPU);
                cout << "YOLO network configured. " << endl; 
                rgb_img = rgb_input;
                depth_img = depth_input;
                boxes.clear(); 
                classIds.clear(); 
                indices.clear(); 
                confidences.clear();
                // boxes.resize(rgb_input.size());
                // classIds.resize(rgb_input.size());
                // indices.resize(rgb_input.size()); 
                // confidences.resize(rgb_input.size());
                if_detected = false;
                if_postprocessed = false;
                

                cout << "RGB and depth images inputed" << endl;
            }
            void reloadImages(Mat rgb_input, Mat depth_input){
                TODO: //每次重新加载图片时需要清除上一次的检测结果。
                rgb_img = rgb_input;
                depth_img = depth_input;
                boxes.clear(); 
                classIds.clear(); 
                indices.clear(); 
                confidences.clear();
                // boxes.resize(rgb_input.size());
                // classIds.resize(rgb_input.size());
                // indices.resize(rgb_input.size()); 
                // confidences.resize(rgb_input.size());

                if_detected = false;
                if_postprocessed = false; // 加了两个flag要注意同步的问题.
                cout << "RGB and depth images reloaded" << endl;
                // cout << "After reloading " << classIds.size() << " " << classIds[0].size() << endl;                                
            }
            ~YOLODetector(){
                net.~Net();
            }
            void LoadClassNames(const string classesFile);
            vector<string> getOutPutNames(const Net& net);
            int detect(vector<Mat>& outs, const int inpWidth, const int inpHeight);
            int postprocess(const vector<Mat> outs, const double confThreshold,
                const double nmsThreshold); // draw bounding box and display, and get final output
            int display();
            void drawPred(const vector<string> classes_label, int classId, float conf, int left, int top, int right,
                 int bottom, Mat& frame);
            void TransferResults(vector<Rect>& boxes_out, vector<int>& classIds_out, 
                                 vector<int>& indices_out, vector<float>& confidences_out,
                                 vector<string>& names_out);
            void TransferResults(vector<Rect>& boxes_out, vector<int>& classIds_out, 
                                 vector<int>& indices_out);
    };
    void YOLODetector::LoadClassNames(const string classesFile)
    {
        ifstream ifs(classesFile.c_str());
        string line;
        while (getline(ifs, line)) class_labels.push_back(line);
    }
    vector<string> YOLODetector::getOutPutNames(const Net& net)
    {
        static vector<String> names;
        if (names.empty())
        {
            //Get the indices of the output layers, i.e. the layers with unconnected outputs
            vector<int> outLayers = net.getUnconnectedOutLayers();
            
            //get the names of all the layers in the network
            vector<String> layersNames = net.getLayerNames();
            
            // Get the names of the output layers in names
            names.resize(outLayers.size());
            for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
        }
        return names;                
    }
    int YOLODetector::detect(vector<Mat>& outs, const int inpWidth, const int inpHeight){
        // Input: RGB & depth images
        // Output: outs
        // Pre-check
        if (rgb_img.size() != depth_img.size())
        {
            printf("number of RGB images doesn't equal number of Depth images");
            return 1;
        }
        // Detection
        // uint32_t size = rgb_img.size();
        Mat rgb   = rgb_img;
        Mat depth = depth_img;
        Mat blob, frame;
        // cout << "rgb: " << rgb.size << endl; // debug
        blobFromImage(rgb, blob, 1/255.0, Size(inpWidth, inpHeight),
            Scalar(0, 0, 0), true, false);
        
        // Core Procedure
        net.setInput(blob);
        // cout << "blob: " << blob.size << endl; // debug
        net.forward(outs, getOutPutNames(net));
        // cout << "outs: " << outs.size() << endl; // debug

        // Efficiency Estimation
        vector<double> layerTime;
        double freq = getTickFrequency() / 1000; // seconds
        double t = net.getPerfProfile(layerTime) / freq;
        string timelabel = format("Inference time for a frame: %.2f ms", t);
        putText(rgb, timelabel, Point(0, 15), FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 0, 255));

        // cout << "After Detecting " << classIds.size() << " " << classIds[0].size() << endl;    
        if_detected = true;
        return 0;
    }
    int YOLODetector::postprocess(const vector<Mat> outs, const double confThreshold, const double nmsThreshold){
        // Input: outs (frames outputed from net model)
        // Output: classIds, confidences, boundingboxes
        if (if_detected == false){
            printf("PostProcessError: Not detected yet\n");
            return 2;
        }
        if (outs.size() == 0)
        {
            printf("PostProcessError: None object detected\n");
            return 1;
        }
        cout << "Postprocess: pre-check." << endl; // debug
        Mat frame = rgb_img;
        // cout << "Processing on " << k << endl;
        for (size_t i = 0; i < outs.size(); i++)
        {
            // Scan through all the bounding boxes output from the network and 
            // keep only the ones with high confidence scores. Assign the box's
            // class label as the class with the highest score for the box. 
            vector<Mat> out = outs;
            float* data = (float*)out[i].data;
            cout << "Processing on " << i << "th output. " << endl;
            cout << "out size: " << out[i].rows << " " << out[i].cols << endl;
            for (int j = 0; j < out[i].rows; ++j, data+= out[i].cols) 
            {
                Mat scores = out[i].row(j).colRange(5, out[i].cols);
                Point classIdPoint;
                double confidence;                
                // Get the value and location of the maximum score
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                // cout << "minMaxLoc done." << endl;         
                if (confidence > confThreshold)
                {
                    cout << "Higher than confThresh found! " << j << endl; 
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;
                    cout << "Size done." << endl;
                    cout << classIds.size() << endl;
                    cout << "Loc: " << classIdPoint.x << classIdPoint.y << endl;
                    classIds.push_back(classIdPoint.x);
                    cout << "CLassId done." << endl;
                    confidences.push_back((float)confidence);
                    cout << "Confi done." << endl;
                    boxes.push_back(Rect(left, top, width, height));
                    cout << "box done. " << endl;
                }            
            }      
            cout << " results size: " << classIds.size() << endl;
            cout << "Processing NMS" << endl;
            dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);            
        }

        if_postprocessed = 1;
        return 0;
    }
    int YOLODetector::display(){
        // Pre-Check
        if (if_postprocessed == false){
            printf("DisplayError: Not postprocessed yet\n");
            return 2;
        }
        vector<int> indi = indices;
        vector<Rect> bo = boxes;
        Mat frame = rgb_img;
        // Mat frame = depth_img[k];
        for (size_t i = 0; i < indi.size(); i++){
            int idx = indi[i];
            Rect box = bo[idx];
            drawPred(class_labels, classIds[idx], confidences[idx], box.x, box.y,
                box.x + box.width, box.y + box.height, frame);
            cout << "Bounding box drawn. " << endl;
        }
        imshow("Detected Object", frame);            
        resizeWindow("Detected Object", 320, 240);
        waitKey(0);
    }
    void YOLODetector::drawPred(const vector<string> classes_label, int classId, float conf, int left,
         int top, int right, int bottom, Mat& frame)
    {
        rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));

        std::string label = format("%.2f", conf);
        if (!classes_label.empty()) {
            CV_Assert(classId < (int)classes_label.size());
            label = classes_label[classId] + ": " + label;
        }

        int baseLine;
        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        top = max(top, labelSize.height);
        rectangle(frame, Point(left, top - labelSize.height),
            Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
        putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
        cout << "Boundingbox: " << label << endl;
    }   
    void YOLODetector::TransferResults(vector<Rect>& boxes_out, vector<int>& classIds_out,
         vector<int>& indices_out, vector<float>& confidences_out, vector<string>& names_out)
    {
        boxes_out = boxes;
        classIds_out = classIds;
        indices_out =  indices;
        confidences_out = confidences;
        for(int i = 0; i < classIds.size(); i++)
        {
            names_out.push_back(class_labels[classIds[i]]);
        }
    }
    void YOLODetector::TransferResults(vector<Rect>& boxes_out, vector<int>& classIds_out,
         vector<int>& indices_out)
    {
        boxes_out = boxes;
        classIds_out = classIds;
        indices_out =  indices;
    }
}