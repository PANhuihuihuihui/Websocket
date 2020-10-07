#ifndef FACEDETECTION_H
#define FACEDETECTION_H

#include <QtCore>
#include <QWidget>
#include <QTime>
#include <QDate>
#include <QImageWriter>
#include <QTime>
#include <QDate>
#include "Datatype.h"
#include "Gaussian.h"
#include "MyLabel.h"
#include <chrono>

#include <opencv2/opencv.hpp>
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/string_util.h"
#include "tensorflow/lite/examples/label_image/get_top_n.h"

using namespace tflite;

class FaceDetection : public QThread
{
    Q_OBJECT
public:
    int frameID; // record total no. of frames
    int peopleFrames; // record how many frames have detected face(s)
    int emptyFrames; // record how many frames have no detected face
    constexpr static int WIDTH=160, HEIGHT=120; // image size, 160x120 for Lepton 3
    int lastID; // personID of the last person
    std::list<samePerson> trackedPeople; // track people across frames
    float imageResizeRatio; // for upscaling image for display locally

    // for recording data to disk
    bool recordData; // toggle data writing
    int outputPathCounter; // save image to a new directory per xx frames
    int peopleOutputPathCounter;
    QString basePath; //path containing log.csv and directories for images
    QString imagePath; //path for storing images without detected faces
    QString peopleImagePath; //path for storing images with faces
    QFile logFile;
    QTextStream logFileStream;
    //QImageWriter imageWriter; // for saving image as .pgm file

    /* Data Logging (Multi-class) */
    // Counters for each type of data to log
    std::map<std::string, unsigned int> logDataCntsMap; 
    // Number of data files to store in each folder
    unsigned short int nFilesPerDir;
    // Root of log directory
    std::string logRootDir;
    // Record the start date of data logging, to prepare for creating
    // separated folders for multiple logging sessions started in a day.
    std::string logStartDate;
    // To count the number of logging started in the day
    unsigned short int logStartDateCnt; 
    
    /* Clock display */
    QDate clock_date; // To get date
    QTime clock_time; // To get time 

    /* BBox Display Colors */
    // QRect for colorbar display
    QRect colorbarRect; 
    // Colors (RGB) to use
    std::vector<QColor> bboxGradientColors;
    // Stop positions of the colors in the color gradient
    std::vector<float> bboxColorGradientStopPos; 
    // Temperatures corrto the color stop positions
    std::vector<float> bboxColorGradientStopTemps; 
    // Arbitrary "values" corr. to the color stop positions
    std::vector<float> bboxColorGradientStopValues;
    // Color gradient for colorbar display
    QLinearGradient bboxColorGradient;


    // Gaussian Mixture Model related
    bool normalizeTemp;
    std::vector<Gaussian> GMMs;
    const int numOfGMMs;
    std::vector<float> bboxSizeForComparison;

    //workaround for camera's temperature fluctuation issue
    //store "potential background pixel" position. Help detect camera read temperature fluctuations
    std::vector<pixPos> bgPixels;
    std::vector<Gaussian> bgGMMs; // one GMM per bgPixel
    int numOfBgGMMs;

    //calculate fps
    int frame, fps;
    std::chrono::steady_clock::time_point startTime;

    // for machine learning
    std::unique_ptr<FlatBufferModel> faceDetectModel;
    std::unique_ptr<FlatBufferModel> maskDetectModel;
    // face detection
    tflite::ops::builtin::BuiltinOpResolver resolver;
    std::unique_ptr<Interpreter> interpreter;

    FaceDetection(bool recordData, bool normalizeTemp, float imageResizeRatio, bool showTemp);

    //methods for face detections
    void findForeheadTemp(std::list<BBox>&, float*);
    void findForeheadTempByMedian(std::list<BBox>&, float*);
    // use BBox size to help find most suitable GMM to model data
    void calcBBoxSize(std::list<BBox>&);
    //void templateMatching(QImage, float*, std::list<BBox>&);
    // Order bounding boxes by confidence scores in desc order
    void orderBBoxByConfidenceDesc(std::list<BBox>&);
    //remove boxes with strange temperature
    void removeNonHumanBoxes(std::list<BBox>&);
    //remove overlapped squares(with certain threshold)
    void nonMaxSuppression(std::list<BBox>&, const float&);

    //methods for after bounding boxes are confirmed

    // deal with camera temperature fluctuation issue
    // adjust detected bounding boxes' temperature
    void offsetTemp(std::list<BBox>&, float*);
    // Use last frame and current frame BBox to update trackedPeople
    // Also detect fever and update GMM
    void trackPeopleAndUpdateGMM(std::list<BBox>&);
    // drawing bounding box, etc. for display
    void drawOnCVImage(const std::list<BBox>&, cv::Mat&);
    void drawOnQImage(const std::list<BBox>&, QImage&, float*);

    void recordToDisk(unsigned char*, const std::list<BBox>&);

    /* Detection Results Logging */
    bool logDetectionResults(unsigned char*, const std::list<BBox>&);
    
    /* Alert System */
    int alertAbnormality(const std::list<samePerson>&); 
    std::chrono::steady_clock::time_point alertStartTime;
    int alertInterval; // Invertal in between sending two alert ON signals (in ms)

    void alertOn(int); // Turn on alert
    void alertOff(int); // Turn off alert

    /* Display-Related */
    bool showTemp; // Whether to show temperature for bbox

    /* Testing */
    // Load detection model
    std::unique_ptr<Interpreter> loadModel(std::string);
    // Read image from file
    cv::Mat readImage(std::string, std::string, cv::Size);
    // Perform detection on images in directory
    void predictFromDir(std::string, std::string);
    // Draw detection resutls on CV image
    void drawDetectionOnCVImage(const std::list<BBox>&, cv::Mat&);
    void drawDetectionOnQImage(
      const std::list<BBox>&, QImage&, float*
    );
    // convert the Qt image to CV:Mat then to base64 string for web server 
    static std::string base64Encode(const unsigned char* Data, int DataByte){
        //encode table
        const char EncodeTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        //return value
        std::string strEncode;
        unsigned char Tmp[4] = { 0 };
        int LineLength = 0;
        for (int i = 0; i < (int)(DataByte / 3); i++)
        {
            Tmp[1] = *Data++;
            Tmp[2] = *Data++;
            Tmp[3] = *Data++;
            strEncode += EncodeTable[Tmp[1] >> 2];
            strEncode += EncodeTable[((Tmp[1] << 4) | (Tmp[2] >> 4)) & 0x3F];
            strEncode += EncodeTable[((Tmp[2] << 2) | (Tmp[3] >> 6)) & 0x3F];
            strEncode += EncodeTable[Tmp[3] & 0x3F];
            if (LineLength += 4, LineLength == 76) { strEncode += "\r\n"; LineLength = 0; }
        }
        //encode the rest data
        int Mod = DataByte % 3;
        if (Mod == 1)
        {
            Tmp[1] = *Data++;
            strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
            strEncode += EncodeTable[((Tmp[1] & 0x03) << 4)];
            strEncode += "==";
        }
        else if (Mod == 2)
        {
            Tmp[1] = *Data++;
            Tmp[2] = *Data++;
            strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
            strEncode += EncodeTable[((Tmp[1] & 0x03) << 4) | ((Tmp[2] & 0xF0) >> 4)];
            strEncode += EncodeTable[((Tmp[2] & 0x0F) << 2)];
            strEncode += "=";
        }
    
    
        return strEncode;
    }

    static std::string Mat2Base64(const cv::Mat &img, std::string imgType)
    {
        //Mat to base64
        std::string img_data;
        std::vector<uchar> vecImg;
        std::vector<int> vecCompression_params;
        vecCompression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        vecCompression_params.push_back(90);
        imgType = "." + imgType;
        cv::imencode(imgType, img, vecImg, vecCompression_params);
        img_data = base64Encode(vecImg.data(), vecImg.size());
        return img_data;
    }

    std:: string QtImg_to_string(QImage &img){
        cv::Mat img_cv = (img.height(), img.width(), cv::CV_8UC1, img.bits(), img.bytesPerLine());
        return Mat2Base64(img_cv,"png");
    }


signals:
    void sendData(std::string); //for network thread
    void updateImage(QImage);
    // void sendHeartBeat();
    

public slots:
    // image and temperature value per pxiel
    void detectFaces(QImage, unsigned char*, float*, bool);
};

#endif // FACEDETECTION_H

