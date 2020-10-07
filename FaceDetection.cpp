#include "FaceDetection.h"
#include "MLFaceDetection.cpp"

#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include <algorithm>
#include <limits>
#include <set>
#include <cmath>
#include <string>
#include <QPainter>

// Controlling GPIO pins of RPi
#include <wiringPi.h> 

// Dirent (for listing files under a directory)
#include <dirent.h>

// https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html
using namespace cv;

FaceDetection::FaceDetection(bool recordData, bool normalizeTemp, float resize, bool showTemp) :
    numOfGMMs(3), recordData(recordData),
    normalizeTemp(normalizeTemp), imageResizeRatio(resize), showTemp(showTemp)
{
    peopleFrames = 0;
    emptyFrames = 0;
    frameID = 0;
    lastID = 0;

    // Create Gaussian Mixture Model for different bounding box sizes
    // to model measured forehead temperature
    for (int i=1; i<=numOfGMMs; i++){
        GMMs.push_back( Gaussian() );
    }

    // Define bounding box sizes for calcBBoxSize()
    bboxSizeForComparison.push_back(1000);
    bboxSizeForComparison.push_back(450);
    //bboxSizeForComparison.push_back(150); //##### TODO: Test for a suitable size

        for (auto &i: bboxSizeForComparison){
            std::cout<<"box size "<<i<<endl;
        }

    // workaround for camera's temperature fluctuation issue
    // Define possible background pixel locations
    // {x, y}
    bgPixels.push_back({10,10});
    bgPixels.push_back({150,10});
    bgPixels.push_back({10,110});
    bgPixels.push_back({150,110});
    bgPixels.push_back({80,10});
    bgPixels.push_back({10,60});
    bgPixels.push_back({80,110});
    bgPixels.push_back({150,60});
    numOfBgGMMs = bgPixels.size();
    for (int i=1; i<=numOfBgGMMs; i++){
        bgGMMs.push_back( Gaussian(1,1,1.5,1,1) );
    }
    
    // TODO (20200906): Deprecated logging procedures, to delete.
    //// create directory for data recording
    //if (recordData){
    //    outputPathCounter = 0;
    //    peopleOutputPathCounter = 0;
    //    for (int i=0; i<10000; i++){
    //        basePath = QString("output/out%1").arg(i);
    //        QDir dir(basePath);
    //        if (!dir.exists()){
    //            dir.mkpath(".");
    //            break;
    //        }
    //    }
    //    logFile.setFileName( QString("%1/log.csv").arg(basePath) );
    //    logFile.open(QIODevice::WriteOnly | QIODevice::Text);
    //    logFileStream.setDevice(&logFile);
    //    logFileStream<<"frameID,x1,x2,y1,y2,BBoxSize,temperature,ifFever,withMask\n";

    //    // set parameters for imageWriter (to save image as .pgm file)
    //    //imageWriter.setFormat("PGM");
    //}

    /* Initialization for Data Logging */ 
    // Set classes of data to log and initialize their counters
    logDataCntsMap.insert({"all", 0}); // Total number of saved frames
    logDataCntsMap.insert({"bkg", 0}); // Class: background
    logDataCntsMap.insert({"face_nomask", 0}); // Class: face without mask
    logDataCntsMap.insert({"face_mask", 0}); // Class: face with mask

    logDataCntsMap.insert({"others", 0}); // Class: others
    logDataCntsMap.insert({"multiple", 0}); // Images capturing objects of multiple classes

    // Set number of file to save in one subdir
    nFilesPerDir = 1000;
    // Root of log directory
    logRootDir = "./log";
    // Record the start date of logging, data will be stored in a folder
    // named by the "date", e.g. "20200901",
    logStartDate = QDate::currentDate()
        .toString("yyyyMMdd").toStdString();
    // If mutliple logging sessions have
    // been started at a day, store data in a folder named by "date" +
    // "counter", e.g. "20200901_1", "20200901_2".
    logStartDateCnt = 0;
    if (QDir((logRootDir+"/"+logStartDate).c_str()).exists()) {
        // If folder, say, "20200901" exists, count the number of 
        // folders created in the same day. and use the count+1 as a 
        // suffix of the folder to be created for the new data 
        // logging session.
        
        // Check if folder, say, "20200901_x" exists, where x=1,2,...
        // Stops at a x when "20200901_x" does not exist.
        logStartDateCnt = 1;

        std::string tempPath;
        while (true) {
            tempPath = 
                logRootDir+"/"+logStartDate+"_"+to_string(logStartDateCnt);
            printf("Check: %s\n", tempPath.c_str());
            if (QDir(tempPath.c_str()).exists()) {
                ++logStartDateCnt;
            } else {
                break;
            }
        }
    }

    /* Initialization of Alert System */
    // Last time of sending alert On signal
    alertStartTime = std::chrono::steady_clock::now();
    // Interval in between sending two alert ON signals (in ms)
    alertInterval = 5000; 

    /**********/

    // TODO: Write header line to log.csv (Current: header omitted)
    // - To write header for log.csv created after the constructure of 
    //   FaceDetection class (e.g. when date changes during a logging 
    //   session)
    // Prepare the main log file
    //QString mainLogPath = logStartDateCnt == 0 ?
    //  QString(%1/%2/log.csv)
    //      .arg(logRootDir.c_str()).arg(logDate.c_str()) :
    //  QString(%1/%2/log.csv)
    //      .arg(logRootDir.c_str())
    //      .arg((logDate+to_string(LogStartDateCnt)).c_str());
    //logFile.setFileName( mainLogPath );
    //logFile.open(QIODevice::WriteOnly | QIODevice::Text);
    //logFileStream.setDevice(&logFile);
    //logFileStream<<"frameID,bboxID,timestamp,class,bboxSize,temperature,ifFever,withMask\n";
    //logFile.close(); // close file after writing

    /* BBox Display Colors */
    // TODO: Define colors in external config file and init colors by 
    // reading values in the config file. 
    
    // Prepare a QRect to display a colorbar on the screen
    // to display the color gradient used to display bbox colors
    colorbarRect = QRect(2*imageResizeRatio, 30*imageResizeRatio,
                         4*imageResizeRatio, 60*imageResizeRatio);
    // Define colors (RGB) to use
    bboxGradientColors.push_back(QColor(0, 255, 0));
    bboxGradientColors.push_back(QColor(255, 165, 0));
    bboxGradientColors.push_back(QColor(255, 0, 0));
    // Define stop positions of the colors in the color gradient
    bboxColorGradientStopPos.push_back(0);
    bboxColorGradientStopPos.push_back(0.75);
    bboxColorGradientStopPos.push_back(1);
    // Define temperatures corresponding to the stop 
    // positions
    bboxColorGradientStopTemps.push_back(35.0);
    bboxColorGradientStopTemps.push_back(39.0);
    bboxColorGradientStopTemps.push_back(40.0);
    // Refer to the color stop positions using 
    // user-defined values
    bboxColorGradientStopValues.push_back(0.0);
    bboxColorGradientStopValues.push_back(0.5);
    bboxColorGradientStopValues.push_back(0.6);
    // Construct color gradient for colorbar display
    // Color changes from bottom to top
    bboxColorGradient = QLinearGradient(
        colorbarRect.bottomRight(), colorbarRect.topLeft()
    );
    for (int i=0; i<bboxGradientColors.size(); ++i) {
        // For each color specified earlier, set color to gradient by:
        //   setColorAt(stop position, color (RGB))
        bboxColorGradient.setColorAt(
            bboxColorGradientStopPos[i], 
            bboxGradientColors[i]
        );
    }


    frame=0; fps=0;
    startTime = std::chrono::steady_clock::now();

    // 1-stage Face Detection YOLO model (2 classes: face_nomask, face_mask)
    faceDetectModel = tflite::FlatBufferModel::BuildFromFile(
                "./model/datav2_2_2/yolov4-tiny-custom-160-fp32/model.tflite");

    // for face detection
    InterpreterBuilder(*faceDetectModel, resolver)(&interpreter);
    // Resize input tensors, if desired.
    interpreter->AllocateTensors();
    interpreter->SetAllowFp16PrecisionForFp32(true);
    interpreter->SetNumThreads(3);
}

// Qt slot function. Called after receiving image from camera
void FaceDetection::detectFaces(QImage image, unsigned char* grayPix, float* pixTemp, bool serverAlive) {
    /* Pre-Set Parameters */
    float overlapThreshold = 0.12; //  Threshold to bboxes IoU used in NMS
    /* Pre-Set Parameters */

    std::list<BBox> bbox; // bounding boxes of detected faces

    //deepcopy grayscale array to openCV format
    Mat cvImg = Mat(HEIGHT, WIDTH, CV_8UC1, grayPix).clone();
    Mat channel3cvImg;
    Mat in[] = {cvImg, cvImg, cvImg};
    cv::merge(in, 3, channel3cvImg);
    // Face Detection YOLO model
    MLFaceDetectYOLO(interpreter, channel3cvImg, bbox);
    // Order bounding boxes by confidence scores (desc order)
    orderBBoxByConfidenceDesc(bbox);

    // process detected bounding boxes
    findForeheadTemp(bbox, pixTemp);
    removeNonHumanBoxes(bbox);
    nonMaxSuppression(bbox, overlapThreshold);
    calcBBoxSize(bbox);
    offsetTemp(bbox, pixTemp);
    trackPeopleAndUpdateGMM(bbox);

    // now bounding boxes are finalized
    for(samePerson const& person: trackedPeople){
        std::cout<<"personID,boxsize,squareidx,framecount,disappear,inThisFrame,x1,y1\n";
        std::cout<<person.personID<<"------"<<person.bboxSize<<"------"<<person.squareIdx
           <<"------"<<person.frameCount<<"------"<<person.disappearCount<<"------"<<person.inThisFrame<<
             "------"<<person.x1<<"------"<<person.y1<<"\n";
    }
    //std::cout<<trackedPeople.size()<<" <---------------------size\n";

    // Data logging (image, bboxes)
    if (recordData) {
        logDetectionResults(grayPix, bbox);
    }

    // Produce alert(s) based on detected bboxes
    alertAbnormality(trackedPeople);
  
    // TODO (20200906): Remove old log procedures below
    //// record data to disk
    //if (recordData){
    //    if (bbox.size()>0){
    //        if (peopleFrames++ % 1024 == 0){
    //            peopleImagePath = QString("%1/people%2").arg(basePath).arg(peopleOutputPathCounter++);
    //            QDir dir(peopleImagePath);
    //            dir.mkpath(".");
    //        }

    //    }
    //    else if (emptyFrames++ % 1024 == 0){
    //        // store images in separate directory per xx frames
    //        imagePath = QString("%1/time%2").arg(basePath).arg(outputPathCounter++);
    //        QDir dir(imagePath);
    //        dir.mkpath(".");
    //    }
    //    recordToDisk(grayPix, bbox);
    //}

    // upscale image for local display
    if (imageResizeRatio != 1)
        image = image.scaled(WIDTH*imageResizeRatio, HEIGHT*imageResizeRatio, Qt::KeepAspectRatio,Qt::SmoothTransformation);
    drawOnQImage(bbox, image, pixTemp);

    int i = 0 ;
    for (Gaussian& gmm: GMMs){
        std::cout<<"GGM "<<i<<" ";
        gmm.print(); 
        i++;
    }

    // prepare data for network transmission
    // data size = (image) + (int) (numOfBBox) +
    //      n*[ (unsigned char) 4 coordinates + float temp + (unsigned char) ifFever+ withMask]
    int bboxSize = bbox.size();
    unsigned int dataSize = WIDTH*HEIGHT + 4 + bboxSize* (4+4+1+1);
    //void* transmitData = malloc( dataSize );
    //memcpy(transmitData, grayPix, WIDTH*HEIGHT); // copy grayscale image
    //unsigned char* ptr = (unsigned char*)transmitData + WIDTH*HEIGHT;
    memcpy(ptr, &bboxSize, 4); // how many detected bboxes
    ptr+=4;
    // now copy bounding box information
    for (BBox const& box: bbox){
        unsigned char* coordinate;
        coordinate = ( (unsigned char*) & box.x1 );
        memcpy(ptr++, coordinate, 1);
        coordinate = ( (unsigned char*) & box.y1 );
        memcpy(ptr++, coordinate, 1);
        coordinate = ( (unsigned char*) & box.x2 );
        memcpy(ptr++, coordinate, 1);
        coordinate = ( (unsigned char*) & box.y2 );
        memcpy(ptr++, coordinate, 1);
        float temp;
        if (normalizeTemp)
            temp = GMMs.at(box.bboxSize).normalizeTempByMean(box.temp);
        else
            temp = box.temp;
        memcpy(ptr, &temp, 4);
        ptr+=4;
        unsigned char ifFever = box.ifFever;
        memcpy(ptr++, &ifFever, 1);
    }

    if (serverAlive)
        // emit sendHeartBeat();
    //emit sendData( (unsigned char*) transmitData, dataSize);
    emit updateImage(image);
    std::string send_data = QtImg_to_string(image);
    emit sendData(send_data);
    frameID++;
    std::cout<<"fps: "<<fps<< "\n";
    frame++;
    std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    if(milliseconds.count() >= 1000){
      fps = frame;
      frame = 0;
      startTime = std::chrono::steady_clock::now();
    }

}

void FaceDetection::orderBBoxByConfidenceDesc(std::list<BBox>& bbox) {
    /* Order Bounding Boxes by Confidence in Descending Order
     * 
     */
    // Use a "greater than" comparison to order the list of bboxes
    // by their confidence scores.
    bbox.sort([](const BBox &b1, const BBox &b2){
        return b1.confidence > b2.confidence;
    });
    // TODO: Remove the debug messages below
    printf("----- BBoxes -----\n");
    printf("#BBox: %d\n", bbox.size());
    if (bbox.size()>0) {
        printf("Confidence scores (classes) in desc order: \n");
        for (auto b : bbox) {
            printf("%.4f (%d) | ", b.confidence, b.bboxClass);
        }
        printf("\n----------\n");
    }
}

void FaceDetection::findForeheadTemp(std::list<BBox>& bbox, float* pixTemp){
    // search max temp in top (2/3) and middle (3/5) region of image
    for (BBox & box: bbox){
        float maxTemp = 0;

        int yAxisIteration = (box.y2- box.y1)/2;
        int xAxisIteration = 3 * (box.x2- box.x1)/5;

        // array location of top left of search region
        int arrayIndex = (box.y1 * WIDTH + box.x1) + ((box.x2- box.x1)/5);

        for (int y=0; y<yAxisIteration; y++){
            for (int x=0; x<xAxisIteration; x++){
                float temp = pixTemp[arrayIndex+x];
                if (temp > maxTemp){
                    maxTemp = temp;
                }
            }
            arrayIndex += WIDTH;
        }
        //maxTemp = pixTemp[60 * WIDTH + 30]; // for debug
        box.temp = maxTemp;
        std::cout<<"In temp method:  ----- "<<maxTemp<<"\n";
    }
}

void FaceDetection::findForeheadTempByMedian(std::list<BBox>& bbox, float* pixTemp){
    // search max temp in top (2/3) and middle (3/5) region of image
    for (BBox & box: bbox){
        std::vector<float> tempVector; // record max xx temperature points
        constexpr int maxData = 200;

        int yAxisIteration = 2* (box.y2- box.y1)/3;
        int xAxisIteration = 3* (box.x2- box.x1)/5;

        // array location of top left of search region
        int arrayIndex = (box.y1 * WIDTH + box.x1) + (box.x2- box.x1)/5;

        for (int y=0; y<yAxisIteration; y++){
            for (int x=0; x<xAxisIteration; x++){
                float temp = pixTemp[arrayIndex+x];
                if (tempVector.size() < maxData)
                    tempVector.push_back(temp);
                else {
                    std::sort(tempVector.begin(), tempVector.end(), greater<float>());
                    if (temp > tempVector.back()){
                        // replace lowest of the recorded data
                        tempVector.pop_back();
                        tempVector.push_back(temp);
                    }
                }

            }
            // finished scanning this row. Advance to next row
            arrayIndex += WIDTH;
        }
        // get middle element in tempSet
        auto itr = tempVector.begin();
        std::advance(itr, tempVector.size()/2);
        box.temp = *itr;
        std::cout<<"In temp method:  ----- "<<*itr<<"\n";
    }
}

void FaceDetection::removeNonHumanBoxes(std::list<BBox>& bbox){
    for (auto boxItr= bbox.begin(); boxItr!= bbox.end(); boxItr++){
        if (boxItr->temp < 25 or boxItr->temp > 40){
            boxItr = bbox.erase(boxItr);
        }
    }
}

void FaceDetection::nonMaxSuppression(std::list<BBox>& bboxes, const float &overlapThreshold){
    /* Non-Maximum Suppression on Predicted Bounding Boxes 
     * 
     * Assumed:
     * - All bboxes are ordered by their confidence scores in desc 
     *   order.
     * 
     * TODO's:
     * - To explore for more efficient implementation (current: O(n^2), 
     *   n=#bboxes). Maybe consider adopting TF's NMS function.
     * 
     */
    
    // To store the reference bbox to compare (with higher confidence, 
    // init to bbox.begin())
    auto bbox_ref = bboxes.begin(); 
    // To store the bbox used to compare with bbox_cuur
    //auto bbox_curr = std::next(bbox_ref, 1);
    std::list<BBox>::iterator bbox_curr;
    
    // Use all bboxes (not yet removed) as reference once 
    while (bbox_ref != bboxes.end()) {
        // Compare reference bbox to all subsequent bboxes,
        // and remove bboxes with IoU higher than threshold.
        bbox_curr = std::next(bbox_ref, 1);
        while (bbox_curr != bboxes.end()) {
            // coordinates for intersection area
            int startX = std::max(bbox_ref->x1, bbox_curr->x1);
            int startY = std::max(bbox_ref->y1, bbox_curr->y1);
            int endX = std::min(bbox_ref->x2, bbox_curr->x2);
            int endY = std::min(bbox_ref->y2, bbox_curr->y2);
            
            float intersectArea = std::max(0.0f, endX - startX + 0.001f)
                                * std::max(0.0f, endY - startY + 0.001f);
                                
            float totalArea =
              (bbox_ref->x2 - bbox_ref->x1)*(bbox_ref->y2 - bbox_ref->y1)
            + (bbox_curr->x2 - bbox_curr->x1)*(bbox_curr->y2 - bbox_curr->y1)
            - intersectArea;

            float overlapRatio = float(intersectArea) / totalArea;
            
            if (overlapRatio > overlapThreshold){
                bbox_curr = bboxes.erase(bbox_curr);
                std::cout<<"NMS      deleted 1 box\n";
            } else{
                bbox_curr++;
            }                
        }
        // Use the next remaining bbox as the reference
        ++bbox_ref;
    }
}

void FaceDetection::calcBBoxSize(std::list<BBox>& bbox){
    for (BBox & box: bbox){
        int boxArea = (box.y2 - box.y1)*(box.x2 - box.x1);
        for (int i=0; i <= numOfGMMs-2; i++){
            if (boxArea > bboxSizeForComparison.at(i)){
                box.bboxSize = i;
                break;
            }
            else if (i == numOfGMMs-2) // last
                box.bboxSize = numOfGMMs-1;
        }
    }
}

void FaceDetection::offsetTemp(std::list<BBox>& bbox, float* pixTemp){
    // update bgPixels' GMM
    int i = 0;
    std::vector<float> diffVector;
    float difference;
    for (const pixPos& pix: bgPixels){
        float bgTemp = pixTemp[pix.y * WIDTH + pix.x];
        if (bgTemp > 15 && bgTemp < 40){
            // bgGMMMean = 0 if bgGMM has no cluster (no data)
            float bgGMMMean = bgGMMs.at(i).firstClusterMean();
            // dont update if this "background pixel" has too much elevated temperature

            float absDiff = std::abs(bgTemp - bgGMMMean);
            // reset bgGMM if the difference is abnormally large
            if (absDiff >= 6 && bgGMMMean != 0){
                bgGMMs.at(i).resetGMM();
                difference = 0;
                for (int i=0; i<numOfGMMs; i++) {
                    // #####
                    printf("Before reset GMM %d...\n", i);
                    GMMs.at(i).resetGMM();
                    printf("After reset GMM %d...\n", i);
                }
            }
            else if (absDiff >= 3.5 && bgGMMMean != 0)
                // assume bgPixel is pointing at non background
                // ignore this bgPixel for this frame
                ;
            else if (absDiff >= 2 && bgGMMMean != 0)
                // difference is large. Avoid updating bgGMM.
                // But still apply offset to detect faces
                difference = bgTemp - bgGMMMean;
            else
                difference = bgGMMs.at(i).update(bgTemp, true, false);
        }
        if (difference != 0)
            diffVector.push_back(difference);
        i++;
    }

    float medianDiff;
    if (diffVector.size() == 0)
        medianDiff = 0;
    else{
        std::sort(diffVector.begin(), diffVector.end());
        auto itr = diffVector.begin();
        std::advance(itr, diffVector.size()/2);
        medianDiff = *itr;

        for (BBox& box: bbox){
            float mean, sd;
            if (GMMs.at(box.bboxSize).clusters.size() == 0){
                mean = 0; sd = 0;
            } else{
                auto itr = GMMs.at(box.bboxSize).clusters.begin();
                mean = (*itr).mean;
                sd = (*itr).sd;
            }
                /*
                float staticPointTemp = pixTemp[60 * WIDTH + 30];
                logFileStream << QString("%1\t%2\n").arg(staticPointTemp)
                                 .arg(staticPointTemp - medianDiff);
                logFileStream << QString("%1\t%2\t%3\t%4\n").arg(box.temp)
                                 .arg(box.temp - medianDiff).arg(mean).arg(sd);
                if (frameID % 16 == 0)
                    logFileStream.flush();
                */

            box.temp -= medianDiff;
        }
    }
    std::cout<<"-------------------------------TempOffset= -" << medianDiff<<"\n";
}

/* TODO (20200902): To remove (only support one detected object)
void FaceDetection::nonMaxSuppression(std::list<BBox>& bbox, const float &overlapThreshold){
    auto itr=bbox.begin();
    itr++;
    while (itr!= bbox.end()){
        // assume list is sorted by Ncc value in desc order
        // first element is most likely a correctly detected face
        auto head = bbox.begin(); 
        auto nestedItr = itr;
        while(nestedItr!=bbox.end()){
            // coordinates for intersection area
            int startX = std::max(head->x1, nestedItr->x1);
            int startY = std::max(head->y1, nestedItr->y1);
            int endX = std::min(head->x2, nestedItr->x2);
            int endY = std::min(head->y2, nestedItr->y2);

            float intersectArea = std::max(0.0f, endX - startX + 0.001f)
                                * std::max(0.0f, endY - startY + 0.001f);
            
            float totalArea =
              (head->x2 - head->x1)*(head->y2 - head->y1)
            + (nestedItr->x2 - nestedItr->x1)*(nestedItr->y2 - nestedItr->y1)
            - intersectArea;

            float overlapRatio = float(intersectArea) / totalArea;
            
            if (overlapRatio > overlapThreshold){
                nestedItr = bbox.erase(nestedItr);
                std::cout<<"NMS      deleted 1 box\n";
            } else{
                nestedItr++;
            }
       }
       itr = nestedItr;
       
    }
}
*/

void FaceDetection::trackPeopleAndUpdateGMM(std::list<BBox>& bbox){
    // remember which current BBoxes contain "same person". Stores its index.
    // mark BBox as tracked. Untracked BBoxes are inserted as new people
    std::set<int> idxSet;

    for(samePerson & p: trackedPeople){
        // assume tracked people are not in this frame. Update later
        p.inThisFrame = false;

        int minDist = std::numeric_limits<int>::max();
        int bboxIdx; // index of bbox which has min distance
        int i = 0;
        auto boxItr = bbox.begin();
        auto minItr = boxItr;
        // loop over all boxes in this frame ro find the closest box
        for(; boxItr != bbox.end(); boxItr++){
            int xDist = p.x1 - boxItr->x1;
            int yDist = p.y1 - boxItr->y1;
            int dist = xDist*xDist + yDist*yDist;
            if (dist < minDist and idxSet.count(bboxIdx) == 0){
                minDist = dist; 
                bboxIdx = i; 
                minItr = boxItr;
            }
            i++;
        }
        // assume is same person if distance between them <= 350
        if (minDist <= 350){
            idxSet.insert(bboxIdx); // remember this BBox is tracked
            // update entry in trackedPeople
            p.inThisFrame = true;
            p.bboxSize = minItr->bboxSize;
            p.frameCount = (p.frameCount+1)%128;
            p.disappearCount = 0;
            p.squareIdx = bboxIdx;
            p.x1 = minItr->x1;
            p.y1 = minItr->y1;
            minItr->personID = p.personID;

            // Remember the detection results for this person
            p.classHistory.push_back(
                (*(std::next(bbox.begin(), bboxIdx))).bboxClass
            );
            p.tempHistory.push_back(
                (*(std::next(bbox.begin(), bboxIdx))).temp
            );
            p.feverHistory.push_back(
                (*(std::next(bbox.begin(), bboxIdx))).ifFever
            );
        }
    }

    // Now delete disappeared people stored in trackedPeople
    auto itr = trackedPeople.begin();
    while (itr != trackedPeople.end()){
        if (itr->inThisFrame == false){
            // allow to disappear for 2 consecutive frame at max
            itr->disappearCount += 1;
            if (itr->disappearCount > 2){
                itr = trackedPeople.erase(itr);
            } else {
                itr++;
            }
        } else{
            itr++;
        }
    }

    // Now insert new people
    // And also detect fever and update GMM while looping
    int idx=0;
    for (BBox & box: bbox){
        if (idxSet.count(idx)==0){
            lastID = (lastID+1)%1024;
            trackedPeople.push_back(samePerson{lastID, box.bboxSize, 1, 0, idx, true, box.x1, box.y1} );
            box.personID = lastID;
        }
        box.ifFever = GMMs.at(box.bboxSize).detectFever(box.temp);
        // update GMM per 4 frames (for each identified person)
        // search personID in trackedPeople
        auto itr = trackedPeople.begin();
        for (; itr != trackedPeople.end(); itr++){
            if (itr->personID == box.personID)
                break;
        }
        if (itr != trackedPeople.end() && itr->frameCount % 4 == 1)
            GMMs.at(box.bboxSize).update(box.temp, true, true);
        idx++;
    }

}

void FaceDetection::drawOnCVImage(const std::list<BBox>&bbox, Mat & cvImg){

    for (BBox const& box: bbox){
        // draw rectangles for bounding boxes
        Rect rec(box.x1, box.y1, box.x2-box.x1, box.y2-box.y1);
        Scalar color; // BGR representation
        if (box.ifFever)
            color = Scalar(3,186,252); // orange
        else
            color = Scalar(0,200,0); // green
        rectangle(cvImg, rec, color, 0.5, 8, 0);

        // write bounding box's forehead temperature
        float temp;
        if (normalizeTemp)
            temp = GMMs.at(box.bboxSize).normalizeTempByMean(box.temp);
        else
            temp = box.temp;
        char tempString [6];
        sprintf(tempString, "%.2f", temp);
        putText(cvImg, tempString, Point(box.x1,box.y1+5), FONT_HERSHEY_COMPLEX, 0.4,
                Scalar(245,135,66), 1, 8, false);

        // debug, write assigned bboxSize
        sprintf(tempString, "%i", box.bboxSize);
        putText(cvImg, tempString, Point(box.x2,box.y2-5), FONT_HERSHEY_DUPLEX, 0.3,
                Scalar(245,135,66), 1, 8, false);
    }
    putText(cvImg, "IOT Device in Testing.", Point(13,8), FONT_HERSHEY_COMPLEX_SMALL, 0.44,
            Scalar(245,135,66), 1, 8, false);
}

QColor interpolateColorRGB(std::vector<QColor> colors , std::vector<float> values, float queryValue){
    /* Get Intermediate Color in Color Gradient by Interpolation
     * 
     * Args:
     * colors: (Intermediate) colors usied in the color gradient
     *         (ordered in asc order)
     * values: position (0-1) of colors in the color gradient
     * queryValue: value to query its corresponding color
     *
     */

    // Compute the position (0-1) of query value in between 
    // the first (smallest) value and the last (largest) value
    float startValue = values.front();
    float endValue = values.back();
    if (queryValue <= startValue) {
        // If queryValue <= startValue --> p=0
        // Simply return the first color without interpolation
        return colors.front();
    } else if (queryValue >= endValue) {
        // If queryValue >= endValue --> p=1
        // Simply return the last color without interpolation
        return colors.back();
    }  
    // If startValue < queryValue < endValue 
    // --> interpolation
                                                                       
    // Determine which interval of values that the query value is
    // located at.
    // Upper bound of interval = first value not less than query value
    auto upperItr = std::lower_bound(
        values.begin(), values.end(), queryValue
    );
    // Lower bound of interval = next value of the lower bound
    // Since startValue < queryValue < endValue (otherwise the 
    // function would have exited above), lowerItr-1 must exists.
    auto lowerItr = upperItr - 1;
    // Find the position of query value in this interval
    float p = (queryValue-*lowerItr) / (*upperItr-*lowerItr);
    
    // Get colors at the two ends of this interval
    QColor startColor = 
        colors[std::distance(values.begin(), lowerItr)];
    QColor endColor = 
        colors[std::distance(values.begin(), upperItr)];

    printf("Lower: %.3f | Upper: %.3d | Query: %.3f | Pos: %.3f\n",
            *lowerItr, *upperItr, queryValue, p);
    printf("Start Color: (%d,%d,%d) | End Color: (%d,%d,%d)\n",
          startColor.red(), startColor.green(), startColor.blue(),
          endColor.red(), endColor.green(), endColor.blue()
    );

    // Return the interpolated color
    return QColor(
        (int)((1-p)*startColor.red() + p*endColor.red()),
        (int)((1-p)*startColor.green() + p*endColor.green()),
        (int)((1-p)*startColor.blue() + p*endColor.blue()) 
    );
};

void FaceDetection::drawOnQImage(const std::list<BBox>&bbox, QImage & image, float* pixTemp){
    QPainter painter (&image);
    painter.setRenderHints(QPainter::TextAntialiasing);
    QPen pen;
    for (BBox const& box: bbox){
        // Skip bbox if for objects of classes not to display
        if (!box.toDisplay) {
            printf("##### Class \"Back\" detected. Skip display. #####\n");
            
            continue;
        }

        // Get coordinates of bbox (and rescale according to display
        // dimensions
        int x1 = box.x1 * imageResizeRatio;
        int y1 = box.y1 * imageResizeRatio;
        int x2 = box.x2 * imageResizeRatio;
        int y2 = box.y2 * imageResizeRatio;

        // draw rectangles for bounding boxes
        QRect rect(x1, y1, x2-x1, y2-y1);
        
        // Draw bbox with discreet colors
        //if (box.ifFever)
        //    pen.setColor(QColor(252,186,3));
        //else
        //    pen.setColor(Qt::green);
        
        // Draw bbox with colors based on temp
        //pen.setColor(interpolateColorRGB(
        //    bboxGradientColors, 
        //    bboxColorGradientStopTemps,
        //    box.temp
        //));
        
        // Compute how many sd's the temperature is deviating from GMM
        // cluster mean. 
        float tempDev = GMMs.at(box.bboxSize).computeDeviation(box.temp);
        float gmmSD = GMMs.at(box.bboxSize).match(box.temp, true)->sd;
        // Draw bbox with colors based on how abnormal the temperature is.
        //if (std::isinf(tempDev)) {
        //    // If tempDev==inf (i.e. sd=0), 
        //    // then the GMM is not ready.
        //    // Draw bbox with grey color.
        if (std::abs(gmmSD) <= 0.00000001f) {
            // If SD of GMM is 0 (init value), 
            // then the GMM is not ready (as later SD is forced to be 
            // greater than some +ve value).
            // Draw bbox with grey color.    
            pen.setColor(QColor(128,128,128));
            printf("GMM model not ready. Draw bbox in gray color.\n");
        } else {
            // Otherwise, get intermeidate color from the color graident 
            // based on the temp devation from GMM model cluster mean.
            pen.setColor(interpolateColorRGB(
                bboxGradientColors, 
                bboxColorGradientStopTemps,
                tempDev
            ));
        }

        printf("Temp: %.4f | BBox Size: %d | Dev: %.4f\n", 
                box.temp, box.bboxSize, tempDev);
        auto c = GMMs.at(box.bboxSize).match(box.temp, true);
        printf("GMM Mean: %.4f | GMM SD: %.4f\n", 
                c->mean, c->sd);

        pen.setWidthF(imageResizeRatio/2);
        painter.setPen(pen);
        painter.drawRect(rect);
        
        // Prepare temp string buffer for text displays
        char tempString [8];
        // Prepare text font and color to use 
        painter.setFont(QFont("Helvetica", imageResizeRatio*3.5));
        pen.setColor(QColor(255,255,255, 255));
        painter.setPen(pen);


        // write bounding box's forehead temperature
        if (showTemp) {
            float temp;
            if (normalizeTemp)
                temp = GMMs.at(box.bboxSize).normalizeTempByMean(box.temp);
            else
                temp = box.temp;

            sprintf(tempString, "%.2f", temp);
            painter.drawText(x1, y1+5*imageResizeRatio, tempString);
        }
    
        // Added (20200905): Hide predicted class if bbox is too "small"
        // "bbox too small": both width and height < 50  px (after resize)
        int bboxSideThresh = 50;
        //float area = (x2-x1)*(y2-y1)/(imageResizeRatio*imageResizeRatio);
        if ( ((x2-x1)<bboxSideThresh) && ((y2-y1)<bboxSideThresh) ) {
            // Do not display predicted class for bbox too small

            // DEBUG: Indicate small bbox
            //QString tempText = "a";
            //tempText = ("small bbox"+to_string(x2-x1)+" "+to_string(y2-y1)+" "+to_string(area)).c_str();
            //painter.drawText(x1, y1+10*imageResizeRatio, tempText);
        } else {
            // show if person is wearing mask
            QString withMask;
            if (box.withMask == true)
                withMask = "With Mask";
            else
                withMask = "No Mask";
            painter.drawText(x1, y1+10*imageResizeRatio, withMask);
        }

        // debug, write assigned bboxSize
        sprintf(tempString, "%i", box.bboxSize);
        painter.drawText(x2, y2-5*imageResizeRatio, tempString);
    }

    pen.setColor(Qt::white);
    painter.setPen(pen);
    painter.setFont(QFont("Helvetica", imageResizeRatio*4));
    painter.drawText(WIDTH/3*imageResizeRatio, 8*imageResizeRatio, "IOT Device in Testing.");        
    
    /* Clock Display */
	clock_date = QDate::currentDate(); // Get current date
	clock_time = QTime::currentTime(); // Get current time
    // Display date (e.g Aug 27 (Thu))
	painter.drawText(1*imageResizeRatio, 5*imageResizeRatio, 
			 		 clock_date.toString("MMM dd (ddd)")); 
    // Display time (e.g. 12:34:56)
	painter.drawText(1*imageResizeRatio, 10*imageResizeRatio,
 					 clock_time.toString("hh:mm:ss")); 

    /* Temperature Colorbar */
    // Draw colorbar to show colors used for displaying temperature
    // with boudning box
    painter.fillRect(colorbarRect, bboxColorGradient);
    
    // Draw message texts near color bar
    QFont fontNormal = painter.font();
    QFont fontS, fontM;
    fontS.setPixelSize(15);
    fontM.setPixelSize(20);

    // Text above color bar
    painter.setFont(fontM);
    painter.drawText(1*imageResizeRatio, 26*imageResizeRatio,
                     "Body Temp.");
    painter.setFont(fontS);
    painter.drawText(1*imageResizeRatio, 29*imageResizeRatio,
                     "(Box Color)");
    // Colorbar text labels
    painter.setFont(fontS);
    painter.drawText(7*imageResizeRatio, 32*imageResizeRatio,
                     "High");
    painter.drawText(7*imageResizeRatio, 88*imageResizeRatio,
                     "Normal");

    painter.setFont(fontNormal);
    /*
        painter.setFont(QFont("Helvetica", imageResizeRatio*3.5));
        char tempString [8];

        QRect rect(30*imageResizeRatio, 60*imageResizeRatio, 2, 2);
        painter.drawRect(rect);
        sprintf(tempString, "%.2f", pixTemp[60 * WIDTH + 30]);
        painter.drawText(30*imageResizeRatio, 55*imageResizeRatio, tempString);


        for (const pixPos & pix: bgPixels){
            painter.drawRect(
                QRect(pix.x*imageResizeRatio, pix.y*imageResizeRatio, 2, 2));
            sprintf(tempString, "%.2f", pixTemp[pix.y * WIDTH + pix.x]);
            painter.drawText(pix.x*imageResizeRatio, pix.y*imageResizeRatio, tempString);
        }
        */
}

// TODO (20200906): Deprecated data logging function, to remove.
//                  To be replaced by logDetectionResults(...)
void FaceDetection::recordToDisk(unsigned char* grayPix, const std::list<BBox>& bbox){}
//void FaceDetection::recordToDisk(unsigned char* grayPix, const std::list<BBox>& bbox){
//    QString imageFileName;
//    if (bbox.size() > 0)
//       imageFileName = QString("%1/frame%2.pgm").arg(peopleImagePath).arg(frameID);
//    else
//       imageFileName = QString("%1/frame%2.pgm").arg(imagePath).arg(frameID);
//    /*
//    imageWriter.setFileName(imageFileName);
//    imageWriter.write(image);
//    */
//    QFile imageFile(imageFileName);
//    imageFile.open(QIODevice::WriteOnly);
//    // .pgm header
//    imageFile.write(QByteArray("P5"));
//    imageFile.write("\n160 120\n255\n");
//    // write pixels
//    QDataStream imageOut(&imageFile);
//    for (int y = 0; y < HEIGHT; y++)
//        for (int x = 0; x < WIDTH; x++)
//            imageOut << grayPix[y*WIDTH+x];
//    imageFile.close();
//
//    for (BBox const& b: bbox)
//        logFileStream << QString("%1,%2,%3,%4,%5,%6,%7,%8,%9\n").arg(frameID).arg(b.x1)
//            .arg(b.x2).arg(b.y1).arg(b.y2).arg(b.bboxSize).arg(b.temp).arg(b.ifFever)
//            .arg(b.withMask);
//    if (frameID % 64 == 0)
//        logFileStream.flush();
//}


// Log detection results
bool FaceDetection::logDetectionResults(unsigned char* grayPix, const std::list<BBox>& bboxes){
    /* Data Logging Procedures
     * 
     * Main Steps:
     * 1. Determine destination path
     * 2. Write data to files:
     *   - Image (frameID.pgm)
     *   - Predicted bboxes in YOLO format (frameID.txt)
     *   - Detailed info of predicted bboxes (in "main" log file)
     *   - GMM models log (gmm/modelID.csv)
     *
     * Log directory structure:
     * log_root_dir
     * |- {date}
     *   |- log.csv ("main" log file)
     *   |- gmm (GMM log files)
     *   |- bkg
     *   |- face_nomask
     *   |- face_mask
     *   |- multiple
     *   |- others
     *   
     * 
     * For each "log data type" subdir (e.g. "bkg"), data are further 
     * separated into folders so that each folder will not contain too 
     * many files, e.g.:
     * "log_type_dir"
     * |- 0
     *   |- frameID.pgm
     *   |- frameID.txt
     * |- 1
     * |- ...
     *
     * Under subfolder "gmm", info of each of the GMM models are log in 
     * separated files (named by "modelID" 0,1,..), e.g.:
     * gmm
     * |- 0.csv
     * |- 1.csv
     * |- ...
     * 
     * TODO's
     * - Move logging-related functions to a separated class/file
     * - Handle arbitrary number of data classes automatically
     *
     * Args:
     * - grayPix: grayscale image from camera
     * - bboxes: list of bbox objects (which contains info of bbox to log)
     * 
     * Return:
     *   true upon successful logging
     */
    /* 1. Determine Log Destination Directories and Paths */
    // Log DataPath format:
    //   log_root_dir/log_date/log_type/cnt/frameID.ext
    
    // Data will be stored in separated folders by their date of collection.
    // Get current date
    std::string logDate = QDate::currentDate().toString("yyyyMMdd").toStdString();
    // If multiple data logging sessions have been started in the day, 
    // append the number of sessions as a suffix to logDate to name the 
    // sub-directory to store the data, e.g. "20200901_1" if this is the
    // 2nd data logging sessions (i.e. "20200901" exists already)
    // 
    // If the date changes during a data logging session, a counter will
    // not be appended (ASSUMED not to have multiple concurrent sessions),
    // e.g. "20200902".
    std::string logDateDir = logDate;
    if ((logDate==logStartDate) &&
        (logStartDateCnt > 0)) {
        // Append counter to date
        logDateDir = logDate+"_"+to_string(logStartDateCnt);
    }

    // Determine the type of data to log (hence the destination directory
    // to save the data file)
    std::string logType;
    if (bboxes.size() == 0) {
        // If no bbox is predicted ==> "bkg" (background)
        logType = "bkg";
    } else { 
        // If some bboxes are predicted for this image, 
        // determine logType based on the predicted classes of the bboxes

        // Get the predicted classes of the bboxes
        std::vector<int> bboxClasses;
        
        for (auto b: bboxes) {
            bboxClasses.push_back(b.bboxClass);
        }
        
        // TODO: Add support to distribution of data to arbitrarily many
        //   number of classes:
        //   - if multiple class --> "multiple"
        //   - else --> predicted class
        bool hasFaceNoMask = (std::find(bboxClasses.begin(), bboxClasses.end(), 0) != 
                              bboxClasses.end());
        bool hasFaceMask = (std::find(bboxClasses.begin(), bboxClasses.end(), 1) != 
                            bboxClasses.end());
        if (hasFaceNoMask || hasFaceMask) {
            // Some "faces" are detected.
            if (!hasFaceMask) {
                // If only faces without masks are detected => "face_nomask"
                logType = "face_nomask";
            } else if (!hasFaceNoMask) {
                // If only faces with masks are detected => "face_mask"
                logType = "face_mask";
            } else {
                // Otherwise, both faces with mask and without masks are detected
                // => "multiple"
                logType = "multiple";
            }
        } else {
            // Other types of detected objects
            logType = "others";
        }
    }

    // To avoid storing too many data files in the same folder, the number
    // of data files to store in a folder is limited (by nFilesPerDir).
    // To calculate the "folder number" to store the current data:
    //    dataDirNum = logDataCnt // nFilesPerDir; 
    unsigned short int dataDirNum = 
        logDataCntsMap.at(logType) / nFilesPerDir;

    // Determine the destination directory to save data files:
    //   logRootDir/logDate/logType/dataDirNum (e.g. ./log/bkg/0)
    QString logDestDirPath = QString("%1/%2/%3/%4") 
        .arg(logRootDir.c_str())
        .arg(logDateDir.c_str())        
        .arg(logType.c_str())
        .arg(dataDirNum);
    // Navigate to the destination directory
    QDir logDestDir(logDestDirPath);
    // Create directory if the folder does not exist
    logDestDir.mkpath(".");
   
    // To save image file
    // Path to save image:
    //   logDestDir/frameID.pgm
    QString logImagePath = QString("%1/%2.pgm")
        .arg(logDestDirPath)
        .arg(logDataCntsMap.at("all"));
    // Write image to file
    QFile imageFile(logImagePath);
    imageFile.open(QIODevice::WriteOnly);
    // .pgm header
    imageFile.write(QByteArray("P5"));
    imageFile.write("\n160 120\n255\n");   
    // write pixels
    QDataStream imageOut(&imageFile);
    for (int y = 0; y < HEIGHT; y++)
        for (int x = 0; x < WIDTH; x++)
            imageOut << grayPix[y*WIDTH+x];
    imageFile.close();
    
    /* 2, Write Log Data to Files */
    // Write bbox annotation file (YOLO format) 
    // In the annotation file (frameID.txt), for each bbox predicted, 
    // write in a new line:
    //   class, bbox.x, bbox.y, bbox.width, bbox.height
    QString annPath = QString("%1/%2.txt")
        .arg(logDestDirPath)
        .arg(logDataCntsMap.at("all"));
    
    logFile.setFileName(annPath);
    logFile.open(QIODevice::WriteOnly | QIODevice::Text |
                 QIODevice::Append);
    logFileStream.setDevice(&logFile);
    for (auto bbox : bboxes) {
        logFileStream << QString("%1 %2 %3 %4 %5\n")
            .arg(bbox.bboxClass)
            .arg(bbox.x)
            .arg(bbox.y)
            .arg(bbox.w)
            .arg(bbox.h);
    }
    logFile.close(); // close fiel after writing

    // Write detailed log info to the main log file
    // (defulat: logRootDir/logDate[_cnt]/log.csv)
    QString mainLogPath = QString("%1/%2/log.csv")
        .arg(logRootDir.c_str())
        .arg(logDateDir.c_str());
    logFile.setFileName(mainLogPath);
    logFile.open(QIODevice::WriteOnly | QIODevice::Text |
                 QIODevice::Append);
    logFileStream.setDevice(&logFile);
    int i_bbox = 0; // Counter of bboxes
    for (auto bbox : bboxes) {
        // For each bbox, write in a new line:
        //   frameID,bboxID,timestamp,class,bboxSize,temp,ifFever,withMask
        logFileStream << QString("%1,%2,%3,%4,%5,%6,%7,%8\n")
            .arg(logDataCntsMap.at("all"))
            .arg(i_bbox)
            .arg(QTime::currentTime().toString("hh:mm:ss"))
            .arg(bbox.bboxClass)
            .arg(bbox.bboxSize)
            .arg(bbox.temp)
            .arg(bbox.ifFever)
            .arg(bbox.withMask);
        
        // Increment counter
        ++i_bbox;
    }
    logFile.close(); // close file after writing

    // Prepare direcotry for logging of GMM
    QString logGMMDirPath = QString("%1/%2/gmm/")
        .arg(logRootDir.c_str())
        .arg(logDateDir.c_str());
    QDir logGMMDir(logGMMDirPath);
    logGMMDir.mkpath(".");

    // Logging of GMM models
    int nGMMs = bboxSizeForComparison.size(); // Number of GMM models
    // Write log for each GMM model
    for (int i_gmm=0; i_gmm<nGMMs; ++i_gmm) {
        // Get cluster
        auto c = GMMs.at(i_gmm).clusters.begin();
        
        // Open log file corr. to this GMM model
        logFile.setFileName(
            QString("%1/%2.csv")
                .arg(logGMMDirPath)
                .arg(i_gmm)
        );
        logFile.open(QIODevice::WriteOnly | QIODevice::Text |
                     QIODevice::Append);
        logFileStream.setDevice(&logFile);
        
        // Write GMM info to log file
        // frameID,timestamp,alpha,mean,sd,weight
        logFileStream <<QString("%1,%2,%3,%4,%5,%6\n")
            .arg(logDataCntsMap.at("all"))
            .arg(QTime::currentTime().toString("hh:mm:ss"))
            .arg(GMMs.at(i_gmm).alpha())
            .arg(c->mean)
            .arg(c->sd)
            .arg(c->weight);


        logFile.close(); // close file after writing
    }
    
    // Increment counts of logged data
    ++logDataCntsMap.at("all"); // Count of all saved frames
    ++logDataCntsMap.at(logType); // Count this logType

    // Return true on successful logging
    return true;
}

// Functions to turn on and off LED light
// ref: https://solarianprogrammer.com/2018/12/23/raspberry-pi-cpp-control-led/
void FaceDetection::alertOn(int ledPIN) {
    digitalWrite(ledPIN, HIGH); 
    printf("!!!!! ALERT ON !!!!!\n");

    // Record alert start time
    alertStartTime = std::chrono::steady_clock::now();
}

void FaceDetection::alertOff(int ledPIN) {
    digitalWrite(ledPIN, LOW);
    printf("***** ALERT OFF *****\n");
}

// Alert system. Alert on abnormal cases.
int FaceDetection::alertAbnormality(const std::list<samePerson>& trackedPeople){
    /* Main Alert System
     *
     * Produce alert on abnormal cases:
     * - abnormally high body temp. (e.g. fever)
     * - no mask with detected face
     * 
     * Execption creitera:
     * - target too far (bboxSize != 0)
     * 
     * Returns:
     *   (int): -1: error
     *           0: no error
     *
     */
    int lastN = 6;
    for (auto p : trackedPeople) {
        if (p.classHistory.size()>=lastN) {
            printf("--- Person: %d ---\n", p.personID);
            printf("Class History (last %d): \n", lastN);
            int n = p.classHistory.size();
            for (int i=lastN; i>0; --i) {
                printf("%d | ", p.classHistory[n-i]);
            }
            printf("\n");
        }
    }    

    // Check if any of the tracked people shows abnormal feature
    // Abnormality only counts if the abnormal feature occurs for
    // at least abCntThresh number of times consecutively.
    int abCntThresh = 4 ;
    std::vector<int> lastClasses; // To store the last few predicts
    //std::vector<float> lastTemps; // To store the last few temp
    std::vector<bool> lastFevers; // To store the last few "ifFever"'s
    bool hasAb = false; // Flag to indicate if abnormality occurs
    for (auto p : trackedPeople) {
        // Skip if the tracked person has not enough history samples
        // (Assume all detection history vectors have the same length)
        if (p.classHistory.size() < abCntThresh) {  
            continue;
        }

        // Skip alert if target is too far from camera
        // (bboxSisze != 0 ==> not largest bbox)
        if (p.bboxSize != 0) {
            continue;
        }
        
        // Check abnormality: face without mask
        // Get the last abCntThresh predictions
        lastClasses.assign(
            p.classHistory.end()-abCntThresh, p.classHistory.end()
        );
        // Class 0: "face_nomask"
        if (std::all_of(lastClasses.begin(), lastClasses.end(),
                        [](int i){return i==0;})) {
            // If true, all last abCntThresh predictions are "face_nomask"
            hasAb = true;
        }
    
        // Check abnormality: abnormal body temp
        // Get the last abCntThresh predictions
        lastFevers.assign(
            p.feverHistory.end()-abCntThresh, p.feverHistory.end()  
        );
        // Check abnormal temps (ifFever)
        if (std::all_of(lastFevers.begin(), lastFevers.end(),
                        [](bool isFever){return isFever;})) {
            // If true, all last abCntThresh detections gives isFever==true
            hasAb = true;
        }

    }

    // Enable use of GPIO pins
    if (wiringPiSetupGpio() < 0 ) {
        printf("Failed to enable GPIO pins.\n");
        // Return -1 ==> error
        return -1;
    } {
        std::cout << "Controlling the GPIO pins with wiringPi\n";
    }
    // Setup PIN for LED
    int ledPIN = 14; // Use GPIO 14 for LED
    pinMode(ledPIN, OUTPUT);
                                                               
    // Raise alert if abnormality occurs
    // Otherwise, turn off alert
    if (hasAb) {
        printf("##### Abnormality Detected #####\n");
        // Turn on alert 
        // (skip sending on signal if already turned on within a short 
        // duration)
        auto alertOnDuration = 
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - alertStartTime
        );
        if (alertOnDuration.count() >= alertInterval) {
            alertOn(ledPIN);
        }
    } else {
        // Turn off 
        alertOff(ledPIN);
    }
       

    // Return 0 ==> no error 
    return 0;
}

/* Testing */
std::unique_ptr<Interpreter> FaceDetection::loadModel(std::string model_path) {
    /* Load Detection Model
     *
     */

    // Load model from path
    unique_ptr<FlatBufferModel> model = 
        tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
    tflite::ops::builtin::BuiltinOpResolver resolver;
    std::unique_ptr<Interpreter> interpreter;
    InterpreterBuilder(*model, resolver)(&interpreter);
    interpreter->AllocateTensors();
    
    // Set model quantization mode
    interpreter->SetAllowFp16PrecisionForFp32(true);

    // Set number of threads for multithreading processing
    interpreter->SetNumThreads(4);

    return interpreter;
}

cv::Mat FaceDetection::readImage(std::string image_path, std::string mode, 
                  cv::Size image_size){
    /* Read and Preprocess Image
     *
     * Mode:
     * - "input": preprocess image as detection model input
     * - "display": load image without preprocessing as model input
     *
     */
    
    // Read image from file
    cv::Mat image = cv::imread(image_path);
    printf("Image read successfully: %s\n", image_path.c_str());
    std::cout << "Image size: " << image.size() << std::endl;

    if (mode == "input") {
        // Resize image to target size
        cv::resize(image, image, image_size);

        // Convert value type to float32, as required by detection model.
        // Normalization is required before converting to float 32, see:
        // https://answers.opencv.org/question/123350/opencv-convertto-need-to-manually-scale-down-inputs/
        cv::normalize(image, image, 0, 255, cv::NORM_MINMAX);
        cv::Mat tempImage;
        image.convertTo(tempImage, CV_32FC3);
        image = tempImage;
        tempImage.release(); // Free memory
        
        // Normalize pixel values
        image = image / 255.0;

        // Convert grayscale image from having 1 channel to 3 channels,
        // by duplicated pixel values
        //std::cout << "Before: " << image.size() << std::endl;
        // cv::Mat tempImageArray[] = (image, image, image);
        // cv::merge(tempImageArray, 3, image);
        // std::cout << "After: " << image.elemSize() << std::endl;

    } else if (mode == "display") {
        // Load image for display
        // Only resize the image to image_size
        cv::resize(image, image, image_size);
    }

    return image;
}

void FaceDetection::drawDetectionOnCVImage(const std::list<BBox>&bbox, Mat & cvImg){
    /* Draw Detection Results on CV Image
     * 
     * TODO
     * - Add support to bbox display with arbitrary number of colors
     * 
     */
    char tempString [6];
    int x1, y1, x2, y2;
    for (BBox const& box: bbox){
        // draw rectangles for bounding boxes
        x1 = box.x1 * imageResizeRatio;
        y1 = box.y1 * imageResizeRatio;
        x2 = box.x2 * imageResizeRatio;
        y2 = box.y2 * imageResizeRatio;
        Rect rec(box.x1, box.y1, box.x2-box.x1, box.y2-box.y1);
        Scalar color; // BGR representation
        switch(box.bboxClass) {
            case 0 :
                color = Scalar(255,0,0); // "nomask" -> red
            case 1 :
                color = Scalar(0,255,0); // "mask" -> green
            case 2 :
                color = Scalar(255,255,255); // "back" - > white
            
        }
        rectangle(cvImg, rec, color, 0.5, 8, 0);
        
        // Write predicted class and confidence inside bbox
        sprintf(tempString, "%i (%.2f)", box.bboxClass, box.confidence);
        putText(cvImg, tempString, Point(box.x1,box.y1+5), FONT_HERSHEY_DUPLEX, 0.3,
                color, 1, 8, false);
    }
    
}

void FaceDetection::drawDetectionOnQImage(
        const std::list<BBox>&bbox, QImage & image, float* pixTemp){
    printf("1\n");
    QPainter painter (&image);
    printf("2\n");
    painter.setRenderHints(QPainter::TextAntialiasing);
    QPen pen;
    for (BBox const& box: bbox){
        // Get coordinates of bbox (and rescale according to display
        // dimensions
        int x1 = box.x1 * imageResizeRatio;
        int y1 = box.y1 * imageResizeRatio;
        int x2 = box.x2 * imageResizeRatio;
        int y2 = box.y2 * imageResizeRatio;

        // draw rectangles for bounding boxes
        QRect rect(x1, y1, x2-x1, y2-y1);
        
        // Draw bbox with discreet colors
        //if (box.ifFever)
        //    pen.setColor(QColor(252,186,3));
        //else
        //    pen.setColor(Qt::green);
        
        // Draw bbox with colors based on temp
        pen.setColor(interpolateColorRGB(
            bboxGradientColors, 
            bboxColorGradientStopTemps,
            box.temp
        ));
        
        /*
        // Compute how many sd's the temperature is deviating from GMM
        // cluster mean. 
        float tempDev = GMMs.at(box.bboxSize).computeDeviation(box.temp);
        float gmmSD = GMMs.at(box.bboxSize).match(box.temp, true)->sd;
        // Draw bbox with colors based on how abnormal the temperature is.
        //if (std::isinf(tempDev)) {
        //    // If tempDev==inf (i.e. sd=0), 
        //    // then the GMM is not ready.
        //    // Draw bbox with grey color.
        if (std::abs(gmmSD) <= 0.00000001f) {
            // If SD of GMM is 0 (init value), 
            // then the GMM is not ready (as later SD is forced to be 
            // greater than some +ve value).
            // Draw bbox with grey color.    
            pen.setColor(QColor(128,128,128));
            printf("GMM model not ready. Draw bbox in gray color.\n");
        } else {
            // Otherwise, get intermeidate color from the color graident 
            // based on the temp devation from GMM model cluster mean.
            pen.setColor(interpolateColorRGB(
                bboxGradientColors, 
                bboxColorGradientStopTemps,
                tempDev
            ));
        }

        printf("Temp: %.4f | BBox Size: %d | Dev: %.4f\n", 
                box.temp, box.bboxSize, tempDev);
        auto c = GMMs.at(box.bboxSize).match(box.temp, true);
        printf("GMM Mean: %.4f | GMM SD: %.4f\n", 
                c->mean, c->sd);
        */
        printf("3\n");
        pen.setWidthF(imageResizeRatio/2);
        painter.setPen(pen);
        painter.drawRect(rect);
        printf("4\n");
        // write bounding box's forehead temperature
        float temp;
        if (normalizeTemp)
            temp = GMMs.at(box.bboxSize).normalizeTempByMean(box.temp);
        else
            temp = box.temp;
        char tempString [8];
        printf("4.1\n");
        sprintf(tempString, "%.2f", temp);
        printf("4.2\n");
        painter.setFont(QFont("Helvetica", imageResizeRatio*3.5));
        printf("4.3\n");
        pen.setColor(QColor(255,255,255, 255));
        printf("4.4\n");
        painter.setPen(pen);
        printf("4.5\n");
        painter.drawText(x1, y1+5*imageResizeRatio, tempString);
        printf("5\n");
        // Added (20200905): Hide predicted class if bbox is too "small"
        // "bbox too small": both width and height < 50  px (after resize)
        int bboxSideThresh = 50;
        //float area = (x2-x1)*(y2-y1)/(imageResizeRatio*imageResizeRatio);
        if ( ((x2-x1)<bboxSideThresh) && ((y2-y1)<bboxSideThresh) ) {
            // Do not display predicted class for bbox too small

            // DEBUG: Indicate small bbox
            //QString tempText = "a";
            //tempText = ("small bbox"+to_string(x2-x1)+" "+to_string(y2-y1)+" "+to_string(area)).c_str();
            //painter.drawText(x1, y1+10*imageResizeRatio, tempText);
        } else {
            // show if person is wearing mask
            QString withMask;
            if (box.withMask == true)
                withMask = "With Mask";
            else
                withMask = "No Mask";
            painter.drawText(x1, y1+10*imageResizeRatio, withMask);
        }
        printf("6\n");
        // debug, write assigned bboxSize
        sprintf(tempString, "%i", box.bboxSize);
        painter.drawText(x2, y2-5*imageResizeRatio, tempString);
    }

    pen.setColor(Qt::white);
    painter.setPen(pen);
    painter.setFont(QFont("Helvetica", imageResizeRatio*4));
    painter.drawText(WIDTH/3*imageResizeRatio, 8*imageResizeRatio, "IOT Device in Testing.");        
    
    /* Clock Display */
	
    // Display date (e.g Aug 27 (Thu))
    clock_date = QDate::currentDate(); // Get current date
	painter.drawText(1*imageResizeRatio, 5*imageResizeRatio, clock_date.toString("MMM dd (ddd)")); 
    // Display time (e.g. 12:34:56)
    clock_time = QTime::currentTime(); // Get current time
	painter.drawText(1*imageResizeRatio, 10*imageResizeRatio, clock_time.toString("hh:mm:ss")); 

    /* Temperature Colorbar */
    // Draw colorbar to show colors used for displaying temperature
    // with boudning box
    painter.fillRect(colorbarRect, bboxColorGradient);
    
    // Draw message texts near color bar
    QFont fontNormal = painter.font();
    QFont fontS, fontM;
    fontS.setPixelSize(15);
    fontM.setPixelSize(20);

    // Text above color bar
    painter.setFont(fontM);
    painter.drawText(1*imageResizeRatio, 26*imageResizeRatio, "Body Temp.");
    painter.setFont(fontS);
    painter.drawText(1*imageResizeRatio, 29*imageResizeRatio, "(Box Color)");
    // Colorbar text labels
    painter.setFont(fontS);
    painter.drawText(7*imageResizeRatio, 32*imageResizeRatio, "High");
    painter.drawText(7*imageResizeRatio, 88*imageResizeRatio, "Normal");

    painter.setFont(fontNormal);
}


void FaceDetection::predictFromDir(std::string model_path, std::string data_dir) {
    /* Perform Detection on Images from Directory
     * 
     * Remarks:
     * - Assumed all files in the directory are images. 
     *   Currently filtering of unwanted files is not supported.
     * 
     * TODO's:
     * - Add support to image batch input to detection model
     *
     */
    /* Pre-Set Parameteres */
    float overlapThreshold = 0.12;
    /**********/
     
    printf("===== Start Detection (From Dir) =====\n");

    // Load TF Lite detection model
    std::unique_ptr<Interpreter> interpreter = 
        loadModel(model_path);
    printf("Loaded detection model.\n");

    // List files under data dir
    // ref: https://stackoverflow.com/questions/306533/how-do-i-get-a-list-of-files-in-a-directory-in-c 
    std::vector<std::string> paths;
    if (auto dir = opendir(data_dir.c_str())) {
        while (auto f = readdir(dir)) {
            if (!f->d_name || f->d_name[0] == '.'){
                continue; // Skip everything that starts with a dot
            }
            paths.push_back(data_dir + f->d_name);
        }
    }
    printf("# Images found: %d\n", paths.size());
    
    /* TODO: Avoid use of FaceDetection class */
    FaceDetection *faceDetectThread = new FaceDetection(false, false, 6, true);

    // Perform detection on each image
    //cv::Mat image;
    cv::Size input_size(160,160); // Image size for model input
    cv::Size display_size(160, 120); // Image size for display
    cv::Mat input_image, display_image;
    float* grayPix; 
    QImage displayQImage = 
        QImage(160, 120, QImage::Format_RGB888);
    QRgb red = qRgb(255,0,0);
    for(int i=0;i<80;i++) {
    	for(int j=0;j<60;j++) {
    		displayQImage.setPixel(i, j, red);
    	}
    }
    //displayQImage = displayQImage.scaled(
    //    120*6, 160*6,
    //    Qt::KeepAspectRatio,Qt::SmoothTransformation
    //);

    for (int i=0; i<paths.size(); ++i) {
        // Load image 
        //cv::Size image_size(160,160);
        //cv::Mat image = readImage(p, "input", image_size);
        
        /* TODO: Build custom detection function which does not include 
         * image preprocessing step. */
         
        printf("Read image: %s\n", paths[i].c_str());
        
        // Image preprocessing performed in MLFaceDetectYOLO
        //input_image = readImage(paths[i], "display", input_size); 
        input_image = readImage(paths[i], "input", input_size); 
        std::list<BBox> bboxes;
        MLFaceDetectYOLO(interpreter, input_image, bboxes);
        auto itr = bboxes.begin();
        printf("1st BBox (detection): (%d, %d), (%d, %d)\n", 
                itr->x1, itr->y1, itr->x2, itr->y2);
        
        
        // Post-processing steps
        printf("#BBoxes (before filtering): %d\n", bboxes.size());
        orderBBoxByConfidenceDesc(bboxes);
        //findForeheadTemp(bboxes, pixTemp);
        //removeNonHumanBoxes(bboxes);
        nonMaxSuppression(bboxes, overlapThreshold);
        //calcBBoxSize(bbox);
        //offsetTemp(bbox, pixTemp);
        //trackPeopleAndUpdateGMM(bboxes);
        printf("#BBoxes (after filtering): %d\n", bboxes.size());
        /***********/
        
        // Prepare display image
        display_image = readImage(paths[i], "display", display_size); 
        // Draw bboxes on image
        drawDetectionOnCVImage(bboxes, display_image); 
        // Save image
        imwrite( "temp_outputs/"+to_string(i)+".jpg", display_image );
        // Log prediction results
        
        printf("Write to: %s\n", ("temp_outputs/"+to_string(i)+".jpg").c_str());
       
        /*
        unsigned char* grayPix;
        Mat cvImg = Mat(
            display_size.height, display_size.width,
            CV_8UC1, grayPix
        );
        Mat channel3cvImg;
        Mat in[] = {cvImg, cvImg, cvImg};
        cv::merge(in, 3, channel3cvImg);
        */


        //display_image = display_image.scaled(
        //    120*6, 160*6,
        //    Qt::KeepAspectRatio,Qt::SmoothTransformation
        //);
        grayPix = display_image.ptr<float>(0);
        drawDetectionOnQImage(bboxes, displayQImage, grayPix);       
        printf("3\n");       
        QImageWriter writer(("temp_outputs/"+to_string(i)+"_2.jpg").c_str());
        writer.write(displayQImage);

        //imwrite( "temp_outputs/"+to_string(i)+"_2.jpg", display_qimage );
    
        if (i%20 == 0) {
            printf("Processed: %d/%d\n", i, paths.size());
        }
        
    }

    printf("===== Finished Detection (From Dir) =====\n");
}


/**********/
