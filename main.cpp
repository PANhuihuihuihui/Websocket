#include <QApplication>
#include <QThread>
#include <QMutex>
#include <QMessageBox>

#include <QColor>
#include <QLabel>
#include <QtDebug>
#include <QString>
#include <QPushButton>

#include "LeptonThread.h"
#include "MyLabel.h"
#include "FaceDetection.h"
#include "NetworkThread.h"

void printUsage(char *cmd) {
        char *cmdname = basename(cmd);
	printf("Usage: %s [OPTION]...\n"
               " -h      display this help and exit\n"
               " -r      record raw image and bounding box information to disk\n"
               " -n      output normalized temperature instead of camera read temp\n"
               " -rs x   resize image by x for local display\n"
               " -cm x   select colormap\n"
               "           1 : rainbow\n"
               "           2 : grayscale[default]\n"
               "           3 : ironblack\n"
               " -tl x   select type of Lepton\n"
               "           2 : Lepton 2.x\n"
               "           3 : Lepton 3.x[default]\n"
               "               [for your reference] Please use nice command\n"
               "                 e.g. sudo nice -n 0 ./%s -tl 3\n"
               " -ss x   SPI bus speed [MHz] (10 - 30)\n"
               "           20 : 20MHz [default]\n"
               " -min x  override minimum value for scaling (0 - 65535)\n"
               "           [default] automatic scaling range adjustment\n"
               "           e.g. -min 30000\n"
               " -max x  override maximum value for scaling (0 - 65535)\n"
               "           [default] automatic scaling range adjustment\n"
               "           e.g. -max 32000\n"
               " -d x    log level (0-255)\n"
               "", cmdname, cmdname);
	return;
}

int main( int argc, char **argv )
{
    int typeColormap = 2; // colormap_grayscale
    int typeLepton = 3; // Lepton 3.x
    int spiSpeed = 20; // SPI bus speed 20MHz
    int rangeMin = 29815; // -1;
    int rangeMax = 31315; // -1;
	int loglevel = 0;

    // custom options
    bool recordData = false;
    bool normalizeTemp = false;
    float resizeImageRatio = 6;
    bool showTemp = false; // Whether to show temp for bbox

	for(int i=1; i < argc; i++) {
		if (strcmp(argv[i], "-h") == 0) {
			printUsage(argv[0]);
			exit(0);
		}
        else if (strcmp(argv[i], "-r") == 0) {
            recordData = true;
        }
        else if (strcmp(argv[i], "-n") == 0) {
            normalizeTemp = true;
        }
        else if ((strcmp(argv[i], "-rs") == 0) && (i + 1 != argc)) {
            resizeImageRatio = std::atof(argv[i + 1]);
            i++;
        }
		else if (strcmp(argv[i], "-d") == 0) {
			int val = 3;
			if ((i + 1 != argc) && (strncmp(argv[i + 1], "-", 1) != 0)) {
				val = std::atoi(argv[i + 1]);
				i++;
			}
			if (0 <= val) {
				loglevel = val & 0xFF;
			}
		}
		else if ((strcmp(argv[i], "-cm") == 0) && (i + 1 != argc)) {
			int val = std::atoi(argv[i + 1]);
			if ((val == 1) || (val == 2)) {
				typeColormap = val;
				i++;
			}
		}
		else if ((strcmp(argv[i], "-tl") == 0) && (i + 1 != argc)) {
			int val = std::atoi(argv[i + 1]);
			if (val == 3) {
				typeLepton = val;
				i++;
			}
		}
		else if ((strcmp(argv[i], "-ss") == 0) && (i + 1 != argc)) {
			int val = std::atoi(argv[i + 1]);
			if ((10 <= val) && (val <= 30)) {
				spiSpeed = val;
				i++;
			}
		}
		else if ((strcmp(argv[i], "-min") == 0) && (i + 1 != argc)) {
			int val = std::atoi(argv[i + 1]);
            if (val == -1){
                i++;
                rangeMin = -1;
                continue;
            }
			if ((0 <= val) && (val <= 65535)) {
				rangeMin = val;
				i++;
			}
		}
		else if ((strcmp(argv[i], "-max") == 0) && (i + 1 != argc)) {
			int val = std::atoi(argv[i + 1]);
            if (val == -1){
                i++;
                rangeMax = -1;
                continue;
            }
			if ((0 <= val) && (val <= 65535)) {
				rangeMax = val;
				i++;
			}
		} else if ((strcmp(argv[i], "-detect_dir") == 0) && (i + 2 != argc)) {
            /* Perform Detection on Images from Directory */
            std::string model_path = argv[i+1];
            std::string data_dir = argv[i+2];
	    
	    FaceDetection *faceDetectThread = 
	      new FaceDetection(recordData, normalizeTemp, resizeImageRatio, showTemp);

            faceDetectThread->predictFromDir(model_path, data_dir);


            return 1;
            
        } else if ((strcmp(argv[i], "-show_temp") == 0) && (i + 2 != argc)) {
            showTemp = true;
            continue;
        }
	}

    argv[0] = "IOT Project";
	//create the app
    QApplication a( argc, argv, true);
	
	QWidget *myWidget = new QWidget;
    myWidget->setGeometry(0, 50, 160*resizeImageRatio, 120*resizeImageRatio);

	//create an image placeholder for myLabel
	//fill the top left corner with red, just bcuz
	QImage myImage;
	myImage = QImage(320, 240, QImage::Format_RGB888);
	QRgb red = qRgb(255,0,0);
	for(int i=0;i<80;i++) {
		for(int j=0;j<60;j++) {
			myImage.setPixel(i, j, red);
		}
	}

    //create a label, and set it's image to the placeholder
    MyLabel myLabel(myWidget);
    myLabel.setGeometry(0, 0, 160*resizeImageRatio, 120*resizeImageRatio);
    myLabel.setPixmap(QPixmap::fromImage(myImage));

//	//create a FFC button
//    QPushButton *button1 = new QPushButton("Perform FFC", myWidget);
//    button1->setGeometry(0, 70*resizeImageRatio, 100, 30);

    FaceDetection *faceDetectThread = new FaceDetection(recordData, normalizeTemp, resizeImageRatio, showTemp);

	//create a thread to gather SPI data
	//when the thread emits updateImage, the label should update its image accordingly
	LeptonThread *thread = new LeptonThread();
	thread->setLogLevel(loglevel);
	thread->useColormap(typeColormap);
	thread->useLepton(typeLepton);
	thread->useSpiSpeedMhz(spiSpeed);
	thread->setAutomaticScalingRange();
	if (0 <= rangeMin) thread->useRangeMinValue(rangeMin);
	if (0 <= rangeMax) thread->useRangeMaxValue(rangeMax);
//    QObject::connect(thread, SIGNAL(updateImage(QImage)), &myLabel, SLOT(setImage(QImage)));
    QObject::connect(faceDetectThread, SIGNAL(updateImage(QImage))
        , &myLabel, SLOT(setImage(QImage)), Qt::DirectConnection);
    QObject::connect(thread, SIGNAL(imageReadFromCamera(QImage, unsigned char*, float*, bool))
        , faceDetectThread, SLOT(detectFaces(QImage, unsigned char*, float*, bool))
        , Qt::BlockingQueuedConnection);
	
	//connect ffc button to the thread's ffc action
//    QObject::connect(button1, SIGNAL(clicked()), thread, SLOT(performFFC()));

    // create Network thread
    NetworkThread* networkThread = new NetworkThread();
    void sendData(unsigned char*, int);
    QObject::connect(faceDetectThread, SIGNAL(sendData(std::string))
        , networkThread, SLOT(sendData(std::string)), Qt::QueuedConnection);
    // QObject::connect(faceDetectThread, SIGNAL(sendHeartBeat())
    //     , networkThread, SLOT(sendHeartBeat()), Qt::QueuedConnection);
    networkThread->start();
	thread->start();
	
    myWidget->show();
	return a.exec();
}

// for quick testing of ML models on static images
/*
#include <opencv2/opencv.hpp>
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/string_util.h"
#include "tensorflow/lite/model.h"

using namespace cv;
using namespace std;
using namespace std::chrono;
using namespace tflite;

int mainS(int argc, char **argv) {

  // Load the classification model
  unique_ptr<FlatBufferModel> model = tflite::FlatBufferModel::BuildFromFile("model/classifier.tflite");
  tflite::ops::builtin::BuiltinOpResolver resolver;
  unique_ptr<Interpreter> interpreter;
  InterpreterBuilder(*model, resolver)(&interpreter);
  interpreter->AllocateTensors();
  interpreter->SetAllowFp16PrecisionForFp32(true);
  interpreter->SetNumThreads(4);

  // Perform mask classification
  Mat src = imread(argv[1]);
  resize(src, src, Size(224,224));
  Mat faceRegion;
  src.convertTo(faceRegion, CV_32FC3);
  memcpy(interpreter->typed_tensor<float>(interpreter->inputs()[0]), faceRegion.data, faceRegion.total()*faceRegion.elemSize());
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  interpreter->Invoke(); // Run Model
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  cout << "Classification Time: " << time_span.count() << " seconds" << endl;

  // Show results
  // ['FaceWithMask', 'FaceWithoutMask']
  float* scores = interpreter->typed_output_tensor<float>(0);
  float scoreForFaceWithMask = scores[0];
  float scoreForFaceWithoutMask = scores[1];
  cout << "FaceWithMask: " << scoreForFaceWithMask << endl;
  cout << "FaceWithoutMask: " << scoreForFaceWithoutMask << endl;
  if (scoreForFaceWithMask > scoreForFaceWithoutMask) {
    cout << "FaceWithMask" << endl;
  } else if (scoreForFaceWithMask < scoreForFaceWithoutMask) {
    cout << "FaceWithoutMask" << endl;
  } else {
    cout << "Unclear" << endl;
  }
  imshow("Face Region", src);
  waitKey(0);
}

*/
