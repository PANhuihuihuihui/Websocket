#include "seasocks/PageHandler.h"
#include "seasocks/PrintfLogger.h"
#include "seasocks/Server.h"
#include "seasocks/WebSocket.h"
#include "seasocks/StringUtil.h"

#include <memory>
#include <set>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <unistd.h>
static std::string base64Encode(const unsigned char* Data, int DataByte)
{
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

// Simple chatroom server, showing how one might use authentication.

using namespace seasocks;
namespace {

struct Handler : WebSocket::Handler {
    std::set<WebSocket*> _cons;
    std::string encode_img2 = "/9j/4AAQSkZJRgABAQEAAAAAAAD/4QBCRXhpZgAATU0AKgAAAAgAAYdpAAQAAAABAAAAGgAAAAAAAkAAAAMAAAABAEoAAEABAAEAAAABAAAAAAAAAAAAAP/bAEMACwkJBwkJBwkJCQkLCQkJCQkJCwkLCwwLCwsMDRAMEQ4NDgwSGRIlGh0lHRkfHCkpFiU3NTYaKjI+LSkwGTshE//bAEMBBwgICwkLFQsLFSwdGR0sLCwsLCwsLCwsLCwsLCwsLCwsLCwsLCwsLCwsLCwsLCwsLCwsLCwsLCwsLCwsLCwsLP/AABEIAncB2gMBIgACEQEDEQH/xAAfAAABBQEBAQEBAQAAAAAAAAAAAQIDBAUGBwgJCgv/xAC1EAACAQMDAgQDBQUEBAAAAX0BAgMABBEFEiExQQYTUWEHInEUMoGRoQgjQrHBFVLR8CQzYnKCCQoWFxgZGiUmJygpKjQ1Njc4OTpDREVGR0hJSlNUVVZXWFlaY2RlZmdoaWpzdHV2d3h5eoOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4eLj5OXm5+jp6vHy8/T19vf4+fr/xAAfAQADAQEBAQEBAQEBAAAAAAAAAQIDBAUGBwgJCgv/xAC1EQACAQIEBAMEBwUEBAABAncAAQIDEQQFITEGEkFRB2FxEyIygQgUQpGhscEJIzNS8BVictEKFiQ04SXxFxgZGiYnKCkqNTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqCg4SFhoeIiYqSk5SVlpeYmZqio6Slpqeoqaqys7S1tre4ubrCw8TFxsfIycrS09TV1tfY2dri4+Tl5ufo6ery8/T19vf4+fr/2gAMAwEAAhEDEQA/APXKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiqU+raLbEifUbKMjqHuIg35E5pNpblwpzqO0E36F2isj/hJvC4OP7Ws/8Av5x+dTw63oE5xFqlg5PYXEQb8ic1KqRezN5YPERV5U5L5M0KKRWVgGVgynoVIIP0Ipas5QooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKwvEHiWx0OLacTX0i5ht1OMDoHlI6L/AD/UTKaguaR0YbDVcVUVGjG8mat5e2OnwtcXk8cEK9WkOMn0UDkn2ArhdU+IEhLxaRbBV5AuboZY+6RDj8z+FcdqOp6jqtwbm+maR+Qi9I4lP8MadAKpV5dXFznpDRH6hlfCOHw6U8X78u32V/n89PIvXmr6zqBJvL65lB/gLlYh9I0wv6VQwKdVlNP1SQbo7G8deuUt5WH5ha49ZPU+vjGjh48sUor5JFTFNIqeWG4hO2aKWJvSVGQ/kwqOlY1spK6JLa91GybfaXdzA3/TCV0B+oU4rpNP8fa/aFVvFhvohgHzAIpse0kYx+amuVIppFaQqSh8LPLxmW4bFK1amn8tfv3PY9I8X6BqxSJZjbXTYAgu8IWPpG+dp/PPtXQ187kV1Gg+NdV0kxwXZe9sBgbJGzPEv/TKRuw9D+ld9LF9JnwWZcKuF54N3/uv9H/n957BRVPTdU07VrZbqxnWWI8MBw8bddkinkGrld6aauj4ecJU5OM1ZoKKKKZAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRwOtRvIQdq8t39FFQ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";
    void update_img(std::string str){
        encode_img = str;
        //send(str);
    }
    void onConnect(WebSocket* con) override {
        _cons.insert(con);
        send(encode_img);
    }
    void onDisconnect(WebSocket* con) override {
        _cons.erase(con);
        //send(con->credentials()->username + " has left");
    }

    void onData(WebSocket* con, const char* data) override {
        std::string rev = std::string(data);
        if( rev == "1"){
            con->send(encode_img);
        }
        else{
            con->send(encode_img2);
        }
        
     }

    void send(const std::string& msg) {
        for (auto* con : _cons) {
            con->send(msg);
        }
    }
}h1;
}
using namespace cv;

int main()
{
    Server server(std::make_shared<PrintfLogger>());
    // server.addPageHandler(std::make_shared<MyAuthHandler>());
    server.addWebSocketHandler("/IoT", std::make_shared<Handler>());

    std::string image_path = "apple.png";
    Mat img = imread(image_path);
    if(img.empty())
    {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return 1;
    }
    std::string encode_png = Mat2Base64(img,"png");
    std::printf("sending img\n");
    server.update_img(encode_png,"/IoT");
    server.serve("IoT_project", 8888);
    
    return 0;
}



