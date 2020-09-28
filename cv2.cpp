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

// Simple chatroom server, showing how one might use authentication.

using namespace seasocks;
namespace {

struct Handler : WebSocket::Handler {
    std::set<WebSocket*> _cons;
    void update_img(std::string str){
        send(str);
    }
    void onConnect(WebSocket* con) override {
        _cons.insert(con);
        //send(con->credentials()->username + " has joined");
    }
    void onDisconnect(WebSocket* con) override {
        _cons.erase(con);
        //send(con->credentials()->username + " has left");
    }

    // void onData(WebSocket* con, const char* data) override {
    //     send(con->credentials()->username + ": " + data);
    // }

    void send(const std::string& msg) {
        for (auto* con : _cons) {
            con->send(msg);
        }
    }
};
using namespace cv;

int main()
{
    Server server(std::make_shared<PrintfLogger>());
    // server.addPageHandler(std::make_shared<MyAuthHandler>());
    server.addWebSocketHandler("/IoT", std::make_shared<Handler>());
    server.serve("src/IoT_project", 8888);
    std::image_path = "apple.png";
    Mat img = imread(image_path, IMREAD_COLOR);
    std::vector<uchar> data_encode;
    while (1)
	{
        if(img.empty())
        {
            std::cout << "Could not read the image: " << image_path << std::endl;
            return 1;
        }
        imencode(".png", img, data_encode);
        auto base64_png = reinterpret_cast<const unsigned char*>(data_encode.data());
                std::string encoded_png = "data:image/png;base64,"+base64_encode(base64_png,data_encode.size());
        Handler.update_img(base64_png);

        int k = waitKey(0); // Wait for a keystroke in the window
    }

    return 0;
}



