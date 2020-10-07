#ifndef NETWORKTHREAD_H
#define NETWORKTHREAD_H

#include <QThread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <vector>
#include <stdio.h>
#include <QMetaType>

#include "seasocks/PageHandler.h"
#include "seasocks/PrintfLogger.h"
#include "seasocks/Server.h"
#include "seasocks/WebSocket.h"
#include "seasocks/StringUtil.h"

#include <memory>
#include <set>
#include <string>


using namespace seasocks;

namespace {
struct Handler : WebSocket::Handler {
    // std::set<WebSocket*> _cons;
    std::set<WebSocket*> _cons;
    std::string encode_img = ""
    void update_img(std::string str){
        encode_img = str;
        send(str);
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
};
}
class NetworkThread : public QThread
{
    Q_OBJECT
    Server server;
    // variables for sending heartbeat to another localhost server
    bool connectHBServer();

public:
    Server server;
    NetworkThread(){
        Server server(std::make_shared<PrintfLogger>());
        // server.addPageHandler(std::make_shared<MyAuthHandler>());
        server.addWebSocketHandler("/IoT", std::make_shared<Handler>());
        // std::string image_path = "apple.png";
        // Mat img = imread(image_path);
        // if(img.empty())
        // {
        //     std::cout << "Could not read the image: " << image_path << std::endl;
        //     return 1;
        // }
        // std::string encode_png = Mat2Base64(img,"png");
        // std::printf("sending img\n");
        // server.update_img(encode_png,"/IoT");
        // server.serve("IoT_project", 8888);
    }
    
    void run{
        server.serve("IoT_project", 8888);
    }


public slots:
    void sendData(std::string);
    // void sendHeartBeat();

signals:
    void updateConfig(int, double, vector<struct region_data>);
    void setPreview(bool);
};

#endif // NETWORKTHREAD_H
