/**

    Class for communicating with player and mouse selection
    NetworkThread is a Qthread

**/

#include <sys/types.h>
#include "NetworkThread.h"
#include <signal.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <net/if_arp.h>


bool NetworkThread::connectHBServer(){
    if (::connect(hb_sock, (struct sockaddr *)&hb_serv_addr, sizeof(hb_serv_addr)) < 0){
        printf("Heartbeat Server connection Failed\n");
        return false;
    }
    return true;
}

/* 
    Qt slot to receive signal and send data to the client
*/
void NetworkThread::sendData(std::string msg) {
    if (server._img != msg)
        server.update_img(msg,"/IoT");
    //free(msg); // msg was malloced in FaceDetection::detectFaces
    //Todo: used shared memeoy instead of senddata through fuction
}

// void NetworkThread::sendHeartBeat(){
//     // try connecting to server once if not already connected
//     /*
//     if (! hbServerConnected){
//         hbServerConnected = connectHBServer();
//     }
//     */
//     if (hbServerConnected) {
//         constexpr unsigned char message = 0xFF;
//         int sentByte = 0;
//         if ((sentByte = send(hb_sock, &message, 1, 0)) < 0){
//             printf("Fail sending data...\n");
//             hbServerConnected = false;
//         } else
//             printf("Data sent successfully. \n");
//     } else
//         printf("No Heartbeat Server. \n");
// }

