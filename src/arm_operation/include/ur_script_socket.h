#pragma once
#include <cstdio>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <chrono>

inline std::string get_unix_time(void){
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch());
  double unix_time = (double)ns.count()/1000000000.0f; // ns -> s
  std::stringstream ss;
  ss << std::fixed << std::setprecision(9) << unix_time;
  return ss.str();
}

class URScriptSocket{
 private:
  int dashBoardSocket, commandSocket, dashBoardPort, commandPort;
  struct sockaddr_in socket_info;
  socklen_t addr_size;
  std::string host;
  const char *UNLOCK_PROTECTIVE = "unlock protective stop\n";
  const char *STOP_PROGRAM      = "sleep(0.1)\n";
  std::string info;
  void connect_socket(int port){
    assert(!host.empty());
    char message[512];
    info = host + ":" + std::to_string(port);
    if(port==dashBoardPort){
      dashBoardSocket = socket(AF_INET, SOCK_STREAM, 0);
      bzero(&socket_info, sizeof(socket_info));
      socket_info.sin_family = AF_INET;
      socket_info.sin_port = htons(port);
      socket_info.sin_addr.s_addr = inet_addr(host.c_str());
      if(dashBoardSocket==-1){
        printf("\033[1;31m[%s] Fail to create a socket.\033[0m\n", info.c_str());
      }
      int err = connect(dashBoardSocket, (struct sockaddr *)&socket_info, sizeof(socket_info));
      if(err==-1){
        printf("\033[1;31m[%s] Connecgtion fail.\033[0m\n", info.c_str());
      }else{
        printf("[%s, %s] Socket connect to %s:%d\033[0m\n", get_unix_time().c_str(), info.c_str(), host.c_str(), port);
      } recv(dashBoardSocket, message, 512, 0);
    } else if(port==commandPort){
      commandSocket = socket(AF_INET, SOCK_STREAM, 0);
      bzero(&socket_info, sizeof(socket_info));
      socket_info.sin_family = AF_INET;
      socket_info.sin_port = htons(port);
      socket_info.sin_addr.s_addr = inet_addr(host.c_str());
      if(commandSocket==-1){
        printf("\033[1;31m[%s] Fail to create a socket.\033[0m\n", info.c_str());
      }
      int err = connect(commandSocket, (struct sockaddr *)&socket_info, sizeof(socket_info));
      if(err==-1){
        printf("\033[1;31m[%s] Connecgtion fail.\033[0m\n", info.c_str());
      }else{
        printf("[%s, %s] Socket connect to %s:%d\033[0m\n", get_unix_time().c_str(), info.c_str(), host.c_str(), port);
      } recv(commandSocket, message, 512, 0);
    }
  }
 public:
  URScriptSocket(): host(""), dashBoardPort(29999), commandPort(30002){}
  URScriptSocket(std::string ip): host(ip), dashBoardPort(29999), commandPort(30002){
    connect_socket(dashBoardPort);
    connect_socket(commandPort);
  }
  ~URScriptSocket() {close(dashBoardSocket); close(commandSocket);}
  void setHost(std::string ip) {host=ip; connect_socket(dashBoardPort); connect_socket(commandPort);}
  void unlock_protective_stop(void){
    printf("[%s, URScriptSocket] Sending command: %s", get_unix_time().c_str(), UNLOCK_PROTECTIVE);
    send(dashBoardSocket, UNLOCK_PROTECTIVE, strlen(UNLOCK_PROTECTIVE), 0);
    char message_receive[512];
    recv(dashBoardSocket, message_receive, sizeof(message_receive), 0);
    printf("[%s, URScriptSocket] Message received: %s\n", get_unix_time().c_str(), message_receive);
  }
  void stop_program(void){
    printf("[%s, URScriptSocket] Sending command: %s", get_unix_time().c_str(), STOP_PROGRAM);
    send(commandSocket, STOP_PROGRAM, strlen(STOP_PROGRAM), 0);
    /*char message_receive[512];
    recv(commandSocket, message_receive, sizeof(message_receive), 0);
    printf("[URScriptSocket] Message received: %s\n", message_receive);*/
  }
};
