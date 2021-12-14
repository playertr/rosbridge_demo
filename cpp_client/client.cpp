// cpp_client/main.cpp
// Example C++ program for connecting for accessing ROSBridge API.
// Modiefied from https://stackoverflow.com/questions/34423092/websocket-library
// Tim Player, playertr@oregonstate.edu, 13 December 2021

#include <iostream>
#include <cpprest/ws_client.h>

using namespace std;
using namespace web;
using namespace web::websockets::client;

int main() {
  // Connect to the ROSBridge websocket on port 9090
  websocket_client client;
  client.connect("ws://0.0.0.0:9090").wait();

  // Send a message using the JSON schema from the ROSBridge protocol
  // https://github.com/RobotWebTools/rosbridge_suite/blob/ros1/ROSBRIDGE_PROTOCOL.md
  // This message requests two random poses from the ROS gen_random_pose service.
  websocket_outgoing_message out_msg;
  string message = "{ \"op\": \"call_service\", \"service\": \"/gen_random_pose\", \"args\": [2] }";
  cout << "Sending Message: " << endl << message << endl;
  out_msg.set_utf8_message(message);
  client.send(out_msg).wait();

  // Print the response
  cout << "Recieving Response: " << endl;
  client.receive().then([](websocket_incoming_message in_msg) {
    return in_msg.extract_string();
  }).then([](string body) {
    cout << body << endl; // test
  }).wait();

  client.close().wait();

  return 0;
}