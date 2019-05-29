#include <mavlink.h>
#include <SoftwareSerial.h>
 
#define RXpin 2
#define TXpin 3
SoftwareSerial SerialMAV(RXpin, TXpin); // sets up serial communication on pins 3 and 2
 
void setup() {
  SerialMAV.begin(57600); //RXTX from Pixhawk (Port 19,18 Arduino Mega)
  Serial.begin(57600); //Main serial port to read the port
 
  mission_count();
  
}
 
void loop() {
 
 MavLink_receive();
 
}
 
//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
void MavLink_receive()
  { 
  mavlink_message_t msg;
  mavlink_status_t status;
 
  while(SerialMAV.available())
  {
    uint8_t c= SerialMAV.read();
 
    //Get new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
 
    //Handle new message from autopilot
      switch(msg.msgid)
      {
      
      // Step 2 uploading a new waypoint - Check for mission replies
      case MAVLINK_MSG_ID_MISSION_REQUEST:
      {
       mavlink_mission_request_t missionreq;
       mavlink_msg_mission_request_decode(&msg, &missionreq);
               
        Serial.print("\nMission Req Sequence: ");Serial.println(missionreq.seq);
        Serial.print("\SysID: ");Serial.println(missionreq.target_system);
        Serial.print("\Compid: ");Serial.println(missionreq.target_component);
 
        if (missionreq.seq == 0) {
        create_home();
        Serial.print("Sent Home: \n");
        }
 
        if (missionreq.seq == 1) {
        create_waypoint();
        Serial.print("Sent Waypoint: \n");
        }
      }
      break;
 
      case MAVLINK_MSG_ID_MISSION_ACK:
      // Step 4 uploading a new waypoint - Receive Mission Ack Message
      {
       mavlink_mission_ack_t missionack;
       mavlink_msg_mission_ack_decode(&msg, &missionack);
       
        Serial.print("\nMission Ack Sequence: ");Serial.println(missionack.type);
        Serial.print("\SysID: ");Serial.println(missionack.target_system);
        Serial.print("\CompID: ");Serial.println(missionack.target_component);
 
        if (missionack.type == 1) {
        Serial.print("\nMission upload FAILED: ");Serial.println(missionack.type);
        }
 
        if (missionack.type == 0) {
        Serial.print("\nMission upload SUCCESSFULL: ");Serial.println(missionack.type);
        }   
      }
      break;
      
      }
    }
  }
}
 
void mission_count() {
  //Step #1 of uploading a new waypoint
  uint8_t _system_id = 255; // system id of sending station. 255 is Ground control software
  uint8_t _component_id = 2; // component id of sending station 2 works fine
  uint8_t _target_system = 1; // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)
 
  uint16_t count = 2; // How many items to upload (HOME coordinates are always the first way-point)
 
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_mission_count_pack(_system_id, _component_id, &msg, _target_system, _target_component, count);
  //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t count
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  SerialMAV.write(buf, len);
  
}
 
void create_home() {
  //Step 3 of uploading a new waypoint (send HOME coordinates)
  uint8_t _system_id = 255; // system id of sending station. 255 is Ground control software
  uint8_t _component_id = 2; // component id of sending station 2 works fine
  uint8_t _target_system = 1; // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)
 
  uint16_t seq = 0; // Sequence number
  uint8_t frame = 0; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Specific command for PX4
  uint8_t current = 0; // Guided mode waypoint
  uint8_t autocontinue = 0; // Always 0
  float param1 = 0; // Loiter time
  float param2 = 0; // Acceptable range from target - radius in meters
  float param3 = 0; // Pass through waypoint
  float param4 = 0; // Desired yaw angle
  float x = 45.464217; // Latitude - degrees
  float y = -1.280222; // Longitude - degrees
  float z = 200; // Altitude - meters
 
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);
  //uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  SerialMAV.write(buf, len);
  
}
 
void create_waypoint() {
  //Step 3 continuation of uploading a new waypoint (send 1st coordinates)
  uint8_t _system_id = 255; // system id of sending station. 255 is Ground control software
  uint8_t _component_id = 2; // component id of sending station 2 works fine
  uint8_t _target_system = 1; // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)
 
  uint16_t seq = 1; // Sequence number
  uint8_t frame = 0; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Specific command for PX4
  uint8_t current = 1; // Guided mode waypoint
  uint8_t autocontinue = 0; // Always 0
  float param1 = 0; // Loiter time
  float param2 = 0; // Acceptable range from target - radius in meters
  float param3 = 0; // Pass through waypoint
  float param4 = 0; // Desired yaw angle
  float x = 31.0529340; // Latitude - degrees
  float y = -162.0703125; // Longitude - degrees
  float z = 200; // Altitude - meters
 
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);
  //uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  SerialMAV.write(buf, len);
  
}
