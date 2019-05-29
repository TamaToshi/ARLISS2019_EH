#include <mavlink.h>
#include <SoftwareSerial.h>
 
#define RXpin 2
#define TXpin 3
SoftwareSerial SerialPix(RXpin, TXpin); // sets up serial communication on pins 3 and 2


void setup() {
  SerialPix.begin(57600); //RXTX from Pixhawk (Port 19,18 Arduino Mega)
  Serial.begin(57600); //Main serial port for console output
  request_datastream();

}
 
void loop() {

   MavLink_receive_GPS();
   MavLink_receive();

}
 

//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
void MavLink_receive_GPS()
  {     
   mavlink_message_t msg;
   mavlink_status_t status;

  while(SerialPix.available())
  {
    uint8_t c= SerialPix.read();
 
    //Get new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
 
    //Handle new message from autopilot
      switch(msg.msgid)
      {
 
        case MAVLINK_MSG_ID_GPS_RAW_INT:
      {
        mavlink_gps_raw_int_t packetGPS;
        mavlink_msg_gps_raw_int_decode(&msg, &packetGPS);
        
        //Serial.print("\nGPS Fix: ");Serial.println(packetGPS.fix_type);
        Serial.print("\nPlane Latitude: ");Serial.println(packetGPS.lat);
        Serial.print("Plane Longitude: ");Serial.println(packetGPS.lon);
        //Serial.print("GPS Speed: ");Serial.println(packetGPS.vel);
        //Serial.print("Sats Visible: ");Serial.println(packetGPS.satellites_visible);
        Serial.print("Plane Altitude(mm): ");Serial.println(packetGPS.alt);
        //時間も取れるから必要あれば送る

      }
        break;
      }

    }
  }
}



void MavLink_receive()
  { 
  mavlink_message_t msg;
  mavlink_status_t status;
 
  while(SerialPix.available())
  {
    uint8_t c= SerialPix.read();
 
    //Get new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
 
    //Handle new message from autopilot
      switch(msg.msgid)
      {
 
      
        case MAVLINK_MSG_ID_ATTITUDE:
      {
        mavlink_attitude_t packetPOS;
        mavlink_msg_attitude_decode(&msg, &packetPOS);

      Serial.print("\nPlane Roll: ");Serial.println(packetPOS.roll);
        Serial.print("Plane Pitch: ");Serial.println(packetPOS.pitch);

      }

    }
  }
}
}



void request_datastream() {
//Request Data from Pixhawk
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x01; //number of times per second to request the data in hex
  uint8_t _start_stop = 1; //1 = start, 0 = stop
 
// STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM and more importantly at:
     https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */
 
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
 
  SerialPix.write(buf, len); //Write data to serial port
}
