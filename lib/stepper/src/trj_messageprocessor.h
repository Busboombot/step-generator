#ifndef _MessageProcessor_h_
#define _MessageProcessor_h_


#include <functional>

#include <FastCRC.h>
#include <PacketSerial.h>
#include "trj_ringbuffer.h"
#include "trj_stepper.h"
#include "trj_stepdriver.h"


#define BUFFER_SIZE 265
#define MAX_PAYLOAD_SIZE  (BUFFER_SIZE-6) // 4 for header, 1 start, one zero at the end. 

/* Enumeration that describe the contents of a command message */
enum class CommandCode : uint8_t {
  
  ACK =     1,
  NACK =    2,
  DONE =    3,  // A Movement command is finished
  EMPTY  =  4, // Queue is empty, nothing to do. 

  RMOVE =   11,  // A relative movement segment, with just the relative distance.   
  AMOVE =   12,  // An absolute movement
  JMOVE =   13,   // A Jog movement. 


  RUN =     21,
  STOP =    22,
  RESET =   23,  // 
  ZERO  =   24,  // Zero positions
  CONFIG =  25, // Reset the configuration
  AXES =    26, // Configure an axis



  MESSAGE = 91,  // Payload is a message; the next packet is text
  ERROR =   92,  // Some error
  ECHO  =   93,  // Echo the incomming header
  DEBUG  =  94,  // 
  INFO   =  95, // Return info messages

  NOOP  =   99,  // Does nothing, but get ACKED

  POSITIONS= 101,  // Position report. ( Unused)
  
};

/* Header on every command packet */
struct PacketHeader {
  uint16_t seq;       // Packet sequence number
  CommandCode code;   // Command code      
  uint8_t crc = 0;    // Payload CRC8
  
};  // 4

// Payload of the move command
struct Moves {
  uint32_t segment_time = 0; // total segment time, in microseconds // 4
  int32_t x[N_AXES];
}; // 8



extern CurrentState current_state;

#define MESSAGE_BUF_SIZE 254

class Loop; 
class MessageProcessor;


class MessageProcessorPacketSerial: public PacketSerial{

  private:

    MessageProcessor* mp;

  public:

    MessageProcessorPacketSerial() : mp(0){}

    MessageProcessorPacketSerial(MessageProcessor *mp) : mp(mp){ }

    MessageProcessor* getMessageProcessor(){ return mp; }

};

class MessageProcessor{

private:

  Stream& stream;
  MessageProcessorPacketSerial ps;


  uint8_t buffer[BUFFER_SIZE]; // Outgoing message buffer

  Loop &loop;

  int lastSegNum=0;


public: 

  MessageProcessor(Stream &stream, Loop& loop) : stream(stream), ps(this), loop(loop) {
    
    ps.setStream(&stream);

    ps.setPacketHandler([](const void* sender, const uint8_t* buffer, size_t size) {
      ((MessageProcessorPacketSerial*)sender)->getMessageProcessor()->processPacket(buffer, size);
    });
  }

  void update() {
    ps.update();
  }

  void setLastSegNum(int v){
    lastSegNum = v;
  }

  void sendAck(uint16_t seq){
   send((const uint8_t*)&current_state, CommandCode::ACK, seq, sizeof(current_state));
  }

  // An ACK with no current state
  void sendEmptyAck(uint16_t seq){
   send(CommandCode::ACK, seq, 0);
  }

  void sendNack(){ 
    send(CommandCode::NACK, lastSegNum, 0); 
  }

  void sendDone(uint16_t seq, CurrentState &current_state){ 
    send((const uint8_t*)&current_state, CommandCode::DONE, seq, sizeof(current_state));
  }

  void sendEmpty(uint16_t seq, CurrentState &current_state){ 
    send((const uint8_t*)&current_state, CommandCode::EMPTY, seq, sizeof(current_state));
  }
 
  void printf(const char* fmt, ...);

  void sendMessage(const char *message_){
    //Serial1.println(message_);
    send((const uint8_t*)message_, CommandCode::MESSAGE, lastSegNum, strlen(message_));
  }


private:

  uint8_t crc(size_t length);

  // Perform operation for each type of message recieved
  void processPacket(const uint8_t* buffer_, size_t size);

  
  void processMove(const uint8_t* buffer_, size_t size);

  void send(size_t length);

  void send(CommandCode code, uint16_t seq, size_t length);

  void send(const uint8_t* buffer_, CommandCode code, uint16_t seq, size_t length);

};


#endif // _MessageProcessor_h_