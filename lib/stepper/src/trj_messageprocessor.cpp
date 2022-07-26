
#include<stdarg.h>

#include "trj_messageprocessor.h"
#include "trj_sdstepper.h"
#include "trj_config.h"
#include "trj_loop.h"
#include "trj_debug.h"

FastCRC8 CRC8;

extern char printf_buffer[1024];

// print to the print buffer
void MessageProcessor::printf(const char* fmt, ...){
    va_list args;
    va_start(args,fmt);
    vsprintf(printf_buffer, fmt,args);
    va_end(args);
    sendMessage(printf_buffer);
}

uint8_t MessageProcessor::crc(size_t length){
    PacketHeader *ph = (PacketHeader*)buffer;
    ph->crc = 0;
    ph->crc = CRC8.smbus(buffer, length);
    return ph->crc;
}

void MessageProcessor::send(size_t length){
    ps.send((const uint8_t*)buffer, length+sizeof(PacketHeader));
}

void MessageProcessor::send(CommandCode code, uint16_t seq, size_t length){
    PacketHeader *ph = (PacketHeader*)&buffer;
    ph->code = code;
    ph->seq = seq;
    crc(length+sizeof(PacketHeader));
    send(length);
}

void MessageProcessor::send(const uint8_t* buffer_, CommandCode code, uint16_t seq, size_t length){
    memcpy(buffer+sizeof(PacketHeader), buffer_, length>MAX_PAYLOAD_SIZE?MAX_PAYLOAD_SIZE:length);
    send(code, seq, length);

}

void MessageProcessor::processPacket(const uint8_t* buffer_, size_t size){
    DEBUG_SET_4
    PacketHeader *ph = (PacketHeader*)buffer_;

    uint8_t that_crc = ph->crc;
    crc(size); // Calc crc on buffer, put back into ph. 
    if (that_crc != ph->crc){
      
      sendNack();
      return;
    } 

    if(ph->code == CommandCode::NOOP){
      //Do Nothing
    } else if(ph->code == CommandCode::ECHO){
      send((const uint8_t*)buffer_ + sizeof(PacketHeader), ph->code, ph->seq, size - sizeof(PacketHeader));
      
      return; // Echos are their own ack.

    } else if(ph->code == CommandCode::RMOVE || ph->code == CommandCode::AMOVE  
              || ph->code == CommandCode::JMOVE  || ph->code == CommandCode::HMOVE ) {
      loop.processMove(buffer_, size);

    } else if(ph->code == CommandCode::RUN){
      loop.start();

    } else if (ph->code == CommandCode::STOP){
      loop.stop();

    } else if (ph->code == CommandCode::RESET) {
      loop.reset();

    } else if (ph->code == CommandCode::ZERO) {
      loop.zero();

    } else if(ph->code == CommandCode::CONFIG) {
      loop.setConfig( (Config*) (buffer_+sizeof(PacketHeader)));

    } else if(ph->code == CommandCode::AXES) {
      loop.setAxisConfig( (AxisConfig*) (buffer_+sizeof(PacketHeader)));


    } else if(ph->code == CommandCode::INFO) {
      loop.printInfo();
    }
  
    sendAck(ph->seq);
    DEBUG_CLEAR_4

  }

