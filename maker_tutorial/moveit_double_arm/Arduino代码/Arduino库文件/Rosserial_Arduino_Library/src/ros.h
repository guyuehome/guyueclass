/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoHardware.h"
#include <avr/pgmspace.h>
#if defined(ESP8266)
  #include "Esp8266Hardware.h"
#endif

#define FLASH_CONVERTER_BUFFER 150

// converter macro for string declared with PROGMEM keyword
#define FCAST( progmem_string_literal) (reinterpret_cast<const __FlashStringHelper *>( progmem_string_literal ))

namespace ros
{
  class FlashReadOutBuffer_ : public DefaultReadOutBuffer_
  {
  private:
    char* buffer_;
    int buffer_index_;
    
    const char* readFromFlash( const __FlashStringHelper * what )
    {
      PGM_P lp_p = (PGM_P) what ;
      
      int offset = buffer_index_;
      
      while ( buffer_index_ < FLASH_CONVERTER_BUFFER - 1 ) 
      {
	buffer_[ buffer_index_ ] = pgm_read_byte( lp_p++ );
		
	if ( buffer_[ buffer_index_ ] == 0)
	{
	  buffer_index_++;
	  
	  break;
	}
	
	buffer_index_++;
      }
      
      if ( buffer_index_ >= FLASH_CONVERTER_BUFFER - 1 )
      {
	read_out_error_ = DefaultReadOutBuffer_::BufferOverflow;
      }
      
      return buffer_ + offset ;
    }
    
  protected:
    ReadoutError read_out_error_;
    
    // overrider of method from DefaultReadOutBuffer_
    virtual const char * readTopic( const char * topic, bool from_flash )
    {
      if ( from_flash )
      {
	return readFromFlash( reinterpret_cast<const __FlashStringHelper *>( topic ) );
      }
      else
      {
	return topic;
      }
    }
        
  public:
    FlashReadOutBuffer_() 
    {
      buffer_index_ = 0;
      
      buffer_ = new char[ FLASH_CONVERTER_BUFFER ];
      
      buffer_[0] = 0;  //zero termination at beginning
      buffer_[ FLASH_CONVERTER_BUFFER - 1 ] = 0; // assure zero termination
    }
    
    virtual ~FlashReadOutBuffer_() 
    {
      delete buffer_;
    }
    
    // for md5sum / msg type
    virtual const char *  readMsgInfo( const char * msg_info )
    {
      return readFromFlash( reinterpret_cast<const __FlashStringHelper*>( msg_info ) );
    }
    
    // for log msgs
    const char *  readLog( const __FlashStringHelper * msg )
    {
      return readFromFlash( msg );
    }
  };
  
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__)
  /* downsize our buffers */
  typedef NodeHandle_<ArduinoHardware, 6, 6, 150, 150, FlashReadOutBuffer_> NodeHandle;

#elif defined(__AVR_ATmega328P__)

  typedef NodeHandle_<ArduinoHardware, 25, 25, 280, 280, FlashReadOutBuffer_> NodeHandle;

#elif defined(ESP8266)

  typedef NodeHandle_<Esp8266Hardware> NodeHandle;
  
#elif defined(SPARK)
  typedef NodeHandle_<ArduinoHardware, 10, 10, 2048, 2048> NodeHandle;

#else

  typedef NodeHandle_<ArduinoHardware, 25, 25, 512, 512, FlashReadOutBuffer_> NodeHandle;

#endif   
  
}

#endif
