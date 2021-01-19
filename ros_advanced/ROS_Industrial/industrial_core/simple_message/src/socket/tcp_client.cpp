/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FLATHEADERS
#include "simple_message/socket/tcp_client.h"
#include "simple_message/log_wrapper.h"
#else
#include "tcp_client.h"
#include "log_wrapper.h"
#endif

namespace industrial
{
namespace tcp_client
{

TcpClient::TcpClient()
{

}

TcpClient::~TcpClient()
{
  LOG_DEBUG("Destructing TCPClient");
}

bool TcpClient::init(char *buff, int port_num)
{

  int rc;
  bool rtn;
  int disableNodeDelay = 1;
  addrinfo *result;
  addrinfo hints = {};

  rc = SOCKET(AF_INET, SOCK_STREAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSockHandle(rc);

    // The set no delay disables the NAGEL algorithm
    rc = SET_NO_DELAY(this->getSockHandle(), disableNodeDelay);
    if (this->SOCKET_FAIL == rc)
    {
      LOG_WARN("Failed to set no socket delay, sending data can be delayed by up to 250ms");
    }

    // Initialize address data structure
    memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
    this->sockaddr_.sin_family = AF_INET;

    // Check for 'buff' as hostname, and use that, otherwise assume IP address
    hints.ai_family = AF_INET;  // Allow IPv4
    hints.ai_socktype = SOCK_STREAM;  // TCP socket
    hints.ai_flags = 0;  // No flags
    hints.ai_protocol = 0;  // Any protocol
    hints.ai_canonname = NULL;
    hints.ai_addr = NULL;
    hints.ai_next = NULL;
    if (0 == GETADDRINFO(buff, NULL, &hints, &result))
    {
      this->sockaddr_ = *((sockaddr_in *)result->ai_addr);
    }
    else 
    {
      this->sockaddr_.sin_addr.s_addr = INET_ADDR(buff);
    }
    this->sockaddr_.sin_port = HTONS(port_num);

    rtn = true;

  }
  else
  {
    LOG_ERROR("Failed to create socket, rc: %d", rc);
    rtn = false;
  }
  return rtn;
}

bool TcpClient::makeConnect()
{
  bool rtn = false;
  int rc = this->SOCKET_FAIL;
  SOCKLEN_T addrSize = 0;

  if (!this->isConnected())
  {
    addrSize = sizeof(this->sockaddr_);
    rc = CONNECT(this->getSockHandle(), (sockaddr *)&this->sockaddr_, addrSize);
    if (this->SOCKET_FAIL != rc)
    {
      LOG_INFO("Connected to server");
      this->setConnected(true);
      rtn = true;
    }
    else
    {
      this->logSocketError("Failed to connect to server", rc, errno);
      rtn = false;
    }
  }

  else
  {
    LOG_WARN("Tried to connect when socket already in connected state");
  }

  return rtn;

}

} //tcp_client
} //industrial

