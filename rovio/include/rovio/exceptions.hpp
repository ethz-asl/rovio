/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_EXCEPTION_HPP_
#define ROVIO_EXCEPTION_HPP_

#include <string>
#include <iostream>
#include <exception>
#include <typeinfo>

#define ROVIO_THROW(exceptionType) {                    \
  throw exceptionType(__FUNCTION__,__FILE__,__LINE__);  \
}

namespace rovio {
  struct ExceptionBase : public std::exception
  {
    std::string message_;
    std::string function_;
    std::string file_;
    int line_;
    ExceptionBase(std::string message,std::string function, std::string file, int line){
      message_ = message;
      function_ = function;
      file_ = file;
      line_ = line;
    }
    virtual ~ExceptionBase(){};
    virtual const char * what () const throw ()
    {
      return (file_ + ":" + std::to_string(line_) + ": " + function_ + "()" + " " + message_).c_str();
    }
  };

  struct CameraNullPtrException : public ExceptionBase
  {
    CameraNullPtrException(std::string function, std::string file, int line): ExceptionBase("Camera pointer is null!",function,file,line){}
  };
}

/* Usage:
 * ROVIO_THROW(rovio::CameraNullPtrException);
 */

#endif /* ROVIO_EXCEPTION_HPP_ */
