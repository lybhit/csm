#ifndef _EXCEPTION_H
#define _EXCEPTION_H

#include <string>
#include <fstream>
#include <limits>
#include <algorithm>
#include <map>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <math.h>
#include <float.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

#include "Types.h"





namespace karto
{
  /**
   * Exception class. All exceptions thrown from Karto will inherit from this class or be of this class
   */
  class KARTO_EXPORT Exception
  {
  public:
    /**
     * Constructor with exception message
     * @param rMessage exception message (default: "Karto Exception")
     * @param errorCode error code (default: 0)
     */
    Exception(const std::string& rMessage = "Karto Exception", kt_int32s errorCode = 0)
      : m_Message(rMessage)
      , m_ErrorCode(errorCode)
    {
    }

    /**
     * Copy constructor
     */
    Exception(const Exception& rException)
      : m_Message(rException.m_Message)
      , m_ErrorCode(rException.m_ErrorCode)
    {
    }

    /**
     * Destructor
     */
    virtual ~Exception()
    {
    }

  public:
    /**
     * Assignment operator
     */
    Exception& operator = (const Exception& rException)
    {
      m_Message = rException.m_Message;
      m_ErrorCode = rException.m_ErrorCode;

      return *this;
    }

  public:
    /**
     * Gets the exception message
     * @return error message as string
     */
    const std::string& GetErrorMessage() const
    {
      return m_Message;
    }

    /**
     * Gets error code
     * @return error code
     */
    kt_int32s GetErrorCode()
    {
      return m_ErrorCode;
    }

  public:
    /**
     * Write exception to output stream
     * @param rStream output stream
     * @param rException exception to write
     */
    friend KARTO_EXPORT std::ostream& operator << (std::ostream& rStream, Exception& rException);

  private:
    std::string m_Message;
    kt_int32s m_ErrorCode;
  };  // class Exception

  }//namespace karto



  #endif