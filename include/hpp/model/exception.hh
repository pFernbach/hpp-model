///
/// Copyright (c) 2011 CNRS
/// Authors: Florent Lamiraux
///
///
// This file is part of hpp-model
// hpp-model is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-model is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-model  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MODEL_EXCEPTION_HH
#define HPP_MODEL_EXCEPTION_HH

#include <string>

namespace hpp {
  namespace model {
    class Exception : public std::exception
    {
    public:
      virtual ~Exception() throw ();
      Exception (const std::string& message) : message_(message) {}
      virtual const char* what () const throw ()
      {
	return message_.c_str ();
      }
    private:
      std::string message_;
    }; // class Exception
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_EXCEPTION_HH
