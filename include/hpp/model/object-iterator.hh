//
// Copyright (c) 2013, 2014 CNRS
// Author: Florent Lamiraux
//
//
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

#ifndef HPP_MODEL_OBJECT_ITERATOR_HH
#define HPP_MODEL_OBJECT_ITERATOR_HH

# include <vector>
# include <hpp/model/config.hh>
# include <hpp/model/fwd.hh>

namespace hpp {
  namespace model {
    /// Iterator over all inner objects of a Device.
    class HPP_MODEL_DLLAPI ObjectIterator {
    public:
      ObjectIterator (Device& device, Request_t type);
      const CollisionObjectPtr_t& operator* ();
      bool operator== (const ObjectIterator& other) const;
      bool operator!= (const ObjectIterator& other) const;
      void operator++ ();
      void setToEnd ();
      bool isEnd () const;
    private:
      Device& device_;
      Request_t type_;
      JointVector_t joints_;
      JointVector_t::iterator jointIt_;
      ObjectVector_t objects_;
      ObjectVector_t::iterator objIt_;

    }; // class ObjectIterator
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_OBJECT_ITERATOR_HH
