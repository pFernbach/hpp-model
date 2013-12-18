///
/// Copyright (c) 2013, 2014 CNRS
/// Author: Florent Lamiraux
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

#ifndef HPP_MODEL_CHILDREN_ITERATOR_HH
# define HPP_MODEL_CHILDREN_ITERATOR_HH

# include <hpp/model/joint.hh>

namespace hpp {
  namespace model {
    class ChildrenIterator {
    public:
      ChildrenIterator (const JointPtr_t& joint) : rootJoint_ (joint),
						   current_ (joint),
						   isEnd_ (false)
      {
      }
      bool end () const
      {
	return isEnd_;
      }
      void operator++ ()
      {
	// depth first
	if (current_->numberChildJoints () != 0) {
	  current_ = current_->childJoint (0);
	} else {
	  bool found = false;
	  while (!found && current_ != rootJoint_) {
	    JointPtr_t parent = current_->parent_;
	    std::size_t rankInParent = current_->rankInParent_;
	    if (rankInParent + 1 == parent->numberChildJoints ()) {
	      current_ = parent;
	    } else {
	      current_ = parent->childJoint (rankInParent + 1);
	      found = true;
	    }
	  }
	  if (current_ == rootJoint_) {
	    isEnd_ = true;
	  }
	}
      }
      JointPtr_t operator* ()
      {
	return current_;
      }

    private:
      JointPtr_t rootJoint_;
      JointPtr_t current_;
      bool isEnd_;
    }; // class ChildrenIterator
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_CHILDREN_ITERATOR_HH
