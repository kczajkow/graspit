//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2004  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under an Research and Educational Use
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author:  Andrew T. Miller (amiller@cs.columbia.edu)
//
// $Id: pincer.h,v 1.1.1.1.10.1 2009/04/27 14:33:10 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the special %Pincer robot class
 */

#ifndef PINCER_H

#include "robot.h"

//! Created for a 2 fingered hand while testing a DOF controller specifically designed for this hand.
/*!  Created for a 2 fingered hand while testing a DOF controller specifically
  designed for this hand.
*/
class Pincer : public Hand {

 public:
  void DOFController(double timeStep);

};

#define PINCER_H
#endif
