//######################################################################
//
// This file is part of GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
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
// Author(s): Andrew T. Miller 
//
// $Id: archBuilderDlg.cpp,v 1.1.2.1 2009/04/24 22:19:24 cmatei Exp $
//
//######################################################################

#include "archBuilderDlg.h"

#include <QValidator>

void ArchBuilderDlg::init()	{
	innerRadiusEdit->setValidator(new QDoubleValidator(0,1.0e+6,6,this));
	outerRadiusEdit->setValidator(new QDoubleValidator(0,1.0e+6,6,this));
	thicknessEdit->setValidator(new QDoubleValidator(0,1.0e+6,6,this));
	QString val;
	innerRadiusEdit->setText( val.setNum(900) );
	outerRadiusEdit->setText( val.setNum(1100) );
	thicknessEdit->setText( val.setNum(200) );
	numberBlocksBox->setValue( 9 );
	supportsCheckBox->setChecked( true );
}
