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
// $Id: main.cpp,v 1.10.2.2 2009/04/27 14:33:14 cmatei Exp $
//
//######################################################################

/*! \mainpage GraspIt! Developer Documentation
  \image html logo.jpg

  These pages document the GraspIt! source code. Please remember this is
  research code. There are still plenty of pieces of code that are unfinished
  and several bugs that need to be fixed.

  More information and original source code for the packages included with
  GraspIt! can be found in the following places:

  - <b>qhull:</b> http://www.qhull.org
  - <b>maxdet:</b> http://www.stanford.edu/~boyd/old_software/MAXDET.html
*/

/*! \file
  \brief Program execution starts here.  Server is started, main window is built, and the interactive loop is started.
 */

#define GRASPITDBG

#include <iostream>
#include <graspitApp.h>
#include "graspitGUI.h"
#include "graspitServer.h"
#include "mainWindow.h"

#ifdef Q_WS_WIN
#include <windows.h>
#include <wincon.h>
#endif

int main(int argc, char **argv)
{
#ifdef GRASPITDBG
#ifdef Q_WS_WIN
  AllocConsole(); 
  freopen("conin$", "r", stdin); 
  freopen("conout$", "w", stdout); 
  freopen("conout$", "w", stderr); 
  //ios::sync_with_stdio();
#endif
#endif

  GraspItApp app(argc, argv);
 
  app.showSplash();
  QApplication::setOverrideCursor( Qt::waitCursor );

  GraspItGUI gui(argc,argv);
  /*
  If unused, the server crashes on deletion, due to what appears to be a bug in Qt
  If you need to use the server, uncomment this, but you will need to deal with the segfault on exit
  */
  //GraspItServer server(4765);
 
  app.setMainWidget(gui.getMainWindow()->mWindow);
  QObject::connect(qApp, SIGNAL(lastWindowClosed()), qApp, SLOT(quit()));

  app.closeSplash();
  QApplication::restoreOverrideCursor();

  if (!gui.terminalFailure()) {
	  gui.startMainLoop();
  }
  return 0;
}
