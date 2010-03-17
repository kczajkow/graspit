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
// Author(s):  Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: mytools.h,v 1.8.2.2 2009/04/27 14:33:10 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Various useful macros and functions for dealing with errors and other common operations.
*/

#ifndef _MY_TOOLS_HXX_
#define _MY_TOOLS_HXX_

#include <qmessagebox.h>
#include <qpixmap.h>
#include <QString>
#include <QTextStream>

#define SUCCESS 0
#define FAILURE -1
#define TERMINAL_FAILURE -2

void show_errors(int, char* = NULL);

#ifndef MAX
#define MAX(A,B)	((A) > (B) ? (A) : (B))
#endif

#ifndef MIN
#define MIN(A,B)	((A) < (B) ? (A) : (B))
#endif

#ifndef ROUND
#define ROUND(A)        ((A) >= 0 ? (int)((A)+.5) : -(int)(.5-(A)))
#endif

#ifndef BATCH_PROCESSING
//! Puts up a QT warning box with the given message
#define QTWARNING(MSG_) QMessageBox::warning(NULL,"GraspIt!",MSG_,QMessageBox::Ok, \
											 Qt::NoButton,Qt::NoButton)
#else
//If batch processing is enabled we supress error messages that require user attention
//alternatively, we could redirect this into a log file
#define QTWARNING(MSG_) 
#endif

//for inlining only when building in release mode
#ifdef GRASPIT_RELEASE
#define INLINE_RELEASE inline
#else
#define INLINE_RELEASE
#endif

QPixmap load_pixmap(const QString &name);

//useful macros for standardizing error printing
#define pr_error(EXPR_)                         \
{                                               \
    fprintf(stderr,">>!>> ");                   \
    fprintf(stderr,EXPR_);                      \
    fprintf(stderr,"\n");                       \
}

// EXPR_ must be of the form: (stderr,"..%...%...%...",args)
#define pr_errArgs(EXPR_)                       \
{                                               \
    fprintf(stderr,">>!>> ");                   \
    fprintf EXPR_;                              \
    fprintf(stderr,"\n");                       \
}

/*!
  Extracts a name from a filename by stripping off any directory path
  preceeding the filename, and discarding any file extension.  \a maxsize
  is the maximum size of the returned name.
*/
inline void
extractName(const char *filename,char *name,int maxSize)
{
  const char *p;
  char *p2;

  //extract just the filename for a longer path
  p=filename+strlen(filename)-1;
  while (p != filename && *p != '/') p--;
  if (*p == '/') p++;
  strncpy(name,p,maxSize);
 
  // discard the ending
  p2 = name;
  while (*p2 != '\0' && *p2 != '.') p2++;
  *p2 = '\0';
}

//! Finds the next line in a stream that is not blank or a comment
int nextValidLine(QTextStream *stream, QString *line);

//! Finds the next line that is an Inventor comment (starts with a #)
int nextCommentLine(QTextStream *stream, QString *line);

//! Finds a keyword on a line in a stream and positions the stream after it
int findString(QTextStream *stream, QString target);

//! Returns the path \a absolutePath made relative to the path \a relativeTo
QString relativePath(QString absolutePath, QString relativeToDir);

#endif
