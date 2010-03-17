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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: mytools.cpp,v 1.5.2.2 2009/04/27 14:33:14 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements the load_pixmap() function.  Other misc. functions could go here too.
*/

#include <Q3MimeSourceFactory>
#include <qmime.h>
#include <q3dragobject.h>
#include <QPixmap>

#include "debug.h"
#include "mytools.h"

/*!
  Read, decode, and return the pixmap of the given name from the
  QMimeSourceFactory.
*/
QPixmap load_pixmap(const QString &name)
{
    const QMimeSource *m = Q3MimeSourceFactory::defaultFactory()->data( name );
    if ( !m )
	return QPixmap();
    QPixmap pix;
    Q3ImageDrag::decode( m, pix );
    return pix;
}

int nextValidLine(QTextStream *stream, QString *line)
{
	while (1) {
		*line = stream->readLine();
		if ( line->isNull() ) break;
		if ( !line->isEmpty() && (*line)[0]!='#' ) break;
	}
	if ( line->isNull() ) return 0;
	else return 1;
}

int nextCommentLine(QTextStream *stream, QString *line)
{
	while (1) {
		*line = stream->readLine();
		if ( line->isNull() ) break;
		if ( !line->isEmpty() && (*line)[0]=='#' ) break;
	}
	if ( line->isNull() ) return 0;
	else return 1;
}

int findString(QTextStream *stream, QString target)
{
	QString line;
	while ( nextValidLine(stream,&line) )
	{
		if ( !(QString::compare(line,target)) )
			return 1;
	}
	return 0;
}

/*! If the two paths have no common root, the returned path is
	identical to \a absolutePath. 
	Code adapted from;
	http://mrpmorris.blogspot.com/2007/05/convert-absolute-path-to-relative-path.html
*/
QString 
relativePath(QString absolutePath, QString relativeToDir )
{
	absolutePath.replace("\\","/");
	relativeToDir.replace("\\","/");
	QStringList absoluteDirectories = absolutePath.split( '/', QString::SkipEmptyParts );
	QStringList relativeDirectories = relativeToDir.split( '/', QString::SkipEmptyParts );

	//Get the shortest of the two paths
	int length = std::min(absoluteDirectories.count(), relativeDirectories.count());
		
	//Use to determine where in the loop we exited
	int lastCommonRoot = -1;
	int index;

	DBGP("Absolute path: " << absolutePath.latin1());
	DBGP("Relative path: " << relativeToDir.latin1());

	//Find common root
	for (index = 0; index < length; index++) {
		if (absoluteDirectories[index] == relativeDirectories[index]) {
			lastCommonRoot = index;
		} else {
			break;
		}
	}
	DBGP("Last common root: " << lastCommonRoot);

	//If we didn't find a common prefix then return full absolute path
	if (lastCommonRoot == -1) {
		return absolutePath;
	}
	
	//Build up the relative path
	QString relativePath;

	//Add on the ..
	for (index = lastCommonRoot + 1; index < relativeDirectories.count(); index++) {
		if (relativeDirectories[index].length() > 0) {
			relativePath.append("../");
		}
	}

	//Add on the folders
	for (index = lastCommonRoot + 1; index < absoluteDirectories.count() - 1; index++) {
		relativePath.append(absoluteDirectories[index] ).append( "/" );
	}
	relativePath.append(absoluteDirectories[absoluteDirectories.count() - 1]);
	return relativePath;
}
