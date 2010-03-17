#include "scanDlg.h"

#include <Q3FileDialog>

#include <fstream>
#include <unistd.h>
#include <time.h>

#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoVertexProperty.h>
#include <Inventor/nodes/SoPackedColor.h>
#include <Inventor/nodes/SoMaterialBinding.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoScale.h>

#include "world.h"
#include "robot.h"
#include "body.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "scanSimulator.h"
#include "scanSimDlg.h"

#include "smartScan.h"
#include "octree.h"
#include "object.h"
#include "objectDetector.h"
#include "graspPoint.h"
#include "graspPlanner.h"
#include "listen_node/rosSystemCalls.h"
#include "listen_node/scanListenNode.h"

void ScanDlg::exitButton_clicked()
{
	if (mNode) {
		mNode->shutdown();
		delete mNode;
		fprintf(stderr,"Node deleted\n");
	}
	while( !mScans->empty() ) {
		delete mScans->back();
		mScans->pop_back();
	}
	delete mScans;
	//Bodies are left in the GraspIt! world
	mBodies->clear();
	delete mBodies;
	if (mOctree) delete mOctree;
	
	fprintf(stderr,"Scan cleaned up\n");

	QDialog::accept();
}


void ScanDlg::init()
{
	fprintf(stderr,"DLG init\n");
	mWorld= graspItGUI->getIVmgr()->getWorld();
  
	actionBox->insertItem("Delaunay");
	actionBox->insertItem("ICP");
	actionBox->insertItem("Save");
	actionBox->insertItem("Save as VRML");
	actionBox->insertItem("Delete");
	actionBox->insertItem("Crop");
	actionBox->insertItem("Remove outliers");
	actionBox->insertItem("Remove grazing points");
	actionBox->insertItem("Remove histogramed plane");
	actionBox->insertItem("Remove ransac plane");
	actionBox->insertItem("Create mesh");
	actionBox->insertItem("Load transform");
	actionBox->insertItem("Join scans");
	actionBox->insertItem("Subtract scans");
	actionBox->insertItem("Find nbrs");
	actionBox->insertItem("Connected components");
	actionBox->insertItem("Principal axes");
	actionBox->insertItem("Object detector");
	actionBox->insertItem("Grasp points");
	actionBox->insertItem("Insert in octree");
	actionBox->insertItem("Trace in octree");
	actionBox->insertItem("Collision test");
	actionBox_activated(0);

	ocxEdit->setText("0");
	ocyEdit->setText("0");
	oczEdit->setText("0");
	odxEdit->setText("1");
	odyEdit->setText("1");
	odzEdit->setText("1");
	odepthEdit->setText("6");

	mScans = new std::vector<SmartScan*>;
	mBodies = new std::vector<Body*>;
	mOctreeBody = NULL;

	scanTypeBox->insertItem("Simulated");

	if ( getROSState() != ROS_READY ) {
		/*
		getScanButton->setEnabled(FALSE);
		scanTypeBox->setEnabled(FALSE);
		*/
		mNode = NULL;
	} else {
		scanTypeBox->insertItem("Last complete scan");
		scanTypeBox->insertItem("Current incompl. scan");
		mNode = new ScanListenNode();
		fprintf(stderr,"Node created\n");
	}
	fprintf(stderr,"DLG init done\n");
}

void ScanDlg::destroy()
{
}

transf ScanDlg::transfFromFloat(float *trd)
{
	mat3 R( vec3(trd[0], trd[4], trd[8]),
		vec3(trd[1], trd[5], trd[9]),
		vec3(trd[2], trd[6], trd[10]) );
	vec3 t(trd[3], trd[7], trd[11]);
	return transf(R,t);
}

void ScanDlg::getScanButton_clicked()
{

	SmartScan *cloud;

	switch ( scanTypeBox->currentItem() ) {
	case 0:
		cloud = simulateScan();
		break;
	case 1:
		cloud = mNode->getFullScan();
		break;
	case 2:
		cloud = mNode->getCurrentScan();
		break;
	default:
		cloud = NULL;
		break;
	}
	if (!cloud) {
		fprintf(stderr,"Cloud unavailable!\n");
		return;
	}

	addScan(cloud, NULL);
}

void ScanDlg::loadScanButton_clicked()
{
	QString fn = QFileDialog::getOpenFileName( QString(getenv("GRASPIT"))+QString("/models/scans"),
						   "ROS Scan files (*.txt)", this ) ;
	if (fn.isEmpty()) return;
	std::fstream fs;
	fs.open(fn.latin1(),std::fstream::in);
	if (fs.fail()){
		fprintf(stderr,"Failed to open file %s\n",fn.latin1());
		return;
	}

	SmartScan *cloud = new SmartScan();
	if (!cloud->readFromFile(fs)) {
		fprintf(stderr,"Failed to read scan from file %s\n",fn.latin1());
		delete cloud; cloud = NULL;
	}
	fs.close();
	QString name = fn.section('/',-1,-1);
	name = name.section('.',0,0);
	if (cloud) addScan(cloud, &name);
}

Body* ScanDlg::createBodyFromScan( SmartScan* cloud )
{
	GraspableBody *newBody = new GraspableBody(mWorld,"Scan_cloud");
	newBody->initializeIV();
	addGeometryFromScan( newBody->getIVGeomRoot(), cloud);
	newBody->addIVMat();
	newBody->addToIvc();
	return newBody;
}

void ScanDlg::addGeometryFromScan( SoGroup *root, SmartScan *cloud )
{
	int maxIntensity = 3000;
	unsigned char red, green, blue;

	//assign each scan a different color so we can tell them apart
	static unsigned char randRed = 200;
	static unsigned char randBlue = 100;
	randRed += 30;
	if (randBlue==100) randBlue = 0;
	else randBlue = 100;

	uint32_t *colors = new uint32_t[cloud->size()];
	SbVec3f *vertices = new SbVec3f[cloud->size()];
	//SbColor *colors = new SbColor[cloud->size()];
	for (int i=0; i<(int)cloud->size(); i++) {
		vertices[i].setValue( cloud->getPoint(i).x, 
				      cloud->getPoint(i).y, 
				      cloud->getPoint(i).z );
		//for now the smart scan does not have intensities
		/*
		blue = 0;
		int rawInt = maxIntensity - 0;
		if (rawInt < 0) rawInt = 0;
		red = (unsigned char)(255 * (float) rawInt / maxIntensity );
		*/
		red = randRed;
		green = 255 - red;
		blue = randBlue;
		colors[i] =  ((uint32_t)red << 24) | ((uint32_t)green << 16) | ((uint32_t)blue << 8) | 0xff;
	}

	SoSeparator *cloudRoot = new SoSeparator;

	//we don't need this anymore; we have killed the notion of a scan tranforms.
	/*
	SoTransform *cloudTrans = new SoTransform;
	float t[16];
	cloud->getTransform(t);
	transf tr = transfFromFloat(t);
	tr.toSoTransform(cloudTrans);
	cloudRoot->addChild(cloudTrans);
	*/

	SoCoordinate3 *coords = new SoCoordinate3;
	coords->point.setValues( 0, cloud->size(), vertices );

	SoMaterialBinding *matBind = new SoMaterialBinding;
	matBind->value = SoMaterialBindingElement::PER_VERTEX;

	SoPackedColor *colorNode = new SoPackedColor;
	colorNode->orderedRGBA.setValues(0,cloud->size(),colors);

	SoPointSet *points = new SoPointSet;
	points->numPoints = cloud->size();

	//	SoVertexProperty *vertexCloud = new SoVertexProperty;
	//vertexCloud->orderedRGBA.setValues(0,cloud->size(),colors);
	//vertexCloud->vertex.setValues(0,cloud->size(), vertices);
	//	points->vertexProperty = vertexCloud;

	//add a cone showing position and direction of scan
	
	SoSeparator *coneSep = new SoSeparator();
	SoTransform *coneTransf = new SoTransform;
	float px,py,pz,dx,dy,dz,ux,uy,uz;
	cloud->getScanner(px,py,pz,dx,dy,dz,ux,uy,uz);
	vec3 x(dx,dy,dz), z(ux,uy,uz),y;
	x = normalise(x);
	z = normalise(z);
	y = z * x;
	mat3 R(x,y,z);
	Quaternion q(R);
	coneTransf->rotation.setValue(q.x, q.y, q.z, q.w);
	coneTransf->translation.setValue( SbVec3f(px,py,pz) );
	coneSep->addChild(coneTransf);
	SoTransform *pointTrans = new SoTransform;
	pointTrans->rotation.setValue( SbVec3f(0,0,1), - M_PI / 2.0);
	//pointTrans->translation.setValue( SbVec3f(0, -0.1, 0));
	coneSep->addChild(pointTrans);
	SoMaterial *coneMat = new SoMaterial();
	coneMat->diffuseColor = SbColor(0.2f,0.0f,0.8f);
	coneMat->ambientColor = SbColor(0.1f,0.0f,0.2f);
	coneMat->emissiveColor = SbColor(0.1f,0.0f,0.4f);
	coneMat->transparency = 0.0f;
	coneSep->addChild(coneMat);
	SoCone *cone = new SoCone();
	cone->height = 0.1;
	cone->bottomRadius = 0.03;
	//cone->height = 30;
	//cone->bottomRadius = 5;
	coneSep->addChild(cone);


	cloudRoot->addChild( coneSep );
	cloudRoot->addChild( coords );
	cloudRoot->addChild( matBind );
	cloudRoot->addChild( colorNode );
	cloudRoot->addChild( points );

	root->addChild(cloudRoot);
}

void ScanDlg::setTransformFromScan(int s)
{
	//obsolete
}

void ScanDlg::goButton_clicked()
{
	if ( mScans->empty() ) return;

	clock_t startTime = clock();

	if (actionBox->currentText() == "Delaunay") {
		delaunay( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Save") {
		saveScan( (*mScans)[target1Box->currentItem()] );
	} else if (actionBox->currentText() == "Save as VRML") {
		saveScanVrml( target1Box->currentItem() );
	} else if (actionBox->currentText() == "ICP") {
		ICP( target1Box->currentItem(), target2Box->currentItem() );
	} else if (actionBox->currentText() == "Crop") {
		crop( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Delete") {
		removeScan( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Remove outliers") {
		removeOutliers( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Remove grazing points") {
		removeGrazingPoints( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Remove histogramed plane") {
		removeHistoPlane( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Remove ransac plane") {
		removeRansacPlane( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Load transform") {
		loadTransform( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Join scans") {
		joinScans( target1Box->currentItem(), target2Box->currentItem() );
	} else if (actionBox->currentText() == "Subtract scans") {
		subtractScans( target1Box->currentItem(), target2Box->currentItem() );
	} else if (actionBox->currentText() == "Find nbrs") {
		findNbrs( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Create mesh") {
		createMesh( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Connected components") {
		connectedComponents( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Principal axes") {
		principalAxes( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Object detector") {
		objectDetector( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Grasp points") {
		graspPoints( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Insert in octree") {
		insertInOctree( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Trace in octree") {
		traceInOctree( target1Box->currentItem() );
	} else if (actionBox->currentText() == "Collision test") {
		collisionTest( target1Box->currentItem() );
	}	

	
	clock_t stopTime = clock();
	double runningTime = (float)(stopTime - startTime) / CLOCKS_PER_SEC;
	fprintf(stderr,"Running time: %.2f seconds\n", runningTime);
	
}

void ScanDlg::actionBox_activated( int )
{
	if (actionBox->currentText() == "Delaunay") {
		target2Box->setEnabled(FALSE);
		setParams("Tol.","Alpha","","",0.001,0.5,0,0);
	} else if (actionBox->currentText() == "Save") {
		target2Box->setEnabled(FALSE);
		setParams("","","","",0,0,0,0);
	} else if (actionBox->currentText() == "Saveas VRML") {
		target2Box->setEnabled(FALSE);
		setParams("","","","",0,0,0,0);
	} else if (actionBox->currentText() == "Delete") {
		target2Box->setEnabled(FALSE);
		setParams("","","","",0,0,0,0);
	} else if (actionBox->currentText() == "ICP") {
		target2Box->setEnabled(TRUE);
		setParams("","","","",0,0,0,0);
	} else if (actionBox->currentText() == "Crop") {
		target2Box->setEnabled(FALSE);
		setParams("x","y","z","d",0.5,0.5,0,2);
	} else if (actionBox->currentText() == "Remove outliers") {
		target2Box->setEnabled(FALSE);
		setParams("radius","nbrs","","",0.01,5,0,0);
	} else if (actionBox->currentText() == "Remove grazing points") {
		target2Box->setEnabled(FALSE);
		setParams("thresh.","outl.","radius","nbrs",10,1,0.01,5);
	} else if (actionBox->currentText() == "Remove histogramed plane") {
		target2Box->setEnabled(FALSE);
		setParams("radius","nbrs","thresh.","",0.02,5,0.02,0);
	} else if (actionBox->currentText() == "Remove ransac plane") {
		target2Box->setEnabled(FALSE);
		setParams("search thresh","iters.","rem. thresh","",0.005,500,0.01,0);
	} else if (actionBox->currentText() == "Load transform") {
		target2Box->setEnabled(FALSE);
		setParams("","","","",0,0,0,0);
	} else if (actionBox->currentText() == "Join scans") {
		target2Box->setEnabled(TRUE);
		setParams("","","","",0,0,0,0);
	} else if (actionBox->currentText() == "Subtract scans") {
		target2Box->setEnabled(TRUE);
		setParams("thresh.","","","",0.01,0,0,0);
	} else if (actionBox->currentText() == "Find nbrs") {
		target2Box->setEnabled(FALSE);
		setParams("x","y","z","radius",0.5,0,0,0.25);
	} else if (actionBox->currentText() == "Create mesh") {
		target2Box->setEnabled(FALSE);
		setParams("resol.","","","",0.01,0,0,0);
	} else if (actionBox->currentText() == "Connected components") {
		target2Box->setEnabled(FALSE);
		setParams("conn.","size","","",0.02,500,0,0);
	} else if (actionBox->currentText() == "Principal axes") {
		target2Box->setEnabled(FALSE);
		setParams("","","","",0,0,0,0);
	} else if (actionBox->currentText() == "Object detector") {
		target2Box->setEnabled(FALSE);
		setParams("","","","",0,0,0,0);
	} else if (actionBox->currentText() == "Grasp points") {
		target2Box->setEnabled(FALSE);
		setParams("resol.","","","",8,0,0,0);
	} else if (actionBox->currentText() == "Insert in octree") {
		target2Box->setEnabled(FALSE);
		setParams("","","","",0,0,0,0);
	} else if (actionBox->currentText() == "Trace in octree") {
		target2Box->setEnabled(FALSE);
		setParams("","","","",0,0,0,0);
	} else if (actionBox->currentText() == "Collision test") {
		target2Box->setEnabled(FALSE);
		setParams("","","","",0,0,0,0);
	}
}

void ScanDlg::setParams(QString q1, QString q2, QString q3, QString q4, float v1, float v2, float v3, float v4)
{
	QString n1,n2,n3,n4;
	n1.setNum(v1); n2.setNum(v2); n3.setNum(v3); n4.setNum(v4);

	if (q1.isEmpty()) {
		p1Label->setText("N/A"); p1Edit->clear(); p1Label->setEnabled(FALSE); p1Edit->setEnabled(FALSE);
	} else {
		p1Label->setText(q1); p1Edit->setText(n1); p1Label->setEnabled(TRUE); p1Edit->setEnabled(TRUE);
	}
	if (q2.isEmpty()) {
		p2Label->setText("N/A"); p2Edit->clear(); p2Label->setEnabled(FALSE); p2Edit->setEnabled(FALSE);
	} else {
		p2Label->setText(q2); p2Edit->setText(n2); p2Label->setEnabled(TRUE); p2Edit->setEnabled(TRUE);
	}
	if (q3.isEmpty()) {
		p3Label->setText("N/A"); p3Edit->clear(); p3Label->setEnabled(FALSE); p3Edit->setEnabled(FALSE);
	} else {
		p3Label->setText(q3); p3Edit->setText(n3); p3Label->setEnabled(TRUE); p3Edit->setEnabled(TRUE);
	}
	if (q4.isEmpty()) {
		p4Label->setText("N/A"); p4Edit->clear(); p4Label->setEnabled(FALSE); p4Edit->setEnabled(FALSE);
	} else {
		p4Label->setText(q4); p4Edit->setText(n4); p4Label->setEnabled(TRUE); p4Edit->setEnabled(TRUE);
	}
}

void ScanDlg::addScan( SmartScan *scan, QString *name )
{
	if (!scan) return;
	Body *body = createBodyFromScan(scan);
	mWorld->addBody(body);
	mScans->push_back(scan);
	mBodies->push_back(body);

	QString s("Scan ");
	QString n; n.setNum( (int)mScans->size() );

	if (!name) s = s+n;
	else s = *name;

	target1Box->insertItem(s);
	target2Box->insertItem(s);

	target1Box->setCurrentItem( mScans->size() - 1 );
	if ( mScans->size() == 1) {
		target2Box->setCurrentItem(0);
	} else if ( mScans->size() == 2) {
		target2Box->setCurrentItem(1);
	}
}

void ScanDlg::saveScan( SmartScan *scan )
{
	//fire up a window to see where we save
	QString fn( QFileDialog::getSaveFileName( QString(getenv("GRASPIT"))+QString("/models/scans"),
						  "ROS Scan files (*.txt)", this ) );
	if ( !fn.isEmpty() ) {
		if (fn.section('.',1).isEmpty()) fn.append(".txt");
		std::fstream fs;
		fs.open(fn.latin1(),std::fstream::out);
		if (fs.fail()){
			fprintf(stderr,"Failed to open file %s\n",fn.latin1());
		} else {
			scan->writeToFile(fs);
			fprintf(stderr,"Scan saved to file: %s\n",fn.latin1());
		}
		fs.close();
	}
}

void ScanDlg::saveScanVrml( int s )
{
	//fire up a window to see where we save
	QString fn( QFileDialog::getSaveFileName( QString(getenv("GRASPIT"))+QString("/models/scans"),
						  "ROS Scan files (*.wrl)", this ) );
	if ( fn.isEmpty() ) return;

	if (fn.section('.',1).isEmpty()) fn.append(".wrl");
	std::fstream fs;
	fs.open(fn.latin1(),std::fstream::out);
	if (fs.fail()){
		fprintf(stderr,"Failed to open file %s\n",fn.latin1());
		return;
	} 

	(*mScans)[s]->writeToFileAsVrml(fs);
	fprintf(stderr,"Scan saved as VRML to file: %s\n",fn.latin1());

	fs.close();
}

void ScanDlg::removeScan(int s)
{
	if ( s >= (int)mScans->size() || s < 0) {
		fprintf(stderr,"Remove out of bounds!\n");
		return;
	}

	mWorld->destroyElement( (*mBodies)[s] );
	delete (*mScans)[s];

	std::vector<SmartScan*>::iterator itS = mScans->begin();
	std::vector<Body*>::iterator itB = mBodies->begin();
	for (int i=0; i<s; i++) {
		itS++; itB++;
	}
	mScans->erase(itS);
	mBodies->erase(itB);

	target1Box->removeItem(s);
	target2Box->removeItem(s);
}

void ScanDlg::delaunay( int s )
{
	bool ok;
	float tol = p1Edit->text().toFloat(&ok);
	if (!ok) {return; fprintf(stderr,"Failed tol conversion\n");}
	float alpha = p2Edit->text().toFloat(&ok);
	if (!ok) {return; fprintf(stderr,"Failed alpha conversion\n");}

	std::vector<scan_utils::Triangle> *triangles = (*mScans)[s]->delaunayTriangulation3D(tol, alpha);
	unsigned int nTri = (unsigned int) triangles->size();
	fprintf(stderr,"Delaunay: %d triangles\n",nTri);

	SbVec3f *points = new SbVec3f[ 3*nTri ];
	int32_t *cIndex = new int32_t[ 4*nTri ];

	if (!points || !cIndex) {
		fprintf(stderr,"Failed to allocate memory for points and indeces!\n");
		triangles->clear(); delete triangles;
		if (points) delete [] points; if (cIndex) delete [] cIndex;
		return;

	}
	unsigned int i;
	for (i=0; i<nTri; i++) {
		points[3*i+0] = SbVec3f( (*triangles)[i].p1.x,  (*triangles)[i].p1.y,  (*triangles)[i].p1.z);
		points[3*i+1] = SbVec3f( (*triangles)[i].p2.x,  (*triangles)[i].p2.y,  (*triangles)[i].p2.z);
		points[3*i+2] = SbVec3f( (*triangles)[i].p3.x,  (*triangles)[i].p3.y,  (*triangles)[i].p3.z);
		cIndex[4*i+0]=3*i+0;
		cIndex[4*i+1]=3*i+1;
		cIndex[4*i+2]=3*i+2;
		cIndex[4*i+3]=-1;
	}

	SoCoordinate3 *coords = new SoCoordinate3;
	coords->point.setValues( 0, 3*nTri, points );

	SoIndexedFaceSet *ifs = new SoIndexedFaceSet;
	ifs->coordIndex.setValues(0, 4*nTri, cIndex);

	SoMaterial *mat = new SoMaterial;
	mat->diffuseColor = SbColor(0.8f,0.0f,0.0f);
	mat->ambientColor = SbColor(0.2f,0.0f,0.0f);
	mat->emissiveColor = SbColor(0.4f,0.0f,0.0f);
	mat->transparency = 0.0f;

	(*mBodies)[s]->getIVGeomRoot()->addChild(mat);
	(*mBodies)[s]->getIVGeomRoot()->addChild(coords);
	(*mBodies)[s]->getIVGeomRoot()->addChild(ifs);

	triangles->clear();
	delete triangles;
	//fprintf(stderr,"All done\n");
}

void ScanDlg::ICP( int s1, int s2)
{
	fprintf(stderr,"Starting ICP of %d to %d.\n",s1,s2);

	float *trf = (*mScans)[s1]->ICPTo( (*mScans)[s2] );
	fprintf(stderr,"Transform computed. Applying...\n");
	(*mScans)[s1]->applyTransform(trf);

	(*mBodies)[s1]->getIVGeomRoot()->removeAllChildren();
	addGeometryFromScan( (*mBodies)[s1]->getIVGeomRoot(), (*mScans)[s1] );
	delete [] trf;
}

void ScanDlg::crop( int s1 )
{
	bool ok;
	float x = p1Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed x conversion\n");return;}
	float y = p2Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed y conversion\n");return;}
	float z = p3Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed z conversion\n");return;}
	float d = p4Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed d conversion\n");return;}
	fprintf(stderr,"Cropping scan\n");
	(*mScans)[s1]->crop(x,y,z,d,d,d);
	(*mBodies)[s1]->getIVGeomRoot()->removeAllChildren();
	addGeometryFromScan( (*mBodies)[s1]->getIVGeomRoot(), (*mScans)[s1] );
}

void ScanDlg::removeOutliers( int s )
{
	bool ok;
	float radius = p1Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed radius conversion\n");return;}
	int nbrs = p2Edit->text().toInt(&ok);
	if (!ok) {fprintf(stderr,"Failed nbrs conversion\n");return;}

	fprintf(stderr,"Removing outliers from scan; args: %f %d\n",radius,nbrs);
	(*mScans)[s]->removeOutliers(radius,nbrs);

	(*mBodies)[s]->getIVGeomRoot()->removeAllChildren();
	addGeometryFromScan( (*mBodies)[s]->getIVGeomRoot(), (*mScans)[s] );
}

void ScanDlg::removeGrazingPoints( int s )
{
	bool ok;
	float thresh = p1Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed threshold conversion\n");return;}
	int outl = p2Edit->text().toInt(&ok);
	if (!ok) {fprintf(stderr,"Failed outliers flag conversion\n");return;}
	bool outlFlag = true; if (!outl) outlFlag = false;
	float radius = p3Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed radius conversion\n");return;}
	int nbrs = p4Edit->text().toInt(&ok);
	if (!ok) {fprintf(stderr,"Failed nbrs conversion\n");return;}

	fprintf(stderr,"Removing grazers from scan; args: %f %d %f %d\n",thresh,outlFlag,radius,nbrs);
	(*mScans)[s]->removeGrazingPoints(thresh,outlFlag,radius,nbrs);

	(*mBodies)[s]->getIVGeomRoot()->removeAllChildren();
	addGeometryFromScan( (*mBodies)[s]->getIVGeomRoot(), (*mScans)[s] );
}

void ScanDlg::removeHistoPlane(int s)
{
	bool ok;
	float radius = p1Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed radius conversion\n");return;}
	int nbrs = p2Edit->text().toInt(&ok);
	if (!ok) {fprintf(stderr,"Failed nbrs conversion\n");return;}

	fprintf(stderr,"Finding plane from scan; args: %f %d\n",radius,nbrs);
	std_msgs::Point3DFloat32 p,n;
	(*mScans)[s]->normalHistogramPlane(p,n,radius, nbrs);

	float thresh = p3Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed threshold conversion\n");return;}

	(*mScans)[s]->removePlane(p,n,thresh);
	(*mBodies)[s]->getIVGeomRoot()->removeAllChildren();
	addGeometryFromScan( (*mBodies)[s]->getIVGeomRoot(), (*mScans)[s] );

}

void ScanDlg::removeRansacPlane(int s)
{
	bool ok;
	float ransThresh = p1Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed ransac thresh conversion\n");return;}
	int its = p2Edit->text().toInt(&ok);
	if (!ok) {fprintf(stderr,"Failed iterations conversion\n");return;}

	fprintf(stderr,"Finding plane from scan; args: %f %d\n",ransThresh, its);
	std_msgs::Point3DFloat32 p,n;
	(*mScans)[s]->ransacPlane(p,n,its, ransThresh);

	float remThresh = p3Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed removal threshold conversion\n");return;}

	(*mScans)[s]->removePlane(p,n,remThresh);
	(*mBodies)[s]->getIVGeomRoot()->removeAllChildren();
	addGeometryFromScan( (*mBodies)[s]->getIVGeomRoot(), (*mScans)[s] );
}
 
void ScanDlg::loadTransform(int s)
{
	QString fn = QFileDialog::getOpenFileName( QString(getenv("GRASPIT"))+QString("/models/scans/transforms"),
						   "Scan transform files (*.txt)", this ) ;
	if (fn.isEmpty()) return;
	std::fstream fs;
	fs.open(fn.latin1(),std::fstream::in);
	if (fs.fail()){
		fprintf(stderr,"Failed to open file %s\n",fn.latin1());
		return;
	}

	float t[16];
	fs >> t[0] >> t[1] >> t[2] >> t[3];
	fs >> t[4] >> t[5] >> t[6] >> t[7];
	fs >> t[8] >> t[9] >> t[10] >> t[11];
	fs >> t[12] >> t[13] >> t[14] >> t[15];

	if (fs.fail()) {
		fprintf(stderr,"Failed to read transform from file\n");
	} else {
		(*mScans)[s]->applyTransform(t);
	}

	fs.close();
	(*mBodies)[s]->getIVGeomRoot()->removeAllChildren();
	addGeometryFromScan( (*mBodies)[s]->getIVGeomRoot(), (*mScans)[s] );
	fprintf(stderr,"Loaded transform from file\n");
}

void ScanDlg::joinScans(int s1, int s2)
{
	if (s1 == s2) {
		fprintf(stderr,"Select two distinct scans\n");
		return;
	}
	fprintf(stderr,"Joining scans %d and %d points\n",(*mScans)[s1]->size(), (*mScans)[s2]->size());
	(*mScans)[s1]->addScan( (*mScans)[s2] );
	fprintf(stderr,"Result: %d points\n",(*mScans)[s1]->size() );
	(*mBodies)[s1]->getIVGeomRoot()->removeAllChildren();
	addGeometryFromScan( (*mBodies)[s1]->getIVGeomRoot(), (*mScans)[s1] );
	//look out! after this, s1 and s2 don't mean the same thing anymore!
	removeScan(s2);
}

void ScanDlg::subtractScans(int s1, int s2)
{
	if (s1 == s2) {
		fprintf(stderr,"Select two distinct scans\n");
		return;
	}

	bool ok;
	float thresh = p1Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed thresh conversion\n");return;}

	(*mScans)[s1]->subtractScan( (*mScans)[s2], thresh );
	(*mBodies)[s1]->getIVGeomRoot()->removeAllChildren();
	addGeometryFromScan( (*mBodies)[s1]->getIVGeomRoot(), (*mScans)[s1] );
}

void ScanDlg::findNbrs(int s1)
{
	bool ok;
	float x = p1Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed x conversion\n");return;}
	float y = p2Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed y conversion\n");return;}
	float z = p3Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed z conversion\n");return;}
	float radius = p4Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed radius conversion\n");return;}

	std::vector<std_msgs::Point3DFloat32> *pts = (*mScans)[s1]->getPointsWithinRadius(x,y,z,radius);
	fprintf(stderr,"Found %d nbrs\n",pts->size());

	for (int i=0; i<(int)pts->size(); i++) {
		SoSeparator *r = new SoSeparator;
		SoTransform *t = new SoTransform;
		t->translation.setValue( (*pts)[i].x, (*pts)[i].y, (*pts)[i].z);
		r->addChild(t);
		SoSphere *s = new SoSphere;
		s->radius.setValue(0.01);
		r->addChild(s);
		(*mBodies)[s1]->getIVGeomRoot()->addChild(r);
	}

	SoSeparator *r = new SoSeparator;
	SoTransform *t = new SoTransform;
	t->translation.setValue( x, y, z );
	r->addChild(t);
	SoSphere *s = new SoSphere;
	s->radius.setValue(0.01);
	r->addChild(s);
	(*mBodies)[s1]->getIVGeomRoot()->addChild(r);

	pts->clear();
	delete pts;
}

void ScanDlg::createMesh(int s1)
{
	bool ok;
	float resolution = p1Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed resolution conversion\n");return;}
	fprintf(stderr,"Resolution: %f\n",resolution);

	std::vector<scan_utils::Triangle> *triangles = (*mScans)[s1]->createMesh(resolution);
	unsigned int nTri = (unsigned int) triangles->size();
	fprintf(stderr,"Mesh: %d triangles\n",nTri);

	SbVec3f *points = new SbVec3f[ 3*nTri ];
	int32_t *cIndex = new int32_t[ 4*nTri ];

	if (!points || !cIndex) {
		fprintf(stderr,"Failed to allocate memory for points and indeces!\n");
		triangles->clear(); delete triangles;
		if (points) delete [] points; if (cIndex) delete [] cIndex;
		return;

	}
	unsigned int i;
	for (i=0; i<nTri; i++) {
		points[3*i+0] = SbVec3f( (*triangles)[i].p1.x,  (*triangles)[i].p1.y,  (*triangles)[i].p1.z);
		//fprintf(stderr,"%f %f %f\n", (*triangles)[i].p1.x,  (*triangles)[i].p1.y,  (*triangles)[i].p1.z);
		points[3*i+1] = SbVec3f( (*triangles)[i].p2.x,  (*triangles)[i].p2.y,  (*triangles)[i].p2.z);
		points[3*i+2] = SbVec3f( (*triangles)[i].p3.x,  (*triangles)[i].p3.y,  (*triangles)[i].p3.z);
		cIndex[4*i+0]=3*i+0;
		cIndex[4*i+1]=3*i+1;
		cIndex[4*i+2]=3*i+2;
		cIndex[4*i+3]=-1;
	}

	SoCoordinate3 *coords = new SoCoordinate3;
	coords->point.setValues( 0, 3*nTri, points );

	SoIndexedFaceSet *ifs = new SoIndexedFaceSet;
	ifs->coordIndex.setValues(0, 4*nTri, cIndex);

	SoMaterial *mat = new SoMaterial;
	mat->diffuseColor = SbColor(0.8f,0.0f,0.0f);
	mat->ambientColor = SbColor(0.2f,0.0f,0.0f);
	mat->emissiveColor = SbColor(0.4f,0.0f,0.0f);
	mat->transparency = 0.0f;

	(*mBodies)[s1]->getIVGeomRoot()->addChild(mat);
	(*mBodies)[s1]->getIVGeomRoot()->addChild(coords);
	(*mBodies)[s1]->getIVGeomRoot()->addChild(ifs);

	triangles->clear();
	delete triangles;
}

SmartScan* ScanDlg::simulateScan()
{
	scanSimDlg *dlg = new scanSimDlg(this, QString::null, TRUE);
	dlg->exec();
	SmartScan *scan = dlg->getScan();
	delete dlg;
	return scan;
}

void ScanDlg::connectedComponents(int s1)
{
	bool ok;
	float thresh = p1Edit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed threshold conversion\n");return;}
	int sizeThresh = p2Edit->text().toInt(&ok);
	if (!ok) {fprintf(stderr,"Failed size threshold conversion\n");return;}

	std::vector<SmartScan*> *scans = (*mScans)[s1]->connectedComponents(thresh);

	QString c("Component ");
	QString n,cn;
	int i=1;
	while (!scans->empty()) {
		n.setNum(i);
		cn = c+n;
		if ( scans->back()->size() >= sizeThresh) {
			addScan( scans->back(), &cn );
			i++;
		}
		scans->pop_back();
	}
	fprintf(stderr,"%d components have enough points\n",i-1);
	removeScan(s1);
}

void ScanDlg::principalAxes(int s1)
{
	libTF::TFPoint c;
	libTF::TFVector a1,a2,a3;

	c = (*mScans)[s1]->centroid();
	(*mScans)[s1]->principalAxes(a1,a2,a3);

	vec3 v1(a1.x, a1.y, a1.z);
	vec3 v2(a2.x, a2.y, a2.z);
	vec3 v3(a3.x, a3.y, a3.z);
	vec3 cen(c.x, c.y, c.z);

	SoSeparator *axesSep = new SoSeparator;
	transf tr( mat3(v1,v2,v3), vec3(c.x, c.y, c.z) );
	SoTransform *axesTran = new SoTransform;
	tr.toSoTransform(axesTran);
	axesSep->addChild(axesTran);

	SoScale *axesScale = new SoScale;
	axesScale->scaleFactor = SbVec3f(0.001,0.001,0.001);
	axesSep->addChild(axesScale);

	axesSep->addChild(graspItGUI->getIVmgr()->getPointers()->getChild(2));

	(*mBodies)[s1]->getIVGeomRoot()->addChild(axesSep);
}

void ScanDlg::objectDetector(int s1)
{
	grasp_module::ObjectDetector *detector = new grasp_module::ObjectDetector;
	std::vector<grasp_module::Object*> *objects = detector->getObjects( (*mScans)[s1] );

	fprintf(stderr,"Got %d components. Displaying...\n",objects->size());

	(*mBodies)[s1]->getIVGeomRoot()->removeAllChildren();
	addGeometryFromScan( (*mBodies)[s1]->getIVGeomRoot(), (*mScans)[s1] );

	libTF::TFPoint c;
	libTF::TFVector a1,a2,a3;
	vec3 v1,v2,v3,cen;

	while (!objects->empty()){
		grasp_module::Object *obj = objects->back();
		objects->pop_back();

		c = obj->getCentroid();
		obj->getAxes(a1,a2,a3);

		v1 = vec3(a1.x, a1.y, a1.z);
		v2 = vec3(a2.x, a2.y, a2.z);
		v3 = vec3(a3.x, a3.y, a3.z);
		cen = vec3(c.x, c.y, c.z);

		std::cerr << "Centroid: " << cen << std::endl << "Axes:" << std::endl << v1 
			  << std::endl << v2 << std::endl << v3;
		
		SoSeparator *axesSep = new SoSeparator;
		transf tr( mat3(v1,v2,v3), vec3(c.x, c.y, c.z) );
		SoTransform *axesTran = new SoTransform;
		tr.toSoTransform(axesTran);
		axesSep->addChild(axesTran);
		
		SoScale *axesScale = new SoScale;
		axesScale->scaleFactor = SbVec3f(0.001,0.001,0.001);
		axesSep->addChild(axesScale);
		
		axesSep->addChild(graspItGUI->getIVmgr()->getPointers()->getChild(2));

		(*mBodies)[s1]->getIVGeomRoot()->addChild(axesSep);

		delete obj;
	}

	delete objects;
	delete detector;
}

void ScanDlg::graspPoints(int s1)
{
	bool ok;
	int resolution = p1Edit->text().toInt(&ok);
	if (!ok) {fprintf(stderr,"Failed resolution conversion\n");return;}

	grasp_module::ObjectDetector detector;
	grasp_module::GraspPlanner planner;

	//detector.setParamGrazing(0,0,0);
	detector.setParamComponents(-1,100);

	std::vector<grasp_module::Object*> *objects = detector.getObjects( (*mScans)[s1]);

	(*mBodies)[s1]->getIVGeomRoot()->removeAllChildren();
	addGeometryFromScan( (*mBodies)[s1]->getIVGeomRoot(), (*mScans)[s1] );

	while ( !objects->empty() ) {
		grasp_module::Object *obj = objects->back();
		objects->pop_back();

		fprintf(stderr,"Object:\n");

		std::vector<grasp_module::GraspPoint*> *grasps = planner.getGraspPoints(obj, resolution);
		while (!grasps->empty()) {
			grasp_module::GraspPoint *grasp = grasps->back();
			grasps->pop_back();

			float *tran;
			grasp->getTran(&tran);
			transf t = transfFromFloat(tran);

			std::cerr << t.translation() << std::endl;
			std::cerr << t.rotation() << std::endl;
			std::cerr << "Quality: " << grasp->getQuality() << std::endl;

			SoSeparator *axesSep = new SoSeparator;
			SoTransform *axesTran = new SoTransform;
			t.toSoTransform(axesTran);
			axesSep->addChild(axesTran);

			/*
			SoMaterial *sphereMat = new SoMaterial();
			sphereMat->diffuseColor = SbColor(grasp->getQuality(),1-grasp->getQuality(),0.0f);
			sphereMat->ambientColor = SbColor(grasp->getQuality(),1-grasp->getQuality(),0.0f);
			sphereMat->transparency = 0.0f;
			axesSep->addChild(sphereMat);
			SoSphere *sphere = new SoSphere;
			sphere->radius = 0.01;
			axesSep->addChild(sphere);
			*/

			float sf = 0.001 + grasp->getQuality() * 1.0e-3;

			SoScale *axesScale = new SoScale;
			axesScale->scaleFactor = SbVec3f(sf,0.001,0.001);
			axesSep->addChild(axesScale);
			
			axesSep->addChild(graspItGUI->getIVmgr()->getPointers()->getChild(2));
			(*mBodies)[s1]->getIVGeomRoot()->addChild(axesSep);

			delete [] tran;
			delete grasp;
		}
		fprintf(stderr,"\n");

		delete grasps;
		delete obj;
	}

	delete objects;
}

void ScanDlg::addTriangles(Body *body, std::list<scan_utils::Triangle> *triangles, float r, float g, float b)
{
	unsigned int nTri = (unsigned int) triangles->size();
	fprintf(stderr,"Mesh: %d triangles\n",nTri);

	SbVec3f *points = new SbVec3f[ 3*nTri ];
	int32_t *cIndex = new int32_t[ 4*nTri ];

	if (!points || !cIndex) {
		fprintf(stderr,"Failed to allocate memory for points and indeces!\n");
		triangles->clear(); delete triangles;
		if (points) delete [] points; if (cIndex) delete [] cIndex;
		return;

	}

	unsigned int i = 0;
	std::list<scan_utils::Triangle>::iterator it = triangles->begin();
	while(it!=triangles->end()) {

		points[3*i+0] = SbVec3f( (*it).p1.x,  (*it).p1.y,  (*it).p1.z);
		points[3*i+1] = SbVec3f( (*it).p2.x,  (*it).p2.y,  (*it).p2.z);
		points[3*i+2] = SbVec3f( (*it).p3.x,  (*it).p3.y,  (*it).p3.z);

		cIndex[4*i+0]=3*i+0;
		cIndex[4*i+1]=3*i+1;
		cIndex[4*i+2]=3*i+2;
		cIndex[4*i+3]=-1;

		it++;
		i++;
	}

	SoCoordinate3 *coords = new SoCoordinate3;
	coords->point.setValues( 0, 3*nTri, points );

	SoIndexedFaceSet *ifs = new SoIndexedFaceSet;
	ifs->coordIndex.setValues(0, 4*nTri, cIndex);

	SoMaterial *mat = new SoMaterial;
	mat->diffuseColor = SbColor(r, g, b);
	mat->ambientColor = SbColor(r/4.0, g/4.0, b/4.0);
	mat->emissiveColor = SbColor(r/2.0, g/2.0, b/2.0);
	mat->transparency = 0.0f;

	body->getIVGeomRoot()->addChild(mat);
	body->getIVGeomRoot()->addChild(coords);
	body->getIVGeomRoot()->addChild(ifs);
}

void ScanDlg::traceInOctree(int s1)
{
	if (!mOctree) octreeSetButton_clicked();
	float sx, sy, sz, foo;
	(*mScans)[s1]->getScanner( sx, sy, sz,
				   foo, foo, foo, foo, foo, foo);
	for (int i=0; i<(*mScans)[s1]->size(); i++) {
		std_msgs::Point3DFloat32 p = (*mScans)[s1]->getPoint(i);
		vec3 d(p.x - sx, p.y - sy, p.z - sz);
		float dist = d.len();
		d = normalise(d);
		mOctree->traceRay( sx, sy, sz, 
				   d.x(), d.y(), d.z(), 
				   dist, (char)2, (char)1 );
	}
}

void ScanDlg::insertInOctree(int s1)
{
	if (!mOctree) octreeSetButton_clicked();
	(*mScans)[s1]->insertInOctree(mOctree, (char)1);
}

void ScanDlg::collisionTest(int s1)
{
	if (!mOctree) {
		fprintf(stderr,"Octree does not exist\n");
		return;
	}

	clock_t startTime = clock();

	int intersections = 0, i;
	for (i=0; i<(*mScans)[s1]->size(); i++) {
		std_msgs::Point3DFloat32 p = (*mScans)[s1]->getPoint(i);

		if (mOctree->get(p.x,p.y,p.z)==1) {
			intersections++;
			/*
			SoSeparator *r = new SoSeparator;
			SoTransform *t = new SoTransform;
			t->translation.setValue( p.x,p.y,p.z );
			r->addChild(t);
			SoSphere *s = new SoSphere;
			s->radius.setValue(0.01);
			r->addChild(s);
			mOctreeBody->getIVGeomRoot()->addChild(r);
			*/
		}		
	}
	
	clock_t stopTime = clock();
	double runningTime = (float)(stopTime - startTime) / CLOCKS_PER_SEC;

	std::cerr << "Ran " << i << " tests in " << runningTime << " seconds" << std::endl;
	std::cerr << "Average: " << runningTime / i << " seconds per test" << std::endl;
	std::cerr << "Intersections: " << intersections << std::endl;
}

void ScanDlg::octreeSetButton_clicked()
{
	bool ok;
	float cx = ocxEdit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed conversion!\n");return;}
	float cy = ocyEdit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed conversion!\n");return;}
	float cz = oczEdit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed conversion!\n");return;}
	float dx = odxEdit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed conversion!\n");return;}
	float dy = odyEdit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed conversion!\n");return;}
	float dz = odzEdit->text().toFloat(&ok);
	if (!ok) {fprintf(stderr,"Failed conversion!\n");return;}
	int depth = odepthEdit->text().toInt(&ok);
	if (!ok) {fprintf(stderr,"Failed conversion!\n");return;}

	if (mOctree) delete mOctree;
	mOctree = new scan_utils::Octree<char>(cx,cy,cz,dx,dy,dz,depth,0);
	mOctree->setAutoExpand(true);
	fprintf(stderr,"Created octree centered at %f %f %f of size %f %f %f, max depth %d and empty value %d\n",
		cx,cy,cz,dx,dy,dz,depth,0);

	if (!mOctreeBody) {
		mOctreeBody = new Body(mWorld,"Octree");
		mOctreeBody->initializeIV();
		mOctreeBody->addIVMat();
		mOctreeBody->addToIvc();
		mWorld->addBody(mOctreeBody);
	} else {
		mOctreeBody->getIVGeomRoot()->removeAllChildren();
	}

}

bool isOne(char c){return c==(char)1;}
bool isZero(char c){return c==(char)0;}

void ScanDlg::octreeTriangulateButton_clicked()
{
	if (!mOctree) octreeSetButton_clicked();
	mOctreeBody->getIVGeomRoot()->removeAllChildren();

	std::list<scan_utils::Triangle> triangles;

	mOctree->getTriangles( triangles, &isOne );
	addTriangles(mOctreeBody, &triangles, 0.8, 0.0, 0.0);
	triangles.clear();
	
	mOctree->getTriangles( triangles, &isZero );
	addTriangles(mOctreeBody, &triangles, 0.0, 0.0, 0.8);
	triangles.clear();
	
	fprintf(stderr,"Octree has %d branches and %d leaves; depth %d\n",
		mOctree->getNumBranches(), mOctree->getNumLeaves(), mOctree->getMaxDepth());
	fprintf(stderr,"Octree occupies %lld bytes\n",mOctree->getMemorySize());
}


void ScanDlg::octreeClearButton_clicked()
{
	if (!mOctree) octreeSetButton_clicked();
	mOctree->clear();
	mOctreeBody->getIVGeomRoot()->removeAllChildren();
}


void ScanDlg::octreeCollisionsButton_clicked()
{
}


void ScanDlg::octreeLoadButton_clicked()
{
	if (!mOctree) octreeSetButton_clicked();
	QString fn = QFileDialog::getOpenFileName( QString(getenv("GRASPIT"))+QString("/models/scans/octrees"),
						   "ROS Octree files (*.txt)", this ) ;
	if (fn.isEmpty()) return;
	std::fstream fs;
	fs.open(fn.latin1(),std::fstream::in);
	if (fs.fail()){
		fprintf(stderr,"Failed to open file %s\n",fn.latin1());
		return;
	}

	mOctree->readFromFile(fs);
	fs.close();
}


void ScanDlg::octreeSaveButton_clicked()
{
	if (!mOctree) return;

	//fire up a window to see where we save
	QString fn( QFileDialog::getSaveFileName( QString(getenv("GRASPIT"))+QString("/models/scans/octrees"),
						  "ROS Octree files (*.txt)", this ) );
	if ( fn.isEmpty() ) return;

	if (fn.section('.',1).isEmpty()) fn.append(".txt");
	std::fstream fs;
	fs.open(fn.latin1(),std::fstream::out);
	if (fs.fail()){
		fprintf(stderr,"Failed to open file %s\n",fn.latin1());
		return;
	} 

	mOctree->writeToFile(fs);
	fs.close();
}


void ScanDlg::testButton_clicked()
{

}
