#ifndef _scandlg_h_
#define _scandlg_h_

//ros and Qt both use the DEBUG keyword which Qt defines as a macro!
#undef DEBUG

ui_scanDlg.h"

#include <QDialog>
#include <vector>
#include <list>

#include "dataTypes.h"
#include "matvec3D.h"

class ScanListenNode;
class SmartScan;
class World;
class Body;
class SoGroup;

class ScanDlg : public QDialog, public Ui::ScanDlgUI
{
	Q_OBJECT
private:
	scan_utils::Octree<char> *mOctree = NULL;
	Body *mOctreeBody;
    ScanListenNode *mNode;
    World *mWorld;
    std::vector<SmartScan*> *mScans;
    std::vector<Body*> *mBodies;

	void init();
	void destroy();
	transf transfFromFloat(float *trd);
	Body* createBodyFromScan( SmartScan* cloud );
	void addGeometryFromScan( SoGroup *root, SmartScan *cloud );
	void setTransformFromScan(int s);
	void setParams(QString q1, QString q2, QString q3, QString q4, float v1, float v2, float v3, float v4);
	void addScan( SmartScan *scan, QString *name );
	void saveScan( SmartScan *scan );
	void saveScanVrml( int s );
	void removeScan(int s);
	void delaunay( int s );
	void ICP( int s1, int s2);
	void crop( int s1 );
	void removeOutliers( int s );
	void removeGrazingPoints( int s );
	void removeHistoPlane(int s);
	void removeRansacPlane(int s);
	void loadTransform(int s);
	void joinScans(int s1, int s2);
	void subtractScans(int s1, int s2);
	void findNbrs(int s1);
	void createMesh(int s1);
	SmartScan* simulateScan();
	void connectedComponents(int s1);
	void principalAxes(int s1);
	void objectDetector(int s1);
	void graspPoints(int s1);
	void addTriangles(Body *body, std::list<scan_utils::Triangle> *triangles, float r, float g, float b);
	void traceInOctree(int s1);
	void insertInOctree(int s1);
	void collisionTest(int s1);

public:
	ScanDlg(QWidget *parent = 0) : QDialog(parent) {
		setupUi(this);
		init();
	}
	~ScanDlg(){destroy();}

public slots:
	void exitButton_clicked();
	void getScanButton_clicked();
	void loadScanButton_clicked();
	void goButton_clicked();
	void actionBox_activated( int );
	void octreeSetButton_clicked();
	void octreeTriangulateButton_clicked();
	void octreeClearButton_clicked();
	void octreeCollisionsButton_clicked();
	void octreeLoadButton_clicked();
	void octreeSaveButton_clicked();
	void testButton_clicked();
};
#endif