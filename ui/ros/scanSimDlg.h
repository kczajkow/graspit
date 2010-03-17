#ifndef _scansimdlg_h_
#define _scansimdlg_h_

#include "ui_scanSimDlg.h"

#include <QDialog>

class SmartScan;

class ScanSimDlg : public QDialog, public Ui::ScanSimDlgUI
{
	Q_OBJECT
private:
	SmartScan *mScan;

	void init();
	void destroy();

public:
	ScanSimDlg(QWidget *parent =  0) : QDialog(parent) {
		setupUi(this);
		init();
	}
	~ScanSimDlg(){destroy();}
	SmartScan* scanSimDlg::getScan() {
		return mScan;
	}

public slots:
	void exitButton_clicked();
	void goButton_clicked();
};

#endif