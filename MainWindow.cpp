#include "MainWindow.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QGroupBox>
#include <QPushButton>
#include "SimulatorView.h"

MainWindow::MainWindow(SimulatorView *view, QWidget *parent) : QMainWindow(parent),
	mSimView(view)
{
	mParamsWidget = new QWidget(this);
	buildParamsWidget();

	mParamsDock = new QDockWidget(tr("Params"), this);
	mParamsDock->setAllowedAreas( Qt::AllDockWidgetAreas );
	mParamsDock->setWidget(mParamsWidget);
	addDockWidget( Qt::RightDockWidgetArea, mParamsDock );

	mDashboardWidget = new QWidget(this);
	buildDashboardWidget();

	mDasboardDock = new QDockWidget(tr("Dashboard"), this);
	mDasboardDock->setAllowedAreas( Qt::AllDockWidgetAreas );
	mDasboardDock->setWidget(mDashboardWidget);
	addDockWidget( Qt::RightDockWidgetArea, mDasboardDock );

	connect( mSimView->simulator(), SIGNAL(simUpdated()), this, SLOT(onSimUpdated()) );

	setCentralWidget(mSimView);
}

void MainWindow::updateSpeedDisplay(double speedKmh)
{
	QLabel *speedMeter = mDashboardWidget->findChild<QLabel*>("speedMeter");
	if( speedMeter )
	{ speedMeter->setText( QString("Speed: %1 km/h").arg(speedKmh) ); }
}

void MainWindow::pixelPerMeterChanged(double newVal)
{
	QSlider *pxPerMeterSlider = mParamsWidget->findChild<QSlider*>("pxPerMeterSlider");
	if( pxPerMeterSlider )
		{ pxPerMeterSlider->setValue( (int)(newVal*10) ); }
}

void MainWindow::onSimUpdated()
{
	updateSpeedDisplay(mSimView->simulator()->car()->speedKmh());

	QLabel *targetSpeedLabel = mDashboardWidget->findChild<QLabel*>("targetSpeed");
	if( targetSpeedLabel )
		{ targetSpeedLabel->setText( QString("Target speed: %1 km/h").arg(mSimView->simulator()->carDriver()->targetSpeedKmh()) ); }

	QLabel *maxAccelRatioLabel = mDashboardWidget->findChild<QLabel*>("maxAccelRatioLabel");
	if( maxAccelRatioLabel )
		{ maxAccelRatioLabel->setText(QString("maxAccelRatio: %1/%2").arg(mSimView->simulator()->carDriver()->currentNetAccel().length(),3,'f',2).arg(mSimView->simulator()->car()->frictionCoeffStatic()*9.81,3,'f',2) ); }

	emit updateMaxAccelRatioDisplay( mSimView->simulator()->currentMaxAccelerationRatio() * 100 );
}

void MainWindow::pxPerMeterChangeReqInt(int sliderVal)
{
	mSimView->setPixelPerMeter( sliderVal/10.0 );
}

void MainWindow::cruiseSpeedValueChanged(double val)
{
	mSimView->simulator()->carDriver()->setCruiseSpeedKmh(val);
}

void MainWindow::onSimRunButtonToggled(bool clicked)
{
	if( clicked )
		{ mSimView->simulator()->start(); }
	else
		{ mSimView->simulator()->stop(); }
}

void MainWindow::buildParamsWidget()
{
	QVBoxLayout *paramsLayout = new QVBoxLayout(mParamsWidget);
	mParamsWidget->setMinimumWidth(200);

	// --- SIMULATION GROUP -------------------
	QGroupBox *simGroup = new QGroupBox;
	simGroup->setLayout(new QVBoxLayout);
	simGroup->setTitle("Simulation");

	// RUN button
	QPushButton *simRunButton = new QPushButton;
	simRunButton->setText("RUN");
	simRunButton->setCheckable(true);
	connect( simRunButton, SIGNAL(clicked(bool)), this, SLOT(onSimRunButtonToggled(bool)) );
	connect( mSimView->simulator(), SIGNAL(simRunStateChanged(bool)), simRunButton, SLOT(setChecked(bool)) );
	simGroup->layout()->addWidget(simRunButton);

	// Pixel per meter slider + label
	QLabel *pxPerMeterLabel = new QLabel(mParamsWidget);
	pxPerMeterLabel->setText(tr("px/m"));
	simGroup->layout()->addWidget(pxPerMeterLabel);
	QSlider *pxPerMeterSlider = new QSlider(Qt::Horizontal, mParamsWidget);
	pxPerMeterSlider->setObjectName("pxPerMeterSlider");
	pxPerMeterSlider->setMinimum(1);
	pxPerMeterSlider->setMaximum(500);
	pxPerMeterSlider->setValue((int)(mSimView->pixelPerMeter()*10));
	connect( pxPerMeterSlider, &QSlider::valueChanged, this, &MainWindow::pxPerMeterChangeReqInt );
	simGroup->layout()->addWidget(pxPerMeterSlider);

	paramsLayout->addWidget(simGroup);
	// /// SIMULATION GROUP -------------------

	QHBoxLayout *cruiseSpeedLayout = new QHBoxLayout;
	QLabel *cruiseSpeedLabel = new QLabel(mParamsWidget);
	cruiseSpeedLabel->setText("Cruise speed:");
	cruiseSpeedLayout->addWidget(cruiseSpeedLabel);
	QDoubleSpinBox *cruiseSpeedSpinbox = new QDoubleSpinBox(mParamsWidget);
	cruiseSpeedSpinbox->setSuffix("km/h");
	cruiseSpeedSpinbox->setDecimals(1);
	cruiseSpeedSpinbox->setSingleStep(5.0);
	cruiseSpeedSpinbox->setValue( mSimView->simulator()->carDriver()->cruiseSpeedKmh() );
	connect( cruiseSpeedSpinbox, SIGNAL(valueChanged(double)), this, SLOT(cruiseSpeedValueChanged(double)) );
	cruiseSpeedLayout->addWidget(cruiseSpeedSpinbox);
	paramsLayout->addLayout(cruiseSpeedLayout);

	paramsLayout->addStretch();
}

void MainWindow::buildDashboardWidget()
{
	QVBoxLayout *dashboardLayout = new QVBoxLayout(mDashboardWidget);
	mDashboardWidget->setMinimumWidth(200);

	// Speed
	QLabel *speedMeter = new QLabel(mDashboardWidget);
	speedMeter->setObjectName("speedMeter");
	dashboardLayout->addWidget(speedMeter);

	// Target speed
	QLabel *targetSpeedLabel = new QLabel(mDashboardWidget);
	targetSpeedLabel->setObjectName("targetSpeed");
	targetSpeedLabel->setText( QString("Target speed: 0 km/h") );
	dashboardLayout->addWidget(targetSpeedLabel);

	// max acceleration ratio
	QLabel *maxAccelRatioLabel = new QLabel(mDashboardWidget);
	maxAccelRatioLabel->setObjectName("maxAccelRatioLabel");
	dashboardLayout->addWidget(maxAccelRatioLabel);
	QSlider *maxAccelRatioSlider = new QSlider(Qt::Horizontal, mDashboardWidget);
	maxAccelRatioSlider->setDisabled(true);
	maxAccelRatioSlider->setObjectName("maxAccelRatioSlider");
	maxAccelRatioSlider->setMinimum(-100);
	maxAccelRatioSlider->setMaximum(100);
	maxAccelRatioSlider->setValue(0);
	connect( this, SIGNAL(updateMaxAccelRatioDisplay(int)), maxAccelRatioSlider, SLOT(setValue(int)) );
	dashboardLayout->addWidget(maxAccelRatioSlider);

	dashboardLayout->addStretch();
}
