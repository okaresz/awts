#include "MainWindow.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QGroupBox>
#include <QPushButton>
#include <qevent.h>
#include <QCheckBox>
#include <QStatusBar>
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

	mStatusBar = new QStatusBar(this);
	setStatusBar(mStatusBar);

	connect( mSimView->simulator(), SIGNAL(simUpdated()), this, SLOT(onSimUpdated()) );
	connect( mSimView->simulator(), SIGNAL(simUpdated()), this, SLOT(update()) );
	connect( mSimView->simulator(), SIGNAL(carUnavoidableTractionLossDetected(double)), this, SLOT(onCarUnavoidableTractionLossDetected(double)) );
	connect( mSimView->simulator(), SIGNAL(carUnavoidableCrashDetected(double)), this, SLOT(onCarUnavoidableCrashDetected(double)) );
	connect( mSimView->simulator(), SIGNAL(carTractionLost(double)), this, SLOT(onCarTractionLost(double)) );
	connect( mSimView->simulator(), SIGNAL(carCrashed(double)), this, SLOT(onCarCrashed(double)) );

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

void MainWindow::onCarTractionLost(double atTravel)
{
	mStatusBar->showMessage(QString("Car traction lost at travel=%1!").arg(atTravel));
}

void MainWindow::onCarCrashed(double atTravel)
{
	mStatusBar->showMessage(QString("Car crashed at travel=%1!").arg(atTravel));
}

void MainWindow::onCarUnavoidableTractionLossDetected(double atTravel)
{
	mStatusBar->showMessage(QString("Car will lose traction at travel=%1!").arg(atTravel));
}

void MainWindow::onCarUnavoidableCrashDetected(double atTravel)
{
	mStatusBar->showMessage(QString("Car will crash at travel=%1!").arg(atTravel));
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

	QSlider *steeringSlider = mParamsWidget->findChild<QSlider*>("steeringSlider");
	if( steeringSlider )
	{
		if( !steeringSlider->isEnabled() || !steeringSlider->hasFocus() )
			{ steeringSlider->setValue(mSimView->simulator()->car()->wheelAngle()/M_PI*180.0); }
	}
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

void MainWindow::onSteeringSliderChanged(int value)
{
	if( ((QSlider*)sender())->isEnabled() && ((QSlider*)sender())->hasFocus() )
		{ mSimView->simulator()->carDriver()->setSteeringControlFeedForward( value/180.0*M_PI ); }
}

void MainWindow::onTargetCrossPosSPinBoxChanged(double value)
{
	mSimView->simulator()->carDriver()->setTargetCrossPos(value);
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
	pxPerMeterLabel->setText(tr("zoom (px/m)"));
	simGroup->layout()->addWidget(pxPerMeterLabel);
	QSlider *pxPerMeterSlider = new QSlider(Qt::Horizontal, mParamsWidget);
	pxPerMeterSlider->setObjectName("pxPerMeterSlider");
	pxPerMeterSlider->setMinimum(1);
	pxPerMeterSlider->setMaximum(500);
	pxPerMeterSlider->setValue((int)(mSimView->pixelPerMeter()*10));
	connect( pxPerMeterSlider, &QSlider::valueChanged, this, &MainWindow::pxPerMeterChangeReqInt );
	simGroup->layout()->addWidget(pxPerMeterSlider);

	paramsLayout->addWidget(simGroup);
	// /// END OF SIMULATION GROUP -------------------

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


	QHBoxLayout *steeringControlLayout = new QHBoxLayout;

	QLabel *steeringControlPLabel = new QLabel(mParamsWidget);
	steeringControlPLabel->setText("P:");
	steeringControlLayout->addWidget(steeringControlPLabel);
	QDoubleSpinBox *steeringControlPSpinBox = new QDoubleSpinBox(mParamsWidget);
	steeringControlPSpinBox->setDecimals(3);
	steeringControlPSpinBox->setSingleStep(0.5);
	steeringControlPSpinBox->setValue( mSimView->simulator()->carDriver()->steeringControlParams().P );
	steeringControlLayout->addWidget(steeringControlPSpinBox);
	steeringControlLayout->addStretch();
	connect( steeringControlPSpinBox, SIGNAL(valueChanged(double)), mSimView->simulator()->carDriver(), SLOT(setSteeringControlP(double)) );

	QLabel *steeringControlILabel = new QLabel(mParamsWidget);
	steeringControlILabel->setText("I:");
	steeringControlLayout->addWidget(steeringControlILabel);
	QDoubleSpinBox *steeringControlISpinBox = new QDoubleSpinBox(mParamsWidget);
	steeringControlISpinBox->setDecimals(3);
	steeringControlISpinBox->setSingleStep(0.5);
	steeringControlISpinBox->setValue( mSimView->simulator()->carDriver()->steeringControlParams().I );
	steeringControlLayout->addWidget(steeringControlISpinBox);
	steeringControlLayout->addStretch();
	connect( steeringControlISpinBox, SIGNAL(valueChanged(double)), mSimView->simulator()->carDriver(), SLOT(setSteeringControlI(double)) );

	QLabel *steeringControlDLabel = new QLabel(mParamsWidget);
	steeringControlDLabel->setText("D:");
	steeringControlLayout->addWidget(steeringControlDLabel);
	QDoubleSpinBox *steeringControlDSpinBox = new QDoubleSpinBox(mParamsWidget);
	steeringControlDSpinBox->setDecimals(3);
	steeringControlDSpinBox->setSingleStep(0.5);
	steeringControlDSpinBox->setValue( mSimView->simulator()->carDriver()->steeringControlParams().D );
	steeringControlLayout->addWidget(steeringControlDSpinBox);
	steeringControlLayout->addStretch();
	connect( steeringControlDSpinBox, SIGNAL(valueChanged(double)), mSimView->simulator()->carDriver(), SLOT(setSteeringControlD(double)) );

	paramsLayout->addLayout(steeringControlLayout);


	QHBoxLayout *manualSteeringLayout = new QHBoxLayout;
	QLabel *manSteerLabel = new QLabel(mParamsWidget);
	manSteerLabel->setText("manualSteering:");
	manualSteeringLayout->addWidget(manSteerLabel);
	QCheckBox *manSteerCheckbox = new QCheckBox(mParamsWidget);
	manSteerCheckbox->setChecked( mSimView->simulator()->carDriver()->manualDrive() );
	connect( manSteerCheckbox, SIGNAL(clicked(bool)), mSimView->simulator()->carDriver(), SLOT(setManualDrive(bool)) );
	manualSteeringLayout->addWidget(manSteerCheckbox);
	paramsLayout->addLayout(manualSteeringLayout);
	QSlider *steeringSlider = new QSlider(Qt::Horizontal,mParamsWidget);
	steeringSlider->setObjectName("steeringSlider");
	steeringSlider->setMinimum(-mSimView->simulator()->car()->maxWheelAngle()/M_PI*180.0);
	steeringSlider->setMaximum(mSimView->simulator()->car()->maxWheelAngle()/M_PI*180.0);
	steeringSlider->setTickInterval(1);
	steeringSlider->setValue( mSimView->simulator()->car()->wheelAngle()/M_PI*180.0 );
	steeringSlider->setEnabled( mSimView->simulator()->carDriver()->manualDrive() );
	connect( steeringSlider, SIGNAL(valueChanged(int)), this, SLOT(onSteeringSliderChanged(int)) );
	connect( manSteerCheckbox, SIGNAL(clicked(bool)), steeringSlider, SLOT(setEnabled(bool)) );
	paramsLayout->addWidget(steeringSlider);


	QHBoxLayout *frictionCoeffLayout = new QHBoxLayout;
	QLabel *frictionCoeffLabel = new QLabel(mParamsWidget);
	frictionCoeffLabel->setText("Friction coeff.:");
	frictionCoeffLayout->addWidget(frictionCoeffLabel);
	QDoubleSpinBox *frictionCoeffSpinBox = new QDoubleSpinBox(mParamsWidget);
	frictionCoeffSpinBox->setDecimals(2);
	frictionCoeffSpinBox->setSingleStep(0.1);
	frictionCoeffSpinBox->setValue( mSimView->simulator()->car()->frictionCoeffStatic() );
	connect( frictionCoeffSpinBox, SIGNAL(valueChanged(double)), mSimView->simulator()->car(), SLOT(setFrictionCoeffStatic(double)) );
	frictionCoeffLayout->addWidget(frictionCoeffSpinBox);
	paramsLayout->addLayout(frictionCoeffLayout);

	QHBoxLayout *targetCrossPosLayout = new QHBoxLayout;
	QLabel *targetCrossPosLabel = new QLabel(mParamsWidget);
	targetCrossPosLabel->setText("targetCrossPos:");
	targetCrossPosLayout->addWidget(targetCrossPosLabel);
	QDoubleSpinBox *targetCrossPosSpinBox = new QDoubleSpinBox(mParamsWidget);
	targetCrossPosSpinBox->setMinimum(-100.0);
	targetCrossPosSpinBox->setMaximum(100.0);
	targetCrossPosSpinBox->setDecimals(2);
	targetCrossPosSpinBox->setSingleStep(0.1);
	targetCrossPosSpinBox->setValue( mSimView->simulator()->carDriver()->targetCrossPos() );
	connect( targetCrossPosSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onTargetCrossPosSPinBoxChanged(double)) );
	targetCrossPosLayout->addWidget(targetCrossPosSpinBox);
	paramsLayout->addLayout(targetCrossPosLayout);

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

	dashboardLayout->addStretch();
}
