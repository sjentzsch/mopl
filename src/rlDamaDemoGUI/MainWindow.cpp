/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include <QApplication>
#include <QDateTime>
#include <QDockWidget>
#include <QFileDialog>
#include <QGLWidget>
#include <QGraphicsView>
#include <QHeaderView>
#include <QLayout>
#include <QMenuBar>
#include <QMutexLocker>
#include <QPainter>
#include <QPrinter>
#include <QRegExp>

#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/Qt/SoQt.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/plan/AddRrtConCon.h>
#include <rl/plan/AdvancedOptimizer.h>
#include <rl/plan/BridgeSampler.h>
#include <rl/plan/DistanceModel.h>
#include <rl/plan/GaussianSampler.h>
#include <rl/plan/Prm.h>
#include <rl/plan/PrmUtilityGuided.h>
#include <rl/plan/RecursiveVerifier.h>
#include "XmlFactory.h"
#include "DamaRrt.h"
#include "DamaModel.h"
#include "DamaSampler.h"
#include "DamaPlanner.h"
#include <rl/plan/Rrt.h>
#include <rl/plan/RrtCon.h>
#include <rl/plan/RrtConCon.h>
#include <rl/plan/RrtDual.h>
#include <rl/plan/RrtExtCon.h>
#include <rl/plan/RrtExtExt.h>
#include <rl/plan/RrtGoalBias.h>
#include <rl/plan/SequentialVerifier.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/SimpleOptimizer.h>
#include <rl/plan/UniformSampler.h>
#include <rl/plan/WorkspaceSphereExplorer.h>
#include <rl/sg/Body.h>
#include <rl/xml/Attribute.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Node.h>
#include <rl/xml/Object.h>
#include <rl/xml/Path.h>

#ifdef HAVE_BULLET
#include <rl/sg/bullet/Scene.h>
#endif // HAVE_BULLET
#ifdef HAVE_ODE
#include <rl/sg/ode/Scene.h>
#endif // HAVE_ODE
#ifdef HAVE_PQP
#include <rl/sg/pqp/Scene.h>
#endif // HAVE_PQP
#ifdef HAVE_SOLID
#include <rl/sg/solid/Scene.h>
#endif // HAVE_SOLID

#include "ConfigurationDelegate.h"
#include "ConfigurationModel.h"
#include "ConfigurationSpaceScene.h"
#include "MainWindow.h"
#include "PlannerModel.h"
#include "Thread.h"
#include "Viewer.h"

MainWindow::MainWindow(QWidget* parent, Qt::WFlags f) :
	QMainWindow(parent, f),
	configurationModel(new ConfigurationModel(this)),
	explorerGoals(),
	explorers(),
	explorerStarts(),
	goal(),
	kin(),
	kin2(),
	mdl(),
	mdl2(),
	model(),
	model2(),
	mutex(),
	planner(),
	plannerModel(new PlannerModel(this)),
	q(),
	sampler(),
	sampler2(),
	sigma(),
	scene(),
	scene2(),
	sceneModel(NULL),
	sceneModel2(NULL),
	start(),
	thread(new Thread(this)),
	verifier(),
	verifier2(),
	viewer(NULL),
	configurationDelegate(new ConfigurationDelegate(this)),
	configurationDockWidget(new QDockWidget(this)),
	configurationSpaceDockWidget(new QDockWidget(this)),
	configurationSpaceScene(new ConfigurationSpaceScene(this)),
	configurationSpaceView(new QGraphicsView(this)),
	configurationView(new QTableView(this)),
	engine(),
	evalAction(new QAction(this)),
	exitAction(new QAction(this)),
	filename(),
	getGoalConfigurationAction(new QAction(this)),
	getRandomConfigurationAction(new QAction(this)),
	getRandomFreeConfigurationAction(new QAction(this)),
	getStartConfigurationAction(new QAction(this)),
	openAction(new QAction(this)),
	plannerDockWidget(new QDockWidget(this)),
	plannerView(new QTableView(this)),
	resetAction(new QAction(this)),
	saveImageAction(new QAction(this)),
	savePdfAction(new QAction(this)),
	saveSceneAction(new QAction(this)),
	setGoalConfigurationAction(new QAction(this)),
	setStartConfigurationAction(new QAction(this)),
	startThreadAction(new QAction(this)),
	toggleCameraAction(new QAction(this)),
	toggleConfigurationAction(new QAction(this)),
	toggleConfigurationEdgesAction(new QAction(this)),
	toggleConfigurationSpaceAction(new QAction(this)),
	toggleConfigurationVerticesAction(new QAction(this)),
	toggleLinesAction(new QAction(this)),
	togglePlannerAction(new QAction(this)),
	togglePointsAction(new QAction(this)),
	toggleSpheresAction(new QAction(this)),
	toggleViewAction(new QAction(this)),
	toggleWorkFramesAction(new QAction(this)),
	wait(true)
{
	MainWindow::singleton = this;
	
	SoQt::init(this);
	SoDB::init();

	this->viewer = new Viewer(this);
	this->setCentralWidget(this->viewer);
	
	this->configurationSpaceView->setEnabled(false);
	this->configurationSpaceView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	this->configurationSpaceView->setScene(this->configurationSpaceScene);
	this->configurationSpaceView->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	this->configurationSpaceView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	
	this->configurationView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
	this->configurationView->horizontalHeader()->hide();
	this->configurationView->setAlternatingRowColors(true);
	this->configurationView->setItemDelegate(this->configurationDelegate);
	this->configurationView->setModel(this->configurationModel);
	this->configurationView->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
	
	this->configurationDockWidget->hide();
	this->configurationDockWidget->resize(160, 320);
	this->configurationDockWidget->setFloating(true);
	this->configurationDockWidget->setWidget(this->configurationView);
	this->configurationDockWidget->setWindowTitle("Configuration");
	
	this->configurationSpaceDockWidget->hide();
	this->configurationSpaceDockWidget->setFloating(true);
	this->configurationSpaceDockWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	this->configurationSpaceDockWidget->setWidget(this->configurationSpaceView);
	this->configurationSpaceDockWidget->setWindowTitle("C-Space");
	
	this->plannerView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
	this->plannerView->setAlternatingRowColors(true);
	this->plannerView->setModel(this->plannerModel);
	this->plannerView->setWordWrap(false);
	this->plannerView->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
	
	this->plannerDockWidget->hide();
	this->plannerDockWidget->resize(160, 320);
	this->plannerDockWidget->setFloating(true);
	this->plannerDockWidget->setWidget(this->plannerView);
	this->plannerDockWidget->setWindowTitle("Planner");
	
	this->init();
	
	QStringList engines;
#ifdef HAVE_ODE
	engines.push_back("ode");
	this->engine = "ode";
#endif // HAVE_ODE
#ifdef HAVE_PQP
	engines.push_back("pqp");
	this->engine = "pqp";
#endif // HAVE_PQP
#ifdef HAVE_BULLET
	engines.push_back("bullet");
	this->engine = "bullet";
#endif // HAVE_BULLET
#ifdef HAVE_SOLID
	engines.push_back("solid");
	this->engine = "solid";
#endif // HAVE_SOLID
	engines.sort();
	
	QRegExp engineRegExp("--engine=(" + engines.join("|") + ")");
	QRegExp helpRegExp("--help");
	QRegExp heightRegExp("--height=(\\d*)");
	QRegExp viewerRegExp("--disable-viewer");
	QRegExp waitRegExp("--disable-wait");
	QRegExp widthRegExp("--width=(\\d*)");
	QRegExp quitRegExp("--enable-quit");
	
	int width = 1024;
	int height = 768;
	
	for (int i = 1; i < QApplication::arguments().size(); ++i)
	{
		if (-1 != engineRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->engine = engineRegExp.cap(1);
		}
		else if (-1 != helpRegExp.indexIn(QApplication::arguments()[i]))
		{
			std::cout << "Usage: rlPlanDemo [PLANFILE] [--engine=" << engines.join("|").toStdString() << "] [--help] [--disable-viewer] [--disable-wait] [--enable-quit] [--width=WIDTH] [--height=HEIGHT]" << std::endl;
		}
		else if (-1 != heightRegExp.indexIn(QApplication::arguments()[i]))
		{
			height = heightRegExp.cap(1).toInt();
		}
		else if (-1 != viewerRegExp.indexIn(QApplication::arguments()[i]))
		{
			QObject::disconnect(this->toggleViewAction, SIGNAL(toggled(bool)), this, SLOT(toggleView(bool)));
			this->toggleViewAction->setChecked(false);
			QObject::connect(this->toggleViewAction, SIGNAL(toggled(bool)), this, SLOT(toggleView(bool)));
		}
		else if (-1 != waitRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->wait = false;
		}
		else if (-1 != widthRegExp.indexIn(QApplication::arguments()[i]))
		{
			width = widthRegExp.cap(1).toInt();
		}
		else if (-1 != quitRegExp.indexIn(QApplication::arguments()[i]))
		{
			this->thread->quit = true;
		}
		else
		{
			this->filename = QApplication::arguments()[i];
		}
	}
	
	this->resize(width, height);
	this->viewer->setMinimumSize(width, height);
	
	this->initAll();

	/*if (this->filename.isEmpty())
	{
		this->open();
	}
	else
	{
		this->load(this->filename);
	}*/
}

MainWindow::~MainWindow()
{
	this->thread->stop();
	
	MainWindow::singleton = NULL;
}

void
MainWindow::clear()
{
	this->explorerGoals.clear();
	this->explorers.clear();
	this->explorerStarts.clear();
	this->goal.reset();
	this->kin.reset();
	this->kin2.reset();
	this->mdl.reset();
	this->mdl2.reset();
	this->model.reset();
	this->model2.reset();
	this->optimizer.reset();
	this->planner.reset();
	this->q.reset();
	this->sampler.reset();
	this->sampler2.reset();
	this->sigma.reset();
	this->scene.reset();
	this->scene2.reset();
	this->sceneModel = NULL;
	this->sceneModel2 = NULL;
	this->start.reset();
	this->verifier.reset();
	this->verifier2.reset();
}

void
MainWindow::connect(const QObject* sender, const QObject* receiver)
{
	QObject::connect(
		sender,
		SIGNAL(configurationRequested(const rl::math::Vector&)),
		receiver,
		SLOT(drawConfiguration(const rl::math::Vector&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(configurationEdgeRequested(const rl::math::Vector&, const rl::math::Vector&, const bool&)),
		receiver,
		SLOT(drawConfigurationEdge(const rl::math::Vector&, const rl::math::Vector&, const bool&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(configurationPathRequested(const rl::plan::VectorList&)),
		receiver,
		SLOT(drawConfigurationPath(const rl::plan::VectorList&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(configurationVertexRequested(const rl::math::Vector&, const bool&)),
		receiver,
		SLOT(drawConfigurationVertex(const rl::math::Vector&, const bool&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(edgeResetRequested()),
		receiver,
		SLOT(resetEdges())
	);
	
	QObject::connect(
		sender,
		SIGNAL(lineRequested(const rl::math::Vector&, const rl::math::Vector&)),
		receiver,
		SLOT(drawLine(const rl::math::Vector&, const rl::math::Vector&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(lineResetRequested()),
		receiver,
		SLOT(resetLines())
	);
	
	QObject::connect(
		sender,
		SIGNAL(pointRequested(const rl::math::Vector&)),
		receiver,
		SLOT(drawPoint(const rl::math::Vector&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(pointResetRequested()),
		receiver,
		SLOT(resetPoints())
	);
	
	QObject::connect(
		sender,
		SIGNAL(resetRequested()),
		receiver,
		SLOT(reset())
	);
	
	QObject::connect(
		sender,
		SIGNAL(sphereRequested(const rl::math::Vector&, const rl::math::Real&)),
		receiver,
		SLOT(drawSphere(const rl::math::Vector&, const rl::math::Real&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(sphereResetRequested()),
		receiver,
		SLOT(resetSpheres())
	);
	
	QObject::connect(
		sender,
		SIGNAL(sweptVolumeRequested(const rl::plan::VectorList&)),
		receiver,
		SLOT(drawSweptVolume(const rl::plan::VectorList&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(vertexResetRequested()),
		receiver,
		SLOT(resetVertices())
	);
	
	QObject::connect(
		sender,
		SIGNAL(workRequested(const rl::math::Transform&)),
		receiver,
		SLOT(drawWork(const rl::math::Transform&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(workEdgeRequested(const rl::math::Vector&, const rl::math::Vector&)),
		receiver,
		SLOT(drawWorkEdge(const rl::math::Vector&, const rl::math::Vector&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(workPathRequested(const rl::plan::VectorList&)),
		receiver,
		SLOT(drawWorkPath(const rl::plan::VectorList&))
	);
}

void
MainWindow::disconnect(const QObject* sender, const QObject* receiver)
{
	QObject::disconnect(sender, NULL, receiver, NULL);
}

void
MainWindow::eval()
{
	this->configurationSpaceScene->eval();
}

void
MainWindow::getGoalConfiguration()
{
	*this->q = *this->goal;
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::getRandomConfiguration()
{
	this->sampler2->generate(*this->q);
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::getRandomFreeConfiguration()
{
	this->sampler2->generateCollisionFree(*this->q);
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::getStartConfiguration()
{
	*this->q = *this->start;
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::init()
{
	QMenu* fileMenu = this->menuBar()->addMenu("File");
	
	this->openAction->setText("Open...");
	this->openAction->setShortcut(QKeySequence::Open);
	QObject::connect(this->openAction, SIGNAL(triggered()), this, SLOT(open()));
	this->addAction(this->openAction);
	fileMenu->addAction(this->openAction);
	
	fileMenu->addSeparator();
	
	this->saveImageAction->setText("Save as PNG");
	this->saveImageAction->setShortcut(QKeySequence("Return"));
	QObject::connect(this->saveImageAction, SIGNAL(triggered()), this, SLOT(saveImage()));
	this->addAction(this->saveImageAction);
	fileMenu->addAction(this->saveImageAction);
	
	this->saveSceneAction->setText("Save as VRML");
	this->saveSceneAction->setShortcut(QKeySequence("Ctrl+Return"));
	QObject::connect(this->saveSceneAction, SIGNAL(triggered()), this, SLOT(saveScene()));
	this->addAction(this->saveSceneAction);
	fileMenu->addAction(this->saveSceneAction);
	
	this->savePdfAction->setText("Save as PDF");
	this->savePdfAction->setShortcut(QKeySequence("Alt+Return"));
	QObject::connect(this->savePdfAction, SIGNAL(triggered()), this, SLOT(savePdf()));
	this->addAction(this->savePdfAction);
	fileMenu->addAction(this->savePdfAction);
	
	fileMenu->addSeparator();
	
	this->exitAction->setText("Exit");
	QObject::connect(this->exitAction, SIGNAL(triggered()), qApp, SLOT(quit()));
	this->addAction(this->exitAction);
	fileMenu->addAction(this->exitAction);
	
	QMenu* configurationMenu = this->menuBar()->addMenu("Configuration");
	
	this->toggleConfigurationAction->setText("Show/Hide");
	this->toggleConfigurationAction->setShortcut(QKeySequence("F5"));
	QObject::connect(this->toggleConfigurationAction, SIGNAL(triggered()), this, SLOT(toggleConfiguration()));
	this->addAction(this->toggleConfigurationAction);
	configurationMenu->addAction(this->toggleConfigurationAction);
	
	configurationMenu->addSeparator();
	
	this->getRandomConfigurationAction->setText("Random");
	this->getRandomConfigurationAction->setShortcut(QKeySequence("F3"));
	QObject::connect(this->getRandomConfigurationAction, SIGNAL(triggered()), this, SLOT(getRandomConfiguration()));
	this->addAction(this->getRandomConfigurationAction);
	configurationMenu->addAction(this->getRandomConfigurationAction);
	
	this->getRandomFreeConfigurationAction->setText("Random (Collision-Free)");
	this->getRandomFreeConfigurationAction->setShortcut(QKeySequence("F4"));
	QObject::connect(this->getRandomFreeConfigurationAction, SIGNAL(triggered()), this, SLOT(getRandomFreeConfiguration()));
	this->addAction(this->getRandomFreeConfigurationAction);
	configurationMenu->addAction(this->getRandomFreeConfigurationAction);
	
	QMenu* cSpaceMenu = this->menuBar()->addMenu("C-Space");
	
	this->toggleConfigurationSpaceAction->setText("Show/Hide");
	this->toggleConfigurationSpaceAction->setShortcut(QKeySequence("F6"));
	QObject::connect(this->toggleConfigurationSpaceAction, SIGNAL(triggered()), this, SLOT(toggleConfigurationSpace()));
	this->addAction(this->toggleConfigurationSpaceAction);
	cSpaceMenu->addAction(this->toggleConfigurationSpaceAction);
	
	cSpaceMenu->addSeparator();
	
	this->evalAction->setText("Evaluate");
	this->evalAction->setShortcut(QKeySequence("F11"));
	QObject::connect(this->evalAction, SIGNAL(triggered()), this, SLOT(eval()));
	this->addAction(this->evalAction);
	cSpaceMenu->addAction(this->evalAction);
	
	QMenu* plannerMenu = this->menuBar()->addMenu("Planner");
	
	this->togglePlannerAction->setText("Show/Hide");
	this->togglePlannerAction->setShortcut(QKeySequence("F7"));
	QObject::connect(this->togglePlannerAction, SIGNAL(triggered()), this, SLOT(togglePlanner()));
	this->addAction(this->togglePlannerAction);
	plannerMenu->addAction(this->togglePlannerAction);
	
	plannerMenu->addSeparator();
	
	this->getStartConfigurationAction->setText("Get Start Configuration");
	this->getStartConfigurationAction->setShortcut(QKeySequence("F1"));
	QObject::connect(this->getStartConfigurationAction, SIGNAL(triggered()), this, SLOT(getStartConfiguration()));
	this->addAction(this->getStartConfigurationAction);
	plannerMenu->addAction(this->getStartConfigurationAction);
	
	this->setStartConfigurationAction->setText("Set Start Configuration");
	this->setStartConfigurationAction->setShortcut(QKeySequence("CTRL+F1"));
	QObject::connect(this->setStartConfigurationAction, SIGNAL(triggered()), this, SLOT(setStartConfiguration()));
	this->addAction(this->setStartConfigurationAction);
	plannerMenu->addAction(this->setStartConfigurationAction);
	
	plannerMenu->addSeparator();
	
	this->getGoalConfigurationAction->setText("Get Goal Configuration");
	this->getGoalConfigurationAction->setShortcut(QKeySequence("F2"));
	QObject::connect(this->getGoalConfigurationAction, SIGNAL(triggered()), this, SLOT(getGoalConfiguration()));
	this->addAction(this->getGoalConfigurationAction);
	plannerMenu->addAction(this->getGoalConfigurationAction);
	
	this->setGoalConfigurationAction->setText("Set Goal Configuration");
	this->setGoalConfigurationAction->setShortcut(QKeySequence("CTRL+F2"));
	QObject::connect(this->setGoalConfigurationAction, SIGNAL(triggered()), this, SLOT(setGoalConfiguration()));
	this->addAction(this->setGoalConfigurationAction);
	plannerMenu->addAction(this->setGoalConfigurationAction);
	
	plannerMenu->addSeparator();
	
	this->startThreadAction->setText("Start");
	this->startThreadAction->setShortcut(QKeySequence("Space"));
	QObject::connect(this->startThreadAction, SIGNAL(triggered()), this, SLOT(startThread()));
	this->addAction(this->startThreadAction);
	plannerMenu->addAction(this->startThreadAction);
	
	this->resetAction->setText("Reset");
	this->resetAction->setShortcut(QKeySequence("F12"));
	QObject::connect(this->resetAction, SIGNAL(triggered()), this, SLOT(reset()));
	this->addAction(this->resetAction);
	plannerMenu->addAction(this->resetAction);
	
	QMenu* viewMenu = this->menuBar()->addMenu("View");
	
	this->toggleViewAction->setCheckable(true);
	this->toggleViewAction->setChecked(true);
	this->toggleViewAction->setText("Active");
	QObject::connect(this->toggleViewAction, SIGNAL(toggled(bool)), this, SLOT(toggleView(bool)));
	this->addAction(this->toggleViewAction);
	viewMenu->addAction(this->toggleViewAction);
	
	viewMenu->addSeparator();
	
	this->toggleConfigurationEdgesAction->setCheckable(true);
	this->toggleConfigurationEdgesAction->setChecked(true);
	this->toggleConfigurationEdgesAction->setText("Configuration Edges");
	QObject::connect(this->toggleConfigurationEdgesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleConfigurationEdges(bool)));
	this->addAction(this->toggleConfigurationEdgesAction);
	viewMenu->addAction(this->toggleConfigurationEdgesAction);
	
	this->toggleConfigurationVerticesAction->setCheckable(true);
	this->toggleConfigurationVerticesAction->setChecked(false);
	this->toggleConfigurationVerticesAction->setText("Configuration Vertices");
	QObject::connect(this->toggleConfigurationVerticesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleConfigurationVertices(bool)));
	this->addAction(this->toggleConfigurationVerticesAction);
	viewMenu->addAction(this->toggleConfigurationVerticesAction);
	
	viewMenu->addSeparator();
	
	this->toggleWorkFramesAction->setCheckable(true);
	this->toggleWorkFramesAction->setChecked(false);
	this->toggleWorkFramesAction->setText("Work Frames");
	QObject::connect(this->toggleWorkFramesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleWorkFrames(bool)));
	this->addAction(this->toggleWorkFramesAction);
	viewMenu->addAction(this->toggleWorkFramesAction);
	
	viewMenu->addSeparator();
	
	this->toggleLinesAction->setCheckable(true);
	this->toggleLinesAction->setChecked(true);
	this->toggleLinesAction->setText("Lines");
	QObject::connect(this->toggleLinesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleLines(bool)));
	this->addAction(this->toggleLinesAction);
	viewMenu->addAction(this->toggleLinesAction);
	
	this->togglePointsAction->setCheckable(true);
	this->togglePointsAction->setChecked(true);
	this->togglePointsAction->setText("Points");
	QObject::connect(this->togglePointsAction, SIGNAL(toggled(bool)), this->viewer, SLOT(togglePoints(bool)));
	this->addAction(this->togglePointsAction);
	viewMenu->addAction(this->togglePointsAction);
	
	this->toggleSpheresAction->setCheckable(true);
	this->toggleSpheresAction->setChecked(true);
	this->toggleSpheresAction->setText("Spheres");
	QObject::connect(this->toggleSpheresAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleSpheres(bool)));
	this->addAction(this->toggleSpheresAction);
	viewMenu->addAction(this->toggleSpheresAction);
	
	viewMenu->addSeparator();
	
	this->toggleCameraAction->setText("Perspective/Orthographic");
	this->toggleCameraAction->setShortcut(QKeySequence("F9"));
	QObject::connect(this->toggleCameraAction, SIGNAL(triggered()), this, SLOT(toggleCamera()));
	this->addAction(this->toggleCameraAction);
	viewMenu->addAction(this->toggleCameraAction);
}

MainWindow*
MainWindow::instance()
{
	if (NULL == MainWindow::singleton)
	{
		new MainWindow();
	}
	
	return MainWindow::singleton;
}

void
MainWindow::initAll()
{
	QMutexLocker lock(&this->mutex);

	this->clear();

	this->filename = QString("DAMA");
	this->setWindowTitle(filename + " - " + this->engine + " - rlDamaDemoGUI");

	this->model2 = boost::shared_ptr< dama::DamaModel >(DamaPlanner::getInstance()->dModelVis);
	this->model = boost::shared_ptr< dama::DamaModel >(DamaPlanner::getInstance()->dModel);

	this->scene = boost::shared_ptr< rl::sg::Scene >(this->model->scene);
	this->scene2 = boost::shared_ptr< rl::sg::so::Scene >(static_cast< rl::sg::so::Scene* >(this->model2->scene));
	this->sceneModel = this->model->model;
	this->sceneModel2 = static_cast< rl::sg::so::Model* >(this->model2->model);
	this->mdl = boost::shared_ptr< rl::mdl::Dynamic >(this->model->mdl);
	this->mdl2 = boost::shared_ptr< rl::mdl::Dynamic >(this->model2->mdl);
	this->sampler = boost::shared_ptr< dama::DamaSampler >(this->model->dRrt->dSampler);
	this->sampler2 = boost::shared_ptr< dama::DamaSampler >(this->model2->dRrt->dSampler);
	this->planner = boost::shared_ptr< dama::DamaRrt >(this->model->dRrt);
	this->start = boost::shared_ptr< rl::math::Vector >(this->planner->start);
	this->q = boost::make_shared< rl::math::Vector >(this->model->getDof());
	*this->q = *this->start;
	this->goal = boost::shared_ptr< rl::math::Vector >(this->planner->goal);
	this->mdl->setPosition(*this->q);
	this->mdl->forwardPosition();
	this->goalDimDefined = boost::shared_ptr< ::std::vector<bool> >(this->planner->goalDimDefined);

	this->sigma = boost::make_shared< rl::math::Vector >(0);
	this->optimizer.reset();

	this->viewer->delta = boost::lexical_cast< rl::math::Real >(0.01f);
	this->viewer->sceneGroup->addChild(this->scene2->root);
	this->viewer->model = this->model2.get();
	this->viewer->customize();

	// TODO: operates on planner model?! wtf??
	this->configurationSpaceScene->model = this->model.get();	// was this->model2.get()

	/*if (this->toggleViewAction->isChecked())
		this->toggleView(true);
	else
		this->toggleView(false);*/

	this->viewer->viewer->setBackgroundColor(SbColor(0.0f, 0.0f, 0.0f));
	this->viewer->viewer->setCameraType(SoOrthographicCamera::getClassTypeId());
	this->viewer->viewer->getCamera()->setToDefaults();
	this->viewer->viewer->viewAll();

	this->viewer->viewer->getCamera()->position.setValue(
		this->viewer->viewer->getCamera()->position.getValue()[0],
		this->viewer->viewer->getCamera()->position.getValue()[1],
		this->viewer->viewer->getCamera()->position.getValue()[2]
	);

	this->viewer->viewer->getCamera()->scaleHeight(1.0f);

	this->evalAction->setEnabled(true);
	this->savePdfAction->setEnabled(true);
	this->toggleConfigurationSpaceAction->setEnabled(true);

	this->configurationSpaceScene->delta = 0.099f;

	this->configurationSpaceScene->x = static_cast< std::size_t >(0);
	this->configurationSpaceScene->y = static_cast< std::size_t >(1);

	this->configurationSpaceScene->eval();

	qreal scale = static_cast< std::size_t >(130.0f);

	this->configurationSpaceView->setEnabled(true);

	rl::math::Vector maximum(this->planner->dModel->getDof(0));
	this->planner->dModel->getMaximum(maximum, 0);
	rl::math::Vector minimum(this->planner->dModel->getDof(0));
	this->planner->dModel->getMinimum(minimum, 0);

	this->configurationSpaceView->setSceneRect(
		minimum(this->configurationSpaceScene->x),
		-maximum(this->configurationSpaceScene->y),
		std::abs(maximum(this->configurationSpaceScene->x) - minimum(this->configurationSpaceScene->x)),
		std::abs(maximum(this->configurationSpaceScene->y) - minimum(this->configurationSpaceScene->y))
	);

	this->configurationSpaceView->resetMatrix();
	this->configurationSpaceView->scale(scale, scale);

	this->configurationSpaceView->adjustSize();
	this->configurationSpaceDockWidget->adjustSize();

	this->configurationSpaceDockWidget->setUpdatesEnabled(false);
	this->configurationSpaceDockWidget->setFloating(!this->configurationSpaceDockWidget->isFloating());
	this->configurationSpaceDockWidget->setFloating(!this->configurationSpaceDockWidget->isFloating());
	this->configurationSpaceDockWidget->setUpdatesEnabled(true);

	this->viewer->drawConfiguration(*this->start);

	this->configurationModel->invalidate();
	this->plannerModel->invalidate();

	this->startThread();	// uncomment if you want to start manually via space-key
}

void
MainWindow::open()
{
	if (NULL != this->planner)
	{
		this->reset();
	}
	
	this->initAll();
}

void
MainWindow::reset()
{
	::std::chrono::steady_clock::duration duration = this->planner->duration;
	
	this->thread->blockSignals(true);
	QCoreApplication::processEvents();
	this->planner->duration = ::std::chrono::duration_cast< ::std::chrono::steady_clock::duration >( ::std::chrono::duration< double >(0));
	this->thread->stop();
	this->planner->duration = duration;
	this->thread->blockSignals(false);
	
	this->planner->reset();
	this->model->reset();
	this->viewer->reset();
	this->configurationSpaceScene->reset();
	
	this->configurationView->setEnabled(true);
	this->evalAction->setEnabled(true);
	this->getGoalConfigurationAction->setEnabled(true);
	this->getRandomConfigurationAction->setEnabled(true);
	this->getRandomFreeConfigurationAction->setEnabled(true);
	this->getStartConfigurationAction->setEnabled(true);
	this->openAction->setEnabled(true);
	this->plannerView->setEnabled(true);
	this->setGoalConfigurationAction->setEnabled(true);
	this->setStartConfigurationAction->setEnabled(true);
	this->startThreadAction->setEnabled(true);
	this->toggleViewAction->setEnabled(true);
}

void
MainWindow::saveImage()
{
	this->viewer->saveImage("planDemo-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".png");
}

void
MainWindow::savePdf()
{
	QPrinter printer;
	printer.setOutputFileName("planDemo-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".pdf");
	printer.setOutputFormat(QPrinter::PdfFormat);
	printer.setPageSize(QPrinter::A4);
	
	QPainter painter(&printer);
	
	this->configurationSpaceScene->render(&painter);
}

void
MainWindow::saveScene()
{
	this->viewer->saveScene("planDemo-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".wrl");
}

void
MainWindow::setGoalConfiguration()
{
	*this->goal = *this->q;
}

void
MainWindow::setStartConfiguration()
{
	*this->start = *this->q;
}

void
MainWindow::startThread()
{
	this->configurationView->setEnabled(false);
	this->evalAction->setEnabled(false);
	this->getGoalConfigurationAction->setEnabled(false);
	this->getRandomConfigurationAction->setEnabled(false);
	this->getRandomFreeConfigurationAction->setEnabled(false);
	this->getStartConfigurationAction->setEnabled(false);
	this->openAction->setEnabled(false);
	this->plannerView->setEnabled(false);
	this->setGoalConfigurationAction->setEnabled(false);
	this->setStartConfigurationAction->setEnabled(false);
	this->startThreadAction->setEnabled(false);
	this->toggleViewAction->setEnabled(false);
	
	this->thread->start();
}

void
MainWindow::toggleCamera()
{
	if (SoPerspectiveCamera::getClassTypeId() == this->viewer->viewer->getCameraType())
	{
		this->viewer->viewer->setCameraType(SoOrthographicCamera::getClassTypeId());
	}
	else
	{
		this->viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
	}
	
	SbVec3f position = this->viewer->viewer->getCamera()->position.getValue();
	SbRotation orientation = this->viewer->viewer->getCamera()->orientation.getValue();
	this->viewer->viewer->getCamera()->setToDefaults();
	this->viewer->viewer->getCamera()->position.setValue(position);
	this->viewer->viewer->getCamera()->orientation.setValue(orientation);
	this->viewer->viewer->viewAll();
}

void
MainWindow::toggleConfiguration()
{
	if (this->configurationDockWidget->isVisible())
	{
		this->configurationDockWidget->hide();
	}
	else
	{
		this->configurationDockWidget->show();
	}
}

void
MainWindow::toggleConfigurationSpace()
{
	if (this->configurationSpaceView->isEnabled())
	{
		if (this->configurationSpaceDockWidget->isVisible())
		{
			this->configurationSpaceDockWidget->hide();
		}
		else
		{
			this->configurationSpaceDockWidget->show();
		}
	}
}

void
MainWindow::togglePlanner()
{
	if (this->plannerDockWidget->isVisible())
	{
		this->plannerDockWidget->hide();
	}
	else
	{
		this->plannerDockWidget->show();
	}
}

void
MainWindow::toggleView(const bool& doOn)
{
	if (doOn)
	{
		this->planner->viewer = this->thread;
		
		if (NULL != this->optimizer)
		{
			this->optimizer->viewer = this->thread;
		}
		
		for (std::vector< boost::shared_ptr< rl::plan::WorkspaceSphereExplorer > >::iterator i = this->explorers.begin(); i != this->explorers.end(); ++i)
		{
			(*i)->viewer = this->thread;
		}
		
		this->connect(this->thread, this->configurationSpaceScene);
		this->connect(this->thread, this->viewer);
	}
	else
	{
		this->disconnect(this->thread, this->configurationSpaceScene);
		this->disconnect(this->thread, this->viewer);
		
		this->planner->viewer = NULL;
		
		if (NULL != this->optimizer)
		{
			this->optimizer->viewer = NULL;
		}
		
		for (std::vector< boost::shared_ptr< rl::plan::WorkspaceSphereExplorer > >::iterator i = this->explorers.begin(); i != this->explorers.end(); ++i)
		{
			(*i)->viewer = NULL;
		}
	}
}

void MainWindow::quitAll()
{
	this->thread->stop();
	this->close();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	// TODO: fucking fix me, it only quits due to seg-fault ...

	MainWindow::singleton = NULL;
	delete configurationModel;
	delete plannerModel;
	thread->exit();
	delete thread;

	event->accept();

	QApplication::quit();
}
