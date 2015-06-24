/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <QAction>
#include <QDockWidget>
#include <QGraphicsView>
#include <QMainWindow>
#include <QMutex>
#include <QTableView>
#include <QCloseEvent>
#include <boost/shared_ptr.hpp>
#include <rl/kin/Kinematics.h>
#include <rl/mdl/Dynamic.h>
#include "XmlFactory.h"
#include "DamaRrt.h"
#include "DamaModel.h"
#include "DamaSampler.h"
#include <rl/plan/DistanceModel.h>
#include <rl/plan/Optimizer.h>
#include <rl/plan/Planner.h>
#include <rl/plan/Sampler.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/WorkspaceSphereExplorer.h>
#include <rl/sg/Model.h>
#include <rl/sg/so/Model.h>
#include <rl/sg/Scene.h>
#include <rl/sg/so/Scene.h>

class ConfigurationDelegate;
class ConfigurationModel;
class ConfigurationSpaceScene;
class PlannerModel;
class Thread;
class Viewer;

class MainWindow : public QMainWindow
{
	Q_OBJECT
	
public:
	virtual ~MainWindow();
	
	static MainWindow* instance();

	ConfigurationModel* configurationModel;
	
	std::vector< boost::shared_ptr< rl::math::Vector3 > > explorerGoals;
	
	std::vector< boost::shared_ptr< rl::plan::WorkspaceSphereExplorer > > explorers;
	
	std::vector< boost::shared_ptr< rl::math::Vector3 > > explorerStarts;
	
	boost::shared_ptr< rl::math::Vector > goal;
	
	boost::shared_ptr< ::std::vector<bool> > goalDimDefined;

	boost::shared_ptr< rl::kin::Kinematics > kin;
	
	boost::shared_ptr< rl::kin::Kinematics > kin2;
	
	boost::shared_ptr< rl::mdl::Dynamic > mdl;
	
	boost::shared_ptr< rl::mdl::Dynamic > mdl2;
	
	boost::shared_ptr< dama::DamaModel > model;
	
	boost::shared_ptr< dama::DamaModel > model2;
	
	QMutex mutex;
	
	boost::shared_ptr< rl::plan::Optimizer > optimizer;
	
	boost::shared_ptr< dama::DamaRrt > planner;
	
	PlannerModel* plannerModel;
	
	boost::shared_ptr< rl::math::Vector > q;
	
	boost::shared_ptr< dama::DamaSampler > sampler;
	
	boost::shared_ptr< dama::DamaSampler > sampler2;
	
	boost::shared_ptr< rl::math::Vector > sigma;
	
	boost::shared_ptr< rl::sg::Scene > scene;
	
	boost::shared_ptr< rl::sg::so::Scene > scene2;
	
	rl::sg::Model* sceneModel;
	
	rl::sg::so::Model* sceneModel2;
	
	boost::shared_ptr< rl::math::Vector > start;
	
	Thread* thread;
	
	boost::shared_ptr< rl::plan::Verifier > verifier;
	
	boost::shared_ptr< rl::plan::Verifier > verifier2;
	
	Viewer* viewer;
	
	bool threadRunning;

	void quitAll();

public slots:
	void eval();
	
	void getGoalConfiguration();
	
	void getRandomConfiguration();
	
	void getRandomFreeConfiguration();
	
	void getStartConfiguration();
	
	void open();
	
	void reset();
	
	void saveImage();
	
	void savePdf();
	
	void saveScene();
	
	void setGoalConfiguration();
	
	void setStartConfiguration();
	
	void startThread();
	
	void toggleCamera();
	
	void toggleConfiguration();
	
	void toggleConfigurationSpace();
	
	void togglePlanner();
	
	void toggleView(const bool& doOn);
	
protected:
	MainWindow(QWidget* parent = NULL, Qt::WindowFlags f = 0);
	
private:
	void clear();
	
	void connect(const QObject* sender, const QObject* receiver);
	
	void disconnect(const QObject* sender, const QObject* receiver);
	
	void init();

	void closeEvent(QCloseEvent *event);

	//void load(const QString& filename);
	
	void initAll();

	ConfigurationDelegate* configurationDelegate;
	
	QDockWidget* configurationDockWidget;
	
	QDockWidget* configurationSpaceDockWidget;
	
	ConfigurationSpaceScene* configurationSpaceScene;
	
	QGraphicsView* configurationSpaceView;
	
	QTableView* configurationView;
	
	QString engine;
	
	QAction* evalAction;
	
	QAction* exitAction;
	
	QString filename;
	
	QAction* getGoalConfigurationAction;
	
	QAction* getRandomConfigurationAction;
	
	QAction* getRandomFreeConfigurationAction;
	
	QAction* getStartConfigurationAction;
	
	QAction* openAction;
	
	QDockWidget* plannerDockWidget;
	
	QTableView* plannerView;
	
	QAction* resetAction;
	
	QAction* saveImageAction;
	
	QAction* savePdfAction;
	
	QAction* saveSceneAction;
	
	QAction* setGoalConfigurationAction;
	
	QAction* setStartConfigurationAction;
	
	static MainWindow* singleton;
	
	QAction* startThreadAction;
	
	QAction* toggleCameraAction;
	
	QAction* toggleConfigurationAction;
	
	QAction* toggleConfigurationEdgesAction;
	
	QAction* toggleConfigurationSpaceAction;
	
	QAction* toggleConfigurationVerticesAction;
	
	QAction* toggleLinesAction;
	
	QAction* togglePlannerAction;
	
	QAction* togglePointsAction;
	
	QAction* toggleSpheresAction;
	
	QAction* toggleViewAction;
	
	QAction* toggleWorkFramesAction;
	
	bool wait;
};

#endif // _MAINWINDOW_H_
