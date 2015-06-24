/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#ifndef _VIEWER_H_
#define _VIEWER_H_

#include <QWidget>
#include <QMutex>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLColor.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLPointSet.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLSwitch.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include "DamaModel.h"
#include <rl/plan/VectorList.h>
#include <rl/plan/Viewer.h>

class Viewer : public QWidget, public rl::plan::Viewer
{
	Q_OBJECT
	
public:
	Viewer(QWidget* parent = NULL, Qt::WindowFlags f = 0);
	
	virtual ~Viewer();
	
	void customize();

	rl::math::Real delta;
	
	dama::DamaModel* model;
	
	SoVRMLGroup* sceneGroup;
	
	SoQtExaminerViewer* viewer;
	
	QMutex draw_mutex;

public slots:
	void drawConfiguration(const rl::math::Vector& q);
	
	void drawConfigurationEdge(const rl::math::Vector& u, const rl::math::Vector& v, const bool& free = true);
	
	void drawConfigurationPath(const rl::plan::VectorList& path);
	
	void drawConfigurationVertex(const rl::math::Vector& q, const bool& free = true);
	
	void drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1);
	
	void drawPoint(const rl::math::Vector& xyz);
	
	void drawSphere(const rl::math::Vector& center, const rl::math::Real& radius);
	
	void drawSweptVolume(const rl::plan::VectorList& path);
	
	void drawWork(const rl::math::Transform& t);
	
	void drawWorkEdge(const rl::math::Vector& u, const rl::math::Vector& v);
	
	void drawWorkPath(const rl::plan::VectorList& path);
	
	void drawWorkVertex(const rl::math::Vector& q);
	
	void reset();
	
	void resetEdges();
	
	void resetLines();
	
	void resetPoints();
	
	void resetSpheres();
	
	void resetVertices();
	
	void saveImage(const QString& filename);
	
	void saveScene(const QString& filename);
	
	void toggleConfigurationEdges(const bool& doOn);
	
	void toggleConfigurationVertices(const bool& doOn);
	
	void toggleLines(const bool& doOn);
	
	void togglePoints(const bool& doOn);
	
	void toggleSpheres(const bool& doOn);
	
	void toggleWorkFrames(const bool& doOn);
	
protected:
	
private:
	SoVRMLSwitch* edges;
	
	SoVRMLSwitch* edgesColliding;
	
	SoVRMLAppearance* edgesCollidingAppearance;
	
	SoVRMLCoordinate* edgesCollidingCoordinate;
	
	SoDrawStyle* edgesCollidingDrawStyle;
	
	SoVRMLIndexedLineSet* edgesCollidingIndexedLineSet;
	
	SoVRMLMaterial* edgesCollidingMaterial;
	
	SoVRMLShape* edgesCollidingShape;
	
	SoVRMLSwitch* edgesFree;
	
	SoVRMLAppearance* edgesFreeAppearance;
	
	SoVRMLCoordinate* edgesFreeCoordinate;
	
	SoDrawStyle* edgesFreeDrawStyle;
	
	SoVRMLIndexedLineSet* edgesFreeIndexedLineSet;
	
	SoVRMLMaterial* edgesFreeMaterial;
	
	SoVRMLShape* edgesFreeShape;
	
	SoVRMLSwitch* edges3;
	
	SoVRMLAppearance* edges3Appearance;
	
	SoVRMLCoordinate* edges3Coordinate;
	
	SoDrawStyle* edges3DrawStyle;
	
	SoVRMLIndexedLineSet* edges3IndexedLineSet;
	
	SoVRMLMaterial* edges3Material;
	
	SoVRMLShape* edges3Shape;
	
	SoVRMLIndexedLineSet* frameIndexedLineSet;
	
	SoVRMLSwitch* lines;
	
	SoVRMLAppearance* linesAppearance;
	
	SoVRMLCoordinate* linesCoordinate;
	
	SoDrawStyle* linesDrawStyle;
	
	SoVRMLIndexedLineSet* linesIndexedLineSet;
	
	SoVRMLMaterial* linesMaterial;
	
	SoVRMLShape* linesShape;
	
	SoVRMLSwitch* path;
	
	SoVRMLAppearance* pathAppearance;
	
	SoVRMLCoordinate* pathCoordinate;
	
	SoDrawStyle* pathDrawStyle;
	
	SoVRMLIndexedLineSet* pathIndexedLineSet;
	
	SoVRMLMaterial* pathMaterial;
	
	SoVRMLShape* pathShape;
	
	SoVRMLSwitch* path3;
	
	SoVRMLAppearance* path3Appearance;
	
	SoVRMLCoordinate* path3Coordinate;
	
	SoDrawStyle* path3DrawStyle;
	
	SoVRMLIndexedLineSet* path3IndexedLineSet;
	
	SoVRMLMaterial* path3Material;
	
	SoVRMLShape* path3Shape;
	
	SoVRMLSwitch* points;
	
	SoVRMLAppearance* pointsAppearance;
	
	SoVRMLCoordinate* pointsCoordinate;
	
	SoDrawStyle* pointsDrawStyle;
	
	SoVRMLPointSet* pointsPointSet;
	
	SoVRMLMaterial* pointsMaterial;
	
	SoVRMLShape* pointsShape;
	
	SoVRMLSwitch* root;
	
	SoVRMLSwitch* scene;
	
	SoDrawStyle* sceneDrawStyle;
	
	SoVRMLSwitch* spheres;
	
	SoVRMLAppearance* spheresAppearance;
	
	SoDrawStyle* spheresDrawStyle;
	
	SoVRMLGroup* spheresGroup;
	
	SoVRMLMaterial* spheresMaterial;
	
	SoVRMLSwitch* swept;
	
	SoVRMLGroup* sweptGroup;
	
	SoVRMLSwitch* vertices;
	
	SoVRMLSwitch* verticesColliding;
	
	SoVRMLAppearance* verticesCollidingAppearance;
	
	SoVRMLColor* verticesCollidingColor;
	
	SoVRMLCoordinate* verticesCollidingCoordinate;
	
	SoDrawStyle* verticesCollidingDrawStyle;
	
	SoVRMLPointSet* verticesCollidingPointSet;
	
	SoVRMLMaterial* verticesCollidingMaterial;
	
	SoVRMLShape* verticesCollidingShape;
	
	SoVRMLSwitch* verticesFree;
	
	SoVRMLAppearance* verticesFreeAppearance;
	
	SoVRMLColor* verticesFreeColor;
	
	SoVRMLCoordinate* verticesFreeCoordinate;
	
	SoDrawStyle* verticesFreeDrawStyle;
	
	SoVRMLPointSet* verticesFreePointSet;
	
	SoVRMLMaterial* verticesFreeMaterial;
	
	SoVRMLShape* verticesFreeShape;
	
	SoVRMLSwitch* work;
	
	SoDrawStyle* workDrawStyle;
	
	SoVRMLTransform* workTransform;
};

#endif // _VIEWER_H_
