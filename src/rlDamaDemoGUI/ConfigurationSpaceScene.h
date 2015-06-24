/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#ifndef _CONFIGURATIONSPACESCENE_H_
#define _CONFIGURATIONSPACESCENE_H_

#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include <QLinkedList>
#include "DamaModel.h"
#include <rl/plan/Viewer.h>

class ConfigurationSpaceThread;

class ConfigurationSpaceScene : public QGraphicsScene, public rl::plan::Viewer
{
	Q_OBJECT
	
public:
	ConfigurationSpaceScene(QObject* parent = NULL);
	
	virtual ~ConfigurationSpaceScene();
	
	rl::math::Real delta;
	
	dama::DamaModel* model;
	
	std::size_t x;
	
	std::size_t y;
	
public slots:
	void addCollision(const qreal& x, const qreal& y, const qreal& w, const qreal& h, const int& rgb);
	
	void clear();
	
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
	
	void eval();
	
	void reset();
	
	void resetEdges();
	
	void resetLines();
	
	void resetPath();
	
	void resetPoints();
	
	void resetSpheres();
	
	void resetVertices();
	
protected:
	void mouseMoveEvent(QGraphicsSceneMouseEvent* mouseEvent);
	
	void mousePressEvent(QGraphicsSceneMouseEvent* mouseEvent);
	
private:
	QLinkedList<QGraphicsLineItem*> edges;
	
	QLinkedList<QGraphicsLineItem*> path;
	
	ConfigurationSpaceThread* thread;
};

#endif // _CONFIGURATIONSPACESCENE_H_
