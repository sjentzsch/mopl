/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#ifndef _THREAD_H_
#define _THREAD_H_

#include <QThread>
#include <rl/plan/Viewer.h>

class Thread : public QThread, public rl::plan::Viewer
{
	Q_OBJECT
	
public:
	Thread(QObject* parent = NULL);
	
	virtual ~Thread();
	
	void drawConfiguration(const rl::math::Vector& q);
	
	void drawConfigurationEdge(const rl::math::Vector& q0, const rl::math::Vector& q1, const bool& free = true);
	
	void drawConfigurationPath(const rl::plan::VectorList& path);
	
	void drawConfigurationVertex(const rl::math::Vector& q, const bool& free = true);
	
	void drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1);
	
	void drawPoint(const rl::math::Vector& xyz);
	
	void drawSphere(const rl::math::Vector& center, const rl::math::Real& radius);
	
	void drawSweptVolume(const rl::plan::VectorList& path);
	
	void drawWork(const rl::math::Transform& t);
	
	void drawWorkEdge(const rl::math::Vector& q0, const rl::math::Vector& q1);
	
	void drawWorkPath(const rl::plan::VectorList& path);
	
	void drawWorkVertex(const rl::math::Vector& q);
	
	void reset();
	
	void resetEdges();
	
	void resetLines();
	
	void resetPoints();
	
	void resetSpheres();
	
	void resetVertices();
	
	void run();
	
	void stop();
	
	bool quit;
	
	bool swept;
	
protected:
	
private:
	bool running;
	
signals:
	void configurationRequested(const rl::math::Vector& q);
	
	void configurationEdgeRequested(const rl::math::Vector& q0, const rl::math::Vector& q1, const bool& free);
	
	void configurationVertexRequested(const rl::math::Vector& q, const bool& free);
	
	void configurationPathRequested(const rl::plan::VectorList& path);
	
	void edgeResetRequested();
	
	void lineRequested(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1);
	
	void lineResetRequested();
	
	void pointRequested(const rl::math::Vector& xyz);
	
	void pointResetRequested();
	
	void resetRequested();
	
	void sphereRequested(const rl::math::Vector& center, const rl::math::Real& radius);
	
	void sphereResetRequested();
	
	void sweptVolumeRequested(const rl::plan::VectorList& path);
	
	void vertexResetRequested();
	
	void workRequested(const rl::math::Transform& t);
	
	void workEdgeRequested(const rl::math::Vector& q0, const rl::math::Vector& q1);
	
	void workPathRequested(const rl::plan::VectorList& path);
	
	void workVertexRequested(const rl::math::Vector& q);
};

#endif // _THREAD_H_
