/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#ifndef _CONFIGURATIONSPACETHREAD_H_
#define _CONFIGURATIONSPACETHREAD_H_

#include <QThread>
#include "DamaModel.h"

class ConfigurationSpaceThread : public QThread
{
	Q_OBJECT
	
public:
	ConfigurationSpaceThread(QObject* parent = NULL);
	
	virtual ~ConfigurationSpaceThread();
	
	void run();
	
	void stop();
	
	rl::math::Real delta;
	
	dama::DamaModel* model;
	
	std::size_t x;
	
	std::size_t y;
	
protected:
	
private:
	bool running;
	
signals:
	void addCollision(const qreal& x, const qreal& y, const qreal& w, const qreal& h, const int& rgb);
};

#endif // _CONFIGURATIONSPACETHREAD_H_
