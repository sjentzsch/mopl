/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include "ConfigurationModel.h"
#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

ConfigurationModel::ConfigurationModel(QObject* parent) :
	QAbstractTableModel(parent)
{
}

ConfigurationModel::~ConfigurationModel()
{
}

int
ConfigurationModel::columnCount(const QModelIndex& parent) const
{
	return 1;
}

QVariant
ConfigurationModel::data(const QModelIndex& index, int role) const
{
	if (NULL == MainWindow::instance()->model)
	{
		return QVariant();
	}
	
	if (!index.isValid())
	{
		return QVariant();
	}
	
	switch (role)
	{
	case Qt::DisplayRole:
	case Qt::EditRole:
		{
			Eigen::Matrix< rl::math::Unit, Eigen::Dynamic, 1 > qUnits(MainWindow::instance()->model->getDof(0));
			MainWindow::instance()->model->getPositionUnits(qUnits);
			
			if (rl::math::UNIT_RADIAN == qUnits(index.row()))
			{
				return (*MainWindow::instance()->q)(index.row()) * rl::math::RAD2DEG;
			}
			else
			{
				return (*MainWindow::instance()->q)(index.row());
			}
		}
	case Qt::TextAlignmentRole:
		return Qt::AlignRight;
		break;
	default:
		break;
	}
	
	return QVariant();
}

Qt::ItemFlags
ConfigurationModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
ConfigurationModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (Qt::DisplayRole == role && Qt::Vertical == orientation)
	{
		return QString::number(section);
	}
	
	return QVariant();
}

void
ConfigurationModel::invalidate()
{
	this->reset();
}

int
ConfigurationModel::rowCount(const QModelIndex& parent) const
{
	if (NULL == MainWindow::instance()->model)
	{
		return 0;
	}
	
	return MainWindow::instance()->model->getDof(0);
}

bool
ConfigurationModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (NULL == MainWindow::instance()->model)
	{
		return false;
	}
	
	if (MainWindow::instance()->thread->isRunning())
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		Eigen::Matrix< rl::math::Unit, Eigen::Dynamic, 1 > qUnits(MainWindow::instance()->model->getDof(0));
		MainWindow::instance()->model->getPositionUnits(qUnits);
		
		if (rl::math::UNIT_RADIAN == qUnits(index.row()))
		{
			(*MainWindow::instance()->q)(index.row()) = value.value< ::rl::math::Real >() * rl::math::DEG2RAD;
		}
		else
		{
			(*MainWindow::instance()->q)(index.row()) = value.value< ::rl::math::Real >();
		}
		
		MainWindow::instance()->viewer->drawConfiguration(*MainWindow::instance()->q);
		
		emit dataChanged(index, index);
		
		return true;
	}
	
	return false;
}
