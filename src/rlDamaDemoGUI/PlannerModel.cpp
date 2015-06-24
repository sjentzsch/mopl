/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include <rl/plan/Prm.h>
#include <rl/plan/Rrt.h>
#include <rl/plan/RrtGoalBias.h>
#include <rl/plan/Verifier.h>

#include "MainWindow.h"
#include "PlannerModel.h"
#include "Thread.h"

PlannerModel::PlannerModel(QObject* parent) :
	QAbstractTableModel(parent)
{

}

PlannerModel::~PlannerModel()
{
}

int
PlannerModel::columnCount(const QModelIndex& parent) const
{
	return 1;
}

QVariant
PlannerModel::data(const QModelIndex& index, int role) const
{
	if (NULL == MainWindow::instance()->planner)
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
		switch (index.row())
		{
		case 0:
			return -1; //FIXME add duration cast: MainWindow::instance()->planner->duration;
			break;
		default:
			break;
		}
		
		if (rl::plan::Prm* prm = dynamic_cast< rl::plan::Prm* >(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 1:
				return static_cast< unsigned int >(prm->degree);
				break;
			case 2:
				return prm->verifier->delta;
				break;
			case 3:
				return static_cast< unsigned int >(prm->k);
				break;
			case 4:
				return prm->radius;
				break;
			default:
				break;
			}
		}
		
		if (rl::plan::Rrt* rrt = dynamic_cast< rl::plan::Rrt* >(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 1:
				return rrt->delta;
				break;
			case 2:
				return rrt->epsilon;
				break;
			default:
				break;
			}
		}
		
		if (rl::plan::RrtGoalBias* rrtGoalBias = dynamic_cast< rl::plan::RrtGoalBias* >(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 3:
				return rrtGoalBias->probability;
				break;
			default:
				break;
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
PlannerModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
PlannerModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (NULL == MainWindow::instance()->planner)
	{
		return QVariant();
	}
	
	if (Qt::DisplayRole == role && Qt::Horizontal == orientation)
	{
		if (0 == section)
		{
			return MainWindow::instance()->planner->getName().c_str();
		}
	}
	
	if (Qt::DisplayRole == role && Qt::Vertical == orientation)
	{
		switch (section)
		{
		case 0:
			return "duration";
			break;
		default:
			break;
		}
		
		if (dynamic_cast< rl::plan::Prm* >(MainWindow::instance()->planner.get()))
		{
			switch (section)
			{
			case 1:
				return "degree";
				break;
			case 2:
				return "delta";
				break;
			case 3:
				return "k";
				break;
			case 4:
				return "radius";
				break;
			default:
				break;
			}
		}
		
		if (dynamic_cast< rl::plan::Rrt* >(MainWindow::instance()->planner.get()))
		{
			switch (section)
			{
			case 1:
				return "delta";
				break;
			case 2:
				return "epsilon";
				break;
			default:
				break;
			}
		}
		
		if (dynamic_cast< rl::plan::RrtGoalBias* >(MainWindow::instance()->planner.get()))
		{
			switch (section)
			{
			case 3:
				return "probability";
				break;
			default:
				break;
			}
		}
	}
	
	return QVariant();
}

void
PlannerModel::invalidate()
{
	this->reset();
}

int
PlannerModel::rowCount(const QModelIndex& parent) const
{
	if (NULL == MainWindow::instance()->planner)
	{
		return 0;
	}
	
	if (dynamic_cast< rl::plan::Prm* >(MainWindow::instance()->planner.get()))
	{
		return 5;
	}
	else if (dynamic_cast< rl::plan::RrtGoalBias* >(MainWindow::instance()->planner.get()))
	{
		return 4;
	}
	else if (dynamic_cast< rl::plan::Rrt* >(MainWindow::instance()->planner.get()))
	{
		return 3;
	}
	
	return 0;
}

bool
PlannerModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (NULL == MainWindow::instance()->planner)
	{
		return false;
	}
	
	if (MainWindow::instance()->thread->isRunning())
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		switch (index.row())
		{
		case 0:
			MainWindow::instance()->planner->duration = ::std::chrono::duration_cast< ::std::chrono::steady_clock::duration >( ::std::chrono::duration< double >(value.value< ::rl::math::Real >()));
			break;
		default:
			break;
		}
		
		if (rl::plan::Prm* prm = dynamic_cast< rl::plan::Prm* >(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 1:
				prm->degree = value.value< ::std::size_t >();
				break;
			case 2:
				prm->verifier->delta = value.value< ::rl::math::Real >();
				break;
			case 3:
				prm->k = value.value< ::std::size_t >();
				break;
			case 4:
				prm->radius = value.value< ::rl::math::Real >();
				break;
			default:
				break;
			}
		}
		
		if (rl::plan::Rrt* rrt = dynamic_cast< rl::plan::Rrt* >(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 1:
				rrt->delta = value.value< ::rl::math::Real >();
				break;
			case 2:
				rrt->epsilon = value.value< ::rl::math::Real >();
				break;
			default:
				break;
			}
		}
		
		if (rl::plan::RrtGoalBias* rrtGoalBias = dynamic_cast< rl::plan::RrtGoalBias* >(MainWindow::instance()->planner.get()))
		{
			switch (index.row())
			{
			case 3:
				rrtGoalBias->probability = value.value< ::rl::math::Real >();
				break;
			default:
				break;
			}
		}
		
		emit dataChanged(index, index);
		
		return true;
	}
	
	return false;
}
