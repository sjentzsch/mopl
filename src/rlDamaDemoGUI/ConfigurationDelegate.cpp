/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include <QDoubleSpinBox>
#include <QModelIndex>

#include "ConfigurationDelegate.h"
#include "MainWindow.h"

ConfigurationDelegate::ConfigurationDelegate(QObject* parent) :
	QItemDelegate(parent)
{
}

ConfigurationDelegate::~ConfigurationDelegate()
{
}

QWidget*
ConfigurationDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	QDoubleSpinBox* editor = new QDoubleSpinBox(parent);
	
	rl::math::Vector maximum(MainWindow::instance()->model->getDof(0));
	MainWindow::instance()->model->getMaximum(maximum, 0);
	rl::math::Vector minimum(MainWindow::instance()->model->getDof(0));
	MainWindow::instance()->model->getMinimum(minimum, 0);
	Eigen::Matrix< rl::math::Unit, Eigen::Dynamic, 1 > qUnits(MainWindow::instance()->model->getDof(0));
	MainWindow::instance()->model->getPositionUnits(qUnits);
	
	if (rl::math::UNIT_RADIAN == qUnits(index.row()))
	{
		editor->setMinimum(minimum(index.row()) * rl::math::RAD2DEG);
		editor->setMaximum(maximum(index.row()) * rl::math::RAD2DEG);
		editor->setSingleStep(1.0f);
	}
	else
	{
		editor->setMinimum(minimum(index.row()));
		editor->setMaximum(maximum(index.row()));
		editor->setSingleStep(0.1f);
	}
	
	QObject::connect(editor, SIGNAL(valueChanged(double)), this, SLOT(valueChanged(double)));
	
	return editor;
}

void
ConfigurationDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
	QDoubleSpinBox* doubleSpinBox = static_cast< QDoubleSpinBox* >(editor);
	doubleSpinBox->setValue(index.model()->data(index, Qt::DisplayRole).toDouble());
}

void
ConfigurationDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
	QDoubleSpinBox* doubleSpinBox = static_cast< QDoubleSpinBox* >(editor);
	doubleSpinBox->interpretText();
	model->setData(index, doubleSpinBox->value(), Qt::EditRole);
}

void
ConfigurationDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	editor->setGeometry(option.rect);
}	

void
ConfigurationDelegate::valueChanged(double d)
{
	emit commitData(static_cast< QWidget* >(QObject::sender()));
}
