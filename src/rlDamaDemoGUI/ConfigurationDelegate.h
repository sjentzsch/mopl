/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#ifndef _CONFIGURATIONDELEGATE_H_
#define _CONFIGURATIONDELEGATE_H_

#include <QItemDelegate>

class ConfigurationDelegate : public QItemDelegate
{
	Q_OBJECT
	
public:
	ConfigurationDelegate(QObject* parent = NULL);
	
	virtual ~ConfigurationDelegate();
	
	QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const;
    
	void setEditorData(QWidget* editor, const QModelIndex& index) const;
    
	void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const;
    
	void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const;	
	
public slots:
	void valueChanged(double d);
	
protected:
	
private:
	
};

#endif // _CONFIGURATIONDELEGATE_H_
