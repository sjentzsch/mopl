/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#ifndef _CONFIGURATIONMODEL_H_
#define _CONFIGURATIONMODEL_H_

#include <QAbstractTableModel>

class ConfigurationModel : public QAbstractTableModel
{
public:
	ConfigurationModel(QObject* parent = NULL);
	
	virtual ~ConfigurationModel();
	
	int columnCount(const QModelIndex& parent = QModelIndex()) const;
	
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
	
	Qt::ItemFlags flags(const QModelIndex &index) const;
	
	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;
	
	void invalidate();
	
	int rowCount(const QModelIndex& parent = QModelIndex()) const;
	
	bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole); 
	
protected:
	
private:
	
};

#endif // _CONFIGURATIONMODEL_H_
