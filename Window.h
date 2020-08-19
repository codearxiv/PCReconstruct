/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef WINDOW_H
#define WINDOW_H

#include "MessageLogger.h"
#include "constants.h"

//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <QWidget>

QT_BEGIN_NAMESPACE
class QSlider;
class QPushButton;
class QMainWindow;
QT_END_NAMESPACE

class GLWidget;


class Window : public QWidget
{
    Q_OBJECT

	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

public:
	Window(QMainWindow *mw, MessageLogger* msgLogger = nullptr);

public slots:
	void setCloud(CloudPtr cloud)
	{ emit cloudChanged(cloud); }

	void getCloud(CloudPtr& cloud)
	{ emit cloudQueried(cloud); }

	void setRandomCloud(size_t nPoints)
	{ emit cloudSetRandom(nPoints); }

	void decimateCloud(size_t nHoles, size_t kNN)
	{ emit cloudDecimate(nHoles, kNN); }

	void sparsifyCloud(float percent)
	{ emit cloudSparsify(percent); }

	void reconstructCloud(
            int kSVDIters, size_t kNN, size_t nfreq, float densify,
			size_t natm, size_t latm, size_t maxNewPoints,
			SparseApprox method)
	{
		emit cloudReconstruct(
                    kSVDIters, kNN, nfreq, densify, natm, latm,
					maxNewPoints, method);
	}

    void setPointSize(float size)
    { emit pointSizeChanged(size); }


signals:
	void cloudChanged(CloudPtr cloud);
	void cloudQueried(CloudPtr& cloud);
	void cloudSetRandom(size_t nPoints);
	void cloudDecimate(size_t nHoles, size_t kNN);
	void cloudSparsify(float percent);
	void cloudReconstruct(
            int kSVDIters, size_t kNN, size_t nfreq, float densify,
			size_t natm, size_t latm, size_t maxNewPoints,
			SparseApprox method);
    void pointSizeChanged(float size);

protected:
    void keyPressEvent(QKeyEvent *event) override;

private:
    QSlider *createSlider();

    GLWidget *glWidget;
	QMainWindow *mainWindow;
	MessageLogger* m_msgLogger;

};

#endif
