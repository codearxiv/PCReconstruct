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
/****************************************************************************
**     Peter Beben: modified this file for the purpose of point cloud
**     visualization.
**     Views ALL points in the point cloud without any pruning (so it
**     must be small enough to fit into video memory!!).
****************************************************************************/

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "BoundBox.h"
#include "Cloud.h"
#include "CloudWorker.h"
//#include "MessageLogger.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <QRecursiveMutex>

class MessageLogger;
class CloudWorker;

QT_BEGIN_NAMESPACE
class QThread;
QT_END_NAMESPACE

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

public:
	GLWidget(QWidget *parent = nullptr, MessageLogger* msgLogger = nullptr);
	~GLWidget();

	static bool isTransparent() { return m_transparent; }
	static void setTransparent(bool t) { m_transparent = t; }

	QSize minimumSizeHint() const override;
	QSize sizeHint() const override;

public slots:
	void setVectRotation(int angle, QVector3D v);
	void setVectTranslation(QVector3D v);
	void cleanup();
	void setCloud(CloudPtr cloud);
	void getCloud(CloudPtr& cloud);
	void setRandomCloud(size_t nPoints);
    void setCloudBBox(float minBBox[3], float maxBBox[3]);
	void decimateCloud(size_t nHoles, size_t kNN)
	{ emit cloudDecimate(nHoles,kNN); }
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
    void setPointSize(float size) { m_pointSize = size; }
    void updateCloud();

signals:
	void vectRotationChanged(int angle, QVector3D v);
	void vectTranslationChanged(QVector3D v);
	//void cloudSetRandom(size_t nPoints);
	void cloudDecimate(size_t nHoles, size_t kNN);
	void cloudSparsify(float percent);
	void cloudReconstruct(
            int kSVDIters, size_t kNN, size_t nfreq, float densify,
			size_t natm, size_t latm, size_t maxNewPoints,
			SparseApprox method);
	void logMessage(const QString& text);

protected:
	void initializeGL() override;
	void paintGL() override;
	void resizeGL(int width, int height) override;
	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;

private:
	void setupVertexAttribs(QOpenGLBuffer vbo);
	void setGLCloud();
	void setGLCloudNorms(float scale);
	void setGLCloudDebug();
	void setGLBBox(
			BoundBox bBox, QOpenGLBuffer vbo, QOpenGLBuffer ebo);

	bool m_core;
	int m_vRot;
	QPoint m_lastMousePos;
	//QPoint m_lastWheelPos;
	Cloud m_cloud;
	BoundBox m_cloudBBox;
	QOpenGLVertexArrayObject m_cloudVao;
	QOpenGLVertexArrayObject m_cloudBBoxVao;
	QOpenGLVertexArrayObject m_cloudNormsVao;
	QOpenGLVertexArrayObject m_cloudDebugVao;
	QOpenGLBuffer m_cloudVbo;
	QOpenGLBuffer m_cloudBBoxVbo;
	QOpenGLBuffer m_cloudBBoxEbo;
	QOpenGLBuffer m_cloudNormsVbo;
	QOpenGLBuffer m_cloudDebugVbo;
	QOpenGLShaderProgram *m_program;
	int m_projMatrixLoc;
	int m_mvMatrixLoc;
	int m_normalMatrixLoc;
	int m_lightPosLoc;
	int m_colorLoc;
    int m_pointSizeLoc;
    float m_pointSize;
	QMatrix4x4 m_proj;
	QMatrix4x4 m_camera;
	QMatrix4x4 m_world;
	QVector3D m_rotVect;
	QVector3D m_movVect;
	static bool m_transparent;
	MessageLogger* m_msgLogger;
	QThread* m_cloudThread;
	CloudWorker* m_cloudWorker;
	QRecursiveMutex m_recMutex;

};

#endif
