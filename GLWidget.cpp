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
**     Peter Beben: heavily modified this file for the purpose of point
**     cloud visualization in this project.
**     Views ALL points in the point cloud without any pruning (so it
**     must be small enough to fit into video memory!!).
****************************************************************************/

#include "GLWidget.h"
#include "BoundBox.h"
#include "Cloud.h"
#include "CloudWorker.h"
#include "MessageLogger.h"
#include "constants.h"

#include <Eigen/Core>
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <QDebug>
#include <QThread>
#include <QRecursiveMutex>
#include <math.h>

//#define SHOW_CLOUD_DBUG


bool GLWidget::m_transparent = false;

//---------------------------------------------------------

GLWidget::GLWidget(QWidget *parent, MessageLogger* msgLogger)
	: QOpenGLWidget(parent),
	  m_msgLogger(msgLogger),
	  m_vRot(0),
	  m_program(0),
	  m_rotVect(0.0f,0.0f,0.0f),
	  m_movVect(0.0f,0.0f,0.0f),
	  m_pointSize(5.0f),
	  m_showCloudNorms(false),
	  m_aspectRatio(1.0f),
	  m_farPlane(100.0f), m_nearPlane(0.01f),
	  m_modelSize(1.0f),
	  m_cloudVbo(QOpenGLBuffer::VertexBuffer),
	  m_cloudBBoxVbo(QOpenGLBuffer::VertexBuffer),
	  m_cloudBBoxEbo(QOpenGLBuffer::IndexBuffer),
	  m_cloudNormsVbo(QOpenGLBuffer::VertexBuffer),
	  m_cloudDebugVbo(QOpenGLBuffer::VertexBuffer),
	  m_cloud(msgLogger, parent), m_cloudBBox(msgLogger)
{
	m_core = QSurfaceFormat::defaultFormat().profile() == QSurfaceFormat::CoreProfile;
	// --transparent causes the clear color to be transparent. Therefore, on systems that
	// support it, the widget will become transparent apart from the cloud.
	if (m_transparent) {
		QSurfaceFormat fmt = format();
		fmt.setAlphaBufferSize(8);
		setFormat(fmt);
	}

	m_cloudThread = new QThread(this);
	m_cloudWorker = new CloudWorker(m_cloud);
	m_cloudWorker->moveToThread(m_cloudThread);

	connect(this, &GLWidget::cloudApproxNorms,
			m_cloudWorker, &CloudWorker::approxCloudNorms);

	connect(this, &GLWidget::cloudDecimate,
			m_cloudWorker, &CloudWorker::decimateCloud);

	connect(this, &GLWidget::cloudSparsify,
			m_cloudWorker, &CloudWorker::sparsifyCloud);

	connect(this, &GLWidget::cloudReconstruct,
			m_cloudWorker, &CloudWorker::reconstructCloud);

	connect(m_cloudWorker, &CloudWorker::finished,
			this, &GLWidget::updateCloud);

	connect(m_cloudThread, &QThread::finished,
			m_cloudWorker, &QObject::deleteLater);

	m_cloudThread->start();

}
//---------------------------------------------------------

GLWidget::~GLWidget()
{
	m_cloudThread->quit();
	m_cloudThread->wait();
//	delete m_cloudThread;
	delete m_cloudWorker;
	cleanup();
}
//---------------------------------------------------------

QSize GLWidget::minimumSizeHint() const
{
	return QSize(50, 50);
}
//---------------------------------------------------------

QSize GLWidget::sizeHint() const
{
	return QSize(400, 400);
}
//---------------------------------------------------------

static void qNormalizeAngle(int &angle)
{
	while (angle < 0)
		angle += 360 * 16;
	while (angle > 360 * 16)
		angle -= 360 * 16;
}

//---------------------------------------------------------

void GLWidget::setVectRotation(int angle, QVector3D v)
{
	qNormalizeAngle(angle);
	m_vRot = angle;
	m_rotVect = v;
	m_movVect = QVector3D(0.0f,0.0f,0.0f);
	emit vectRotationChanged(angle, v);
	update();
}
//---------------------------------------------------------

void GLWidget::setVectTranslation(QVector3D v)
{
	m_vRot = 0.0f;
	m_rotVect = QVector3D(0.0f,0.0f,0.0f);
	m_movVect = v;
	emit vectTranslationChanged(v);
	update();
}
//---------------------------------------------------------

void GLWidget::cleanup()
{
	if (m_program == nullptr)
		return;
	makeCurrent();
	m_cloudVbo.destroy();
	m_cloudNormsVbo.destroy();
	m_cloudBBoxVbo.destroy();
	m_cloudDebugVbo.destroy();
	delete m_program;
	m_program = 0;
	doneCurrent();	
}
//---------------------------------------------------------

static const char *vertexShaderSourceCore =
		"#version 150\n"
		"in vec4 vertex;\n"
		"in vec3 normal;\n"
		"out vec3 vert;\n"
		"out vec3 vertNormal;\n"
		"uniform mat4 projMatrix;\n"
		"uniform mat4 mvMatrix;\n"
		"uniform mat3 normalMatrix;\n"
        "uniform float pointSize;\n"
        "void main() {\n"
		"   vert = vertex.xyz;\n"
		"   vertNormal = normalMatrix * normal;\n"
		"   gl_Position = projMatrix * mvMatrix * vertex;\n"
        "   gl_PointSize = pointSize/(0.1+10.0*abs(gl_Position.z));\n"
//		"   gl_PointSize = 50.0;\n"
		"}\n";

static const char *fragmentShaderSourceCore =
		"#version 150\n"
		"in highp vec3 vert;\n"
		"in highp vec3 vertNormal;\n"
		"out highp vec4 fragColor;\n"
		"uniform highp vec3 lightPos;\n"
		"uniform vec3 vertColor;\n"
		"void main() {\n"
		"   highp vec3 L = normalize(lightPos - vert);\n"
		"   highp float NL = max(abs(dot(normalize(vertNormal), L)), 0.0);\n"
		"   highp vec3 color = vertColor;\n"
		"   highp vec3 col = clamp(color * (0.3 + 0.8*NL), 0.0, 1.0);\n"
//		"   highp vec3 col = color;\n"
		"   fragColor = vec4(col, 1.0);\n"
		"}\n";

static const char *vertexShaderSource =
		"attribute vec4 vertex;\n"
		"attribute vec3 normal;\n"
		"varying vec3 vert;\n"
		"varying vec3 vertNormal;\n"
		"uniform mat4 projMatrix;\n"
		"uniform mat4 mvMatrix;\n"
		"uniform mat3 normalMatrix;\n"
		"uniform float pointSize;\n"
		"void main() {\n"
		"   vert = vertex.xyz;\n"
		"   vertNormal = normalMatrix * normal;\n"
		"   gl_Position = projMatrix * mvMatrix * vertex;\n"
		"   gl_PointSize = pointSize/(0.1+10.0*abs(gl_Position.z));\n"
//		"   gl_PointSize = 500.0;\n"
		"}\n";

static const char *fragmentShaderSource =
		"varying highp vec3 vert;\n"
		"varying highp vec3 vertNormal;\n"
		"uniform highp vec3 lightPos;\n"
		"uniform highp vec3 vertColor;\n"
		"void main() {\n"
		"   highp vec3 L = normalize(lightPos - vert);\n"
		"   highp float NL = max(abs(dot(normalize(vertNormal), L)), 0.0);\n"
		"   highp vec3 color = vertColor;\n"
		"   highp vec3 col = clamp(color * (0.3 + 0.8*NL), 0.0, 1.0);\n"
//		"   highp vec3 col = color;\n"
		"   gl_FragColor = vec4(col, 1.0);\n"
		"}\n";


//---------------------------------------------------------

void GLWidget::initializeGL()
{
	connect(context(), &QOpenGLContext::aboutToBeDestroyed,
			this, &GLWidget::cleanup);

	initializeOpenGLFunctions();
	glClearColor(0, 0, 0, m_transparent ? 0 : 1);

	m_program = new QOpenGLShaderProgram;
	m_program->addShaderFromSourceCode(
				QOpenGLShader::Vertex, m_core ?
					vertexShaderSourceCore : vertexShaderSource);
	m_program->addShaderFromSourceCode(
				QOpenGLShader::Fragment, m_core ?
					fragmentShaderSourceCore : fragmentShaderSource);
	m_program->bindAttributeLocation("vertex", 0);
	m_program->bindAttributeLocation("normal", 1);
	m_program->link();

	m_program->bind();
	m_projMatrixLoc = m_program->uniformLocation("projMatrix");
	m_mvMatrixLoc = m_program->uniformLocation("mvMatrix");
	m_normalMatrixLoc = m_program->uniformLocation("normalMatrix");
	m_lightPosLoc = m_program->uniformLocation("lightPos");
	m_colorLoc = m_program->uniformLocation("vertColor");
    m_pointSizeLoc = m_program->uniformLocation("pointSize");

	m_cloudVao.create();
	QOpenGLVertexArrayObject::Binder vaoBinderCloud(&m_cloudVao);
	setGLCloud();

	m_cloudBBoxVao.create();
	QOpenGLVertexArrayObject::Binder vaoBinderCloudBBox(&m_cloudBBoxVao);
	setGLBBox(m_cloudBBox, m_cloudBBoxVbo, m_cloudBBoxEbo);

	m_cloudNormsVao.create();
	QOpenGLVertexArrayObject::Binder vaoBinderCloudNorms(&m_cloudNormsVao);
	setGLCloudNorms(1.0f);

#ifdef SHOW_CLOUD_DBUG
	m_cloudDebugVao.create();
	QOpenGLVertexArrayObject::Binder vaoBinderCloudDebug(&m_cloudDebugVao);
	setGLCloudDebug();
#endif

	setGLView();

	m_program->setUniformValue(m_lightPosLoc, QVector3D(0, 0, 2*m_modelSize));

	m_program->release();

}

//---------------------------------------------------------
void GLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_PROGRAM_POINT_SIZE);

	m_world.rotate(-m_vRot / 8.0f, m_rotVect);
	m_camera.translate(-m_movVect / 2000.0f);

	m_program->bind();
	m_program->setUniformValue(m_projMatrixLoc, m_proj);
	m_program->setUniformValue(m_mvMatrixLoc, m_camera * m_world.transposed());
	QMatrix3x3 normalMatrix = m_world.normalMatrix();
	m_program->setUniformValue(m_normalMatrixLoc, normalMatrix);
    m_program->setUniformValue(m_pointSizeLoc, m_pointSize);
	m_program->setUniformValue(m_lightPosLoc, QVector3D(0, 0, 2*m_modelSize));

	size_t npoints = m_cloud.pointCount();
	size_t npointsOrig = m_cloud.pointCountOrig();
	size_t npointsNew = npoints - npointsOrig;

	m_program->setUniformValue(m_colorLoc, QVector3D(0.4f, 1.0f, 0.0f));
	QOpenGLVertexArrayObject::Binder vaoBinder(&m_cloudVao);
	glDrawArrays(GL_POINTS, 0, npointsOrig);

	if( npointsNew > 0 ){
		m_program->setUniformValue(m_colorLoc, QVector3D(1.0f, 0.2f, 1.0f));
		glDrawArrays(GL_POINTS, npointsOrig-1, npointsNew);
	}

	m_program->setUniformValue(m_colorLoc, QVector3D(1.0f, 0.5f, 0.0f));
	QOpenGLVertexArrayObject::Binder vaoBinder2(&m_cloudBBoxVao);
	int idxCount = (m_cloudBBox.vertCount()==0) ? 0 : 24;
	glDrawElements(GL_LINES, idxCount, GL_UNSIGNED_INT, 0);
//	glDrawElements(GL_LINES, idxCount, GL_UNSIGNED_INT, m_cloudBBox.elemGLData());

	if( m_showCloudNorms ){
		m_program->setUniformValue(m_colorLoc, QVector3D(1.0f, 0.0f, 1.0f));
		QOpenGLVertexArrayObject::Binder vaoBinder3(&m_cloudNormsVao);
		glDrawArrays(GL_LINES, 0, 2*npoints);
	}

#ifdef SHOW_CLOUD_DBUG
	m_program->setUniformValue(m_colorLoc, QVector3D(0.0f, 0.0f, 0.5f));
	QOpenGLVertexArrayObject::Binder vaoBinder4(&m_cloudDebugVao);
	glDrawArrays(GL_LINES, 0, 2*m_cloud.debugCount());
#endif

	m_program->release();
}
//---------------------------------------------------------

void GLWidget::resizeGL(int w, int h)
{
	m_aspectRatio = float(w)/h;
	m_proj.setToIdentity();
	m_proj.perspective(45.0f, m_aspectRatio, m_nearPlane, m_farPlane);
}

//---------------------------------------------------------

void GLWidget::setGLView()
{
	m_rotVect = QVector3D(0.0f,0.0f,0.0f);
	m_movVect = QVector3D(0.0f,0.0f,0.0f);

	m_nearPlane = 1e-5*m_modelSize;
	m_farPlane = 100.0f*m_modelSize;
	m_proj.setToIdentity();
	m_proj.perspective(45.0f, m_aspectRatio, m_nearPlane, m_farPlane);

	m_world.setToIdentity();
	m_camera.setToIdentity();
	m_camera.translate(0, 0, -m_modelSize);
}
//---------------------------------------------------------

void GLWidget::setGLCloud()
{
	size_t npoints = m_cloud.pointCount();	

	makeCurrent();

	// Setup our vertex buffer object for point cloud.
	m_cloudVbo.create();
	m_cloudVbo.bind();
	m_cloudVbo.allocate(
				m_cloud.vertGLData(), 6*npoints*sizeof(GLfloat));
	// Store the vertex attribute bindings for the program.
	setupVertexAttribs(m_cloudVbo);

}

//---------------------------------------------------------

void GLWidget::setGLCloudNorms(float scale)
{
	size_t npoints = m_cloud.pointCount();

	makeCurrent();

	m_cloudNormsVbo.create();
	m_cloudNormsVbo.bind();
	m_cloudNormsVbo.allocate(
				m_cloud.normGLData(scale),
				12*npoints*sizeof(GLfloat));
	setupVertexAttribs(m_cloudNormsVbo);

}

//---------------------------------------------------------

void GLWidget::setGLCloudDebug()
{
	size_t ndebug = m_cloud.debugCount();

	makeCurrent();

	m_cloudDebugVbo.create();
	m_cloudDebugVbo.bind();
	m_cloudDebugVbo.allocate(
				m_cloud.debugGLData(),
				12*ndebug*sizeof(GLfloat));
	setupVertexAttribs(m_cloudDebugVbo);

}

//---------------------------------------------------------

void GLWidget::setGLBBox(
		BoundBox bBox, QOpenGLBuffer vbo, QOpenGLBuffer ebo)
{
	makeCurrent();

	vbo.create();
	vbo.bind();
	vbo.allocate(bBox.vertGLData(), 6*bBox.vertCount()*sizeof(GLfloat));
	ebo.create();
	ebo.bind();
	int idxCount = (bBox.vertCount()==0) ? 0 : 24;
	ebo.allocate(bBox.elemGLData(), idxCount*sizeof(GLuint));
	setupVertexAttribs(vbo);

}

//---------------------------------------------------------

void GLWidget::setupVertexAttribs(QOpenGLBuffer vbo)
{
	vbo.bind();
	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	f->glEnableVertexAttribArray(0);
	f->glEnableVertexAttribArray(1);
	f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), 0);
	f->glVertexAttribPointer(
				1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat),
				reinterpret_cast<void *>(3 * sizeof(GLfloat)));
	vbo.release();

}

//---------------------------------------------------------

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	m_lastMousePos = event->pos();
	//***
//	if(event->buttons() & Qt::MidButton){
//		setRandomCloud(35000);
//	}

}
//---------------------------------------------------------

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - m_lastMousePos.x();
	int dy = event->y() - m_lastMousePos.y();

	bool leftButton = (event->buttons() & Qt::LeftButton);
	bool rightButton = (event->buttons() & Qt::RightButton);

	if ( leftButton && rightButton ) {
		float norm = frobeniusNorm4x4(m_camera);
		setVectTranslation(QVector3D(-norm*dx,norm*dy,0.0f));
	}
	else if ( leftButton ) {
		int angle = abs(dx) + abs(dy);
		setVectRotation(angle, QVector3D(dy,dx,0.0f));
	}
	else if ( rightButton ) {
		int angle = abs(dx) + abs(dy);
		setVectRotation(angle, QVector3D(dy,0.0f,-dx));
	}

	m_lastMousePos = event->pos();
}
//---------------------------------------------------------

void GLWidget::wheelEvent(QWheelEvent *event)
{
	float norm = frobeniusNorm4x4(m_camera) + 1e-3;
	setVectTranslation(QVector3D(0.0f,0.0f,norm*event->delta()));
}
//---------------------------------------------------------

void GLWidget::setCloud(CloudPtr cloud)
{
	QMutexLocker locker(&m_recMutex);

	m_cloud.fromPCL(cloud);

	updateCloud(true);
	setGLView();
}


//---------------------------------------------------------

void GLWidget::setRandomCloud(size_t nPoints)
{
	QMutexLocker locker(&m_recMutex);

	Eigen::VectorXf C = Eigen::VectorXf::Random(5);
	Eigen::VectorXf D = Eigen::VectorXf::Random(8);
	Eigen::VectorXf E = Eigen::VectorXf::Random(5);
	Eigen::VectorXf F = Eigen::VectorXf::Random(8);

	auto heightFun = [&C, &D, &E, &F](float xu, float xv){
		float pu = 15*xu, pv = 15*xv;
		float height =
				C(0)*cos(D(0)*pu) +
				C(1)*cos(D(1)*pv) +
				C(2)*cos(D(2)*pu)*cos(D(3)*pu) +
				C(3)*cos(D(4)*pu)*cos(D(5)*pv) +
				C(4)*cos(D(6)*pv)*cos(D(7)*pv) +
				E(0)*sin(F(0)*pu) +
				E(1)*sin(F(1)*pv) +
				E(2)*sin(F(2)*pu)*sin(F(3)*pu) +
				E(3)*sin(F(4)*pu)*sin(F(5)*pv) +
				E(4)*sin(F(6)*pv)*sin(F(7)*pv);
		return 0.05f*height;
	};

	Eigen::Vector3f norm(0.0f,1.0f,0.0f);
	m_cloud.fromRandomPlanePoints(norm, nPoints, heightFun);

	updateCloud(true);

	setGLView();
}


//---------------------------------------------------------

void GLWidget::getCloud(CloudPtr& cloud)
{
	QMutexLocker locker(&m_recMutex);

	m_cloud.toPCL(cloud);
}

//---------------------------------------------------------

void GLWidget::undoCloud()
{
	QMutexLocker locker(&m_recMutex);

	m_cloud.restore();

	updateCloud(false);
}


//---------------------------------------------------------

void GLWidget::setCloudBBox(float minBBox[3], float maxBBox[3])
{
    QMutexLocker locker(&m_recMutex);

    m_cloudBBox.set(minBBox, maxBBox);
	m_cloudBBox.logMessageBBox();
	emit bBoxFieldsChanged(minBBox, maxBBox);
	m_cloud.setBoundBox(&m_cloudBBox);
    setGLBBox(m_cloudBBox, m_cloudBBoxVbo, m_cloudBBoxEbo);
    update();

}

//---------------------------------------------------------

void GLWidget::viewGLCloudNorms(bool enabled)
{
	m_showCloudNorms = enabled;
	update();
}

//---------------------------------------------------------

void GLWidget::updateCloud(bool updateBBox)
{
	QMutexLocker locker(&m_recMutex);

	if( updateBBox ){
		float minBBox[3], maxBBox[3];
		m_cloudBBox.set(m_cloud);
		m_cloudBBox.rescale(0.01f);
		m_cloudBBox.logMessageBBox();
		m_cloudBBox.getExtents(minBBox, maxBBox);
		emit bBoxFieldsChanged(minBBox, maxBBox);
		m_cloud.setBoundBox(&m_cloudBBox);
		m_modelSize = m_cloudBBox.diagonalSize();
		setGLBBox(m_cloudBBox, m_cloudBBoxVbo, m_cloudBBoxEbo);
	}

	setGLCloud();
	setGLCloudNorms(1e-2*m_modelSize);

#ifdef SHOW_CLOUD_DBUG
	setGLCloudDebug();
#endif

	update();
}

//---------------------------------------------------------

float GLWidget::frobeniusNorm4x4(QMatrix4x4 M)
{
	float normSq =
			M(0,0)*M(0,0) +
			M(1,0)*M(1,0) +
			M(2,0)*M(2,0) +
			M(3,0)*M(3,0) +
			M(0,1)*M(0,1) +
			M(1,1)*M(1,1) +
			M(2,1)*M(2,1) +
			M(3,1)*M(3,1) +
			M(0,2)*M(0,2) +
			M(1,2)*M(1,2) +
			M(2,2)*M(2,2) +
			M(3,2)*M(3,2) +
			M(0,3)*M(0,3) +
			M(1,3)*M(1,3) +
			M(2,3)*M(2,3) +
			M(3,3)*M(3,3);
	return sqrt(normSq);
}
//---------------------------------------------------------
