#include "glquad.h"

#include "controllooper.h"
#include "globals.h"
// #include "observer.h"
#include "path.h"
#include "quad.h"
#include "imu.h"
#include "propellers.h"

#include <QtGui>
#include <QtOpenGL>

#include <math.h>

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

int texture[1];

GLQuad::GLQuad(QWidget *parent)
   : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
   QTimer *timer = new QTimer(this);
   connect( timer, SIGNAL(timeout()), this, SLOT(updateGL()) );
   timer->start( 50 );
}

GLQuad::~GLQuad()
{
}


QSize GLQuad::minimumSizeHint() const
{
   return QSize(50, 50);
}
QSize GLQuad::sizeHint() const
{
   return QSize(400, 400);
}


void GLQuad::initializeGL()
{
   glEnable( GL_TEXTURE_2D );
   glShadeModel(GL_SMOOTH);
   glClearColor(1,1,1,0);
   glClearDepth(1.0f);
   glEnable( GL_DEPTH_TEST );
   
   glDepthFunc(GL_LEQUAL);
   glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
   
   
   float lightDiffuse[] = {1,1,1,0.5};
   float lightAmbient[] = {0.5,0.5,0.5,0.1};
   //Ambient light component
   glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
   //Diffuse light component
   glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
   
   glEnable( GL_LIGHTING );
   glEnable( GL_LIGHT0 );
   glEnable(GL_COLOR_MATERIAL);
}


void GLQuad::setupView()
{
   glLoadIdentity();
   
   glTranslated( -12, -12, -80 );
   
   // Ensures that the z axis is pointing upwars, y to right and x coming out of screen
   glRotatef( -120, 1, 1, 1 );
}

void GLQuad::drawAxes()
{
   int l = 20;
   int s = 2;
   
   glBegin(GL_LINES);
   
   // x axis
   glColor3f(1,0,0);
   glVertex3f(0,0,0);
   glVertex3f(l,0,0);
   for ( int i = 1; i < l; i+=s )
   {
      glVertex3f( i, -0.5, 0 );
      glVertex3f( i, +0.5, 0 );
   }
   
   // y axis
   glColor3f(0,1,0);
   glVertex3f(0,0,0);
   glVertex3f(0,l,0);
   for ( int i = 1; i < l; i+=s )
   {
      glVertex3f( -0.5, i, 0 );
      glVertex3f( +0.5, i, 0 );
   }
   
   // z axis
   glColor3f(0,0,1);
   glVertex3f(0,0,0);
   glVertex3f(0,0,l);
   for ( int i = 1; i < l; i+=s )
   {
      glVertex3f( 0, -0.5, i );
      glVertex3f( 0, +0.5, i );
   }
   
   glEnd();
   
   glColor3f(0,0,0);
   renderText( l, 0, 0, "x" );
   renderText( 0, l, 0, "y" );
   renderText( 0, 0, l, "z" );
}

void GLQuad::drawIntercept() {
  shared_ptr<Path> intercept = Globals::self().simulatedQuad()->path_;
  if (intercept == nullptr) {
    return;
  }
  glBegin(GL_LINES);
  int count = 50;
  double duration = intercept->duration();
  Vector3d prev;
  for (int i = 0; i < count; ++i) {
    double t = i * duration / count;
    Vector3d next = intercept->position(t);
    if (i > 0) {
      glVertex3f(prev.x(), prev.y(), prev.z());
      glVertex3f(next.x(), next.y(), next.z());
    }
    prev = next;
    if (i == 0) {
      // Draw the acceleration arrow
      Vector3d tip = prev + 4 * intercept->initialAccelerationDirection();
      glColor3f(1,0,0);
      glVertex3f(prev.x(), prev.y(), prev.z());
      glVertex3f(tip.x(), tip.y(), tip.z());
      glColor3f(0,0,0);
    }
//     if (i == count-1) {
//       // Draw the final 
  }
  glEnd();
}

void GLQuad::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   
  setupView();
  drawAxes();
  drawIntercept();

  glColor3f(0.8,0.5,0.5);
  Quad *quad = Globals::self().simulatedQuad();
  drawQuad(quad->state(), quad->propInput());
  glColor3f(0.5,0.5,0.8);
  drawQuad(Globals::self().imu()->lastState(), Globals::self().propellers()->input());
  glColor3f(0.5,0.5,0.5);
  quad = Globals::self().remoteQuad();
  drawQuad(quad->state(), quad->propInput());
}

void GLQuad::drawQuad(QuadState const& state, Vector4d const& prop_input)
{
   glPushMatrix();
   
   glTranslated( state.pos.x(), state.pos.y(), state.pos.z() );
   
   GLdouble matrix[16];
   createMatrix( state.orient.conjugate(), matrix );
   //    createMatrix( state.orient, matrix );
   glMultMatrixd( matrix );
   
   
//    glRotatef
   
//    glRotatef( -45, 0, 1, 0 );
//    glRotatef( 30, 1, 0, 0 );
   
//    /5--6
//    1--2|
//    |7-|8
//    3--4/

   
   double scale = 0.5;
   
   drawCube(-1*scale,1*scale,-1*scale,1*scale,-0.5*scale,0.5*scale);
   
   drawCube(-3*scale,-1*scale,-0.5*scale,0.5*scale,-0.5*scale,0.5*scale);
   drawCube(1*scale,3*scale,-0.5*scale,0.5*scale,-0.5*scale,0.5*scale);
   
   drawCube(-0.5*scale,0.5*scale,-3*scale,-1*scale,-0.5*scale,0.5*scale);
   drawCube(-0.5*scale,0.5*scale,1*scale,3*scale,-0.5*scale,0.5*scale);
   
   double s = 4 *scale;
   
   glTranslated( 2.5*scale, 0, 0 );
   drawArrow( -prop_input[0]*s );
//    renderText( 1.2*scale, 0, 0, "1" );
   
   glTranslated( -2.5*scale, 2.5*scale, 0 );
   drawArrow( -prop_input[1]*s );
//    renderText( 0, 1.2*scale, 0, "2" );
   
   glTranslated( -2.5*scale, -2.5*scale, 0 );
   drawArrow( -prop_input[2]*s );
//    renderText( -1.2*scale, 0, 0, "3" );
   
   glTranslated( 2.5*scale, -2.5*scale, 0 );
   drawArrow( -prop_input[3]*s );
//    renderText( 0, -1.2*scale, 0, "4" );
   
   glPopMatrix();
}

void GLQuad::drawArrow( double length )
{
   glColor3f( 0,0,0 );
   glBegin( GL_LINES );
   glVertex3f( 0,0,0 );
   glVertex3f( 0,0,length );
   glEnd();
}

void GLQuad::drawCube( GLdouble x1, GLdouble x2, GLdouble y1, GLdouble y2, GLdouble z1, GLdouble z2 )
{
   glBegin(GL_QUADS);
   
   glNormal3f( -1, 0, 0 );
   glVertex3f( x1, y1, z1 );
   glVertex3f( x1, y1, z2 );
   glVertex3f( x1, y2, z2 );
   glVertex3f( x1, y2, z1 );
   
   glNormal3f( 1, 0, 0 );
   glVertex3f( x2, y1, z1 );
   glVertex3f( x2, y1, z2 );
   glVertex3f( x2, y2, z2 );
   glVertex3f( x2, y2, z1 );
   
   glNormal3f( 0, -1, 0 );
   glVertex3f( x1, y1, z1 );
   glVertex3f( x1, y1, z2 );
   glVertex3f( x2, y1, z2 );
   glVertex3f( x2, y1, z1 );
   
   glNormal3f( 0, 1, 0 );
   glVertex3f( x1, y2, z1 );
   glVertex3f( x1, y2, z2 );
   glVertex3f( x2, y2, z2 );
   glVertex3f( x2, y2, z1 );
   
   glNormal3f( 0, 0, -1 );
   glVertex3f( x1, y1, z1 );
   glVertex3f( x1, y2, z1 );
   glVertex3f( x2, y2, z1 );
   glVertex3f( x2, y1, z1 );
   
   glNormal3f( 0, 0, 1 );
   glVertex3f( x1, y1, z2 );
   glVertex3f( x1, y2, z2 );
   glVertex3f( x2, y2, z2 );
   glVertex3f( x2, y1, z2 );
   
   
   glEnd();
}

void
gluPerspective(GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar)
{
   GLdouble xmin, xmax, ymin, ymax;
   
   ymax = zNear * tan(fovy * M_PI / 360.0);
   ymin = -ymax;
   xmin = ymin * aspect;
   xmax = ymax * aspect;
   
   
   glFrustum(xmin, xmax, ymin, ymax, zNear, zFar);
}

void GLQuad::resizeGL(int width, int height)
{
   if ( height == 0 )
      height = 1;
   glViewport(0,0,width,height);
   
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(45.0f,(GLdouble)width/(GLdouble)height,0.1f,100.0f);
   glMatrixMode(GL_MODELVIEW);						// Select The Modelview Matrix
   glLoadIdentity();							// Reset The Modelview Matrix
}


void GLQuad::createMatrix( Quaterniond q, GLdouble *m )
{
   Matrix3d m3 = q.toRotationMatrix();
   
   m[0] = m3(0,0);
   m[1] = m3(1,0);
   m[2] = m3(2,0);
   m[3] = 0;
   
   m[4] = m3(0,1);
   m[5] = m3(1,1);
   m[6] = m3(2,1);
   m[7] = 0;
   
   m[8] = m3(0,2);
   m[9] = m3(1,2);
   m[10] = m3(2,2);
   m[11] = 0;
   
   m[12] = 0;
   m[13] = 0;
   m[14] = 0;
   m[15] = 1;
}
