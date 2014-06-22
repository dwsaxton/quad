#ifndef GLQUAD_H
#define GLQUAD_H

#include <QGLWidget>

#include <Eigen/Geometry>
using namespace Eigen;

class QuadState;

class GLQuad : public QGLWidget
{
   Q_OBJECT
   
   public:
      GLQuad(QWidget *parent = 0);
      ~GLQuad();
      
      QSize minimumSizeHint() const;
      QSize sizeHint() const;
      
      void update();
      
   protected:
      void initializeGL();
      void paintGL();
      void resizeGL(int width, int height);
      
   private:
      void setupView();
      void drawAxes();
      void drawQuad( const QuadState & state );
      void createMatrix( Quaterniond q, GLdouble *m );
      void drawCube( GLdouble x1, GLdouble y1, GLdouble z1, GLdouble x2, GLdouble y2, GLdouble z2 );
      void drawArrow( double length );
};

#endif
