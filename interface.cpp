#include <QtGui>

#include "controllooper.h"
#include "globals.h"
#include "highercontroller.h"
#include "imu.h"
#include "interface.h"
#include "observer.h"
#include "propellers.h"
#include "quad.h"
#include "sensors.h"

#include <cmath>
#include <iomanip>
#include <iostream>
using namespace std;

Interface::Interface()
{
  is_running_ = false;
  control_automatic_ = true;
  setupUi(this);
  
  m_propInputs[0] = prop1;
  m_propInputs[1] = prop2;
  m_propInputs[2] = prop3;
  m_propInputs[3] = prop4;
  
  for ( int i = 0; i < 4; ++i ) {
    connect( m_propInputs[i], SIGNAL(valueChanged(int)), this, SLOT(propInputsChanged()) );
  }
  
  connect(runPauseButton, SIGNAL(clicked()), this, SLOT(runPauseQuad()));
  connect(resetButton, SIGNAL(clicked()), this, SLOT(resetQuad()));
  connect(controlAutomatic, SIGNAL(toggled(bool)), this, SLOT(setControlAutomatic(bool)));

  QTimer *timer = new QTimer(this);
  connect( timer, SIGNAL(timeout()), this, SLOT(updateLabels()) );
  timer->start( 50 );
  
  runPauseQuad();
  
  updateLabels();
}


Interface::~Interface()
{
   delete Globals::self().controlLooper();
}

double Interface::propInput( int i ) const
{
   return double(m_propInputs[i]->value()) / double(m_propInputs[i]->maximum());
}

void Interface::propInputsChanged()
{
  Propellers *p = Globals::self().propellers();
  for ( int i = 0; i < 4; ++i ) {
    p->setInput(i, propInput(i));
  }
}


QString fixed( double x, int prec = 2 )
{
   return QString::number( x, 'f', prec );
}

QString toCoords( const VectorXd & v, int prec = 2 )
{
   QString c("(");
   for ( int i = 0; i < v.size(); ++i )
   {
      if ( i > 0 )
         c += ",";
      c += fixed( v(i), prec );
   }
   c += ")";
   return c;
}

VectorXd qCoeffs_wxyz( const Quaterniond & q )
{
   VectorXd v(4);
   v << q.w(),q.x(),q.y(),q.z();
   return v;
}

void Interface::updateLabels()
{
   Quad *quad = Globals::self().simulatedQuad();
   
  QuadState state = quad->state();

  positionLabel->setText( toCoords( state.pos ) );
  velocityLabel->setText( toCoords( state.vel ) );
//   avLabel->setText( toCoords( state.omega ) );
//   orientationLabel->setText( toCoords( qCoeffs_wxyz(state.orient) ) );
//   windLabel->setText( toCoords( World::self()->wind() ) );
   
//    Observer *o = Globals::self().controlLooper()->observer();
//    QuadState pred = o->state();
   
//    ArrayXd sd = o->cov().diagonal().array();
   
//    predPositionLabel->setText( toCoords( pred.pos ) + "\n+-" + toCoords(sd.segment(0,3).sqrt()) );
//    predVelocityLabel->setText( toCoords( pred.vel ) + "\n+-" + toCoords(sd.segment(3,3).sqrt()) );
//    predavLabel->setText( toCoords( pred.omega ) );
//    predOrientationLabel->setText( toCoords( qCoeffs_wxyz(pred.orient) ) );
   
   
//    timeLabel->setText( fixed( World::self()->time() ) );
//    OmegaLabel->setText( toCoords( 100*World::self()->simulatedQuad()->propInput(), 1 ) );
   
//    Sensors *s = Globals::self().controlLooper()->sensors();

  Vector3d accel = Globals::self().imu()->lastAcceleration();
  Vector3d gyro = Globals::self().imu()->lastAngularAcceleration();
   
//   accelerometersLabel->setText(toCoords(accel) + "\n+-" + toCoords(s->var_ba().array().sqrt(),4) );
//   gyroLabel->setText( toCoords(gyro, 3) + "\n+-" + toCoords(s->var_bg().array().sqrt(),4) );
  accelerometersLabel->setText(toCoords(accel));
  gyroLabel->setText(toCoords(gyro, 3));
//    gpsLabel->setText( toCoords( s->readGPS() ) + "\n+-" + toCoords(s->var_bgps().array().sqrt(),1) );
   
   // Update plot 
//    double duration = quad->path_ != nullptr ? quad->path_->duration() : 0;
//    interceptPlot->setIntercept(quad->intercept, duration);

  runPauseButton->setText(is_running_ ? "Pause" : "Start");
   
  Vector4d propInput = Globals::self().propellers()->input();
      
  // Update slider values
  for ( int i = 0; i < 4; ++i ) {
    double p = propInput[i];
    int v = int(p * m_propInputs[i]->maximum());
    m_propInputs[i]->setValue( v );
  }
}

void Interface::runPauseQuad()
{
  is_running_ = !is_running_;
  Globals::self().controlLooper()->setRunning(control_automatic_ && is_running_);
  Globals::self().setSimulatedQuadRunning(is_running_);
//    updateLabels();
   
//    MatrixXd cov = World::self()->observer()->cov();
//    for ( int i = 0; i < cov.rows(); ++i )
//    {
//       for ( int j = 0; j < cov.cols(); ++j )
//       {
//          if ( abs( cov(i,j) ) < 1e-9 )
//             cov(i,j) = 0;
//       }
//    }
//   cout << "Covariance matrix of observed state:" << endl;
//   cout << "C=[";
//   for ( int i = 0; i < cov.rows(); ++i )
//   {
//      if ( i > 0 )
//         cout << ";";
// //      cout << "[";
//      for ( int j=0; j < cov.cols(); ++j )
//      {
//         if ( j > 0 )
//            cout << " ";
//         cout << setiosflags(ios::fixed) << setprecision(8) << cov(i,j);
//      }
// //      cout << "]";
//   }
//   cout << "]" << endl;

//   cout << cov << endl;
}

void Interface::resetQuad()
{
  Globals::self().reset();
//    propInputsChanged();
//    updateLabels();
}

void Interface::setControlAutomatic(bool automatic) {
  control_automatic_ = automatic;
  Globals::self().controlLooper()->setRunning(control_automatic_ && is_running_);
}
