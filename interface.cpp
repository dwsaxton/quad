#include <QtGui>

#include "controller.h"
#include "interface.h"
#include "observer.h"
#include "quad.h"
#include "sensors.h"
#include "world.h"

#include <cmath>
#include <iomanip>
#include <iostream>
using namespace std;

Interface::Interface()
{
   new World;
   
   setupUi(this);
   
   m_propInputs[0] = prop1;
   m_propInputs[1] = prop2;
   m_propInputs[2] = prop3;
   m_propInputs[3] = prop4;
   
   for ( int i = 0; i < 4; ++i )
      connect( m_propInputs[i], SIGNAL(valueChanged(int)), this, SLOT(propInputsChanged()) );
   
   connect( runPauseButton, SIGNAL(clicked()), this, SLOT(runPauseQuad()) );
   connect( resetButton, SIGNAL(clicked()), this, SLOT(resetQuad()) );
   connect( controlAutomatic, SIGNAL(toggled(bool)), World::self()->controller(), SLOT(giveControl(bool)) );
   
   connect( environmentSimulation, SIGNAL(clicked()), this, SLOT(setEnvironmentSimulation()) );
   connect( environmentActual, SIGNAL(clicked()), this, SLOT(setEnvironmentActual()) );
   
   World::self()->controller()->giveControl( controlAutomatic->isChecked() );
   
   QTimer *timer = new QTimer(this);
   connect( timer, SIGNAL(timeout()), this, SLOT(updateLabels()) );
   timer->start( 50 );
   
   if ( environmentSimulation->isChecked() )
      setEnvironmentSimulation();
   else
      setEnvironmentActual();
   
   runPauseQuad();
   
   updateLabels();
}


Interface::~Interface()
{
   delete World::self();
}

void Interface::setEnvironmentSimulation()
{
   World::self()->setEnvironment( World::Simulation );
   
   positionLabel->setVisible( true );
   velocityLabel->setVisible( true );
//    avLabel->setVisible( true );
//    orientationLabel->setVisible( true );
//    windLabel->setVisible( true );
   
   updateLabels();
}

void Interface::setEnvironmentActual()
{
   World::self()->setEnvironment( World::Actual );
   
   positionLabel->setVisible( false );
   velocityLabel->setVisible( false );
//    avLabel->setVisible( false );
//    orientationLabel->setVisible( false );
//    windLabel->setVisible( false );
   
   updateLabels();
}

double Interface::propInput( int i ) const
{
   return double(m_propInputs[i]->value()) / double(m_propInputs[i]->maximum());
}

void Interface::propInputsChanged()
{
   for ( int i = 0; i < 4; ++i )
      World::self()->simulatedQuad()->setPropInput( i, propInput(i) );
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
   bool simulation = World::self()->environment() == World::Simulation;
   
   if ( simulation )
   {
      QuadState state = World::self()->simulatedQuad()->state();
   
      positionLabel->setText( toCoords( state.pos ) );
      velocityLabel->setText( toCoords( state.vel ) );
//       avLabel->setText( toCoords( state.omega ) );
//       orientationLabel->setText( toCoords( qCoeffs_wxyz(state.orient) ) );
//       windLabel->setText( toCoords( World::self()->wind() ) );
      
//       connStatus->setText( "Simulation" );
   }
   else
   {
//       if ( World::self()->transceiver()->isConnected() )
//          connStatus->setText( "Have connection" );
//       else
//          connStatus->setText( "No connection established" );
   }
   
   Observer *o = World::self()->observer();
   QuadState pred = o->state();
   
   ArrayXd sd = o->cov().diagonal().array();
   
//    predPositionLabel->setText( toCoords( pred.pos ) + "\n+-" + toCoords(sd.segment(0,3).sqrt()) );
//    predVelocityLabel->setText( toCoords( pred.vel ) + "\n+-" + toCoords(sd.segment(3,3).sqrt()) );
//    predavLabel->setText( toCoords( pred.omega ) );
//    predOrientationLabel->setText( toCoords( qCoeffs_wxyz(pred.orient) ) );
   
   
//    timeLabel->setText( fixed( World::self()->time() ) );
//    OmegaLabel->setText( toCoords( 100*World::self()->simulatedQuad()->propInput(), 1 ) );
   
   Sensors *s = World::self()->sensors();
   
   accelerometersLabel->setText( toCoords( s->readAccelerometer() ) + "\n+-" + toCoords(s->var_ba().array().sqrt(),4) );
   gyroLabel->setText( toCoords( s->readGyroscope(), 3 ) + "\n+-" + toCoords(s->var_bg().array().sqrt(),4) );
   gpsLabel->setText( toCoords( s->readGPS() ) + "\n+-" + toCoords(s->var_bgps().array().sqrt(),1) );
   
   
   
   if ( World::self()->isRunning() )
      runPauseButton->setText( "Pause" );
   else
      runPauseButton->setText( "Start" );
   
   if ( World::self()->controller()->haveControl() )
   {
      Vector4d propInput = World::self()->simulatedQuad()->propInput();
      
      // Update slider values
      for ( int i = 0; i < 4; ++i )
      {
         double p = propInput[i];
         int v = int(p * m_propInputs[i]->maximum());
         m_propInputs[i]->setValue( v );
      }
   }
}

void Interface::runPauseQuad()
{
   World::self()->runPause();
   updateLabels();
   
   MatrixXd cov = World::self()->observer()->cov();
   for ( int i = 0; i < cov.rows(); ++i )
   {
      for ( int j = 0; j < cov.cols(); ++j )
      {
         if ( abs( cov(i,j) ) < 1e-9 )
            cov(i,j) = 0;
      }
   }
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
   World::self()->reset();
   propInputsChanged();
   updateLabels();
}
