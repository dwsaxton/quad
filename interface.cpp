#include "interface.h"

#include <QtGui>

#include "controllooper.h"
#include "globals.h"
#include "imu.h"
#include "propellers.h"
#include "quad.h"
#include "highercontroller.h"

Interface::Interface()
{
  control_automatic_ = true;
  setupUi(this);
  
  m_propInputs[0] = prop1;
  m_propInputs[1] = prop2;
  m_propInputs[2] = prop3;
  m_propInputs[3] = prop4;
  
  for ( int i = 0; i < 4; ++i ) {
    connect( m_propInputs[i], SIGNAL(valueChanged(int)), this, SLOT(onPropSlidersAdjusted()) );
  }

  runPauseSimulationButton->setVisible(Globals::self().isSimulation());

  connect(controlAutomatic, SIGNAL(toggled(bool)), this, SLOT(setControlAutomatic(bool)));
  connect(goButton, SIGNAL(clicked()), this, SLOT(onAutoGoClicked()));
  connect(runPauseSimulationButton, SIGNAL(clicked()), this, SLOT(onRunPauseSimulationClicked()));
  connect(resetButton, SIGNAL(clicked()), this, SLOT(onResetClicked()));

  QTimer *timer = new QTimer(this);
  connect( timer, SIGNAL(timeout()), this, SLOT(updateLabels()) );
  timer->start( 50 );
}


Interface::~Interface()
{
   delete Globals::self().controlLooper();
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

void Interface::updateLabels()
{
  bool isSimulation = Globals::self().isSimulation();
  Quad *quad;
  if (isSimulation) {
    quad = Globals::self().simulatedQuad();
  } else {
    quad = Globals::self().remoteQuad();
  }
   
  QuadState state = quad->state();

  positionLabel->setText( toCoords( state.pos ) );
  velocityLabel->setText( toCoords( state.vel ) );

  Vector3d accel = Globals::self().imu()->lastAcceleration();
  Vector3d gyro = Globals::self().imu()->lastAngularAcceleration();

  accelerometersLabel->setText(toCoords(accel));
  gyroLabel->setText(toCoords(gyro, 3));

  // Update plot 
  double duration = quad->path_ != nullptr ? quad->path_->duration() : 0;
  interceptPlot->setIntercept(quad->intercept, duration);

  runPauseSimulationButton->setText(Globals::self().isSimulatedQuadRunning() ? "Pause" : "Run");

  Vector4d propInput = Globals::self().propellers()->input();

  // Update slider values
  for ( int i = 0; i < 4; ++i ) {
    double p = propInput[i];
    int v = int(p * m_propInputs[i]->maximum());
    m_propInputs[i]->setValue( v );
  }
}

void Interface::setControlAutomatic(bool automatic) {
  control_automatic_ = automatic;
  bool runControlLooper = control_automatic_ && (!Globals::self().isSimulation() || Globals::self().isSimulatedQuadRunning());
  Globals::self().controlLooper()->setRunning(runControlLooper);
}

void Interface::onAutoGoClicked() {
  Vector3d pos = {go_x->value(), go_y->value(), go_z->value()};
  Globals::self().controlLooper()->controller()->setTargetPos(pos);
}

double Interface::propInput( int i ) const {
  return double(m_propInputs[i]->value()) / double(m_propInputs[i]->maximum());
}

void Interface::onPropSlidersAdjusted()
{
  Propellers *p = Globals::self().propellers();
  for ( int i = 0; i < 4; ++i ) {
    p->setInput(i, propInput(i));
  }
}

void Interface::onRunPauseSimulationClicked() {
  Globals::self().setSimulationRunning(!Globals::self().isSimulatedQuadRunning());
}

void Interface::onResetClicked() {
  Globals::self().reset();
}

