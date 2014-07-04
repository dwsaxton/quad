#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include <QVector>

#include "mpc.h"
#include "quad.h"

typedef Matrix<double, QUAD_STATE_SIZE, QUAD_STATE_SIZE> MatrixA;
typedef Matrix<double, QUAD_STATE_SIZE, 4> MatrixB;

namespace QP { class Problem; }

/**
 * Linearized state of the quadrocopter.
 */
struct LinearQuad
{
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
   MatrixA Ac;
   MatrixB Bc;
   QuadStateVector f0;
   QuadStateVector x0;
   Vector4d um;
};
LinearQuad linearize( const QuadState & qsx0, const Vector4d & u0 );


class ControlledOutput
{
public:
   ControlledOutput();
   
   void reset();
   
   bool used;
   QVector<double> value;
   double weight;
   void setConstValue( double v );
};


class Controller : public QObject
{
   Q_OBJECT
   
   public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
      Controller();
      ~Controller();
      
      void step();
	  bool haveControl() const { return m_haveControl; }
      
   public Q_SLOTS:
      void giveControl( bool haveControl );
      
   private:
      void initMpc();
	  void updateMpc();
	  void addPropConstraints();
	  
	  void updateControlledOutputs( const QuadState & state );
	  void updateControlledPosition( const QuadState & state );
      void updateControlledOrientation( const QuadState & state );
      void updateToStopSpin( const QuadState & state );
	  QVector<ControlledOutput> m_outputs;
	  
      bool m_haveControl;
	  VectorXd m_predX;
	  
	  // Previous disturbance
	  VectorXd m_pred_d;
	  
	  // Previously applied input
	  Vector4d m_prev_u;
      
      Mpc m_mpc;
};

#endif
