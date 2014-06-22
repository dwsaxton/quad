#ifndef MPC_H
#define MPC_H

#include <QVector>

#include <Eigen/Geometry>
using namespace Eigen;

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>

void mpcTest();

class Mpc
{
public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
   Mpc();
   ~Mpc();
   
   void checkDimensions();
   
   VectorXd f0;
   VectorXd x0;
   VectorXd um; // u_{-1}
   MatrixXd Ac;
   MatrixXd Bc;
   VectorXd d; // disturbance
   double Ts;
   
   /**
	* n is the dimension of x and w, the number of variables in the system.
	*/
   int n;
   /**
	* l is the dimension of u, the number of controls.
	*/
   int l;
   /**
	* m is the dimension of y=z, the number of measured/controlled outputs.
	*/
   int m;
   /**
	* Prediction horizon; how long do we compare the predicted trajectory
	* with the target trajectory?
	*/
   int Hp;
   /**
	* Time for which the input u is allowed to change.
	*/
   int Hu;
   /**
	* When we start penalizing changes from the reference trajectory.
	*/
   int Hw;
   /**
	* Reference trajectory for x.
	* Vector of length m*(Hp-Hw+1).
	*/
   VectorXd r;
   /**
	* How / what is measured from x.
	* Matrix of dimension m x n.
	*/
   SparseMatrix<double> C;
   /**
	* Set this to true if permC is valid.
	*/
   bool permCValid;
   QVector<int> permC;
   /**
	* Penalize deviations from the reference trajectory.
	* Square matrix with m*(Hp-Hw+1) rows
	*/
   MatrixXd Q;
//    SparseMatrix<double> Q;
   /**
	* If Q is a diagonal matrix, then set this to true to speed up calculations.
	*/
   bool QIsDiagonal;
   /**
	* Penalize changes in the input.
	* Square matrix with l*Hu rows.
	*/
   MatrixXd R;
   /**
	* Constraint on control input change:
	* E \Delta U <= E0
	*/
    MatrixXd E;
   /**
	* Constraint on control input change:
	* E \Delta U <= E0
	*/
    VectorXd E0;
   /**
	* Constraint on control input:
	* F (u_0, \ldots, u_{H_u-1}) \leq F0
	*/
    MatrixXd F;
   /**
	* Constraint on control input:
	* F (u_0, \ldots, u_{H_u-1}) \leq F0
	*/
    VectorXd F0;
   /**
	* Constraint on trajectory:
	* G (x_1, \ldots, x_{H_p}) \leq G0
	*/
    MatrixXd G;
   /**
	* Constraint on trajectory:
	* G (x_1, \ldots, x_{H_p}) \leq G0
	*/
    VectorXd G0;
   
   /**
    * Calculate the optimal change in input u to apply. This function also
	* returns what it expects the next x value to be in predX (minus the current
	* correction for disturbance), used for disturbance correction.
    */
   VectorXd calc_du( VectorXd *predX = 0 );
   
   VectorXd J;
   MatrixXd A;
   MatrixXd B;
   
   VectorXd K;
   
private:
   void calc_JAB();
   
   /**
	* sumAiB[i] = \sum_{j=0}^i  A^j B
	*/
   QVector<MatrixXd> sumAiB;
   void calc_sumAiB();
   
   /**
	* C D_k, so Dk[i] = C D_i, i = 1, ..., H_p
	*/
   QVector<MatrixXd> Dk;
   QVector<MatrixXd> CDk;
   void calc_Dk();
   void calc_CDk();
   
   /**
	* Free response of system. Thus first n entries are w_1, the next n entries
	* are w_2, and so fourth.
	*/
   VectorXd w;
   void calc_w();
   
   /**
	* The matrices for the QP problem.
	*/
   MatrixXd phi;
   MatrixXd Phi;
   void calc_QP();
   
   MatrixXd H;
   void calc_K();
   void calc_H();
   
   void calc_phi();
   void calc_Phi();
   
   MatrixXd Omega;
   VectorXd omega;
   void calcConstraints();
};

#endif
