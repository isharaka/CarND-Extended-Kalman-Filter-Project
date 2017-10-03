#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  static MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * A helper method to normalise angle within .-pi and +pi
  */
  static float NormaliseAngle(const float angle);


  /**
  * A helper method to convert cartesian measurement vector to polar
  */
  static int Cartesian2Polar(const VectorXd& cartesian, VectorXd& polar);


  /**
  * A helper method to convert polar measurement vector to cartesian
  */
  static int Polar2Cartesian(const VectorXd polar, VectorXd& cartesian);

};

#endif /* TOOLS_H_ */
