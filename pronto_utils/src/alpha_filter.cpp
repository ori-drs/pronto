#include "pronto_utils/alpha_filter.hpp"
using namespace Eigen;
using namespace std;

namespace EstimateTools {

AlphaFilter::AlphaFilter(double alpha_): 
     alpha_(alpha_){
  init_ = false;
  verbose_ = false;

}


void AlphaFilter::processSample(Eigen::VectorXd& x, 
                    Eigen::VectorXd &x_filtered){
  if (!init_){
     x_filtered = x;
     x_filtered_prev_ = x_filtered;
     init_ = true;
     return;
  }

  x_filtered <<  alpha_*x_filtered_prev_  + (1.0 - alpha_)*x;
  //std::cout << "pre: " << x.transpose()
  //          << " | post: " << x_filtered.transpose()
  //          << "\n";

  x_filtered_prev_ = x_filtered;
}


}
