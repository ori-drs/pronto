#include "pronto_core/indexed_meas_module.hpp"
#include <iostream>

namespace pronto {

IndexedMeasurementModule::IndexedMeasurementModule(const RBISUpdateInterface::sensor_enum& sensor) :
    indexed_sensor(sensor){

}

RBISUpdateInterface* IndexedMeasurementModule::processMessage(const IndexedMeasurement *msg,
                                         StateEstimator *est)
{
    return new RBISIndexedMeasurement(msg->z_indices,
                                      msg->z_effective,
                                      msg->R_effective,
                                      indexed_sensor,
                                      msg->utime);

}

bool IndexedMeasurementModule::processMessageInit(const IndexedMeasurement *msg,
                                                  const std::map<std::string, bool> &sensor_initialized,
                                                  const RBIS &default_state,
                                                  const RBIM &default_cov,
                                                  RBIS &init_state, RBIM &init_cov)
{
    RBISIndexedMeasurement * update = (RBISIndexedMeasurement *) processMessage(msg, NULL);

    if (update == NULL) {
      return false;
    }

    int m = update->index.rows();
    for (int ii = 0; ii < m; ii++) {
      for (int jj = 0; jj < m; jj++) {
        init_cov(update->index(ii), update->index(jj)) = update->measurement_cov(ii, jj);
      }
      init_state.vec(update->index(ii)) = update->measurement(ii);
    }
    init_state.chiToQuat();

    std::cout << "Initializing indices:" << std::endl << update->index << std::endl
              << "with:" << std::endl << update->measurement << std::endl
              << "and cov:" << std::endl << update->measurement_cov << std::endl;
    delete update;

    return true;
}


} // namespace pronto
