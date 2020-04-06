#pragma once

#include <fstream>      // std::ofstream

#include <map>
#include <filter_tools/simple_kalman_filter.hpp> // SimpleKalmanFilter

#include "pronto_biped_core/FootContact.hpp"
#include "pronto_biped_core/FootContactAlt.hpp"
#include "pronto_biped_core/foot_contact_classify.hpp"
#include "pronto_biped_core/biped_forward_kinematics.hpp"
#include <memory>

namespace pronto {
namespace biped {

constexpr int NUM_FILT_JOINTS = 30;

struct LegOdometerConfig {
    ControlMode control_mode = ControlMode::CONTROLLER_UNKNOWN;
    FilterJointMode filter_mode = FilterJointMode::NONE;
    bool use_controller_input = false;
    std::string initialization_mode;
    std::string left_foot_name = "l_foot";
    std::string right_foot_name = "r_foot";
    bool filter_contact_events = false;
    bool publish_diagnostics = false;
    float total_force = 0.0;
    float standing_schmitt_level = 0.0;
    float schmitt_low_threshold = 0;
    float schmitt_high_threshold = 0;
    int schmitt_low_delay = 0;
    int schmitt_high_delay = 0;
};

class LegEstimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    LegEstimator(BipedForwardKinematics& fk, const LegOdometerConfig& cfg);
    
    virtual ~LegEstimator();
    
    inline void setPoseBody(const Eigen::Isometry3d& world_to_body_in ){
      world_to_body_ = world_to_body_in; 
      world_to_body_init_ = true;
    }
    
    inline void setFootSensing(const FootSensing& lfoot_sensing_in,
                               const FootSensing& rfoot_sensing_in )
    {
      lfoot_sensing_ = lfoot_sensing_in;
      rfoot_sensing_ = rfoot_sensing_in;
    }

    inline void setControlContacts(int n_control_contacts_left_in,
                                   int n_control_contacts_right_in)
    {
      n_control_contacts_left_  = n_control_contacts_left_in;
      n_control_contacts_right_ = n_control_contacts_right_in;
    }
    
    // Update the running leg odometry solution
    // returns: odometry_status - a foot contact classification
    // which can be used to determine a suitable covariance
    // 0 -> 1 float
    // 0 is very accurate   
    // 1 very inaccuracy    - foot breaks
    // -1 unuseable/invalid - foot strikes
    float updateOdometry(const std::vector<std::string> &joint_name,
                         const std::vector<double> &joint_position,
                         const std::vector<double> &joint_velocity,
                         const int64_t &utime);

    // returns a validity label, currently alaways true
    bool getLegOdometryDelta(Eigen::Isometry3d &odom_to_body_delta,
                             int64_t &current_utime,
                             int64_t &previous_utime);
    
    // returns a validity label
    bool getLegOdometryWorldConstraint(Eigen::Isometry3d &world_to_body_constraint,
                                       int64_t &current_utime);
    
    inline Eigen::Isometry3d getRunningEstimate() const {
        return odom_to_body_;
    }
    
    inline void setInitializationMode(const std::string& initialization_mode_in)
    {
        initialization_mode_ = initialization_mode_in;
    }

private:
    BipedForwardKinematics& fk_;
    // original method from Dehann uses a very conservative Schmitt trigger
    ContactStatusID footTransition();
    // a more aggressive trigger with different logic
    ContactStatusID footTransitionAlt();
    /// Integration Methods:
    bool initializePose(const Eigen::Isometry3d& body_to_foot);

    bool prepInitialization(const Eigen::Isometry3d& body_to_l_foot,
                            const Eigen::Isometry3d& body_to_r_foot,
                            ContactStatusID contact_status);
    // Foot position, as with above. For subsequent ticks, foot quaternion is updated using the pelvis quaternion
    // The pelvis position is then backed out using this new foot positon and fk.
    // return: true on initialization, else false
    bool legOdometryGravitySlavedAlways(const Eigen::Isometry3d& body_to_l_foot,
                                        const Eigen::Isometry3d& body_to_r_foot,
                                        const ContactStatusID  & contact_status);

    // related method to determine position constraint
    void determinePositionConstraintSlavedAlways(const Eigen::Isometry3d &body_to_l_foot,
                                                 const Eigen::Isometry3d &body_to_r_foot);

  private:
    // joint position filters, optionally used
    std::vector<LowPassFilter*> lpfilter_; // previously were not pointers
    std::vector<EstimateTools::SimpleKalmanFilter*> joint_kf_;

    /// Parameters
    int verbose_;
    // How the position will be initialized
    std::string initialization_mode_;
    // Which link is assumed to be stationary - typically [lr]_foot or [lr]_talus
    std::string l_standing_link_;
    std::string r_standing_link_;
    // use a heavy low pass filter on the input joints
    FilterJointMode filter_joint_positions_;
    // detect and filter kinematics when contact occurs
    bool filter_contact_events_;
    // Publish Debug Data e.g. kinematic velocities and foot contacts
    bool publish_diagnostics_;    
    
    /// Foot Contact Classifiers
    // most recent measurements for the feet forces (typically synchronised with joints measurements
    FootSensing lfoot_sensing_, rfoot_sensing_; // unfiltered... check
    std::shared_ptr<biped::FootContact> foot_contact_logic_; // dehann, conservative
    std::shared_ptr<biped::FootContactAlt> foot_contact_logic_alt_; // mfallon
    // Classify Contact Events (seperate from the above)
    std::shared_ptr<FootContactClassifier> foot_contact_classify_;

    // output from the FootContact class(es), not directly related to primary_foot_, but close
    FootID standing_foot_;
    
    /// Current High-level controller mode
    // Used to determine certain classifiers to use
    ControlMode control_mode_;
    
    // Use information from the controller about the contact points it is exerting
    bool use_controller_input_;
    // which contacts is the controller using
    int n_control_contacts_left_, n_control_contacts_right_;
    
    /// State Variables
    // Current time from current input msg 
    int64_t current_utime_;
    int64_t previous_utime_;
    
    // Current position of pevis in nominal odometry frame
    Eigen::Isometry3d odom_to_body_;
    Eigen::Isometry3d previous_odom_to_body_;
    // The incremental motion of the pelvis: transform between previous_odom_to_body_ and odom_to_body_
    Eigen::Isometry3d odom_to_body_delta_;
    bool leg_odo_init_ = false; // has the leg odometry been initialized. (set to false when an anomoly is detected)
    FootID primary_foot_; // the foot assumed to be fixed for the leg odometry. TODO: unify the foot ids
    Eigen::Isometry3d odom_to_primary_foot_fixed_; // Position in the odom frame in which the fixed foot is kept
    Eigen::Isometry3d odom_to_secondary_foot_; // Ditto for moving foot (entirely defined by kinematics)

    // Pelvis Position produced by mav-estimator
    // TODO: this should be the same as the RBIS state
    Eigen::Isometry3d world_to_body_;
    bool world_to_body_init_;
    // Free running feet positions in world frame
    // HOWEVER: primary are NOT fixed as they are the positions after sensor fusion with INS+Lidar
    // hence these frames will slide around (by as much as 2cm) during a stride
    Eigen::Isometry3d world_to_primary_foot_slide_; 
    Eigen::Isometry3d world_to_secondary_foot_; 
    
    // .. in contrast this frame is the position of primary AT THE TIME OF TRANSITION
    // If we assumed that a foot did not move after first contact, it shouldn't leave this position
    // But it at least can be used as a measurement constraint on XYZ
    // (TODO: this position to be fed into a Filter to update it slowly)
    Eigen::Isometry3d world_to_primary_foot_transition_; 
    bool world_to_primary_foot_transition_init_;
    // This is the position of the foot later on during the step
    // if the kinematics was perfect this would be the same position foot.
    // NB: the XYZ value of this position is the same as world_to_primary_foot_transition_
    // This assumption is the basis of our position constraint
    Eigen::Isometry3d world_to_primary_foot_constraint_; 
    Eigen::Isometry3d world_to_secondary_foot_constraint_; 
    // ... and finally the position the pelvis would have if no sliding had occured.
    Eigen::Isometry3d world_to_body_constraint_; 
    bool world_to_body_constraint_init_; // is this constraint valid/up-to-date?
    
    Eigen::Isometry3d previous_body_to_l_foot_; // previous FK positions. Only used in one of the integration methods
    Eigen::Isometry3d previous_body_to_r_foot_;

    std::vector<double> filtered_joint_position_;
    std::vector<double> filtered_joint_velocity_;
    std::vector<double> filtered_joint_effort_;
};    

}
}
