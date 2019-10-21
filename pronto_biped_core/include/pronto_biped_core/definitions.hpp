#pragma once

namespace pronto {
namespace biped {
// TODO: these enum duplicate one another and also something in Dehann's classifier, clean this up

enum class FootID {
  UNKNOWN=-1,
  LEFT  =0,
  RIGHT =1,
};

enum class ContactStatusID {
  UNKNOWN = -1,
  LEFT_NEW   = 0, // just decided that left is in (primary) contact
  RIGHT_NEW   = 1, // just decided that right is in (primary) contact
  LEFT_FIXED = 2, // left continues to be in primary contact
  RIGHT_FIXED = 3, // right continues to be in primary contact
};

enum class WalkMode {
  UNKNOWN = -1,
  LEFT_PRIME_RIGHT_STAND = 0, //0 both in contact, left has been for longer
  LEFT_PRIME_RIGHT_BREAK = 1, //1 ....
  LEFT_PRIME_RIGHT_SWING = 2, //2 left in contact, right raised
  LEFT_PRIME_RIGHT_STRIKE= 3, //3
  LEFT_STAND_RIGHT_PRIME = 4, //4
  LEFT_BREAK_RIGHT_PRIME = 5, //5
  LEFT_SWING_RIGHT_PRIME = 6, //6
  LEFT_STRIKE_RIGHT_PRIME= 7, //7

};

// These ids should match drc_controller_status_t.lcm
enum class ControlMode {
  CONTROLLER_UNKNOWN  = 0,
  CONTROLLER_STANDING = 1,
  CONTROLLER_WALKING  = 2,
  CONTROLLER_TOE_OFF  = 8, // not in drc_controller_status_t
};

enum class FilterJointMode {
  NONE    = 0,
  LOWPASS = 1,
  KALMAN  = 2,
};

inline bool isEqualTo(FootID one, ContactStatusID two) {
    return static_cast<int>(one) == static_cast<int>(two);
}

inline bool isEqualTo(ContactStatusID one, FootID two){
    return isEqualTo(two, one);
}

} // namespace biped
} // namespace pronto
