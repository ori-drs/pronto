// Bring in my package's API, which is what I'm testing
#include <lcmtypes/bot_core/ins_t.hpp>
#include <bot_frames/bot_frames.h>
#include <Eigen/Dense>
#include <eigen_utils/eigen_rigidbody.hpp>
#include <eigen_utils/eigen_lcm.hpp>
// Bring in gtest
#include <gtest/gtest.h>
#include <random>

// Declare a test
TEST(BotTrans, transAndRotation)
{
    for(int i=0; i<250; i++){
        // check back and forward conversion for 250 random transformations

        // fill random positions and rotations
        Eigen::Vector3d v(Eigen::Vector3d::Random());
        Eigen::Quaterniond q(Eigen::Quaterniond::UnitRandom());

        // fill an Eigen Affine with those
        Eigen::Affine3d eigen_t1(Eigen::Affine3d::Identity());
        eigen_t1.translate(v);
        eigen_t1.rotate(q);

        // fill a bot transformation with the random vector/quaternion
        BotTrans bot_trans;
        bot_trans.trans_vec[0] = v(0);
        bot_trans.trans_vec[1] = v(1);
        bot_trans.trans_vec[2] = v(2);
        eigen_utils::quaternionToBotDouble(bot_trans.rot_quat,q);

        // create a new Affine with the values from the bot trans
        Eigen::Affine3d eigen_t2(Eigen::Affine3d::Identity());
        eigen_t2.translate(Eigen::Map<Eigen::Vector3d>(bot_trans.trans_vec));
        Eigen::Quaterniond temp_quat;
        eigen_utils::botDoubleToQuaternion(temp_quat,bot_trans.rot_quat);
        eigen_t2.rotate(temp_quat);

        // check that the two Affine are the same
        ASSERT_TRUE(eigen_t1.isApprox(eigen_t2));

        // create an array of double and fill with random values
        double bot_temp1[3];
        double lower_bound = 0;
        double upper_bound = 10000;
        std::uniform_real_distribution<double> unif(lower_bound,upper_bound);
        std::default_random_engine re;

        bot_temp1[0] = unif(re);
        bot_temp1[1] = unif(re);
        bot_temp1[2] = unif(re);

        // rotate the random array with the rotational part of the bot trans
        // from before
        double bot_temp2[3];
        bot_quat_rotate_to(bot_trans.rot_quat, bot_temp1, bot_temp2);
        Eigen::Map<Eigen::Vector3d> eigen_temp3(bot_temp2);

        // rotate the same array but with the affine from before, which should
        // be equal to the bot trans because we checked already
        Eigen::Vector3d eigen_temp4(eigen_t1.rotation() * Eigen::Map<Eigen::Vector3d>(bot_temp1));

        // check the two results are the same
        EXPECT_TRUE(eigen_temp4.isApprox(eigen_temp3));
    }

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
