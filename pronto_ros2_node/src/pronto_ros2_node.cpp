
#include <rclcpp/rclcpp.hpp>
#include "pronto_ros/ros_frontend.hpp"
#include "pronto_ros/ins_ros_handler.hpp"
#include "pronto_ros/pose_msg_ros_handler.hpp"
#include "pronto_ros/scan_matcher_ros_handler.hpp"

#include "pronto_quadruped_ros/conversions.hpp"
#include "pronto_quadruped_ros/stance_estimator_ros.hpp"
#include "pronto_quadruped_ros/leg_odometer_ros.hpp"
#include "pronto_quadruped_ros/bias_lock_handler_ros.hpp"
#include "pronto_quadruped_ros/legodo_handler_ros.hpp"
#include "pronto_ros2_node/wheeled_odometry.hpp"

#include "pronto_quadruped_ros/pronto_mammal_utilities.hpp"

#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <algorithm>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <string>
#include <vector>
#include "pinocchio/algorithm/model.hpp"


#include "pronto_quadruped_ros/quad_model_parser.hpp"
#include "pronto_quadruped_ros/bias_lock_handler_ros.hpp"
#include "pronto_ros2_node/qualysis_mt.hpp"

const pinocchio::JointModelFreeFlyer root_fb;
namespace pronto
{
    namespace pronto_node
    {
        using JointsStates = pi3hat_moteus_int_msgs::msg::JointsStates;
        using IMU = sensor_msgs::msg::Imu;
        class Pronto_Ros2 : public rclcpp::Node
        {
            using SensorList = std::vector<std::string>;
            public:
            Pronto_Ros2():
            Node("Pronto_ROS2_Node")
            {
                    this->declare_parameter<std::string>("urdf_file","");
                    this->declare_parameter<std::vector<std::string>>("init_sensors",std::vector<std::string>());
                    this->declare_parameter<std::vector<std::string>>("active_sensors",std::vector<std::string>());
            };
            ~Pronto_Ros2() = default;

            // get the urdf file to create the robot dynamic and the sensors list
            void get_params()
            {
                // get the urdf file path and the sensors used
                try
                {
                urdf_file_ = this->get_parameter("urdf_file").as_string();
                // RCLCPP_INFO(get_logger(),"the urdf file is %s",urdf_file_.c_str());
                init_sensors_ = this->get_parameter("init_sensors").as_string_array();
                active_sensors_ = this->get_parameter("active_sensors").as_string_array();
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), e.what() << '\n');
                    throw std::logic_error("YAML file is not correct");
                }

                // RCLCPP_INFO_STREAM(this->get_logger(),"The urdf file path is "<<urdf_file_.c_str());

                RCLCPP_INFO_STREAM(get_logger(),init_sensors_.size()<<" "<<active_sensors_.size());
                for(auto &init:init_sensors_)
                {
                    if (std::find(all_sensors_.begin(), all_sensors_.end(), init) == all_sensors_.end()) {
                        all_sensors_.push_back(init);
                    }
                    RCLCPP_INFO_STREAM(get_logger(),"Init sensor "<<init.c_str());
                }
                for(auto &active:active_sensors_)
                {
                    RCLCPP_INFO_STREAM(get_logger(),"Active sensor "<<active.c_str());
                    if (std::find(all_sensors_.begin(), all_sensors_.end(), active) == all_sensors_.end()) {
                        all_sensors_.push_back(active);
                    }
                }




            }

            void init_front_end(bool subscribe=true)
            {
                bool init = false;
                bool active = false;
                bool roll_forward = false;
                bool publish_head = false;
                std::string topic;
                Eigen::VectorXd reference_config;
                // parse the urdf to get imu base tf
                auto mod_parse_ = std::make_unique<Model_Parser>(urdf_file_);
                int dof;
                Axis_Direction ax_ker;
                // pinocchio::urdf::buildModelFromXML(urdf_file_,pinocchio::JointModel::fl)
                try
                {
                    dof = mod_parse_->get_robot_DOF();
                    ax_ker = mod_parse_->get_ker_dir();
                    if(ax_ker == Axis_Direction::error)
                    {
                        throw(std::logic_error("not correct xacro file"));
                    }
                    RCLCPP_INFO_STREAM(get_logger(),"The parse robot has "<<dof<<" DOF and "<<
                    ax_ker<<" as kernel direction ");
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR_STREAM(get_logger() ,e.what());
                    throw(std::logic_error("not correct xacro file"));
                }
                std::vector<std::string> jnt_n =  {
                                                    "LF_HAA",
                                                    "LF_HFE",
                                                    "LF_KFE",
                                                    "RF_HAA",
                                                    "RF_HFE",
                                                    "RF_KFE",
                                                    "LH_HAA",
                                                    "LH_HFE",
                                                    "LH_KFE",
                                                    "RH_HAA",
                                                    "RH_HFE",
                                                    "RH_KFE"
                                                }, jnt_pin ;
                std::vector<pinocchio::JointIndex> pin_jnt_ind;
                
                std::vector<int> conv_pro2pin;
                jnt_pin.resize(dof);
                conv_pro2pin.resize(dof);
                mod_parse_->get_jnt_names(jnt_pin);
                bool exist;
                for(size_t i = 0; i < jnt_n.size(); i++)
                {
                    exist = false;
                    for(size_t j = 0; j < jnt_pin.size(); j++)
                    {
                        if(jnt_n[i].compare(jnt_pin[j])==0)
                        {
                            conv_pro2pin[j] = i;
                            exist = true;
                        }

                    }
                    if(!exist)
                    {
                        RCLCPP_WARN(get_logger(),"Joint %s is not cointaned into Pinocchio joints list",jnt_n[i].c_str());
                        if(!(i == 0 || i== 3 || i == 6 || i== 9))
                            throw(std::logic_error("error in URDF joints nomenclature"));
                    }
                

                }
                RCLCPP_ERROR(this->get_logger(),"%d",conv_pro2pin.size());
                for(size_t i = 0; i< dof ; i++)
                {
                    RCLCPP_INFO(get_logger(),"%s--%s--%d",jnt_n[conv_pro2pin[i]].c_str(),jnt_pin[i].c_str(),conv_pro2pin[i]);
                }
                pinocchio::urdf::buildModelFromXML(urdf_file_,root_fb,model_);
                

                // create front end
                ros_fe_ = std::make_shared<pronto::ROSFrontEnd>(this->shared_from_this(),true);

                for (SensorList::iterator it = all_sensors_.begin(); it != all_sensors_.end(); ++it)
                {
                    RCLCPP_INFO(get_logger(),"allocate sensor %s",it->c_str());
                    declare_parameter<bool>(*it + ".roll_forward_on_receive",false);
                    declare_parameter<bool>(*it + ".publish_head_on_message",false);
                    declare_parameter<std::string>(*it + ".topic","");

                    if (!this->get_parameter(*it + ".roll_forward_on_receive", roll_forward)) {
                        RCLCPP_WARN_STREAM(this->get_logger(),"Not adding sensor \"" << *it << "\".");
                        RCLCPP_WARN(this->get_logger(), "Param \"roll_forward_on_receive\" not available.");
                        continue;
                    }
                    if (!this->get_parameter(*it + ".publish_head_on_message", publish_head)) {
                        RCLCPP_WARN_STREAM(this->get_logger(),"Not adding sensor \"" << *it << "\".");
                        RCLCPP_WARN(this->get_logger(), "Param \"publish_head_on_message\" not available.");
                        continue;
                    }
                    if (!this->get_parameter(*it + ".topic", topic)) {
                        RCLCPP_WARN_STREAM(this->get_logger(),"Not adding sensor \"" << *it << "\".");
                        RCLCPP_WARN(this->get_logger(), "Param \"topic\" not available.");
                        continue;
                    }
                    // check if the sensor is also used to initialize
                    init = (std::find(init_sensors_.begin(), init_sensors_.end(), *it) != init_sensors_.end());
                    active = (std::find(active_sensors_.begin(), active_sensors_.end(), *it) != active_sensors_.end());

                    // add sensor module to front end
                    if (it->compare("ins") == 0)
                    {
                        const std::string ins_param_prefix = "ins.";
                        std::string imu_frame = "imu";
                        std::string base_frame = "base_link";
                        Eigen::Isometry3d i2bl_trans;
                        
                        // get the urdf imu and base link name 
                        this->declare_parameter<std::string>(ins_param_prefix + "frame",imu_frame); 
                        this->declare_parameter<std::string>(ins_param_prefix + "base_link_name",base_frame);
                        imu_frame = this->get_parameter(ins_param_prefix + "frame").as_string();
                        base_frame = this->get_parameter(ins_param_prefix +"base_link_name").get_value<std::string>();
                        
                        mod_parse_->get_imu_base_tranform(base_frame,imu_frame,i2bl_trans);

                        
                        ins_handler_ = std::make_shared<InsHandlerROS>(this->shared_from_this(), i2bl_trans);
                        if (active)
                        {
                            ros_fe_->addSensingModule(*ins_handler_, *it, roll_forward, publish_head, topic, subscribe);
                        }
                        if (init)
                        {
                            ros_fe_->addInitModule(*ins_handler_, *it, topic, subscribe);
                        }

                    }
                    else if (it->compare("legodo") == 0)
                    {
                        //try to make the model and the pinocchio's matrix
                        declare_parameter<bool>(*it + ".sim",false);
                        bool sim ;
                        get_parameter(*it+".sim",sim);
                        
                        feet_force_ = pronto_pinocchio::Pinocchio_Feet_Force(model_,ax_ker,dof,conv_pro2pin);
                        jacs_ = pronto_pinocchio::Pinocchio_Jacobian(&feet_force_);
                        fk_ = pronto_pinocchio::Pinocchio_FK(&feet_force_);

                        stance_est_ = std::make_shared<quadruped::StanceEstimatorROS>(shared_from_this(),feet_force_);
                        leg_odom_ = std::make_shared<quadruped::LegOdometerROS>(shared_from_this(),jacs_,fk_);
                        if(sim)
                            lo_pin_handler_sim_ = std::make_shared<quadruped::LegodoHandlerPinRos_Sim>(shared_from_this(),stance_est_.get(),leg_odom_.get(),jnt_n);
                        else
                            lo_pin_handler_ = std::make_shared<quadruped::LegodoHandlerPinRos>(shared_from_this(),stance_est_.get(),leg_odom_.get(),jnt_n);
                        if (active)
                            {
                                if(sim)
                                    ros_fe_->addSensingModule(*lo_pin_handler_sim_, *it, roll_forward, publish_head, topic, subscribe);
                                else
                                    ros_fe_->addSensingModule(*lo_pin_handler_, *it, roll_forward, publish_head, topic, subscribe);
                            }
                            if (init)
                            {
                                if(sim)
                                    ros_fe_->addInitModule(*lo_pin_handler_sim_, *it, topic, subscribe);
                                else
                                    ros_fe_->addInitModule(*lo_pin_handler_, *it, topic, subscribe);
                            }
                    }
                    else if(it->compare("bias_lock")==0)
                    {
                        declare_parameter<bool>(*it + ".sim",false);
                        declare_parameter<std::string>(*it + ".secondary_topic","");
                        bool sim ;
                        std::string sec_topic;
                        get_parameter(*it+".sim",sim);
                        get_parameter(*it + ".secondary_topic",sec_topic);
                        const std::string ins_param_prefix = "ins.";
                        std::string imu_frame = "imu";
                        std::string base_frame = "base_link";
                    
                        Eigen::Isometry3d i2bl_trans;
                        
                        // get the urdf imu and base link name
                        imu_frame = this->get_parameter(ins_param_prefix + "frame").as_string();
                        base_frame = this->get_parameter(ins_param_prefix +"base_link_name").get_value<std::string>();
                        
                        mod_parse_->get_imu_base_tranform(base_frame,imu_frame,i2bl_trans);

                     if(sim)
                            ibl_handler_sim_ = std::make_shared<quadruped::ImuBiasLockROS_Sim>(shared_from_this(),i2bl_trans);
                        else

                            ibl_handler_ = std::make_shared<quadruped::ImuBiasLockROS>(shared_from_this(),i2bl_trans);

                        if (active)
                            {
                                if(sim)
                                {
                                    ros_fe_->addSensingModule(*ibl_handler_sim_, *it, roll_forward, publish_head, topic, subscribe);
                                    ros_fe_->addSecondarySensingModule(*ibl_handler_sim_,*it,sec_topic,subscribe);
                                }
                                else
                                {
                                    ros_fe_->addSensingModule(*ibl_handler_, *it, roll_forward, publish_head, topic, subscribe);
                                    ros_fe_->addSecondarySensingModule(*ibl_handler_,*it,sec_topic,subscribe);
                                }
                            }
                            if (init)
                            {
                                if(sim)
                                    ros_fe_->addInitModule(*ibl_handler_sim_, *it, topic, subscribe);
                                else
                                    ros_fe_->addInitModule(*ibl_handler_, *it, topic, subscribe);
                            }
                    }
                    else if(it->compare("qualysis_mt") == 0)
                    {
                            qual_mt_ = std::make_shared<QualysisMTRosHandler>(this->shared_from_this());
                            if (active)
                            {
                                ros_fe_->addSensingModule(*qual_mt_, *it, roll_forward, publish_head, topic, subscribe);
                            }
                            if (init)
                            {
                                ros_fe_->addInitModule(*qual_mt_, *it, topic, subscribe);
                            }

                    }
                    else if(it->compare("scan_matcher") == 0)
                    {
                        scan_match_handler_ = std::make_shared<ScanMatcherHandler>(this->shared_from_this());
                        if (active)
                        {
                            ros_fe_->addSensingModule(*scan_match_handler_, *it, roll_forward, publish_head, topic, subscribe);
                        }
                        if (init)
                        {
                            ros_fe_->addInitModule(*scan_match_handler_, *it, topic, subscribe);
                        }
                    }
                    else if(it->compare("w_odom") == 0)
                    {
                        w_odom_handler_ = std::make_shared<WheeledOdometry>(this->shared_from_this());
                        if (active)
                        {
                            ros_fe_->addSensingModule(*w_odom_handler_, *it, roll_forward, publish_head, topic, subscribe);
                        }
                        if (init)
                        {
                            ros_fe_->addInitModule(*w_odom_handler_, *it, topic, subscribe);
                        }
                    }
                }
            }

            private:
                SensorList init_sensors_;
                SensorList active_sensors_;
                SensorList all_sensors_;
                std::string urdf_file_;

                //declare the mammal utils stuff and the ros_frontend
                std::shared_ptr<pronto::ROSFrontEnd> ros_fe_;
                std::shared_ptr<quadruped::StanceEstimatorROS> stance_est_;
                std::shared_ptr<quadruped::LegOdometerROS> leg_odom_;
                pronto_pinocchio::Pinocchio_Feet_Force feet_force_;
                pronto_pinocchio::Pinocchio_Jacobian jacs_;
                pronto_pinocchio::Pinocchio_FK fk_;
                pinocchio::Model model_;


                //declare shared ptr to proprioceptive sensors handler
                std::shared_ptr<InsHandlerROS> ins_handler_;
                std::shared_ptr<quadruped::LegodoHandlerPinRos> lo_pin_handler_;
                std::shared_ptr<quadruped::LegodoHandlerPinRos_Sim> lo_pin_handler_sim_;
                std::shared_ptr<quadruped::ImuBiasLockROS> ibl_handler_;
                std::shared_ptr<quadruped::ImuBiasLockROS_Sim> ibl_handler_sim_;
                std::shared_ptr<QualysisMTRosHandler> qual_mt_;
                std::shared_ptr<ScanMatcherHandler> scan_match_handler_;
                std::shared_ptr<WheeledOdometry> w_odom_handler_;
        };

    }; // namespace pronto_node

};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);



    std::shared_ptr<pronto::pronto_node::Pronto_Ros2> node = std::make_shared<pronto::pronto_node::Pronto_Ros2>();
    node->get_params();
    node->init_front_end();
    rclcpp::spin(node);



    return 0;
}

