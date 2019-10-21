    //Roll pitch and yaw in Radians
    //double roll = 1.5707, pitch = 0, yaw = 0.707;    
    double roll = 0.3707, pitch = -0.30, yaw = 0.407;    


    ////////////////////////////////////////////////////////////////
    // Old Way - depreciated
    Eigen::Quaterniond q0 = euler_to_quat(roll, pitch, yaw); // rpy
    std::stringstream ss0;
    print_Quaterniond(q0, ss0);
    std::cout << ss0.str() << " q OLD\n";

    double rpy0[3];
    quat_to_euler(q0, rpy0[0], rpy0[1], rpy0[2]);

    std::cout << rpy0[0] << " "
              << rpy0[1] << " "
              << rpy0[2] << " " << " e OLD\n\n";

 
    ////////////////////////////////////////////////////////////////
    // Recommended Way
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    std::stringstream ss;
    print_Quaterniond(q, ss);
    std::cout << ss.str() << " q NEW\n";
    std::cout << q.toRotationMatrix().eulerAngles(2, 1, 0).reverse().transpose() << " e NEW\n\n";


    ////////////////////////////////////////////////////////////////
    // Consistent ROS way - avoid using internally
    tf::Quaternion qTF = tf::createQuaternionFromRPY ( roll, pitch, yaw);
    qTF.setRPY(roll, pitch, yaw);  
    std::cout << qTF.w() << " "
              << qTF.x() << " "
              << qTF.y() << " "
              << qTF.z() << " " << " qTF\n";  
    double rpy_tf[3];  
    tf::Matrix3x3(qTF).getRPY(rpy_tf[0], rpy_tf[1], rpy_tf[2]);
    std::cout << rpy_tf[0] << " "
              << rpy_tf[1] << " "
              << rpy_tf[2] << " " << " e TF\n\n";