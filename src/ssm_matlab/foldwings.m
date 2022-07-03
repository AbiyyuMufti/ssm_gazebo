function foldwings()
%FOLDWINGS Summary of this function goes here
%   Detailed explanation goes here
    try
        rosinit('http://MDY:11311/');
    catch exception
        disp("Already Connected to ROS MASTER");
    end
    fins1pub = rospublisher("/ex_mm40b3_controller/fin_fold1_position_controller/command", "std_msgs/Float64");
    fins2pub = rospublisher("/ex_mm40b3_controller/fin_fold2_position_controller/command", "std_msgs/Float64");
    fins3pub = rospublisher("/ex_mm40b3_controller/fin_fold3_position_controller/command", "std_msgs/Float64");
    fins4pub = rospublisher("/ex_mm40b3_controller/fin_fold4_position_controller/command", "std_msgs/Float64");
    wing1pub = rospublisher("/ex_mm40b3_controller/wings_fold1_position_controller/command", "std_msgs/Float64");
    wing2pub = rospublisher("/ex_mm40b3_controller/wings_fold2_position_controller/command", "std_msgs/Float64");
    wing3pub = rospublisher("/ex_mm40b3_controller/wings_fold3_position_controller/command", "std_msgs/Float64");
    wing4pub = rospublisher("/ex_mm40b3_controller/wings_fold4_position_controller/command", "std_msgs/Float64");
    
    foldmsg = rosmessage('std_msgs/Float64');
    foldmsg.Data = deg2rad(123);
    fins1pub.send(foldmsg)
    fins2pub.send(foldmsg)
    fins3pub.send(foldmsg)
    fins4pub.send(foldmsg)
    wing1pub.send(foldmsg)
    wing2pub.send(foldmsg)
    wing3pub.send(foldmsg)
    wing4pub.send(foldmsg)
    pause(3);
end
