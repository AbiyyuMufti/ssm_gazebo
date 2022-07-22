clc; clear;


target_directory = "SimulinkMsgs";
msgs_directory = "msgs";

targetPath = fullfile(pwd,target_directory);
sourcePath = fullfile(pwd,msgs_directory);

if exist(targetPath, 'dir')
    disp('SimulinkMsgs exisiting already, removing ...')
    rmdir(targetPath, 's')
end
disp('Create SimulinkMsgs directory')
mkdir(target_directory);


disp('Copying custom msgs to target directory')
msgs_files = dir(fullfile(sourcePath,"*.proto"));
for k = 1:length(msgs_files)
    proto_name = msgs_files(k).name;
    copyfile(fullfile(sourcePath, proto_name), fullfile(targetPath, proto_name));
end

msg_togenerate = ["gazebo.msgs.Vector3d","gazebo.msgs.Image", ... 
    "gazebo.msgs.Int", "gazebo.msgs.Collision",...
    "gazebo.msgs.ImagesStamped", "gazebo.msgs.Inertial",...
    "gazebo.msgs.Contact", "gazebo.msgs.Contacts",...
    "gazebo.msgs.IMUSensor", "gazebo.msgs.IMU",...
    "gazebo.msgs.Time", "gazebo.msgs.Wrench", 
    ];


gazebogenmsg(targetPath,"GazeboMessageList",msg_togenerate);

addpath(fullfile(targetPath,'install'));
savepath
out = packageGazeboPlugin('./../ssm_simulink_connect',targetPath)

disp("Package createde in ");
disp(out);
