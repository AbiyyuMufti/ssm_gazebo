folderPath = fullfile(pwd,'customMessage');
gazebogenmsg(folderPath,"GazeboMessageList", ["gazebo.msgs.Vector3d","gazebo.msgs.Image", "gazebo.msgs.Int"])
addpath('/home/amani/ssm_gazebo/src/ssm_plugin/customMessage/install')