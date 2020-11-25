clear;close all;clc
% Escolhe um arquivo '.bag'
bagfile = 'onlinefusion.bag';
bag = rosbag(bagfile);
bagselect0 = select(bag,'topic','/tf');
bagselect1 = select(bag,'topic','/dji_sdk/acceleration_ground_fused');
bagselect2 = select(bag,'topic','/dji_sdk/velocity');
tfmsgs = readMessages(bagselect0);
accmsgs = readMessages(bagselect1);
velmsgs = readMessages(bagselect2);

%% Convert to matlab 
fused_data = [];
gps_data = [];
rtk_data = [];
Accs = zeros(length(accmsgs),4);
Vels = zeros(length(velmsgs),4);
for i=1:length(tfmsgs)
   if ( strcmp(tfmsgs{i}.Transforms.ChildFrameId,'drone_gps'))
       time = tfmsgs{i}.Transforms.Header.Stamp.Sec + tfmsgs{i}.Transforms.Header.Stamp.Nsec*1e-9;
       gps_data = [gps_data;
       time tfmsgs{i}.Transforms.Transform.Translation.X tfmsgs{i}.Transforms.Transform.Translation.Y tfmsgs{i}.Transforms.Transform.Translation.Z];
   elseif(strcmp(tfmsgs{i}.Transforms.ChildFrameId,'fused'))
      time = tfmsgs{i}.Transforms.Header.Stamp.Sec + tfmsgs{i}.Transforms.Header.Stamp.Nsec*1e-9;
       fused_data = [fused_data;
       time tfmsgs{i}.Transforms.Transform.Translation.X tfmsgs{i}.Transforms.Transform.Translation.Y tfmsgs{i}.Transforms.Transform.Translation.Z]; 
       
   else
       time = tfmsgs{i}.Transforms.Header.Stamp.Sec + tfmsgs{i}.Transforms.Header.Stamp.Nsec*1e-9;
        rtk_data = [rtk_data;
       time tfmsgs{i}.Transforms.Transform.Translation.X tfmsgs{i}.Transforms.Transform.Translation.Y tfmsgs{i}.Transforms.Transform.Translation.Z];
   end   
end


for i=1:length(accmsgs)
    time = accmsgs{i}.Header.Stamp.Sec + accmsgs{i}.Header.Stamp.Nsec*1e-9;
 Accs(i,:) = [time accmsgs{i}.Vector.X accmsgs{i}.Vector.Y accmsgs{i}.Vector.Z];
end


for i=1:length(velmsgs)
    time = velmsgs{i}.Header.Stamp.Sec + velmsgs{i}.Header.Stamp.Nsec*1e-9;
 Vels(i,:) = [time velmsgs{i}.Vector.X velmsgs{i}.Vector.Y velmsgs{i}.Vector.Z];
end

%% Save
clc
datafile = strtok(bagfile,'.');
save(datafile,'gps_data','rtk_data','Accs','Vels','fused_data');
disp('dados salvo com sucesso');


