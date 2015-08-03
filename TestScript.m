
%--------------------------------------------------------------------------
%           ___________Dataset Example Script Usage_________
%                        Copyright (C) 2010-2013
%
%--------------------------------------------------------------------------
% This script intents to facilitate the usage of a dataset collected using 
% an industrial robot and a set of IMU/MARG sensors retrieving raw data.
%                                                  
% Can be used to test filter fusion algorithms for orientation and position
% estimate.
%                                                    
% Data was collected in an non-perfect environment with many unmodelled 
% magnetic distortions and accelerations.       
%
% https://github.com/miguelrasteiro/IMU_dataset
%
%-DESCRIPTION--------------------------------------------------------------
%
% Simulation Run (main cycle)
%
%-DISCLAIMER---------------------------------------------------------------
% This program is distributed in the hope that it will be useful,but
% WITHOUT ANY WARRANTY;
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR
% A PARTICULAR PURPOSE.
% You can use this source code without licensing fees only for
% NON-COMMERCIAL research and EDUCATIONAL purposes only.
% You cannot repost this file without prior written permission from 
% the authors.
%
%-AUTHORS------------------------------------------------------------------
%   Miguel Rasteiro*
%   Luis Conde Bento*
%   Hugo Costelha*
%   Pedro Assunção*
%
% *School of Technology and Management - Polytechnic Institute of Leiria
%--------------------------------------------------------------------------

clear all;
close all;
clc;

addpath ('quaternion_library');

%% Data Load 
% Folder system is has it follows: DataSets/velocity/path/sensor
%
%    6 different velocities: -> 150  mm/s (eV50p) aka safe mode
%                            -> 300  mm/s (V_300)
%                            -> 500  mm/s (V_500)
%                            -> 800  mm/s (V_800)
%                            -> 1000 mm/s (V1000)
%                            -> 1500 mm/s (V1500)
%    4 different paths: -> path1 (Sequencia1)
%                       -> path2 (Sequencia2) - for position (not available for all senssors)
%                       -> path3 (Sequencia3)
%                       -> path4 (Sequencia4)
%            Note: increasing complexity in rotations and dislocations
%                  plot path to observe (last code section)
%
%    4 sensors retrieving raw data: -> LSM9DS0 (LSM9DS0)
%                                   -> MPU9150 (MPU9150)
%                                   -> MPU6500 + RM3100 (MPU6500RM3100)
%                                   -> MPU6050 + RM3100 (MPU6050RM3100)
%    4 reference sensors retrieving orientation: 
%                                   -> PNI SENTRAL M&M Blue (SENTRAL)
%                                   -> XSENS MTi-30 (XSENS)
%                                   -> MPU6500 (MPU6500DMP)
%                                   -> MPU6050 (MPU6050DMP)

% Reference Sensors (Orientation only)
% retrieve a quaternion
MPU6050DMP   = load ('DataSets/eV50p/Sequencia1/MPU6050DMP',   'MATLABtime', 'ABBquaternion', 'ABBposition', 'SENSquat'); % Invensense IMFA
MPU6500DMP   = load ('DataSets/eV50p/Sequencia1/MPU6500DMP',   'MATLABtime', 'ABBquaternion', 'ABBposition', 'SENSquat'); % Invensense IMFA
SENTRAL      = load ('DataSets/eV50p/Sequencia1/SENTRAL',      'MATLABtime', 'ABBquaternion', 'ABBposition', 'SENSquat'); % PNI SENTRAL M&M Blue module
XSENS        = load ('DataSets/eV50p/Sequencia3/XSENS',        'MATLABtime', 'ABBquaternion', 'ABBposition', 'SENSquat'); % XSENS MTi-30
% Raw data:
%   - Acc in G's
%   - Gyro in deg/s
%   - Mag in miliGauss for the MPU9150 and in uT for the other sensors
LSM9DS0      = load ('DataSets/eV50p/Sequencia1/LSM9DS0',      'MATLABtime', 'ABBquaternion', 'ABBposition', 'SENSgyro', 'SENSacc', 'SENSmag'); % ST
MPU9150      = load ('DataSets/eV50p/Sequencia1/MPU9150',      'MATLABtime', 'ABBquaternion', 'ABBposition', 'SENSgyro', 'SENSacc', 'SENSmag'); % Invensense
MPU6050RM3100= load ('DataSets/eV50p/Sequencia1/MPU6050RM3100','MATLABtime', 'ABBquaternion', 'ABBposition', 'SENSgyro', 'SENSacc', 'SENSmag'); % Invensense + PNI
MPU6500RM3100= load ('DataSets/eV50p/Sequencia1/MPU6500RM3100','MATLABtime', 'ABBquaternion', 'ABBposition', 'SENSgyro', 'SENSacc', 'SENSmag'); % Invensense + PNI

%% Data Treatment 
% Sensor axis alignement with ABB axis
% MPU9150
aux = MPU9150.SENSmag(1,:);                 % magx = ABBy
MPU9150.SENSmag(1,:)= MPU9150.SENSmag(2,:); % magy = ABBx
MPU9150.SENSmag(2,:)= aux;
MPU9150.SENSmag(3,:)=-MPU9150.SENSmag(3,:); % magz = -ABBz
% MPU6050 + RM3100
aux = MPU6050RM3100.SENSmag(1,:);                        % magx = -ABBy
MPU6050RM3100.SENSmag(1,:)= -MPU6050RM3100.SENSmag(2,:); % magy = ABBx
MPU6050RM3100.SENSmag(2,:)= -aux;
MPU6050RM3100.SENSmag(3,:)= -MPU6050RM3100.SENSmag(3,:); % magz = -ABBz
% MPU6500 + RM3100
aux = MPU6500RM3100.SENSmag(1,:);                        % magx = -ABBy
MPU6500RM3100.SENSmag(1,:)= -MPU6500RM3100.SENSmag(2,:); % magy = ABBx
MPU6500RM3100.SENSmag(2,:)= -aux;
MPU6500RM3100.SENSmag(3,:)= -MPU6500RM3100.SENSmag(3,:); % magz = -ABBz
% LSM9DS0
aux = LSM9DS0.SENSgyro(1,:);                    % gyrox = ABBy
LSM9DS0.SENSgyro(1,:)= -LSM9DS0.SENSgyro(2,:);  % gyroy = -ABBx
LSM9DS0.SENSgyro(2,:)= aux;
aux = LSM9DS0.SENSacc(1,:);                     % accx = ABBy
LSM9DS0.SENSacc(1,:) = -LSM9DS0.SENSacc(2,:);   % accy = -ABBx
LSM9DS0.SENSacc(2,:) = aux;
aux = LSM9DS0.SENSmag(1,:);                     % magx = ABBy
LSM9DS0.SENSmag(1,:)= -LSM9DS0.SENSmag(2,:);    % magy = -ABBx
LSM9DS0.SENSmag(2,:)= aux;
LSM9DS0.SENSmag(3,:) = -LSM9DS0.SENSmag(3,:);   % magz = -ABBz
% DMP axis alignment
MPU6050DMP.SENSquat(4,:) = -MPU6050DMP.SENSquat (4,:);
MPU6500DMP.SENSquat(4,:) = -MPU6500DMP.SENSquat (4,:);
% SENTRAL axis alignment
aux=SENTRAL.SENSquat(3,:);
SENTRAL.SENSquat(3,:)=  SENTRAL.SENSquat(2,:);
SENTRAL.SENSquat(2,:)=  aux;
SENTRAL.SENSquat(4,:)=  SENTRAL.SENSquat(4,:);
% XSENS axis alignment
XSENS.SENSquat=XSENS.SENSquat';

%% Test Algorithm 
% Example of data fusion
% Madgwick Filter (Complementary filter)
% Initializations
MPU9150.AHRS = MahonyAHRS ('SamplePeriod' , 0.01, 'Kp', 0.01, 'Ki', 0.001);
MPU9150.quat=zeros(length(MPU9150.MATLABtime),4);
MPU9150.quat=[1 0 0 0];
MPU9150.SENSgyro = deg2rad(MPU9150.SENSgyro);
% Filtering
for i = 1:length(MPU9150.MATLABtime)
    MPU9150.AHRS.Update(MPU9150.SENSgyro(:,i)',MPU9150.SENSacc(:,i)',MPU9150.SENSmag(:,i)');
    MPU9150.quat(i,:) = MPU9150.AHRS.Quaternion;
end
        
%% Results adaptation to ABB ground truth 
% May vary according to filter implementation or test 
% Merely exemplificative

% align initial orientation with ground-truth
Qt=quaternProd(SENTRAL.ABBquaternion(:,10)',quaternConj(SENTRAL.SENSquat(:,10)'));
SENTRAL.SENSquat=quaternProd(SENTRAL.SENSquat',Qt);

Qt=quaternProd(XSENS.ABBquaternion(:,10)',quaternConj(XSENS.SENSquat(:,10)'));
XSENS.SENSquat=quaternProd(XSENS.SENSquat',Qt);

Qt = quaternProd(MPU9150.ABBquaternion(:,5)',quaternConj(MPU9150.quat(5,:)));
MPU9150.quat = quaternProd(MPU9150.quat,Qt);
    
% Compatibilize with industrial robot data 
% ABB's real component of the retrieved quaternion is never negative, 
% instead, they invert all signals

% PNI SENTRAL M&M Blue example
for i=1:length(SENTRAL.SENSquat)
    if SENTRAL.SENSquat(i,1)<0
        SENTRAL.SENSquat(i,:)=SENTRAL.SENSquat(i,:)*(-1);
    end
end
% XSENS example
for i=1:length(XSENS.SENSquat)
    if XSENS.SENSquat(i,1)<0
        XSENS.SENSquat(i,:)=XSENS.SENSquat(i,:)*(-1);
    end
end
% MPU example
MPU9150.quat = quaternConj(MPU9150.quat); 
for i=1:length(MPU9150.quat)
    if MPU9150.quat(i,1)<0
        MPU9150.quat(i,:)=MPU9150.quat(i,:)*(-1);
    end
end
% transpose ()
MPU9150.quat = MPU9150.quat';
SENTRAL.SENSquat = quaternConj(SENTRAL.SENSquat);
SENTRAL.SENSquat = SENTRAL.SENSquat';
XSENS.SENSquat = XSENS.SENSquat';

%% Get some Results 

    %%%%%%%%%%%%%%%%%%%%%%%
    % MPU9150 %%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%

% RPY Angles conversion
[MPU9150.yawSENS, MPU9150.pitchSENS, MPU9150.rollSENS] = quat2angle(MPU9150.quat');
[MPU9150.yawABB, MPU9150.pitchABB, MPU9150.rollABB] = quat2angle(MPU9150.ABBquaternion');
% from rad to deg
MPU9150.yawSENS=radtodeg(MPU9150.yawSENS); MPU9150.pitchSENS=radtodeg(MPU9150.pitchSENS); MPU9150.rollSENS=radtodeg(MPU9150.rollSENS);
MPU9150.yawABB=radtodeg(MPU9150.yawABB);   MPU9150.pitchABB=radtodeg(MPU9150.pitchABB);   MPU9150.rollABB=radtodeg(MPU9150.rollABB);
% Prevent error due to angles proprieties
for i = 1 : length(MPU9150.MATLABtime)
    if (MPU9150.rollABB(i)-MPU9150.rollSENS(i)) > 300
        MPU9150.rollSENS(i) = MPU9150.rollSENS(i) +360;
    elseif (MPU9150.rollABB(i)-MPU9150.rollSENS(i)) < - 300
        MPU9150.rollSENS(i) = MPU9150.rollSENS(i) -360;
    end
    if (MPU9150.rollABB(i)-MPU9150.pitchSENS(i)) > 150
        MPU9150.pitchSENS(i) = MPU9150.pitchSENS(i) +180;
    elseif (MPU9150.pitchABB(i)-MPU9150.pitchSENS(i)) < - 150
        MPU9150.pitchSENS(i) = MPU9150.pitchSENS(i) -180;
    end
    if (MPU9150.yawABB(i)-MPU9150.yawSENS(i)) > 300
        MPU9150.yawSENS(i) = MPU9150.yawSENS(i) +360;
    elseif (MPU9150.yawABB(i)-MPU9150.yawSENS(i)) < - 300
        MPU9150.yawSENS(i) = MPU9150.yawSENS(i) -360;
    end
end
% calculate sample error
MPU9150.error = [MPU9150.rollABB-MPU9150.rollSENS  MPU9150.pitchABB-MPU9150.pitchSENS  MPU9150.yawABB-MPU9150.yawSENS];

% Dont consider Errors superior to a certain limit
error_limit = 50; % in degrees
j=length(MPU9150.error);
i=1; 
while i<j
    if abs(MPU9150.error(i,1)) > error_limit
        MPU9150.error(i,:) = []; j=j-1; i=i-1;
        if i<1, i=1; end
    end
    if abs(MPU9150.error(i,2)) > error_limit
        MPU9150.error(i,:) = []; j=j-1; i=i-1; 
        if i<1, i=1; end
    end
    if abs(MPU9150.error(i,3)) > error_limit
        MPU9150.error(i,:) = []; j=j-1; i=i-1; 
        if i<1, i=1; end
    end
    i=i+1;
end
%Save error mean, std, and rms
MPU9150.mean_error = mean(MPU9150.error);
MPU9150.std_error  = std(MPU9150.error);
MPU9150.rms_error  = rms(MPU9150.error);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SENTRAL (same process) %%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
[SENTRAL.yawSENS, SENTRAL.pitchSENS, SENTRAL.rollSENS] = quat2angle(SENTRAL.SENSquat');
[SENTRAL.yawABB, SENTRAL.pitchABB, SENTRAL.rollABB] = quat2angle(SENTRAL.ABBquaternion');
%to deg
SENTRAL.yawSENS=radtodeg(SENTRAL.yawSENS); SENTRAL.pitchSENS=radtodeg(SENTRAL.pitchSENS); SENTRAL.rollSENS=radtodeg(SENTRAL.rollSENS);
SENTRAL.yawABB=radtodeg(SENTRAL.yawABB);   SENTRAL.pitchABB=radtodeg(SENTRAL.pitchABB);   SENTRAL.rollABB=radtodeg(SENTRAL.rollABB);
% circularity
for i = 1 : length(SENTRAL.MATLABtime)
    if (SENTRAL.rollABB(i)-SENTRAL.rollSENS(i)) > 300
        SENTRAL.rollSENS(i) = SENTRAL.rollSENS(i) +360;
    elseif (SENTRAL.rollABB(i)-SENTRAL.rollSENS(i)) < - 300
        SENTRAL.rollSENS(i) = SENTRAL.rollSENS(i) -360;
    end
    if (SENTRAL.rollABB(i)-SENTRAL.pitchSENS(i)) > 150
        SENTRAL.pitchSENS(i) = SENTRAL.pitchSENS(i) +180;
    elseif (SENTRAL.pitchABB(i)-SENTRAL.pitchSENS(i)) < - 150
        SENTRAL.pitchSENS(i) = SENTRAL.pitchSENS(i) -180;
    end
    if (SENTRAL.yawABB(i)-SENTRAL.yawSENS(i)) > 300
        SENTRAL.yawSENS(i) = SENTRAL.yawSENS(i) +360;
    elseif (SENTRAL.yawABB(i)-SENTRAL.yawSENS(i)) < - 300
        SENTRAL.yawSENS(i) = SENTRAL.yawSENS(i) -360;
    end
end
% SAve errors
SENTRAL.error = [SENTRAL.rollABB-SENTRAL.rollSENS  SENTRAL.pitchABB-SENTRAL.pitchSENS  SENTRAL.yawABB-SENTRAL.yawSENS];

% Dont consider Errors superior to a certain limit
error_limit = 50;
j = length ( SENTRAL.error );
i=1; 
while i<j
    if abs(SENTRAL.error(i,1))>error_limit
        SENTRAL.error(i,:)=[]; j=j-1; i=i-1;
        if i<1, i=1; end
    end
    if abs(SENTRAL.error(i,2))>error_limit
        SENTRAL.error(i,:)=[]; j=j-1; i=i-1;
        if i<1, i=1; end
    end
    if abs(SENTRAL.error(i,3))>error_limit
        SENTRAL.error(i,:)=[]; j=j-1; i=i-1;
        if i<1, i=1; end
    end
    i=i+1;
end

SENTRAL.mean_error = mean(SENTRAL.error);
SENTRAL.std_error  = std(SENTRAL.error);
SENTRAL.rms_error  = rms(SENTRAL.error);

    %%%%%%%%%%%%%%%%%%%%%%
    % XSENS  %%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%
    
[XSENS.yawSENS, XSENS.pitchSENS, XSENS.rollSENS] = quat2angle(XSENS.SENSquat');
[XSENS.yawABB, XSENS.pitchABB, XSENS.rollABB] = quat2angle(XSENS.ABBquaternion');
XSENS.yawSENS=radtodeg(XSENS.yawSENS); XSENS.pitchSENS=radtodeg(XSENS.pitchSENS); XSENS.rollSENS=radtodeg(XSENS.rollSENS);
XSENS.yawABB=radtodeg(XSENS.yawABB);   XSENS.pitchABB=radtodeg(XSENS.pitchABB);   XSENS.rollABB=radtodeg(XSENS.rollABB);

for i = 1 : length(XSENS.MATLABtime)
    if (XSENS.rollABB(i)-XSENS.rollSENS(i)) > 300
        XSENS.rollSENS(i) = XSENS.rollSENS(i) +360;
    elseif (XSENS.rollABB(i)-XSENS.rollSENS(i)) < - 300
        XSENS.rollSENS(i) = XSENS.rollSENS(i) -360;
    end
    if (XSENS.rollABB(i)-XSENS.pitchSENS(i)) > 150
        XSENS.pitchSENS(i) = XSENS.pitchSENS(i) +180;
    elseif (XSENS.pitchABB(i)-XSENS.pitchSENS(i)) < - 150
        XSENS.pitchSENS(i) = XSENS.pitchSENS(i) -180;
    end
    if (XSENS.yawABB(i)-XSENS.yawSENS(i)) > 300
        XSENS.yawSENS(i) = XSENS.yawSENS(i) +360;
    elseif (XSENS.yawABB(i)-XSENS.yawSENS(i)) < - 300
        XSENS.yawSENS(i) = XSENS.yawSENS(i) -360;
    end
end

XSENS.error = [XSENS.rollABB-XSENS.rollSENS  XSENS.pitchABB-XSENS.pitchSENS  XSENS.yawABB-XSENS.yawSENS];

% Dont consider Errors superior to a certain limit
error_limit = 50;
j = length ( XSENS.error );
i=1; 
while i<j
    if abs(XSENS.error(i,1))>error_limit
        XSENS.error(i,:)=[]; j=j-1; i=i-1;
        if i<1, i=1; end
    end
    if abs(XSENS.error(i,2))>error_limit
        XSENS.error(i,:)=[]; j=j-1; i=i-1;
        if i<1, i=1; end
    end
    if abs(XSENS.error(i,3))>error_limit
        XSENS.error(i,:)=[]; j=j-1; i=i-1;
        if i<1, i=1; end
    end
    i=i+1;
end
    
XSENS.mean_error = mean(XSENS.error);
XSENS.std_error  = std(XSENS.error);
XSENS.rms_error  = rms(XSENS.error);

%% Plot Results 
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%% GroundTruth versus meaured %%%%%%%%%%%%%%%%%%
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i=1; 
figure('Name', 'Angles with Mahony Filter on MPU9150'); 
subplot(3,1,i); grid on; hold on; ylabel ('roll (º)')
plot (MPU9150.MATLABtime, MPU9150.rollABB,'k')
plot (MPU9150.MATLABtime, MPU9150.rollSENS,'r')
hold off; 
i=i+1; 
subplot(3,1,i); grid on; hold on; ylabel ('pitch (º)')
plot (MPU9150.MATLABtime, MPU9150.pitchABB,'k')
plot (MPU9150.MATLABtime, MPU9150.pitchSENS,'r')
hold off; 
i=i+1; 
subplot(3,1,i); grid on; hold on; ylabel ('yaw (º)'); xlabel ('time (s)')
plot (MPU9150.MATLABtime, MPU9150.yawABB,'k')
plot (MPU9150.MATLABtime, MPU9150.yawSENS,'r')
legend('ABB','MPU9150')
hold off

i=1; 
figure('Name', 'Angles from XSENS'); 
subplot(3,1,i); grid on; hold on; ylabel ('roll (º)')
plot (XSENS.MATLABtime, XSENS.rollABB,'k')
plot (XSENS.MATLABtime, XSENS.rollSENS,'r')
hold off; 
i=i+1; 
subplot(3,1,i); grid on; hold on; ylabel ('pitch (º)')
plot (XSENS.MATLABtime, XSENS.pitchABB,'k')
plot (XSENS.MATLABtime, XSENS.pitchSENS,'r')
hold off; 
i=i+1; 
subplot(3,1,i); grid on; hold on; ylabel ('yaw (º)'); xlabel ('time (s)')
plot (XSENS.MATLABtime, XSENS.yawABB,'k')
plot (XSENS.MATLABtime, XSENS.yawSENS,'r')
legend('ABB','XSENS')
hold off

          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%%%%%%%%%%%%%%%% Error plot Example %%%%%%%%%%%%%%%%%%%%
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i=1; 
figure('Name', 'Angle Error with SENTRAL')
subplot(3,1,i); grid on; hold on; ylabel ('roll (º)');
plot (SENTRAL.error(:,i),'r')
hold off; i=i+1; subplot(3,1,i); grid on ;hold on; ylabel ('pitch (º)')
plot (SENTRAL.error(:,i),'g'); 
hold off; i=i+1; subplot(3,1,i); grid on; hold on; ylabel ('yaw (º)'); xlabel ('sample')
plot (SENTRAL.error(:,i),'b')
hold off

          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%% Statistical plot Examples %%%%%%%%%%%%%%%%%%
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Mean_Data= [ MPU9150.mean_error; SENTRAL.mean_error; XSENS.mean_error];
Std_Data = [ MPU9150.std_error;  SENTRAL.std_error;  XSENS.std_error ];
Rms_Data = [ MPU9150.rms_error;  SENTRAL.rms_error;  XSENS.rms_error ];
names =    ['MPU9150';          'SENTRAL';        '  XSENS'          ];

[numberofsensors,~]=size(names);

% Mean and std
figure('Name', 'Mean and Std Results');

hold on; grid on; ylabel ('Degrees');
axis([0 numberofsensors+1 -20 20])
errorbar((1:numberofsensors)-0.1,Mean_Data(:,1),Std_Data(:,1),'rx','linewidth',2)
set(gca,'xticklabel', names, 'xtick',1:numberofsensors);

hold on; grid on; ylabel ('Degrees');
axis([0 numberofsensors+1 -20 20])
errorbar((1:numberofsensors),Mean_Data(:,2),Std_Data(:,2),'gx','linewidth',2)
set(gca,'xticklabel', names, 'xtick',1:numberofsensors);
hold on; grid on; ylabel ('Degrees');

axis([0 numberofsensors+1 -20 20])
errorbar((1:numberofsensors)+0.1,Mean_Data(:,3),Std_Data(:,3),'bx','linewidth',2)
set(gca,'xticklabel', names, 'xtick',1:numberofsensors);
hold off

% RMSE
figure('Name', 'RMSE Results');

subplot(3,1,1);  hold on; grid on; ylabel ('Degrees');
stem(Rms_Data(:,1),'r-o','linewidth',2)
set(gca,'xticklabel', names, 'xtick',1:numberofsensors);
axis([0 numberofsensors+1 0 7])

subplot(3,1,2);  hold on; grid on; ylabel ('Degrees');
stem(Rms_Data(:,2),'g-o','linewidth',2)
set(gca,'xticklabel', names, 'xtick',1:numberofsensors);
axis([0 numberofsensors+1 0 6])

subplot(3,1,3);  hold on; grid on; ylabel ('Degrees');
stem(Rms_Data(:,3),'b-o','linewidth',2)
set(gca,'xticklabel', names, 'xtick',1:numberofsensors);
axis([0 numberofsensors+1 0 10])

%% Full path plot example

% Draw data:
figure('Name', 'GroundTruth Position and Orientation');
title('Pos');
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);
grid on

ox = SENTRAL.ABBposition(1,:)';   oy = SENTRAL.ABBposition(2,:)';   oz = SENTRAL.ABBposition(3,:)';
R = quat2dcm((SENTRAL.ABBquaternion'));
ux = permute(R(1,1,:),[3 2 1]);   vx = permute(R(2,1,:),[3 2 1]);   wx = permute(R(3,1,:),[3 2 1]);
uy = permute(R(1,2,:),[3 2 1]);   vy = permute(R(2,2,:),[3 2 1]);   wy = permute(R(3,2,:),[3 2 1]);
uz = permute(R(1,3,:),[3 2 1]);   vz = permute(R(2,3,:),[3 2 1]);   wz = permute(R(3,3,:),[3 2 1]);
hold on
lx = line(ox,oy,oz,'Color','k');
quivX = quiver3(ox,oy,oz,ux,vx,wx, 'r', 'AutoScaleFactor', 2);
quivY = quiver3(ox,oy,oz,uy,vy,wy, 'g', 'AutoScaleFactor', 2);
quivZ = quiver3(ox,oy,oz,uz,vz,wz, 'b', 'AutoScaleFactor', 2);
axis equal
