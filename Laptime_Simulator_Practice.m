%% Define Vehicle Capability

g = 9.81;
ayMax = 2*g;
axMax = 2*g;

%% Define Track Parameters

Turn1_Rad = 100;
Turn1_Arc = 334;

Turn2_Rad = 50;
Turn2_Arc = 147;

Straight1_Len = 250;
Straight2_Len = 500;
Straight3_Len = 250;

%% Define Mesh and Track Segments

nMeshSegments = 10; % Number of segments per track section

dxStraight1_Intvl = Straight1_Len/nMeshSegments; % Find mesh size
dxStraight1 = [zeros(1,1), ones(1,nMeshSegments)*dxStraight1_Intvl]'; % Vector of intervals
xStraight1 = [0:dxStraight1_Intvl:Straight1_Len]'; % Cumulative length of track

dxTurn1_Intvl = Turn1_Arc/nMeshSegments;
dxTurn1 = [zeros(1,1), ones(1,nMeshSegments)*dxTurn1_Intvl]';
xTurn1 = [0:dxTurn1_Intvl:Turn1_Arc]';

dxStraight2_Intvl = Straight2_Len/nMeshSegments;
dxStraight2 = [zeros(1,1), ones(1,nMeshSegments)*dxStraight2_Intvl]'; 
xStraight2 = [0:dxStraight2_Intvl:Straight2_Len]';

dxTurn2_Intvl = Turn2_Arc/nMeshSegments;
dxTurn2 = [zeros(1,1), ones(1,nMeshSegments)*dxTurn2_Intvl]';
xTurn2 = [0:dxTurn2_Intvl:Turn2_Arc]';

dxStraight3_Intvl = Straight3_Len/nMeshSegments;
dxStraight3 = [zeros(1,1), ones(1,nMeshSegments)*dxStraight3_Intvl]'; 
xStraight3 = [0:dxStraight3_Intvl:Straight3_Len]';

dxFullTrack = [dxStraight1; dxTurn1; dxStraight2; dxTurn2; dxStraight3];

xFullTrack = [zeros(1,length(dxFullTrack)-1)]';
for i = 2:length(dxFullTrack)
    xFullTrack(i) = xFullTrack(i-1) + dxFullTrack(i);
end

%% On-Throttle from Turn 1 Calculations

% Calculating speeds for Accelerating out of Turn 1

vThrotT1_Turn1 = ones(length(xTurn1),1)*sqrt(ayMax*Turn1_Rad);

vThrotT1_Straight2 = [vThrotT1_Turn1(end), zeros(1,length(dxStraight2)-1)]';
for i = 2:length(dxStraight2)
    vThrotT1_Straight2(i) = sqrt(vThrotT1_Straight2(i-1)^2 + 2*axMax*dxStraight2(i)); 
end

%% On-Throttle from Turn 2 Calculations

% Calculating speeds for Accelerating out of Turn 2

vThrotT2_Turn2 = ones(length(xTurn2),1)*sqrt(ayMax*Turn2_Rad);

vThrotT2_Straight3 = [vThrotT2_Turn2(end), zeros(1,length(dxStraight3)-1)]';
for i = 2:length(dxStraight3)
    vThrotT2_Straight3(i) = sqrt(vThrotT2_Straight3(i-1)^2 + 2*axMax*dxStraight3(i)); 
end

vThrotT2_Straight1 = [vThrotT2_Straight3(end), zeros(1,length(dxStraight1)-1)]';
for i = 2:length(dxStraight1)
    vThrotT2_Straight1(i) = sqrt(vThrotT2_Straight1(i-1)^2 + 2*axMax*dxStraight1(i)); 
end

%% On-Brakes to Turn 1 Calculations

% Calculating speeds for Braking into Turn 1

vBrakeT1_Straight1 = [zeros(1,length(dxStraight1)-1), vThrotT1_Turn1(1)]';
for i = length(dxStraight1)-1:-1:1
    vBrakeT1_Straight1(i) = sqrt(vBrakeT1_Straight1(i+1)^2 + 2*axMax*dxStraight1(i+1)); 
end

vBrakeT1_Straight3 = [zeros(1,length(dxStraight3)-1), vBrakeT1_Straight1(1)]';
for i = length(dxStraight3)-1:-1:1
    vBrakeT1_Straight3(i) = sqrt(vBrakeT1_Straight3(i+1)^2 + 2*axMax*dxStraight3(i+1)); 
end

%% On-Brakes to Turn 2 Calculations

% Calculating speeds for Braking into Turn 2

vBrakeT2_Straight2 = [zeros(1,length(dxStraight2)-1), vThrotT2_Turn2(1)]';
for i = length(dxStraight2)-1:-1:1
    vBrakeT2_Straight2(i) = sqrt(vBrakeT2_Straight2(i+1)^2 + 2*axMax*dxStraight2(i+1)); 
end

%% Take Minimum Speeds for all Track Mesh Segments

vFullTrack = [min(vThrotT2_Straight1,vBrakeT1_Straight1); vThrotT1_Turn1;...
    min(vThrotT1_Straight2,vBrakeT2_Straight2); vThrotT2_Turn2;...
    min(vThrotT2_Straight3,vBrakeT1_Straight3)];

%% Calculate Laptime

tFullTrack = dxFullTrack ./ vFullTrack;
Final_Laptime = sum(tFullTrack);

disp("Final Laptime = " + Final_Laptime);