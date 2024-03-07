%% Define Vehicle Capability

g = 9.81;
ayMax = 2*g;
axThrotMax = 2*g;
axBrakeMax = 2*g;

vehTopSpeed = 400/3.6;

%% Plot GG Diagram of Vehicle Limit

FyGG = linspace(0,ayMax,100);
FxGG_Throt = sqrt((1-(FyGG/ayMax).^2)*axThrotMax^2);
FxGG_Brake = sqrt((1-(FyGG/ayMax).^2)*axBrakeMax^2);

Gy_reference = linspace(0,g,100);
Gx_reference = sqrt(g^2-Gy_reference.^2);

xline(0,'HandleVisibility','off')
hold on
yline(0,'HandleVisibility','off')
plot(Gy_reference,Gx_reference, "LineWidth",1,"Color","k")
plot(Gy_reference,-Gx_reference, "LineWidth",1,"Color","k",'HandleVisibility','off')
plot(-Gy_reference,Gx_reference, "LineWidth",1,"Color","k",'HandleVisibility','off')
plot(-Gy_reference,-Gx_reference, "LineWidth",1,"Color","k",'HandleVisibility','off')
plot(FyGG, FxGG_Throt, "LineWidth",1,"Color","b")
plot(-FyGG, FxGG_Throt, "LineWidth",1,"Color","b",'HandleVisibility','off')
plot(FyGG, -FxGG_Brake, "LineWidth",1,"Color","b",'HandleVisibility','off')
plot(-FyGG, -FxGG_Brake, "LineWidth",1,"Color","b",'HandleVisibility','off')
hold off
grid on
title("Vehicle GG Plot")
xlabel("Lateral Acceleration Capability (ms^{-2})")
ylabel("Longitudinal Acceleration Capability (ms^{-2})")
ylim([-ceil(axBrakeMax/10)*10, ceil(axThrotMax/10)*10]);
%xlim([-ceil(ayMax/10)*10, ceil(ayMax/10)*10]);
legend('1g Reference','Vehicle Limit')
axis equal

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

%% Define Full Track Radius Vector

radFullTrack = [zeros(length(xStraight1),1); ones(length(xTurn1),1).*Turn1_Rad;...
        zeros(length(xStraight2),1); ones(length(xTurn2),1).*Turn2_Rad;...
        zeros(length(xStraight3),1)];

%% On-Throttle from Turn 1 Calculations

% Calculating speeds for Accelerating out of Turn 1

vThrotT1_Turn1 = ones(length(xTurn1),1)*sqrt(ayMax*Turn1_Rad);

vThrotT1_Straight2 = [vThrotT1_Turn1(end), zeros(1,length(dxStraight2)-1)]';
for i = 2:length(dxStraight2)
    vThrotT1_Straight2(i) = sqrt(vThrotT1_Straight2(i-1)^2 + 2*axThrotMax*dxStraight2(i)); 
end

%% On-Throttle from Turn 2 Calculations

% Calculating speeds for Accelerating out of Turn 2

vThrotT2_Turn2 = ones(length(xTurn2),1)*sqrt(ayMax*Turn2_Rad);

vThrotT2_Straight3 = [vThrotT2_Turn2(end), zeros(1,length(dxStraight3)-1)]';
for i = 2:length(dxStraight3)
    vThrotT2_Straight3(i) = sqrt(vThrotT2_Straight3(i-1)^2 + 2*axThrotMax*dxStraight3(i)); 
end

vThrotT2_Straight1 = [vThrotT2_Straight3(end), zeros(1,length(dxStraight1)-1)]';
for i = 2:length(dxStraight1)
    vThrotT2_Straight1(i) = sqrt(vThrotT2_Straight1(i-1)^2 + 2*axThrotMax*dxStraight1(i)); 
end

%% On-Brakes to Turn 1 Calculations

% Calculating speeds for Braking into Turn 1

vBrakeT1_Straight1 = [zeros(1,length(dxStraight1)-1), vThrotT1_Turn1(1)]';
for i = length(dxStraight1)-1:-1:1
    vBrakeT1_Straight1(i) = sqrt(vBrakeT1_Straight1(i+1)^2 + 2*axBrakeMax*dxStraight1(i+1)); 
end

vBrakeT1_Straight3 = [zeros(1,length(dxStraight3)-1), vBrakeT1_Straight1(1)]';
for i = length(dxStraight3)-1:-1:1
    vBrakeT1_Straight3(i) = sqrt(vBrakeT1_Straight3(i+1)^2 + 2*axBrakeMax*dxStraight3(i+1)); 
end

%% On-Brakes to Turn 2 Calculations

% Calculating speeds for Braking into Turn 2

vBrakeT2_Straight2 = [zeros(1,length(dxStraight2)-1), vThrotT2_Turn2(1)]';
for i = length(dxStraight2)-1:-1:1
    vBrakeT2_Straight2(i) = sqrt(vBrakeT2_Straight2(i+1)^2 + 2*axBrakeMax*dxStraight2(i+1)); 
end

%% Take Minimum Speeds for all Track Mesh Segments

vFullTrack = [min(vThrotT2_Straight1,vBrakeT1_Straight1); vThrotT1_Turn1;...
    min(vThrotT1_Straight2,vBrakeT2_Straight2); vThrotT2_Turn2;...
    min(vThrotT2_Straight3,vBrakeT1_Straight3)];

for i = 1:length(vFullTrack)
    if vFullTrack(i) > vehTopSpeed
        vFullTrack(i) = vehTopSpeed;
    end
end

%% Calculate Lateral and Longitudinal Accelerations

tFullTrack = dxFullTrack ./ vFullTrack;

% LatAcc Calculation for each Turn
LatAcc = zeros(length(vFullTrack),1);
for i = 1:length(radFullTrack)
    if radFullTrack(i) == 0
        LatAcc(i) = 0;
    else
        LatAcc(i) = vFullTrack(i).^2 ./ radFullTrack(i);
    end
end

% LongAcc Calculation
LongAcc = zeros(length(vFullTrack),1);
for i = 1:length(tFullTrack)
    if tFullTrack(i) == 0
        LongAcc(i) = 0;
    else
        LongAcc(i) = (vFullTrack(i) - vFullTrack(i-1))./tFullTrack(i);
    end
end


%% Plot Speed and Acceleration Traces

figure(2)
plot(xFullTrack,vFullTrack)
grid on
title("Full Lap Speed Trace")
xlabel("Lap Distance (m)")
ylabel("Vechicle Speed (m/s)")
xticks(0:100:1500);
ylim([0, ceil(max(vFullTrack)/10)*10]);

figure(3)
plot(xFullTrack,LatAcc)
hold on
plot(xFullTrack,LongAcc)
grid on
title("Full Lap Acceleration Trace")
xlabel("Lap Distance (m)")
ylabel("Vechicle Acceleration (ms^{-2})")
xticks(0:100:1500);
legend('LatAcc', 'LongAcc',"Location","southeast")

%% Calculate and Display Laptime and Maximum Speed

Final_Laptime = sum(tFullTrack);

vMax = max(vFullTrack);

disp("Final Laptime = " + Final_Laptime + " s");
disp("Maximum Speed = " + vMax + " m/s " + "(" + vMax*3.6 + " kph)");
