%Sorry for messy code but it works fine
Cx = 0.19; %Drag coefficient
Cr = 0.0048; %Rolling coefficient
m = 1580; %vehicle mass with 2 person
A = 1.8; %front surface
eff = 0.85; %drivetrain efficiency
g = 9.81; %gravity
alpha = 0; %slope of road
dens = 1.25; %air density

bat_cap = 22; %battery capacity
charge = 6.6; %charging power rate
ch_eff = 0.95; %charging efficiency
fd=0; %other forces
SoE = [11; 13.6; 12]; %SoE of each car
speedEV1 = [0 3 5 6 8 9 14 14 12 11 11 10 10 6 2 0]; %EV-1's speed
speedEV2 = [0 5 8 8 12 10 9 8 7 3 2 0 ]; %EV-2's speed
speedEV3 = [0 5 8 8 15 12 10 8 8 7 7 7 7 6 4 2 0]; %EV-3's speed

%------------------------------------------------------------%
disp("EV-1");

v1 = speedEV1.*(1000/3600); %km/h to m/s
dt = 60; %Accel Force deltaT is 60 sec
deltaT = 1; %delta ts = s.*(1000/3600); %velocity m/s
square1 = v1.^2; %for Fa velocity needed squared
vel1 = diff(v1); %acceleration
zero = [0]; %after diff one var is empty so
dv1 =[zero vel1]; %first integer is must be = 0

f1 = m.*dv1./dt; %Newton law of motion
fa1 = 0.5*dens*A*Cx*square1; %Aerodynamic Drag Force
fr1 = m*Cr*g*cos(alpha); %Rolling Friction
fg1 = m*g*sin(alpha); %hill climbing force
ft1 = f1+fa1+fr1+fg1+fd; %Total Force
Pv1 = v1.*ft1; %Instantaneous vehicle power needed in watts
P1 = zeros(size(Pv1));  %Instantaneous Power
for i = 1:length(Pv1)
    if Pv1(i)>0 %if positive divide 0.85
        P1(i) = Pv1(i)./(eff*1000); %Instanteneous Power in kWs 
    elseif Pv1(i)<0 %if negative multiple 0.85
        P1(i) = Pv1(i).*eff./1000; %Instanteneous Power in kWs
    end
end

%discharge mode for EV
disp("Discharge rates");
for ii = 1:length(P1)
    SoE(1) = SoE(1) - ch_eff*P1(ii)*deltaT/60; %formula
    disp(SoE(1));
end
table1 = [v1; dv1; f1; fa1; ft1; Pv1; P1]; 

disp("Charging rates");
i=1; %counter
pout1(i)=0; %power consumption counter
t1 = 0; %2.25pm, EV-3 comes to D point
deltaT = 5/60; %delta is changing by 5 min. We need hour so divide 60
while SoE(1)<bat_cap
    if t1>=0 %2.25, EV-1 arives D
        SoE(1) = SoE(1) + ch_eff*charge*deltaT;
        t1=t1+5; %timer
        i=i+1; %ordinary counter
        pout1(i)=charge; %6.6kW power used
        disp(t1);
        disp(SoE(1));
        t_list1(i,:)=[SoE;t1];
    else
        t_list1(i,:)=[SoE; t1];
        SoE(1) = SoE(1);
        t1=t1+5; %timer
        i=i+1; %ordinary counter
        pout1(i)=0; %0 power used
    end
end
fprintf("EV-1 charged at %.2f o'clock\n", 2.25+t1*0.0166667); %0.0166667 is min to hour

%-----------------------------------------------------------%
disp("EV-2");

v2 = speedEV2.*(1000/3600); %km/h to m/s
dt = 60; %Accel Force deltaT is 60 sec
deltaT = 1; %delta ts = s.*(1000/3600); %velocity m/s
square2 = v2.^2; %for Fa velocity needed squared
ve2 = diff(v2); %acceleration 
zero = [0]; %after diff one var is empty so
dv2 =[zero ve2]; %first integer is needed = 0

f2 = m.*dv2./dt; %Newton law of motion
fa2 = 0.5*dens*A*Cx*square2; %Aerodynamic Drag Force
fr2 = m*Cr*g*cos(alpha); %Rolling Friction
fg2 = m*g*sin(alpha); %hill climbing force
ft2 = f2+fa2+fr2+fg2+fd; %Total Force
Pv2 = v2.*ft2; %Instantaneous vehicle power needed in watts
P2= zeros(size(Pv2));  %Instantaneous Power
for i = 1:length(Pv2)
    if Pv2(i)>0 %if positive divide 0.85
        P2(i) = Pv2(i)./(eff*1000); %Instanteneous Power in kWs
    elseif Pv2(i)<0 %if negative multiple 0.85
        P2(i) = Pv2(i).*eff./1000; %Instanteneous Power in kWs
    end
end

%discharge mode for EV
disp("Discharge rates");
for ii = 1:length(P2)
    SoE(2) = SoE(2) - ch_eff*P2(ii)*deltaT/60; %formula
    disp(SoE(2));
end
table2 = [v2; dv2; f2; fa2; ft2; Pv2; P2]; %

disp("Charging rates");
i=1; %ordinary counter
pout2=0; %power consumption counter
t2=0; %3.16pm, EV-3 comes to D point
deltaT = 5/60; %delta changing by 5min. We need hour so divided 60
while SoE(2)<bat_cap
    if t2>=21 %2.25+21 = 2.46, EV-2 arives D.
        SoE(2) = SoE(2) + ch_eff*charge*deltaT;
        t2=t2+5; %timer
        i=i+1; %ordinary counter
        pout2(i)=charge; %6.6kW power used
        disp(t2);
        disp(SoE(2));
        t_list2(i,:)=[SoE;t2];
    else
        t_list2(i,:)=[SoE;t2];
        SoE(2) = SoE(2);
        t2=t2+5; %timer
        i=i+1; %ordinary counter
        pout2(i)=0; %0kW power used
    end
end
fprintf("EV-2 charged at %.2f o'clock\n", 2.25+t2*0.0166667); %0.0166667 is min to hour

%--------------------------------------------------------------%
disp("EV-3");

v3 = speedEV3.*(1000/3600); %km/h to m/s
dt = 60; %Accel Force deltaT is 60 sec
deltaT = 1; %delta ts = s.*(1000/3600); %velocity m/s
square3 = v3.^2; %for Fa velocity needed squared
vel3 = diff(v3); %acceleration
zero = [0]; %after diff one var is empty so
dv3 =[zero vel3]; %first integer is needed = 0

f3 = m.*dv3./dt; %Newton law of motion
fa3 = 0.5*dens*A*Cx*square3; %Aerodynamic Drag Force
fr3 = m*Cr*g*cos(alpha); %Rolling Friction
fg3 = m*g*sin(alpha); %hill climbing force
ft3 = f3+fa3+fr3+fg3+fd; %Total Force
Pv3 = v3.*ft3; %Instantaneous vehicle power needed in watts
P3 = zeros(size(Pv3));  %Instantaneous Power
for i = 1:length(Pv3)
    if Pv3(i)>0 %if positive divide 0.85
        P3(i) = Pv3(i)./(eff*1000); %Instanteneous Power in kWs
    elseif Pv3(i)<0 %if negative multiple 0.85
        P3(i) = Pv3(i).*eff./1000; %Instanteneous Power in kWs
    end
end

%discharge mode for EV
disp("Discharge rates");
for ii = 1:length(P3)
    SoE(3)= SoE(3) - ch_eff*P3(ii)*deltaT/60;
    disp(SoE(3));
end
table3 = [v3; dv3; f3; fa3; ft3; Pv3; P3];

disp("Charging rates");
i=1; %ordinary counter
pout3=0; %power consumption counter
t3=0; %timer
deltaT=5/60; %delta is 5 min. we need hour so divided by 60
while SoE(3)<bat_cap
    if t3>=41 %2.25+41 = 3.06, EV-2 arives D
        SoE(3) = SoE(3) + ch_eff*charge*deltaT; %formula
        t3=t3+5; %timer
        i=i+1; %ordinary counter
        pout3(i)=charge; %6.6kW power used
        disp(t3);
        disp(SoE(3));
        t_list3(i,:)=[SoE;t3];
    else
        t_list3(i,:)=[SoE;t3];
        SoE(3) = SoE(3); 
        t3=t3+5; %time timer
        i=i+1; %ordinary counter
        pout3(i)=0; %power variation timer for EV-3
    end
end
fprintf("EV-3 charged at %.2f o'clock\n", 2.25+t3*0.0166667); %0.0166667 is min to hour

pout1(length(pout1)+1:length(pout3))=0; %the lengths was not equal to add each other
pout2(length(pout2)+1:length(pout3))=0; %the lengths was not equal to add each other
pout=pout1+pout2+pout3; %power variation
stairs(pout); %graph
xlabel("Time, 2.25 is 0, min");
ylabel("Total Power Consumption");
