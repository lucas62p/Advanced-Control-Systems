function call_plot(q)
%#codegen
% Define the extrinsic functions

coder.extrinsic('pause')

if ~ishandle(1) %Initialize Figure
     figure(1);
end

clf

PlotS2A(q);
pause(0.02)
end

function PlotS2A(q,c)

q1 = q(1);
q2 = q(2);
c  = q(3); 
r = 0.5; % Wheel radius
l1 = 2;
p0 = [q1;0;0]; % Base
Rotz_q2 = RotZ(q2);
p2 = Rotz_q2*[0;1;0] * l1 + p0; % Tip of the Arm / Head
AngleWheel = -q1/r;
RotWheel = RotZ(AngleWheel);

%% Arm
line([p0(1);p2(1)],[p0(2);p2(2)],  'LineWidth',10,   'Color','red');
hold on 

%% Cart

rectangle('Position',[p0(1)-r,p0(2)-0.2,2*r,2*r
    ],...
    'Curvature',[1 1],'FaceColor','black')
% Wheel
p_WheelCenter = p0+[0;r-0.2;0];
p1_Wheel = p_WheelCenter + RotWheel*[0;r*0.95;0];
p2_Wheel = p_WheelCenter + RotWheel*RotZ(120*pi/180)*[0;r*0.95;0];
p3_Wheel = p_WheelCenter + RotWheel*RotZ(-120*pi/180)*[0;r*0.95;0];

plot(p_WheelCenter(1),p_WheelCenter(2),'o','LineWidth',7,'Color','blue')
line([p_WheelCenter(1);p1_Wheel(1)],[p_WheelCenter(2);p1_Wheel(2)],...
    'LineWidth',5,   'Color','red');
line([p_WheelCenter(1);p2_Wheel(1)],[p_WheelCenter(2);p2_Wheel(2)],...
    'LineWidth',5,   'Color','red');
line([p_WheelCenter(1);p3_Wheel(1)],[p_WheelCenter(2);p3_Wheel(2)],...
    'LineWidth',5,   'Color','red');
%% Head
 p_head_left = p2-Rotz_q2*[0.2;0;0];
 p_head_right = p2+Rotz_q2*[0.2;0;0];
 plot(p_head_right(1),p_head_right(2),'*','LineWidth',5,   'Color','blue')
 line([p_head_left(1);p_head_right(1)],[p_head_left(2);p_head_right(2)],...
     'LineWidth',10,   'Color','black');
hold on


%% Ground
% c1 = fix(rem(0.5*(abs(c)+1),2))-1 ;
% 2*floor((a+1)/2)-1;

c1 = (2*floor((c+1)/2)-1)-6

for i=1:7
  
   if  mod((c1-1)/2,2)
   rectangle('Position',[c1,-0.2-0.1,2,0.1],'FaceColor','black') 
   else
   rectangle('Position',[c1,-0.2-0.1,2,0.1],'FaceColor',[0.6 0.6 0.7])
   end
   c1 = c1 + 2;
   
end

% rectangle('Position',[c1-7,-0.2-0.1,2,0.1],'FaceColor','black')
% rectangle('Position',[c1-5,-0.2-0.1,2,0.1],'FaceColor',[0.6 0.6 0.7])
% rectangle('Position',[c1-3,-0.2-0.1,2,0.1],'FaceColor','black')
% rectangle('Position',[c1-1,-0.2-0.1,2,0.1],'FaceColor',[0.6 0.6 0.7])
% rectangle('Position',[c1+1,-0.2-0.1,2,0.1],'FaceColor','black')
% rectangle('Position',[c1+3,-0.2-0.1,2,0.1],'FaceColor',[0.6 0.6 0.7])
% rectangle('Position',[c1+5,-0.2-0.1,2,0.1],'FaceColor','black')

%% Axis
% grid on
axis([c-5 c+5 -1 4])
grid on;
xlabel('X')
ylabel('Y')


end

function R = RotZ(qz)
    R = [cos(qz)  -sin(qz) 0;sin(qz) cos(qz) 0; 0 0 1];
end


