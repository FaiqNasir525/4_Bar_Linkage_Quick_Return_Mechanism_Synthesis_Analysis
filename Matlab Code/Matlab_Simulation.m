clc
clear all
% Link Lengths (in cm)
Link1 = 4.8; %Ground Link
Link2 = 1.65; %Crank
Link3 = 4.65; %Coupler
Link4_A = 5; %Rocker (O4 to C)
Link4_B = 3; %Rocker (O4 to D)
Link5 = 1.75; %Driller Coupler
Link6= 4; %Driller
%crank angle variation
theta2 = 0:1:360;
for i = 1:length(theta2)

 %Distance between A and O4
 A_O4(i) = sqrt(Link1^2 + Link2^2 - 2*Link1*Link2*cosd(theta2(i)));
 %Angle between A_O4 and x-axis
 beta(i) = acosd((Link1^2 + A_O4(i)^2 - Link2^2)/(2*Link1*A_O4(i)));
 %Angle between Link3 and A_O4
 psi(i) = acosd((Link3^2 + A_O4(i)^2 - Link4_A^2)/(2*Link3*A_O4(i)));
 %Angle between Link4 and A_O4
 lamda(i) = acosd((Link4_A^2 + A_O4(i)^2 - Link3^2)/(2*Link4_A*A_O4(i)));
 %Angle between Link4 and negative x-axis
 alpha(i) = 180 - beta(i) - lamda(i);
 %Angle of Link3 with positive x-axis
 theta3(i) = psi(i) - beta(i);
 %Angle of Link4 with positive x-axis
 theta4(i) = 180 - beta(i) - lamda(i);
 %when crank is in the third and fourth quadrants
 if(theta2(i) > 180)
 %Angle of Link3 with positive x-axis
 theta3(i) = psi(i) + beta(i);
 %Angle of Link4 with positive x-axis
 theta4(i) = 180 - lamda(i) + beta(i);
 %Angle between Link4 and negative x-axis
 alpha(i) = 180 - lamda(i) + beta(i);
 end
 % Positions of Joints

 %Position of O2
 O2_x(i) = 0;
 O2_y(i) = 0;

 %Position of A
 A_x(i) = O2_x(i) + Link2*cosd(theta2(i));
 A_y(i) = O2_y(i) + Link2*sind(theta2(i));
 %Position of B
 B_x(i) = O2_x(i) + A_x(i) + Link3*cosd(theta3(i));
 B_y(i) = O2_y(i) + A_y(i) + Link3*sind(theta3(i));

 %Position of O4
 O4_x(i) = Link1;
 O4_y(i) = 0;

  %Position of D
 D_x(i) = O4_x(i) + Link4_B*cosd(theta4(i)-180);
 D_y(i) = O4_y(i) + Link4_B*sind(theta4(i)-180);

 %Position of E
 E_x(i) = D_x(i) - Link5;
 E_y(i) = -3;

 %Postion of F;
 F_x(i) = E_x(i) - Link6;
 F_y(i) = -3;

 %Plot of All the Joints
 plot([O2_x(i) A_x(i)], [O2_y(i) A_y(i)],[A_x(i) B_x(i)], [A_y(i) B_y(i)],[B_x(i) O4_x(i)], [B_y(i) O4_y(i)], [O4_x(i) D_x(i)], [O4_y(i) D_y(i)], [D_x(i) E_x(i)], [D_y(i) E_y(i)], [E_x(i) F_x(i)], [E_y(i) F_y(i)], 'LineWidth', 4);

 title('Pump Jack 201111,201180,201075 ');

 %Turn on grid
 grid on;
 %Axis limits
 axis([-2 10 -5 6]);
 %rotate(, 90, [0,0]);
 %modifies the plot
 drawnow;
end
