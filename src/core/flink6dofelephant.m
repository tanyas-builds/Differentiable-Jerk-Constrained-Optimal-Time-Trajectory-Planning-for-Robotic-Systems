function [x,y,z,h]=flink6dofelephant(theta1,theta2,theta3, theta4, theta5, theta6)
% this routine computes x y z coordinates by transformation
% conver to radians]
d1=131.56;d4=66.39;d5=73.18;d6=43.6;
a2=110.4;a3=96;
h=trans(theta1,d1,0,pi/2)*trans(theta2-pi/2,0,-a2,0)*trans(theta3,0,-a3,0)...
    *trans(theta4-pi/2,d4,0,pi/2)*trans(theta5+pi/2,d5,0,-pi/2)*trans(theta6,d6,0,0);
x=h(1,4);
y=h(2,4);
z=h(3,4);
sqrt(x*x + y*y + z*z);
function T=trans(theta,d,a,alpha)
 T=[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
 sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
 0 sin(alpha) cos(alpha) d;
 0 0 0 1];
end
end
