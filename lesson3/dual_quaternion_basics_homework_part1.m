% Clean up routine to remove variables from your workspace
clear all;
close all;
% Initialize the DQ Robotics namespace
include_namespace_dq

%% Solutions
%% Problem 1
t1 = i_;
r1 = cos(pi/16) + k_*sin(pi/16);
x1 = r1 + 0.5*E_*t1*r1;
plot(x1);
title('x1');

%% Problem 2
t2 = k_;
r2 = cos(-(pi/4)) + i_*sin(-(pi/4));
x2 = r2 + 0.5*E_*t2*r2;
plot(x2);
title('x2');

% Problem 3
x0 = 1;
x3 = x0*x1*x2;
plot(x3);
title('x3');

%% Problem 4
x3 = r3

%% Problem 5
x3 = t3

%% Problem 6
r4 = cos(pi/16) + k_*sin(pi/16);
t4 = i_;
x4 = r4 + 0.5*E_*r4*t4;
plot(x4);
title('x4');



