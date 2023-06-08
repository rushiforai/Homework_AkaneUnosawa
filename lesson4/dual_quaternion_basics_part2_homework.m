%% Homework example
% Clean up routine to remove variables from your workspace
clear all;
close all;
% Initialize the DQ Robotics namespace
include_namespace_dq

%% Solutions
% Question 1
l1 = j_
p_l1 = DQ(0)
%プリュッカー線は直線上の無限遠方の点を表現するため、直線の方向や位置に関連する情報のみが必要。プリュッカー線は平行移動や回転の影響を受けない。そのため、プリュッカー線を表現するデュアル四元数では、スカラー成分を常に0とし、ベクトル成分に直線の方向を表すモーメントが格納される。
%スカラー成分が0であることにより、デュアル四元数の演算や表現が簡素化され、直線上の点の移動や回転を考慮する必要がなくなる。
%よってプリュッカー線のデュアル四元数表現では、スカラー成分は常に0とすることが一般的
m1 = cross(p_l1,l1)
l1_dq = l1 + E_*m1

l2 = j_
p_l2 = DQ(1)
m2 = cross(p_l2,l2)
l2_dq = l2 + E_*m2

l3 = j_
p_l3 = DQ(2)
m3 = cross(p_l3,l3)
l3_dq = l3 + E_*m3

l4 = j_
p_l4 = DQ(3)
m4 = cross(p_l4,l4)
l4_dq = l4 + E_*m4

%All Plucker lines are the same.

%% Question 2
% Point coordinates
p1 = 2*i_ + j_ + k_;
% Calculate distance
d_p_p1 = norm(cross(p1,l1)-m1)

% Question 3
n_pi1 = i_
p_pi1 = DQ(0)
d_pi1 = dot(p_pi1,n_pi1)
pi1 = n_pi1 + E_*d_pi1
figure(10000000),plot(pi1,'plane', 5, 'location', p_pi1); 
title('figure pi1')

n_pi2 = -i_
p_pi2 = DQ(0)
d_pi2 = dot(p_pi2,n_pi2)
pi2 = n_pi2 + E_*d_pi2
 figure,plot(pi2,'plane', 5, 'location', p_pi2); 
 title('figure pi2')

% Question4
% distance between p1 and pi1
d_p1_pi1 = dot(p1,n_pi1)-d_pi1
% distance between p2 and pi2  
d_p1_pi2 = dot(p1,n_pi2)-d_pi2
%Yes, these signs are different. That's because the normal signs are
%different.

% Question5

n_pi3 = i_
p_pi3 = 2.5*i_
d_pi3 = dot(p_pi3,n_pi3)
pi3 = n_pi3 + E_*d_pi3
figure,plot(pi3,'plane', 5, 'location', p_pi3)

n_pi4 = j_
p_pi4 = 2.5*j_
d_pi4 = dot(p_pi4,n_pi4)
pi4 = n_pi4 + E_*d_pi4
hold on
plot(pi4,'plane', 5, 'location', p_pi4)

n_pi5 = k_
p_pi5 = 2.5*k_
d_pi5 = dot(p_pi5,n_pi5)
pi5 = n_pi5 + E_*d_pi5
hold on
plot(pi5,'plane', 5, 'location', p_pi5)

n_pi6 = i_
p_pi6 = -2.5*i_
d_pi6 = dot(p_pi6,n_pi6)
pi6 = n_pi6 + E_*d_pi6
hold on
plot(pi6,'plane', 5, 'location', p_pi6)

n_pi7 = j_
p_pi7 = -2.5*j_
d_pi7 = dot(p_pi7,n_pi7)
pi7 = n_pi7 + E_*d_pi7
hold on
plot(pi7,'plane', 5, 'location', p_pi7)

n_pi8 = k_
p_pi8 = -2.5*k_
d_pi8 = dot(p_pi8,n_pi8)
pi8 = n_pi8 + E_*d_pi8
hold on
plot(pi8,'plane', 5, 'location', p_pi8)

% Question6


















