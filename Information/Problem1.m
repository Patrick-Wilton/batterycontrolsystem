%% Parameters

% Material Paramters
E1 = 121; % GPa
E2 = 8.6; % GPa
v12 = 0.27;
v21 = (E2 / E1) * v12;
G12 = 4.7; % GPa
t = 0.25; % mm
laminate1 = [0,30,30,0]; % degrees
laminate2 = [30,0,0,30]; % degrees
sizeL = size(laminate1, 2);
zarray = linspace(-t*(sizeL/2), t*(sizeL/2), sizeL + 1);


% Maximum Stain Criterion
eLpos = 0.0167;
eLneg = 0.0108;
eTpos = 0.0032;
eTneg = 0.0192;
eLT = 0.012;

% Calculate Q Values
Q11 = E1 / (1 - v12 * v21);
Q12 = (v12 * E2) / (1 - v12 * v21);
Q22 = E2 / (1 - v12 * v21);
Q66 = G12;

Q30 = matQ(Q11, Q12, Q22, Q66, 30);
Q0 = matQ(Q11, Q12, Q22, Q66, 0);

QuadMatA1 = SumMat(Q0, Q30, Q30, Q0, 'A', zarray);
QuadMatA2 = SumMat(Q30, Q0, Q0, Q30, 'A', zarray);
QuadMatD1 = SumMat(Q0, Q30, Q30, Q0, 'D', zarray);
QuadMatD2 = SumMat(Q30, Q0, Q0, Q30, 'D', zarray);



%% Q Matrix Function

function [Q] = matQ(Q11, Q12, Q22, Q66, a)

c = cosd(a);
s = sind(a);
c2 = cosd(a)^2;
s2 = cosd(a)^2;
c3 = cosd(a)^3;
s3 = sind(a)^3;
c4 = cosd(a)^4;
s4 = sind(a)^4;

Qmat11 = Q11*c4 + Q22*s4 + 2*(Q12 + 2*Q66)*s2*c2;
Qmat12 = (Q11 + Q22 - 4 * Q66)*s2*c2 + Q12*(c4 + s4);
Qmat22 = Q11*s4 + Q22*c4 + 2*(Q12 + 2*Q66)*s2*c2;
Qmat16 = (Q11 - Q12 - 2*Q66)*c3*s - (Q22 - Q12 - 2*Q66)*c*s3;
Qmat26 = (Q11 - Q12 - 2*Q66)*c*s3 - (Q22 - Q12 - 2*Q66)*c3*s;
Qmat66 = (Q11 + Q22 - 2*Q12 - 2*Q66)*s2*c2 + Q66*(s4 + c4);

Q = [Qmat11, Qmat12, Qmat16;Qmat12, Qmat22, Qmat26;Qmat16, Qmat26, Qmat66];
end

%% Quadrant Matrix Function

function QuadMat = SumMat(Q1, Q2, Q3, Q4, quad, zarray)

% Determines what quadrant we are in
if quad == 'A'
    p = 1;
elseif quad == 'B'
    p = 2;
elseif quad == 'D'
    p = 3;
else
    disp('invalid input')
end

% Matrix Summing
QuadMat = Q1* (zarray(2)^p - zarray(1)^p) + Q2 * (zarray(3)^p - zarray(2)^p) + Q3 * (zarray(4)^p - zarray(3)^p) + Q4 * (zarray(5)^p - zarray(4)^p);

end

%% Strain Function

function [a] = strain(b)

end