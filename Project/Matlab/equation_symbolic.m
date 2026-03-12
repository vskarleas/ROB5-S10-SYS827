close all;
clear;
clc;

syms d1
syms a2
syms a3 
syms d4
syms d5
syms d6

syms q1
syms q2
syms q3
syms q4
syms q5
syms q6

syms q23 
syms q234


%% DH Parameters (Modified DH Convention)
ai = [0; 0; a2; a3; 0; 0];
alphai = [sym(0); sym(pi)/2; sym(0); sym(0); sym(pi)/2; -sym(pi)/2];
di = [d1; 0; 0; d4; d5; d6];
thetai = [q1;q2;q3;q4;q5;q6];

%% Modified DH Transformation Matrix
function T = DH_Modified_Transform(alpha, a, d, theta)
    % T = Rot_x(alpha) * Trans_x(a) * Trans_z(d) * Rot_z(theta)
    T = [cos(theta),              -sin(theta),             0,            a;
         sin(theta)*cos(alpha),   cos(theta)*cos(alpha),   -sin(alpha),  -sin(alpha)*d;
         sin(theta)*sin(alpha),   cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d;
         0,                       0,                       0,            1];
end


%% Individual transformation matrices T_{i-1}^{i}
T = cell(6,1);
for i = 1:6
    T{i} = simplify(DH_Modified_Transform(alphai(i), ai(i), di(i), thetai(i)));
    fprintf('\n--- T_%d-%d ---\n', i-1, i);
    disp(T{i});
end

%% Cumulated transformation matrices T_0^{i}
T_cumul = cell(6,1);
T_cumul{1} = T{1};
for i = 2:6
    T_cumul{i} = simplify(T_cumul{i-1} * T{i}, 'Steps', 50);
end

%% Trigonometric simplification and display
for i = 1:6
    fprintf('\n=== T_0-%d (simplified) ===\n', i);
    
    Ti = simplify(T_cumul{i}, 'Steps', 50);
    Ti = rewrite(Ti, 'sincos');
    
    % cos(q2+q3) and sin(q2+q3)
    Ti = subs(Ti, cos(q2)*cos(q3) - sin(q2)*sin(q3), cos(q2+q3));
    Ti = subs(Ti, cos(q2)*sin(q3) + sin(q2)*cos(q3), sin(q2+q3));
    Ti = subs(Ti, sin(q2)*cos(q3) + cos(q2)*sin(q3), sin(q2+q3));
    Ti = simplify(Ti, 'Steps', 50);
    Ti = subs(Ti, q2+q3, q23);
    
    % cos(q23+q4) and sin(q23+q4)
    Ti = subs(Ti, cos(q23)*cos(q4) - sin(q23)*sin(q4), cos(q23+q4));
    Ti = subs(Ti, cos(q23)*sin(q4) + sin(q23)*cos(q4), sin(q23+q4));
    Ti = subs(Ti, sin(q23)*cos(q4) + cos(q23)*sin(q4), sin(q23+q4));
    Ti = simplify(Ti, 'Steps', 50);
    Ti = subs(Ti, q23+q4, q234);
    
    disp(Ti);
    
    % Store back
    T_cumul{i} = Ti;
end

%% ========== JACOBIAN (Position) ==========

% Recompute cumulated matrices WITHOUT trig substitutions
T_cumul_raw = cell(6,1);
T_cumul_raw{1} = T{1};
for i = 2:6
    T_cumul_raw{i} = simplify(T_cumul_raw{i-1} * T{i}, 'Steps', 50);
end

% Extract position vector from RAW T_0-6
T06_raw = T_cumul_raw{6};
px = T06_raw(1, 4);
py = T06_raw(2, 4);
pz = T06_raw(3, 4);

% Joint variables
q = [q1, q2, q3, q4, q5, q6];

% Compute Jacobian matrix (3x6) from raw expressions
Jp = sym(zeros(3, 6));
for j = 1:6
    Jp(1, j) = diff(px, q(j));
    Jp(2, j) = diff(py, q(j));
    Jp(3, j) = diff(pz, q(j));
end

% Simplify
Jp = simplify(Jp, 'Steps', 50);

% NOW apply trig substitutions on the Jacobian
% cos/sin(q2+q3)
Jp = subs(Jp, cos(q2)*cos(q3) - sin(q2)*sin(q3), cos(q2+q3));
Jp = subs(Jp, cos(q2)*sin(q3) + sin(q2)*cos(q3), sin(q2+q3));
Jp = subs(Jp, sin(q2)*cos(q3) + cos(q2)*sin(q3), sin(q2+q3));
Jp = simplify(Jp, 'Steps', 50);
Jp = subs(Jp, q2+q3, q23);

% cos/sin(q23+q4)
Jp = subs(Jp, cos(q23)*cos(q4) - sin(q23)*sin(q4), cos(q23+q4));
Jp = subs(Jp, cos(q23)*sin(q4) + sin(q23)*cos(q4), sin(q23+q4));
Jp = subs(Jp, sin(q23)*cos(q4) + cos(q23)*sin(q4), sin(q23+q4));
Jp = simplify(Jp, 'Steps', 50);
Jp = subs(Jp, q23+q4, q234);

Jp = simplify(Jp, 'Steps', 50);

%% Display Jacobian
fprintf('\n========== POSITION JACOBIAN Jp (3x6) ==========\n');
disp(Jp);

for j = 1:6
    fprintf('\n--- dP/dq%d ---\n', j);
    disp(Jp(:, j));
end