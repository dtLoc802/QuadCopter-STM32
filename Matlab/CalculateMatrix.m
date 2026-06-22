clc
clear

syms q1 q2 q3 q4 ax ay az wx wy wz B mx my mz bx by bz vx vy vz g real
% Dat dieu kien q1^2 + q2^2 + q3^2 + q4^2 == 1
assume(q1^2 + q2^2 + q3^2 + q4^2 == 1)


%Tinh vecto tu truong trai dat
m = [0 mx my mz]
q = [q1 q2 q3 q4]
q = real(q)
m = real(m)
q_conj = quatconj(q)
h = quatmultiply(quatmultiply(q, m), q_conj)
simplify(h(2)) %bx = sqrt(hx*hx + hy*hy)
simplify(h(3))
simplify(h(4)) %bz = hz

q = [q1 ; q2 ; q3 ; q4 ]
Omega = [ 0   -wx  -wy  -wz;
          wx   0    wz  -wy;
          wy  -wz   0    wx;
          wz   wy  -wx   0 ]

%Ma tran Jacobian co Magnetometer
Jb = [-2*bz*q3, 2*bz*q4, -4*bx*q3-2*bz*q1, -4*bx*q4+2*bz*q2;
      -2*bx*q4+2*bz*q2, 2*bx*q3+2*bz*q1, 2*bx*q2+2*bz*q4, -2*bx*q1+2*bz*q3;
      2*bx*q3, 2*bx*q4-4*bz*q2, 2*bx*q1-4*bz*q3, 2*bx*q2]

%Ma tran sai so co Magnetometer
fb = [2*bx*(0.5 - q3^2 - q4^2) + 2*bz*(q2*q4 - q1*q3) - mx;
      2*bx*(q2*q3 - q1*q4) + 2*bz*(q1*q2 + q3*q4) - my;
      2*bx*(q1*q3 + q2*q4) + 2*bz*(0.5 - q2^2 - q3^2) - mz]

%Ma tran Jacobian khong co Magnetometer
Jg = [-2*q3  2*q4  -2*q1  2*q2;
      2*q2  2*q1   2*q4  2*q3;
      0    -4*q2  -4*q3  0]

%Ma tran sai so khong co Manetometer
fg = [2*(q2*q4 - q1*q3) - ax;
      2*(q1*q2 + q3*q4) - ay;
      2*(0.5 - q2^2 - q3^2) - az]

%Chuyen vi hai ma tran
JgT = Jg.'
JbT = Jb.'

%Phuong trinh dong hoc quaternion
q_dot = 0.5 * Omega * q

%Gradient descent
detaf1 = JgT * fg
detaf2 = JgT * fg + JbT * fb
ccode(simplify(detaf1))
ccode(simplify(detaf2))

%Kiem tra ma tran Jacobian tinh bang matlab va tinh tay
Jg_auto = jacobian(fg, [q1,q2,q3,q4])
simplify(Jg - Jg_auto)
Jb_auto = jacobian(fb, [q1,q2,q3,q4])
simplify(Jb - Jb_auto)

%Kiem tra lai ham gradient detaf1
detaf1_correct = Jg_auto.' * fg;
ccode(simplify(detaf1_correct))

%Kiem tra lai ham gradient detaf2
detaf2_correct = Jg_auto.' * fg + Jb_auto.' * fb;
ccode(simplify(detaf2_correct))

%Tinh ma tran accelerometer earth frame
I = eye(3)
va = [0 vx vy vz] %Vecto truoc khi quay
q = [q1 q2 q3 q4]
q_conj = [q1 -q2 -q3 -q4]

%Ham Hamilton
function out = quatmul(a, b)
    out = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
           a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
           a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
           a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end

%Kiem tra ma tran xoay (direction matrix) tinh tay va tinh bang matlab
temp = quatmul(q_conj, va)
vb = quatmul(temp, q)
vb_vec = vb(2:4) %Bo phan scalar vb(1)
v = [vx; vy; vz]
Rcheck = jacobian(vb_vec, v) %Tach he so 
Rcheck = simplify(Rcheck) %neu Rcheck bang R la dung

%Tinh toan ma tran gia toc theo earth frame
R = [1-2*(q3*q3 + q4*q4) 2*(q2*q3 + q1*q4) 2*(q2*q4 - q1*q3);
     2*(q2*q3 - q1*q4) 1-2*(q2*q2 + q4*q4) 2*(q3*q4 + q1*q2);
     2*(q2*q4 + q1*q3) 2*(q3*q4 - q1*q2) 1-2*(q2*q2 + q3*q3)]
gravity = [0;
           0;
           g]
a = [ax; 
     ay; 
     az]
a_L = R*a + gravity
ccode(simplify(a_L))
