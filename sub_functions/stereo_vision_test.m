% stereo vision test
clear;
Origin = [400; -1000; 0];
width = 0.07;   % stereo vision 사이 간격 (사람의 눈 간격과 동일하게 설정)
focus = 0.022;
OxL = [Origin(1)-width; Origin(2); Origin(3)];
OxR = [Origin(1)+width; Origin(2); Origin(3)];

target = [0; 0; 0];

mL = OxL(2)/OxL(1);
mR = OxR(2)/OxR(1);

fxL = [(Origin(2)+focus)/mL; Origin(2)+focus; 0];
fxR = [(Origin(2)+focus)/mR; Origin(2)+focus; 0];

% f1 = fxL(1)-OxL(1);
% f2 = fxR(1)-OxR(1);

% Newton-Raphson method
X = [10; 1];    % X=[t;s]; initial value
X_prev = X;

J = [(fxL(1)-OxL(1)), -(fxR(1)-OxR(1));
    (fxL(2)-OxL(2)), -(fxR(2)-OxR(2));
    (fxL(3)-OxL(3)), -(fxR(3)-OxR(3))];

f = [X(1)*(fxL(1)-OxL(1))-X(2)*((fxR(1)-OxR(1)))+OxL(1)-OxR(1);
    X(1)*(fxL(2)-OxL(2))-X(2)*((fxR(2)-OxR(2)))+OxL(2)-OxR(2);
    X(1)*(fxL(3)-OxL(3))-X(2)*((fxR(3)-OxR(3)))+OxL(3)-OxR(3);];

X = X - pinv(J)*f;

iter = 0;

while norm(X - X_prev) > 10^-5   
    X_prev = X;
    
    J = [(fxL(1)-OxL(1)), -(fxR(1)-OxR(1));
        (fxL(2)-OxL(2)), -(fxR(2)-OxR(2));
        (fxL(3)-OxL(3)), -(fxR(3)-OxR(3))];
    
    f = [X(1)*(fxL(1)-OxL(1))-X(2)*((fxR(1)-OxR(1)))+OxL(1)-OxR(1);
        X(1)*(fxL(2)-OxL(2))-X(2)*((fxR(2)-OxR(2)))+OxL(2)-OxR(2);
        X(1)*(fxL(3)-OxL(3))-X(2)*((fxR(3)-OxR(3)))+OxL(3)-OxR(3);];
    
    X = X - pinv(J)*f;
    
    iter = iter + 1;
    
    if iter > 1000
        break
    end
    
%     fprintf('iter=%d, X=%.3f, Y=%.3f, Z=%.3f\n',iter, X(1), X(2), X(3));
end

u1 = [(fxL(1)-OxL(1)); (fxL(2)-OxL(2)); (fxL(3)-OxL(3))];
u2 = [(fxR(1)-OxR(1)); (fxR(2)-OxR(2)); (fxR(3)-OxR(3))];

P1 = [OxL(1) + X(1)*u1(1); OxL(2) + X(1)*u1(2); OxL(3) + X(1)*u1(3)];
P2 = [OxR(1) + X(2)*u2(1); OxR(2) + X(2)*u2(2); OxR(3) + X(2)*u2(3)];