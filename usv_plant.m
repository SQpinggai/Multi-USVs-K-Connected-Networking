function [deta, dv] = usv_plant(stack_eta, stack_v, stack_tao, stack_v0_hat, index)
eta = stack_eta{index};
v = stack_v{index};
v0 = stack_v0_hat{index};
tao = [stack_tao{index}(1); 0; stack_tao{index}(2)];

phi = eta(3);
su = v(1);
sv = v(2);
sr = v(3);

Rphi = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
M = [25.8 0 0; 0 33.8 1.0115; 0 1.0948 2.76];
m_11 = 25.8; m_22 = 33.8; m_33 = 2.76;
c13 = -33.8*sv ;
c23 = 25.8*su;
c31 = 33.8*sv;
c32 = -25.8*su;
C = [0 0 c13; 0 0 c23; c31 c32 0];
d11 = 0.72 + 1.33 * abs(su) + 5.87 * (su)^2;
d22 = 0.8896 + 36.5 * abs(sv) + 0.805 * abs(sr);
d23 = 7.25 + 0.845 * abs(sv) + 3.45 * abs(sr);
d32 = 0.0313 + 3.96 * abs(sv) + 0.13 * abs(sr);
d33 = 1.9 - 0.08 * abs(sv) + 0.75 * abs(sr);
D = [d11 0 0; 0 d22 d23; 0 d32 d33];

deta = Rphi*v;
dsu = (m_22*sv*sr -d11*su + stack_tao{index}(1))/m_11;
dsv = (-m_11*su*sr -d22*sv)/m_22;
dsr = ((m_11 - m_22)*su*sv -d33*sr + stack_tao{index}(2))/m_33;
dv = [dsu; dsv; dsr];
end
    