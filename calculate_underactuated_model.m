function [trans_p, trans_q, F, G] = calculate_underactuated_model(stack_eta, stack_v, l, index)

if index == 0
    eta = stack_eta;
    v = stack_v;

    phi = eta(3);
    su = v(1);
    sv = v(2);
    sr = v(3);

    trans_px = eta(1) + l*cos(phi);
    trans_py = eta(2) + l*sin(phi);
    trans_p = [trans_px; trans_py];

    trans_qu = su*cos(phi) - sv*sin(phi) - l*sr*sin(phi);
    trans_qv = su*sin(phi) + sv*cos(phi) + l*sr*cos(phi);
    trans_q = [trans_qu; trans_qv];

    m_11 = 25.8; m_22 = 33.8; m_33 = 2.76;
    d11 = 0.72 + 1.33 * abs(su) + 5.87 * (su)^2;
    d22 = 0.8896 + 36.5 * abs(sv) + 0.805 * abs(sr);
    d23 = 7.25 + 0.845 * abs(sv) + 3.45 * abs(sr);
    d32 = 0.0313 + 3.96 * abs(sv) + 0.13 * abs(sr);
    d33 = 1.9 - 0.08 * abs(sv) + 0.75 * abs(sr);
    f_u = (m_22*sv*sr -d11*su)/m_11;
    f_r = ((m_11 - m_22)*su*sv -d33*sr)/m_33;
    F_1 = f_u*cos(phi) -su*sr*sin(phi) -sv*sr*cos(phi) - (sin(phi)/m_22)*(-m_11*su*sr -d22*sv) - l*f_r*sin(phi) - l*sr^2 *cos(phi);
    F_2 = f_u*sin(phi) -su*sr*cos(phi) -sv*sr*sin(phi) + (cos(phi)/m_22)*(-m_11*su*sr -d22*sv) + l*f_r*cos(phi) - l*sr^2 *sin(phi);
    F = [F_1; F_2];

    G_1 = cos(phi)/m_11; G_2 = -l*sin(phi)/m_33;
    G_3 = sin(phi)/m_11; G_4 = l*cos(phi)/m_33;
    G = [G_1, G_2; G_3, G_4];
else
    eta = stack_eta{index};
    v = stack_v{index};

    phi = eta(3);
    su = v(1);
    sv = v(2);
    sr = v(3);

    trans_px = eta(1) + l*cos(phi);
    trans_py = eta(2) + l*sin(phi);
    trans_p = [trans_px; trans_py];

    trans_qu = su*cos(phi) - sv*sin(phi) - l*sr*sin(phi);
    trans_qv = su*sin(phi) + sv*cos(phi) + l*sr*cos(phi);
    trans_q = [trans_qu; trans_qv];

    m_11 = 25.8; m_22 = 33.8; m_33 = 2.76;
    d11 = 0.72 + 1.33 * abs(su) + 5.87 * (su)^2;
    d22 = 0.8896 + 36.5 * abs(sv) + 0.805 * abs(sr);
    d23 = 7.25 + 0.845 * abs(sv) + 3.45 * abs(sr);
    d32 = 0.0313 + 3.96 * abs(sv) + 0.13 * abs(sr);
    d33 = 1.9 - 0.08 * abs(sv) + 0.75 * abs(sr);
    f_u = (m_22*sv*sr -d11*su)/m_11;
    f_r = ((m_11 - m_22)*su*sv -d33*sr)/m_33;
    F_1 = f_u*cos(phi) -su*sr*sin(phi) -sv*sr*cos(phi) - (sin(phi)/m_22)*(-m_11*su*sr -d22*sv) - l*f_r*sin(phi) - l*sr^2 *cos(phi);
    F_2 = f_u*sin(phi) -su*sr*cos(phi) -sv*sr*sin(phi) + (cos(phi)/m_22)*(-m_11*su*sr -d22*sv) + l*f_r*cos(phi) - l*sr^2 *sin(phi);
    F = [F_1; F_2];

    G_1 = cos(phi)/m_11; G_2 = -l*sin(phi)/m_33;
    G_3 = sin(phi)/m_11; G_4 = l*cos(phi)/m_33;
    G = [G_1, G_2; G_3, G_4];
end




