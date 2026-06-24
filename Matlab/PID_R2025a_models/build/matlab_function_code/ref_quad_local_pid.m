function ref = fcn(t, p)
%#codegen
k = p(4);
z_amp = p(5);
x_amp = p(6);
y_amp = p(7);
psi_ref_value = p(8);

[z_ref, dz_sigmoid] = sigmoid_ref(t, z_amp, k, 4.0);
[x_ref, dx_ref] = sigmoid_ref(t, x_amp, k, 5.0);
[y_ref, dy_ref] = sigmoid_ref(t, y_amp, k, 5.0);

% The original Victor Fcn z controller damps measured dz only; it does not
% subtract dz_ref. Keep this channel zero to make that convention explicit.
dz_ref = 0.0;
psi_ref = psi_ref_value;
ref = [x_ref; dx_ref; y_ref; dy_ref; z_ref; dz_ref; psi_ref];
end

function [r, dr] = sigmoid_ref(t, amp, k, center)
q = exp(-k*(t - center));
den = 1.0 + q;
r = amp/den;
dr = amp*k*q/(den*den);
end
