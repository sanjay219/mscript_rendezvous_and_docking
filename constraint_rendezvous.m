function [c, ceq] = constraint_rendezvous(p, params)
% Nonlinear constraints for two-impulse CW rendezvous
% Equality: final state after second burn equals desired rendezvous state
% Inequality: simple soft safety surrogate can be added here later

ceq = rendezvous_constraint_eq(p, params);

% Placeholder inequality constraint for future work:
% c <= 0
% Examples you can add later:
%  - minimum radial corridor
%  - line-of-sight cone
%  - max intermediate range
c = [];
end
