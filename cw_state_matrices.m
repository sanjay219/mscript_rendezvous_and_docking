function [A,B] = cw_state_matrices(n)
A = cwA(n);
B = [0 0 0;
     0 0 0;
     0 0 0;
     1 0 0;
     0 1 0;
     0 0 1];
end
