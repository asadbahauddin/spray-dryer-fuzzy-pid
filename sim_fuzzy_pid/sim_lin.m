function X = sim_lin(P, x0, W, dt)
%SIM_LIN  Exact (ZOH) propagation of the linear plant for piecewise-constant
%   inputs. W is 3-by-N (columns = input held over each step of length dt).
%   Returns X (3-by-N+1) including the initial state.
[A,B] = plant_lin(P);
n = size(A,1); m = size(B,2);
M = expm([A B; zeros(m,n+m)]*dt);
Ad = M(1:n,1:n); Bd = M(1:n,n+1:end);
N = size(W,2);
X = zeros(n,N+1); X(:,1) = x0(:);
for k = 1:N
    X(:,k+1) = Ad*X(:,k) + Bd*W(:,k);
end
end
