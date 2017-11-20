
function xhat = EKFstudent(t, z)
  % In this exercise, you will batch-process this data: you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
  xhat = zeros(2,length(t));
  dt = 0.02;
  X = [0;0];
  A = [1,dt;  0,1];
  Q = diag([1,1]);
  R = diag([1,1,2]);
  P = diag([2,2]);
  
  for i = 1:size(z,2)
     %predict state
     xp = A*X;
     %project covariance matrix
     Pk = A*P*A' + Q;
     
     %compute H measurement matrix
     H = [1,0;  X(1),0;   0,1];
     
     K = Pk * H'*(H*P*H' + R)';
     X = xp + K*(z(:,i) - [sin(X(1)); cos(X(1)); X(2)]);
     P = (eye(2)-K*H)*Pk;
     xhat(:,i) = X;
     
  end

  % Student completes this
end
