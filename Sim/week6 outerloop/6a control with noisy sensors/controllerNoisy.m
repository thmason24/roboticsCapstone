
function u = controllerNoisy(params, t, obs)
  % This is the starter file for the week5 assignment
  % Now you only receive noisy measurements for phi, and must use your EKF from week 3 to filter the data and get an estimate of the state
  % obs = [ay; az; gx] with a* in units of g's, and gx in units of rad/s

  % This template code calls the function EKFupdate that you must complete below
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);
  
  % The rest of this function should ideally be identical to your solution in week 4
  % Student completes this
  
  Kp = 100;
  Kd = 2;
  
  u = Kp*phi + Kd*phidot;
  %u=0;
end

function xhatOut = EKFupdate(params, t, z)
  % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
  % You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
  % Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.
  persistent X P t_last
  
  if t == 0
     X = [0;0];
     P = diag([1,1]);
     t_last = 0;
     dt = 0;
  else
     dt = t-t_last;
     t_last = t;
  end
  
  Qphi = 1e-2;
  Rphi = 1;
  
  Qw = 1e9;
  Rw = 1;
  Q = diag([Qphi,Qw]);
  R = diag([Rphi,Rphi,Rw]);
  
  A = [1,dt;  0,1];
  %predict state
  xp = A*X;
  %project covariance matrix
  Pk = A*P*A' + Q;

  %compute H measurement matrix
  H = [1,0;  -X(1),0;   0,1];

  %compute innovation
  y = z - [sin(xp(1)); cos(xp(1)); xp(2)];
  %compute innovation covariance
  S = H*Pk*H' + R;
  % compute near optimal kalman gain
  K = (Pk * H')/S;
  %update state
  X = xp + K*(y);
  
  %update convariance
  P = (eye(2)-K*H)*Pk;
  xhatOut = X;
  
  
end
