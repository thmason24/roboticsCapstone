
function u = controllerNoisyEnc(params, t, obs, th, dth)
  % This is the starter file for the week5 assignment
  % Now you only receive noisy measurements for theta, and must use your EKF from week 3 to filter the data and get an estimate of the state
  % obs = [ay; az; gx] (same as last week)
  % New for 6b: you also have access to params.traj(t)

  % Template code (same as last week)
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  dphi = xhat(2);
  
  x_des = params.traj(t);
  x = params.r*(phi+th);
  x_error = x_des-x;
  xdot = params.r*(dphi + dth);
  
  
  kp = 1.0;
  kd = -0.09;
  
  Kphi_p = 10;
  Kphi_d = 0.1;
  
  
  %best
  %Kphi_p = 200;
  %Kphi_d = 0.1;
  
  %works ok for step
  %kp = 0.006;
  %kd = 0.0002
  
  %x_error = 0.1 + 0.2*sin(t);
  ddx_p = kp*x_error;
  ddx_v = kd*xdot;
  ddx = ddx_p + ddx_v;
  beta = 1;
  phides = asin(ddx);
  if false      
      phiMax = 60*pi/180;
      if phides > phiMax
          phides = phiMax;
      end
      if phides < -phiMax
          phides = -phiMax;
      end
  end
  
  %Kphi_p = 200;
  %Kphi_d = 0.1;
  %phides = 0.00 + 0.4*sin(t);
  %phides = pi/4;
  %works pretty well   
  %Kphi_p = 300;
  %Kphi_d = 0.1;

  %works for stable angle
  %Kphi_p = 300;
  %Kphi_d = 1;

  u = Kphi_p*sin(phi-phides)+Kphi_d*dphi;
  if mod(t,0.05) == 0 && true
      fprintf('t: %2.2f  phides %2.2f phi %2.2f dx %2.2f x %2.2f xerror %2.2f ddx_p %2.2f ddx_v %2.4f\n' , t, phides, phi, xdot, x,  x_error, ddx_p, ddx_v)
  end
      %u = 0;
  
end

function xhatOut = EKFupdate(params, t, z)
  % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
  % You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
  % Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.

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
