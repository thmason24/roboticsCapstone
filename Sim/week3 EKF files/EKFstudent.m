
function xhat = EKFstudent(t, z)
  % In this exercise, you will batch-process this data: you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
  xhat = zeros(2,length(t));
  X = [0;0];
  
  Qphi = 1e-2;
  Rphi = 1;
  
  Qw = 1e9;
  Rw = 1;
  Q = diag([Qphi,Qw]);
  R = diag([Rphi,Rphi,Rw]);
  P = diag([1,1]);
  
  P1_vec = zeros(numel(t));
  P2_vec = zeros(numel(t));
  K1_vec = zeros(numel(t));
  K2_vec = zeros(numel(t));
  
  for i = 2:size(z,2)
      
     dt = t(i)-t(i-1);
     %dt = 1.2*dt;
     %dt = 0.005;
     %dt = 0;
     Z = z(:,i);
     Z(3) = deg2rad(Z(3));
     %Z(1:2) = [-1,1];
     %Z(1:2) = Z(1:2)/norm(Z(1:2)); 
     %Z = [0,1,0]';
     %Z(3) = 20;
     if false
         A = [1,dt;  0,1];
         X(2) = z(3,i);
         X = A*X;
         xhat(:,i) = X;
     
     else
         A = [1,dt;  0,1];
         %predict state
         xp = A*X;
         %xp(1) = X(1) + dt*z(3,i);
         %project covariance matrix
         Pk = A*P*A' + Q;

         %compute H measurement matrix
         H = [1,0;  -X(1),0;   0,1];
         %H = [1,0;  -X(1),0;   0,1];
         
         %compute innovation
         y = Z - [sin(xp(1)); cos(xp(1)); xp(2)];
         %compute innovation covariance
         S = H*Pk*H' + R;
         % compute near optimal kalman gain
         K = (Pk * H')/S;
         %update state
         X = xp + K*(y);
         %X(2) = Z(3);

         %update convariance
         P = (eye(2)-K*H)*Pk;
         P1_vec(i) = P(1,1);
         P2_vec(i) = P(2,2);
         K1_vec(i) = K(1,1);
         K2_vec(i) = K(1,2);
         xhat(:,i) = rad2deg(X);
     end
  end
  
  
  if false
      figure()
      subplot(4,1,1)
      plot(P1_vec)
      title('P1')
      subplot(4,1,2)
      plot(P2_vec)
      title('P2')
      subplot(4,1,3)
      plot(K1_vec)
      title('k1')
      subplot(4,1,4)
      plot(K2_vec)
      title('k2')
      figure()
      
  end
  
  K

  % Student completes this
end
