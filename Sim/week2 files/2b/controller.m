
function u = controller(params, t, X)

  
  m = params.m;
  l = params.l;
  g = params.g;
  th1 = X(1);
  th2 = X(2);
  dth1 = X(3);
  dth2 = X(4);
  % 1. write out the forward kinematics, such that p = FK(theta1, theta2)
  %calculate xy position of tip of second arm,  
  p = l*[cos(th1);sin(th1)] + l*[cos(th1+th2);sin(th1+th2)];
  
  % 2. Let e = p - params.traj(t) be the task-space error
  e = p - params.traj(t);
  %e = p - [-0.0;-0.5];
  
  
  
  % 3. Calculate the manipulator Jacobian J = d p / d theta
  %J = d0P
  J = zeros(2,2);
  %p1 = l * (cos(th1) + cos(th1 + th2))
  %p2 = l * (sin(th1) + sin(th1 + th2))
  %dx/dtheta1
  J(1,1) = -l * (sin(th1)  + sin(th1 + th2));
  %dx/dtheta2
  J(1,2) = -l * sin(th1 + th2);
  %dy/dtheta1
  J(2,1) =  l * (cos(th1)  + cos(th1 + th2));
  %dy/dtheta2
  J(2,2) =  l * cos(th1 + th2);
   
  % 4. Use a "natural motion" PD controller, u = - kp * J^T * e - kd * [dth1; dth2]
  
  
  kp = 2500;
  kd = 5;
  
  %kp = evalin('base','kp');
  %kd = evalin('base','kd');
  u = -kp*J'*e - kd*[dth1;dth2];
  %[th1,th2]
  
  
  
  
end

