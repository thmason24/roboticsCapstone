function qdd = eom(params, theta, phi, thetadot, phidot, u)
  % This is the starter file for the week5 assignment

  % Provided params are
  % params.g: gravitational constant
  % params.mr: mass of the "rod"
  % params.ir: rotational inertia of the rod
  % params.d: distance of rod CoM from the wheel axis
  % params.r: wheel radius
  m = params.mr;
  g = params.g;
  I = params.ir;
  l  = params.d;
  r  = params.r;

  % Provided states are:
  % th: wheel angle (relative to body)
  % phi: body pitch
  % dth, dphi: time-derivatives of above
  % u: torque applied at the wheel
  
  ddphi = (- m*cos(phi)*sin(phi)*l^2*phidot^2 + g*m*sin(phi)*l)/(I + l^2*m - l^2*m*cos(phi)^2);
  ddtheta = (m*sin(phi)*l^3*phidot^2 + m*r*cos(phi)*sin(phi)*l^2*phidot^2 - g*m*cos(phi)*sin(phi)*l^2 + I*sin(phi)*l*phidot^2 - g*m*r*sin(phi)*l)/(I*r + l^2*m*r - l^2*m*r*cos(phi)^2);
 
  
  qdd = [ddtheta;ddphi];
  % THE STUDENT WILL FILL THIS OUT
end