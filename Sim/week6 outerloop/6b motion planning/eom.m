function qdd = eom(params, theta, phi, thetadot, phidot, tau)
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
  
  %ddphi = (- m*cos(phi)*sin(phi)*l^2*dphi^2 + g*m*sin(phi)*l)/(I + l^2*m - l^2*m*cos(phi)^2);
  %ddtheta = (m*sin(phi)*l^3*dphi^2 + m*r*cos(phi)*sin(phi)*l^2*dphi^2 - g*m*cos(phi)*sin(phi)*l^2 + I*sin(phi)*l*dphi^2 - g*m*r*sin(phi)*l)/(I*r + l^2*m*r - l^2*m*r*cos(phi)^2);
  ddphi  = -(r*tau + l*tau*cos(phi) - g*l*m*r*sin(phi) + l^2*m*phidot^2*r*cos(phi)*sin(phi))/(I*r + l^2*m*r - l^2*m*r*cos(phi)^2);
  ddtheta = (sin(phi)*l^3*m^2*phidot^2*r + cos(phi)*sin(phi)*l^2*m^2*phidot^2*r^2 - g*cos(phi)*sin(phi)*l^2*m^2*r + tau*l^2*m - g*sin(phi)*l*m^2*r^2 + I*sin(phi)*l*m*phidot^2*r + 2*tau*cos(phi)*l*m*r + tau*m*r^2 + I*tau)/(l^2*m^2*r^2 + I*m*r^2 - l^2*m^2*r^2*cos(phi)^2);
  qdd = [ddtheta;ddphi];
  % THE STUDENT WILL FILL THIS OUT
end