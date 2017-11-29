syms m r l I g phi theta phidot thetadot ddphi ddtheta
syms velX(r,theta,phi,l)
syms velY(phi,l)

posX =  r * (theta + phi) + l * sin(phi);
velX =  r * (thetadot + phidot) + l * phidot * cos(phi);
velY =  -l * phidot * sin(phi); 

K_trans = 0.5 * m * (velX^2 + velY^2);
K_rot   = 0.5 * I * phidot^2;
T = K_trans + K_rot;
V = m * g * l * cos(phi);
L = T - V;
dLdthetadot = expand(diff(L,thetadot))
dLdtheta = expand(diff(L,theta))
phiEquation = m*r^2*(ddphi + ddtheta) + m*l^2*ddphi + 2*m*l*r*ddphi*cos(phi) - m*l*r*phidot^2*sin(phi) ...
              + m*l*r*ddtheta*cos(phi) - m*g*l*sin(phi) + I*ddphi;
          
thetaEquation = m*r^2*(ddphi + ddtheta) + m*l*r*ddphi*cos(phi) - m*l*r*phidot^2*sin(phi);


dLdphidot = expand(diff(L,phidot))
dLdphi    = expand(diff(L,phi))

[ddphi, ddtheta] = solve(phiEquation == 0, thetaEquation == 0, ddphi,ddtheta)  
