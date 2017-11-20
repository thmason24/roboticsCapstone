
function robotSim()
  % This file is starter code
  %system constants
  animate = false;
  animate = true;
  params = struct();
  params.m = 1;
  params.l = 0.5;
  params.g = 9.81;
  params.traj = @trajSin;
  T = 10;
  tspan=[0,T];
  if false
      params.traj = @trajSquare;
      params.l = 1;
  end
  
  for kp = 2500
      for kd = 5 
  %for kp = 2500:50:3000
  %    for kd = 1:10:100
          
          
          y0=[0,1,0,0,0,0];
          [t,X, u] = ode45(@(t,y) dynamics(params,t,y,kp,kd),tspan,y0);
          truthTraj = params.traj(t');
          l = params.l;
          p = l*[cos(X(:,1)');sin(X(:,1)')] + l * [cos(X(:,2)' + X(:,1)');sin(X(:,2)' + X(:,1)')];
 
          figure(3)
          subplot(5,1,1)
          hold off
          plot(t,truthTraj(1,:))
          hold on
          plot(t,p(1,:))
          title('X')
          
          subplot(5,1,2)
          hold off
          plot(t,truthTraj(2,:))
          hold on
          plot(t,p(2,:))
          title('Y')
          
          subplot(5,1,3)
          hold off
          plot(t, X(:,1))
          hold on
          plot(t, X(:,2))
          title('theta')
          
          subplot(5,1,4)
          hold off
          plot(t, X(:,3))
          hold on
          plot(t, X(:,4))
          title('dtheta')

          subplot(5,1,5)
          hold off
          plot(t, X(:,5))
          hold on
          plot(t, X(:,6))
          title('u')
          
          totalError = sum(vecnorm(truthTraj-p));
          fprintf('kp , kd, normError %f %f %d \n', kp,kd,round(totalError))
          
      
      end
  end
      
  %return
  % size(X)

  close('all')
  if false
      figure(1)
      subplot(3,1,1)
      hold all
      plot(t, X(:,1))
      plot(t, X(:,2))
      hold off
      ylabel('$\theta$ (rad)','Interpreter','latex')
      subplot(3,1,2)
      hold all
      plot(t, X(:,3))
      plot(t, X(:,4))
      hold off
      ylabel('$\dot\theta$ (rad/s)','Interpreter','latex')
      subplot(3,1,3)
      plot(t, X(:,1))
      ylabel('$\theta$ (rad)','Interpreter','latex')
      xlabel('t (sec)')
  end
  
  figure(3)
  truthTraj = params.traj(t');
  l = 0.5;
  p = l*[cos(X(:,1)');sin(X(:,1)')] + l * [cos(X(:,2)' + X(:,1)');sin(X(:,2)' + X(:,1)')];
  
  subplot(2,1,1)
  hold off
  plot(t,truthTraj(1,:))
  hold on
  plot(t,p(1,:))
  subplot(2,1,2)
  hold off
  plot(t,truthTraj(2,:))
  hold on
  plot(t,p(2,:))
  
  % animate
  if animate
      framedel = 0.05;
      slowmo = 1;
      tdraw = 0:framedel:t(end);
      Xdraw = interp1(t, X, tdraw);
      l = params.l;
      th1 = Xdraw(:,1);
      th2 = Xdraw(:,2);
      dth1 = Xdraw(:,3);
      dth2 = Xdraw(:,4);
      p1 = l*[cos(th1),sin(th1)];
      p2 = p1 + l*[cos(th1+th2),sin(th1+th2)];

      saveanim = 0;

      if saveanim
        v = VideoWriter('anim.avi','Uncompressed AVI');
        open(v);
      end

      % size(Xdraw)
      for i=1:numel(tdraw)
        figure(2)
        clf
        hold all
        % body
        line([0,p1(i,1),p2(i,1)],[0,p1(i,2),p2(i,2)],'Color',[.5,.5,.5],'Linewidth',3);
        % path
        plot(p2(1:i,1),p2(1:i,2))
        % traj
        trajNow = params.traj(tdraw(i));
        plot(trajNow(1),trajNow(2),'*')
        % t
        text(-0.1,1.5,['t = ',num2str(tdraw(i))])
        hold off
        axis equal
        xlim([-2,2]);
        ylim([-2,2]);
        drawnow
        if saveanim
          ax = gca;
          ax.Units = 'pixels';
          pos = ax.Position;
          aa = get(gcf);
          marg = 30;
          rect = [-marg, -marg, pos(3)+2*marg, pos(4)+2*marg];
          F = getframe(gca,rect);
          writeVideo(v,F);
        end
        pause(slowmo*framedel)
      end

      if saveanim
        close(v);
      end
  end
end

function [Xd] = dynamics(params, t, X,kp,kd)

  Xd = zeros(6,1);

  th1 = X(1);
  th2 = X(2);
  dth1 = X(3);
  dth2 = X(4);

  u = controllerSim(params, t, X,kp,kd);
  assert(numel(u)==2, 'output from controller should be a 2-element vector')
  u1 = -u(1);
  u2 = -u(2);

  m = params.m;
  l = params.l;
  g = params.g;

  Xd(1:2) = X(3:4);
  Xd(3:4) = [(-1).*l.^(-2).*m.^(-1).*((-2)+cos(th2).^2).^(-1).*((-1).*u1+u2+( ...
  -2).*g.*l.*m.*cos(th1)+u2.*cos(th2)+g.*l.*m.*cos(th2).*cos(th1+ ...
  th2)+2.*dth1.*dth2.*l.^2.*m.*sin(th2)+dth2.^2.*l.^2.*m.*sin(th2)+ ...
  dth1.^2.*l.^2.*m.*(1+cos(th2)).*sin(th2)),l.^(-2).*m.^(-1).*((-2)+ ...
  cos(th2).^2).^(-1).*((-1).*u1+3.*u2+(-2).*g.*l.*m.*cos(th1)+(-1).* ...
  u1.*cos(th2)+2.*u2.*cos(th2)+(-2).*g.*l.*m.*cos(th1).*cos(th2)+2.* ...
  g.*l.*m.*cos(th1+th2)+g.*l.*m.*cos(th2).*cos(th1+th2)+dth2.^2.* ...
  l.^2.*m.*(1+cos(th2)).*sin(th2)+dth1.*dth2.*l.^2.*m.*csc((1/2).* ...
  th2).^2.*sin(th2).^3+dth1.^2.*l.^2.*m.*(3.*sin(th2)+sin(2.*th2))) ...
  ];
 Xd(5:6) = u;

end

function x = trajSin(t)
  x = 0.5*[cos(pi*t); sin(pi*t)];
end

function x = trajSquare(t)
  spd = 0.2;
  if t<2.5
    x = [0.5;0] + spd * [-1; 1] * (t);
  elseif t<5
    x = [0;0.5] + spd * [-1; -1] * (t-2.5);
  elseif t<7.5
    x = [-0.5;0] + spd * [1; -1] * (t-5);
  else
    x = [0;-0.5] + spd * [1; 1] * (t-7.5);
  end
end
