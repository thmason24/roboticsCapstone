
function u = controller(params, t, phi, phidot)
  % STUDENT FILLS THIS OUT
  % 
  % Initialize any added state like this:
  % 
  % persistent newstate
  % if isempty(newstate)
  %   % initialize
  %   newstate = 0;
  % end
  % 
  u = 0;
  Kp = 100;
  Kd = 2;
  
  u = Kp*phi + Kd*phidot;
  
end

