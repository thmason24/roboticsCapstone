
function u = controller(params, t, x, xd)
  % x = current position
  % xd = current velocity

  % Use params.traj(t) to get the reference trajectory
  % e.g. (x - params.traj(t)) represents the instaneous trajectory error
  % params can be initialized in the initParams function, which is called before the simulation starts
  truth = params.traj(t);
  params.Kp = 27;
  params.Kd = -8;
  
  % SOLUTION GOES HERE -------------
  u =params.Kp*(truth-x) + params.Kd * xd;
end