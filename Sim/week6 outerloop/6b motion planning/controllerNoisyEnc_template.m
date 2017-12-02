
function u = controllerNoisyEnc(params, t, obs, th, dth)
  % This is the starter file for the week5 assignment
  % Now you only receive noisy measurements for theta, and must use your EKF from week 3 to filter the data and get an estimate of the state
  % obs = [ay; az; gx] (same as last week)
  % New for 6b: you also have access to params.traj(t)

  % Template code (same as last week)
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);

  % fill this out
  u=0;
end

function xhatOut = EKFupdate(params, t, z)
  % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
  % You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
  % Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.

  % Student completes this
  xhatOut = [0; 0];
end
