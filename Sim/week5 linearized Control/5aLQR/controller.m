
function u = controller(params, t, X)
  % You have full state feedback available
  %K = [ -0.2774  -31.4238   -0.3457   -3.8648];
  %K = [-0.1000  -11.3571   -0.1246   -1.3937];  %gain = 100
  K = [-0.0316   -3.6206   -0.0394   -0.4409];  %gain = 1000
  u = -K*X;
  %u = -u;
  %fprintf('t %f\n', t)
  % After doing the steps in simLinearization, you should be able to substitute the linear controller u = -K*x
  
end

