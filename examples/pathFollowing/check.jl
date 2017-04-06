

t0 = 3.0; # this is the initial time that the interpolation is defined at
t0_sample = 3.0 - 2*eps();  # sample time

if  t0_sample < t0
  if t0!=(t0_sample+t0)/2
    error("increase t0!")
  else
    warn("increased t0 a little")
  end
end
