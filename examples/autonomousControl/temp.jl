using Juno
function y(x)
  x^2
end

Juno.@step y(8)
