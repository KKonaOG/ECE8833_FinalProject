function [point] = RandomPoint(a,b)
%RANDOMPOINT Generates a random (floating point) number between a and b.
%   Derived from: https://www.mathworks.com/help/matlab/ref/rand.html
    point = a + (b - a).*rand(1, 2);
end

