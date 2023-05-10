function   d = in_distance(a,b)
    % Eucledian Distance
    d = sqrt( ( a(:,1) - b (:,1) ) .^ 2  + ( a(:,2) - b(:,2) ) .^ 2 );
end