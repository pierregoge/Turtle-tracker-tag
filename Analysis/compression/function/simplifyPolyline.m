function [B]= simplifyPolyline(A, tol)
    % simplifyPolyline() simplifies polyline to the required detail by removing vertices. 
    % The detail level is controlled by tol. The simplification is done by decimating 
    % the polyline to fewer number of vertices by using Ramer–Douglas–Peucker algorithm.
    % 
    % [B] =  simplifyPolyline(A, tol) returns a simplified polyline,
    %        B, of input polyline A. 
    % 
    % Input 
    %   A    : polyline coordinates [nPoints x nDim]
    %   tol  : threshold distance, if the point lies outside the threshold then
    %          it is included i.e.coarser the value greater the simplification.
    % Output 
    %   B    : simplified polyline coordinates [mPoints x nDim], 
    %          where mPoints <= nPoints
    %   
    % @author    Naveen Somasundaram, Novemeber 2021
    
      nPoints = size(A, 1);
      if nPoints <= 2
        B = A;
        return 
      end
      
      % Fetch Points
      x1 = A(1, :);             % Start Point
      x2 = A(nPoints, :);       % End Point
      x  = A(2:nPoints - 1, :); % Intermediate points
      % Find shortest distance from intermediate points (x) to line 
      % segment joining (x1) and (x2)
      t = -sum((x2 - x1).* (x1 - x), 2) / sum((x2 -x1).^2);
      t = min(1, max(t, 0));
      d = sqrt(sum((x1 + t.*(x2-x1) - x).^2, 2));
      
      % Find largest distance and corresponding point 
      [dmax, ind] = max(d);
      ind = ind + 1;
      
      % If dmax is greater than tol, split the line into two and repeat; else
      % return the line approximated by start and end point
      if(dmax > tol)
        B1 = simplifyPolyline(A(1:ind, :), tol);
        B2 = simplifyPolyline(A(ind:nPoints, :), tol);
        B  = [B1(1:end-1, :); B2(1:end, :)];
      else
        B  = [A(1, :); A(nPoints, :)];
      end
 end