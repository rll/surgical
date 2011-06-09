function norms = normArray(A, dim)
% Calculates L2 norm of array A of vectors
%
% A = array of vectors.
% If dim is passed in, assume vectors go in that dimension. If dim=1,
% assume A is an array of column vectors, and return a 1xn array of norms.
%
% If dim is not passed in, checks to see if A is an 2xn,nx2,3xn, or nx3
% array, in that order. 

if numel(size(A)) > 2
    error(['normArray not defined for arrays of dimension: ' num2str(numel(size(A)))]);
end

if nargin >= 2
    sqs = A.*A;
    sums = sum(sqs, dim);
    norms = sqrt(sums);
    return
end


sz = size(A);
if sz(1) == 2
    normsSq = A(1,:).*A(1,:) + A(2,:).*A(2,:);
    norms = sqrt(normsSq);
elseif sz(2) == 2
    normsSq = A(:,1).*A(:,1) + A(:,2).*A(:,2);
    norms = sqrt(normsSq);
elseif sz(1) == 3
    normsSq = A(1,:).*A(1,:) + A(2,:).*A(2,:) + A(3,:).*A(3,:);
    norms = sqrt(normsSq);
elseif sz(2) == 3
    normsSq = A(:,1).*A(:,1) + A(:,2).*A(:,2) + A(:,3).*A(:,3);
    norms = sqrt(normsSq);
end