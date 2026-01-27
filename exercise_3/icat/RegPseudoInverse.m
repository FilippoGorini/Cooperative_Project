% Regularized Pseudo Inverse
% 
% This function computes the pseudo inverse of X, performing a singular
% value oriented regularization.
% It uses the two parameters lambda and threshold to compute a bell shaped
% regularizing factor. No regularization occurs if the singular value is
% above the threshold value.
% 
% Return values:
% - Xinv the computed regularized pseudo inverse
% - p a vector of the regularizing parameters
% - diag_S a vector with all the singular values of the input matrix

function [Xinv, p, diag_S] = RegPseudoInverse(X, lambda, threshold)
    [U,S,V] = svd(X);

    diag_S = diag(S);       % NEW: return full vector of singular values

    sizeS = size(S);
    Sinv = zeros(sizeS(2), sizeS(1));
    for i = 1:min(sizeS)
        p(i) = DecreasingBellShapedFunction(0, threshold, 0, lambda, S(i,i));
        Sinv(i,i) = S(i,i)/(S(i,i)^2 + p(i));
    end
    Xinv = V*Sinv*U';
end

% NB: we modified the function to also return the full vector of singular
% values, which is useful to debug situations where we are near
% singularities and regularization is applied when doing the
% pseudoinversion