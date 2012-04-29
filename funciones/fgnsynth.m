function J = fgnsynth (I,Sn,Alpha)
% function J = fgnsynth (I,Sn,Alpha)

I = double(I);
[M,N,C] = size(I);

J = I.*10.^((sqrt(Sn).*randn(M,N,C))/Alpha);

end
