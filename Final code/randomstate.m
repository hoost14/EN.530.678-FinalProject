function qrand=randomstate(ndim,dim)
%  generate a random configuration
%  e.g configuration of EduMIP:(x, y, omega)
%  randomstate(3,[xmin xmax, ymin , ymax, omegamin, omegamax]

l=floor(length(dim)/2);
if (ndim~=l)
    disp('Error: ndim and the given dimensions do not match');
    qrand=[];
    return;
end

for j=1:ndim
    qrand(j)=  dim(2*j-1)+(dim(2*j) -dim(2*j-1))*rand();
end

qrand=qrand';

end

