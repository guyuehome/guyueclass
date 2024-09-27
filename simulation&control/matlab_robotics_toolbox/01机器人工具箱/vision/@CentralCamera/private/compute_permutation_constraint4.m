function K=compute_permutation_constraint4(V)

% Copyright (C) <2007>  <Francesc Moreno-Noguer, Vincent Lepetit, Pascal Fua>
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the version 3 of the GNU General Public License
% as published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
% General Public License for more details.       
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.
%
% Francesc Moreno-Noguer, CVLab-EPFL, September 2007.
% fmorenoguer@gmail.com, http://cvlab.epfl.ch/~fmoreno/ 



%[B11,B12,...,B33]=lambda1*v1+lambda2*v2+lambda3*v3

N=size(V,2); %dimension of the kernel
n=4; %dimension of Bij
idx=[1 2 3 4; 2 5 6 7; 3 6 8 9; 4 7 9 10];

%1.-Generation of the first set of equations Bii.Bjj=Bij.Bii  (n(n-1)/2 eqs).
nrowsK=n*(n-1)/2+n*(n-1)*n/2;
ncolsK=N*(N+1)/2;
K=zeros(nrowsK,ncolsK);

t=1;
for i=1:n
    for j=i+1:n
        offset=1;
        for a=1:N
            for b=a:N
                if a==b
                    K(t,offset)=V(idx(i,i),a)*V(idx(j,j),a)-V(idx(i,j),a)*V(idx(i,j),a);
                else
                    K(t,offset)=V(idx(i,i),a)*V(idx(j,j),b)-V(idx(i,j),a)*V(idx(i,j),b)+...
                                V(idx(i,i),b)*V(idx(j,j),a)-V(idx(i,j),b)*V(idx(i,j),a);
                end
                offset=offset+1;
            end
            
        end
        t=t+1;
        %fprintf('t:%d\t offset:%d\n',t,offset);
    end
end


for k=1:n
    for j=k:n
        for i=1:n
            if (i~=j & i~=k)
                offset=1;
                for a=1:N
                    for b=a:N
                        if a==b
                            K(t,offset)=V(idx(i,j),a)*V(idx(i,k),a)-V(idx(i,i),a)*V(idx(j,k),a);
                        else
                            K(t,offset)=V(idx(i,j),a)*V(idx(i,k),b)-V(idx(i,i),a)*V(idx(j,k),b)+...
                                        V(idx(i,j),b)*V(idx(i,k),a)-V(idx(i,i),b)*V(idx(j,k),a);
                        end
                        offset=offset+1;
                    end
                    
                end
                t=t+1;
                %fprintf('t:%d\t offset:%d\n',t,offset);
            end
        end
    end
end
                
                
         
                    
                    
