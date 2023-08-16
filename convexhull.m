function Nx = convexhull(input)
m=dlmread('input.txt');
N=m(3,1);
k=1;
for i = 1 : ( N+3 )
    if i < 3
        x(:,k)=m(k,1);
        y(:,k)=m(k,2);
    else if i==3
            k=k-1;
        else
            x(:,k)=m(i,1);
            y(:,k)=m(i,2);
            k=k+1;
            x(:,k)=m(i,3);
            y(:,k)=m(i,4);
        end
    end
    k=k+1;
end
X=[x;y];
a=[ m(1,3)-m(1,1) ; m(1,4)-m(1,2) ];
b=[ m(1,5)-m(1,1) ; m(1,6)-m(1,2) ];
for v = 1 : 2*N
    X(:,k)=[X(1,v+2);X(2,v+2)]+a;
    k=k+1;
    X(:,k)=[X(1,v+3);X(2,v+3)]+b;
    k=k+1;
end
for i=1:N
    rx=[ X(1,(2*i)+1) , X(1,2*((2*i)+1)+3) , X(1,2*((2*i)+1)+4) , X(1,(2*i)+2) , X(1,2*((2*i)+2)+3) , X(1,2*((2*i)+2)+4) ];
    ry=[ X(2,(2*i)+1) , X(2,2*((2*i)+1)+3) , X(2,2*((2*i)+1)+4) , X(2,(2*i)+2) , X(2,2*((2*i)+2)+3) , X(2,2*((2*i)+2)+4) ];
    R((2*i-1):(2*i) , 1:6)=[rx;ry];
    qq(:,i)=(size(convhull(rx,ry),1) +1);
    Nxx(1 : size(convhull(rx,ry),1) ,i)=convhull(rx,ry);
end
j=1;
s=1;
e=1;
Nx=zeros(2*N , 2*N );
for z = 1:N
    while j ~= qq(1,z)
        Nx( (2*e-1):(2*e) , j) = [ R(s,Nxx(j,z) ) ; R(s+1,Nxx(j,z)) ];
        j=j+1;
    end
    j=1;
    e=e+1;
    s=s+2;
end
end