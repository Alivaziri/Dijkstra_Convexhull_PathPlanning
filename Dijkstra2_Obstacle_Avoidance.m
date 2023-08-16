function path = Dijkstra2_Obstacle_Avoidance(input)
clc
clear
pause('on')
Nx = convexhull('input.txt')
m=dlmread('input.txt');
N=m(3,1);
k=1;
h=1;
for i = 1 : ( N+2 )
    if i < 3
        x(:,k)=m(k,1);
        y(:,k)=m(k,2);
    else
        x(:,k)=Nx(2*i-5,1);
        y(:,k)=Nx(2*i-4,1);
        k=k+1;
        x(:,k)=Nx(2*i-5,2);
        y(:,k)=Nx(2*i-4,2);
        k=k+1;
        x(:,k)=Nx(2*i-5,3);
        y(:,k)=Nx(2*i-4,3);
        k=k+1;
        x(:,k)=Nx(2*i-5,4);
        y(:,k)=Nx(2*i-4,4);
        k=k+1;
        x(:,k)=Nx(2*i-5,5);
        y(:,k)=Nx(2*i-4,5);
        k=k+1;
        x(:,k)=Nx(2*i-5,6);
        y(:,k)=Nx(2*i-4,6);
    end
    k=k+1;
end
XX=[x;y];
i=1;
kk=1;
while i <= size(XX,2)
    X(:,kk)=[XX(1,i),XX(2,i)];
    i=i+1;
    kk=kk+1;
    if (XX(1,i)==0) && (XX(2,i)==0)
        i=i+1;
    end
end
X1(:,1)=X(:,1);
ww=0;
for i = 1 : size(X,2)
    ww=0;
    for j = 1 : (i-1)
        if (X(1,j)==X(1,i)) && (X(2,j)==X(2,i))
            ww=ww+1;
        end
    end
    if ww==0
        X1(:,h)=X(:,i);
        h=h+1;
    end
end
n=size(X1,2);
% Group of uncrossed points. All points are members of this group at the
% beginning.
nopass=1:n;
% Distance of each point defined as infinite at the beginning of the
% algorithm.
dis(1,1:n)=inf;
% Setting final point as current point
CP=2;
% Setting its distance to zero.
dis(2)=0;
% Matrice of possible destination points for each point.
Possible=zeros(n);
n1=size(X,2);
for i = 1 : n
    a=0;
    for u = 1 : n
        if i~=u
            aa=0;
            
            for z = 1 : n1
                if z < (n1-2)
                    L=[ X1(1,i) ; X1(2,i) ];
                    M=[ X1(1,u) ; X1(2,u) ];
                    A =[ X(1,z+2) ; X(2,z+2) ] ;
                    B =[ X(1,z+3) ; X(2,z+3) ] ;
                    alfa = inv( [L-M , -(B-A)] ) * (A - M);
                    aa(:,z) = (  abs(alfa(1,1) >=1) |  abs(alfa(1,1) <=0) |  abs(alfa(2,1) >=1)|  abs(alfa(2,1) <=0) );
                end
            end
            if all( aa )
                a = a + 1;
                Possible(i,a)=u;
            end
        end
    end
end
X2=X;
X=X1;
Possible
while CP~=1
    % Omitting Current Point from the list of unpassed points
    nopass = setdiff(nopass,CP);
    % Displaying
    dis
    pause(2) %Pausing execution to display distance changes for the vertices
    % GATHERING POSSIBLE POINTS FOR NEXT MOVE TOGETHER
    CurrentPossible = intersect(Possible(CP,:),nopass);
    % NUMBER OF POSSIBLE VERTICES FOR LATER USE
    P = size(CurrentPossible);
    P = P(2);
    % MATRIX FOR COMPARING "DISTANCES"
    comp = zeros(2,P);
    % CORRECTING "DISTANCE" OF POINTS
    for i=1:P
        delX = X(:,CP)-X(:,CurrentPossible(i));
        if dis(CurrentPossible(i))> dis(CP)+sqrt(delX(1)^2+delX(2)^2)
            dis(CurrentPossible(i)) = dis(CP)+sqrt(delX(1)^2+delX(2)^2);
        end
        comp(:,i)=[dis(CurrentPossible(i));CurrentPossible(i)];
    end
    % DETECTING MINIMUM "DISTANCE" AMONG ADJACENT VERTICES
    mindis = min(comp(1,:));
    [i,j] = find(comp(1,:)== mindis);
    % J =size(j);
    % J = J(2);
    % if J~=1
    %     j=j(1);
    % end
    CP = comp(2,j);
end
% Path Planning
CP = 1; % SETTING START POINT
nopass = 1:n;
Path(1)=1;
counter = 2;
while CP~=2
    nopass = setdiff(nopass,CP);
    CurrentPossible = intersect(Possible(CP,:),nopass);
    P = size(CurrentPossible);
    P = P(2);
    flag = 0;
    i=1;
    while flag~=1
        if isempty(CurrentPossible)
            CP=CP+1;
           % nopass = setdiff(nopass,CP);
            %CurrentPossible = intersect(Possible(CP,:),nopass);
           % CP = CurrentPossible(i);
            break;
        end
        if (i <= P)
            
            delX = X(:,CP)-X(:,CurrentPossible(i));
            if dis(CP) == dis(CurrentPossible(i))+sqrt(delX(1)^2+delX(2)^2)
                CP = CurrentPossible(i);
                Path(counter)=CP;
                counter = counter+1;
                flag=1;
            end
            i=i+1;
        else
            nopass = setdiff(nopass,CP);
            CurrentPossible = intersect(Possible(CP,:),nopass);
            CP = CurrentPossible(i-1);
            break;
        end
    end
end

Path
p=Path;
hold off
rx1(:,1)=1;
ry1(:,1)=1;
for i=4:(N+3)
    rx=[ m(i,1) , m(i,3) ];
    ry=[ m(i,2) , m(i,4) ];
    plot(rx,ry)
    hold on
end

for g=1:size(p,2)
    rx1(:,g)=[ X(1,p(1,g)) ];
    ry1(:,g)=[ X(2,p(1,g)) ];
    
    plot(rx1,ry1)
    hold on
end

end