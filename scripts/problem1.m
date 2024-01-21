function [D,E,P] = problem1(N,R,drawingOn)

% Cálculo de los drones necesarios para formar un semiesfera
% utilizando la máxima distancia en horizontal 
% permitida entre drones

%X= [altura_anillo drones_anillo radio_anillo]

% Maximum distance in horizontal between drones of ring

d = (sqrt(3)*sqrt(pi)*sqrt((16*N-16)*sqrt(3)+3*pi)+3*pi)*R/(6*N-6);
eb = ceil(2*pi*R/d);


X=[0 eb R];
h=Inf;

while (eb<N && h>0)
 h=-((-sqrt(3)*pi*R^2+(-4*N + 4*eb + 4)*R^2 + sqrt(-3*(-(8*pi*R^2*(N - eb - 1)*sqrt(3))/3 + ((N - eb - 1)*d - pi*R)*((N - eb - 1)*d + pi*R))*R^2))*d^2*sqrt(3)/(16*pi*R^3 + 3*pi*R*d^2));    
 radius_ring = sqrt(2*R*h - h^2);
 drones = ceil(2*pi*radius_ring/d);

 
 if (isreal(drones) && isreal(h) && drones>0) 
     X=[X; R-h drones radius_ring];
 elseif ~isreal(h) || ~isreal(drones)
     X=[X; R N-eb 0];
 elseif (h==0 && drones==0)
      X=[X; R N-eb 0];
 end
 eb=eb+drones;

end

total_drones=sum(X(:,2));

% Posicionamiento de los drones
% X: [altura_anillo drones_anillo radio_anillo max_dis_diag_anillo_inf max_dista_ver_anillo_infer max_dis_hor separacion_grados ]
% P: coordenadas cartesianas de drones
% E: aristas entre drones
% D: longitud de las aritas



X_cart=[];
X_sph=[];

radio_dome = X(1,3);
theta_r_aux= 0;
% Obtención de coordenadas esféricas y cartesianas de un anillo
for i=1:size(X,1)
    altura_ring=X(i,1);
    n_drones_ring=X(i,2);
    radio_ring=X(i,3);
    theta_r=2*pi/n_drones_ring;
    phi_r= atan(altura_ring/radio_ring);
    
    % Cáculo del theta auxiliar de los drones del anillo
    if i>1 && i<size(X,1)
        diff_longitud= -0.5*(2*pi*radio_ring-2*pi*X(i-1,3));
        theta_r_aux= 0; %diff_longitud/radio_ring;
    end
    
    % Coordenadas de los drones del anillo
    for j=1:n_drones_ring
        X_sph=[X_sph; theta_r_aux+theta_r*(j-1) phi_r radio_dome];
        [X1, Y1, Z1]= sph2cart(theta_r_aux+theta_r*(j-1), phi_r, radio_dome);
        X_cart=[X_cart; X1 Y1 Z1];
    end
    
end

if drawingOn
    figure();
    plot(X_cart(:,1)*1.2, X_cart(:,2)*1.2,'o');
    % Dibujar anillos
    hold on;
    for i=1:size(X,1)
        t=0:pi/30:2*pi;
        x=X(i,3)*cos(t)*1.2;y=X(i,3)*sin(t)*1.2;
        plot(x,y);
    end
    axis 'square';
    xlabel('X [m]')
    ylabel('Y [m]')
    grid minor
    set(gca, 'FontSize', 16,'fontname','times')
    box on
end


DT = delaunayTriangulation(X_cart(:,1),X_cart(:,2),X_cart(:,3));

[K,~] = convexHull(DT);

newK=[];
H=[];
% Eliminación de las conexiones de la base
for i=1:size(K,1)
    
    vert1=K(i,1);
    vert2=K(i,2);
    vert3=K(i,3);
    Z1=X_cart(vert1,3);
    Z2=X_cart(vert2,3);
    Z3=X_cart(vert3,3);
    if Z1==0 && Z2==0 && Z3==0
        % it is a base
        H=[H vert1 vert2 vert3];
    else
        newK=[newK; K(i,:)];
    end
end

% Convex hull
H=unique(H);

TR=triangulation(newK, X_cart(:,1)*1.2,X_cart(:,2)*1.2,X_cart(:,3));

if drawingOn
    figure();
    trisurf(TR,'FaceAlpha',0.3);
    hold on;
    plot3(X_cart(:,1)*1.2,X_cart(:,2)*1.2,X_cart(:,3),'o');
    axis equal
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')
    grid minor
    set(gca, 'FontSize', 16,'fontname','times')
    box on
end


% Obtención de aristas y distancias a partir de TR
E=[];
D=[];
P=TR.Points;
% Aristas entre drones
for i=1:size(TR.ConnectivityList,1)
    
    Triangule=TR.ConnectivityList(i,:);
    E=[E; Triangule(1) Triangule(2); ...
        Triangule(1) Triangule(3); ...
        Triangule(2) Triangule(3)];
end

% Eliminación de aristas duplicadas
E =sort(E,2);
E =unique(E,'rows');

% Distancias entre drones
for i=1:size(E,1)
    D=[D; pdist2(TR.Points(E(i,1),:), TR.Points(E(i,2),:))];
end

end