function J = fgdenoise(I,Sv,Gamma)
%I = FGDENOISE(J,Sv,Gamma) Film Grain Denoise
%   fgdenoise remueve el ruido de grano de la imagen de entrada de un canal
%   y devuelve la imagen J con ruido removido.
%
%   Parametros:
%   Gamma = 1.3;        % Parámetro de DMRF 
%   Sv = 0.1;           % Ruido de Observación % sigma_v^2 = 0.1
%   
%   Implementado en base al artículo:
%   Importance Sampling-Based Unscented Kalman Filter for Film-Grain Noise
%   Removal. Subrahmanyam, Rajagopalan, Aravind.

[M,N]=size(I);
Sampler = 2;        % Sampling uniforme

%% Parámetros

nx = 1;             % Dimensión del estado
nu = 0;             % Dimensión del ruido de estadi
nv = 1;             % Dimensión del ruido de observaciones
na = nx + nu + nv;  % Dimensión del estado aumentado para UKF

alpha = 5;          % Pendiente curva de pelicula fotográfica

% Parametros Unscented Transformation
But = 0;
Aut = 1;
Kappa = 1;
Lda = Aut^2*(na + Kappa) - na;
c1 = sqrt(na+Lda);
Wm      = (1/(2*(Lda + na)))*ones(1,1+2*na);
Wm(1)   = 2*Lda*Wm(1);
Wc      = Wm;
Wc(1)   = Wm(1)+(1-Aut^2+But);

% Parametros Importance Sampling
L = 100;            % Numero de Muestras
w = zeros(L,1);     % Weights del IS
s1 = 400; % gamma de Cauchy pdf

%% Monte Carlo sampling UKF

% Inicialización
P = 100;
s = I;
Ns  = zeros(4,2);
Ns(:,2) = [1; 2; 1; 2];     % Distancias

tic
% Filtro
for i=2:M
    for j=2:N-1
        
        %   Neighbors
        Ns(:,1) = [s(i,j-1) s(i-1,j-1:j+1)]';
        
        switch Sampler
            case 1
                % Cauchy Sampler
                k1 = 0.25*ones(1,4)*Ns(:,1);
                u = rand(L,1);
                z = k1+s1*tan(pi*(u-0.5)); % Inverse Cauchy cdf
                
                % Cálculo de eta(k,1) = sum(((z(k)-Ns(:,1)).^2)./Ns(:,2))
                Z = repmat(z,1,4);
                Ns1 = repmat(Ns(:,1)',L,1);
                Ns2 = repmat(Ns(:,2)',L,1);
                eta = ((Z-Ns1).^2./Ns2)*ones(4,1);
                
                % Importance Sampling
                w = exp(-Gamma.*log(1+(1/Gamma)*((1/P)*eta)))./...
                    1./(pi*s1*(1+((z-k1)./s1).^2));          
            case 2
                % MonteCarlo Sampling (Sampling Uniforme)
                u = 255*rand(L,1);
                z = u;
                Z = repmat(z,1,4);
                Ns1 = repmat(Ns(:,1)',L,1);
                Ns2 = repmat(Ns(:,2)',L,1);
                eta = ((Z-Ns1).^2./Ns2)*ones(4,1);
                w = exp(-Gamma.*log(1+(1/Gamma)*((1/P)*eta)))*255;              
        end
        W = ones(1,L)*w;
        Mup = (w'*z)/W;
        Sgp = (w'*(z - Mup).^2)/W;
                                     
        % Predicción
        x_ = Mup;
        P_ = Sgp;
        
%         % Aumento estado
%         xa = [ x_ ; 0];
%         Pa = [ P_ 0 ; 0 Sv ];
        
        % Matriz de sigma points estimados
        Xa(:,1)     = [x_;0];            
        Xa(:,2)     = [x_ + c1*sqrt(P_);0];
        Xa(:,3)     = [x_ - c1*sqrt(P_);0];
        Xa(:,4)     = [x_ ; + c1*sqrt(Sv)];
        Xa(:,5)     = [x_ ; - c1*sqrt(Sv)];
        % Otras yerbas
        Xx = Xa(1,:);
        Xv = Xa(2,:);
        
        % Propagación en el exposure domain
        Y_ = Xx.*10.^(Xv/alpha);
        % Según otro paper...
            % Y_ = Xx + 5*Xv.*Xx.^(0.5);
            
        % Estimación de estadisticas
        y_ = Wm*Y_';
        Pyy = (Wc.*(Y_-y_))*(Y_-y_)';
        Pxy = (Wc.*(Xx-x_))*(Y_-y_)';
        % Ganancia de Kalman
        K = Pxy./Pyy;
        % Updates
        x = x_ + K*(s(i,j)-y_);
        P = P_ - K*Pyy*K;
        
        s(i,j) = x;
    end
%     clc
%     display('Progreso(%): ')
%     disp(round(100*i/M))
end
toc

J = s;
end
