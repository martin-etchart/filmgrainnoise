%% Film Grain Noise

close all; clear all; clc;

%% Parámetros por defecto
NombreImagen = 'johnnycash.png';

Sv = 0.01;      % Varianza del ruido de observaciones
alpha = 5;      % Pendiente de la curva de la película fotográfica
Gamma = 1.3;    % Parámetro de DAMRF.

%% Menu
while 1
choice = menu('Tratamiento estadístico de señales | Film Grain Noise',...
              'Ingresar parámetros',...
              'Degradación Sintética',...
              'Degradación real',...
              'Salir');

    switch choice
        case 1
%% Parámetros
NombreImagen = input('Nombre de la imagen: ','s');
Sv = input('Varianza del ruido de observación (0.01-0.1): ');  % Varianza del ruido de observaciones
Gamma = input('Gamma del modelo (1.1-1.8):');    % Parámetro de DAMRF.

        case 2
%% Degradación Sintética
Sintetica

        case 3
%% Degradación Real
Real

        case 4
            break;
    end
end