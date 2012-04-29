%% Degradación Sintetica

im = double(imread(NombreImagen));
[M,N,C]=size(im);

%% Ruido Sintetico
re = fgnsynth(im,Sv,alpha);

%% Film Grain Denoise
s = zeros(M,N,C);
for i=1:C
    s(:,:,i) = fgdenoise(re(:,:,i),Sv,Gamma);
end
%% Medidas de desempeño

ISNR = 10*log10(sum((re(:) - im(:)).^2)/sum((s(:) - im(:)).^2))
MSE = round(mean((s(:)-im(:)).^2))

%% Perfil de evolución
figure
plot(re(150,:),'b'); hold on
plot(s(150,:),'r'); hold on
title('Perfil de evolución para fila 150'); legend('Ruidosa','Filtrada')
ylabel('Intensidad'); xlabel('Indices'); axis('tight')

%% Imágenes
figure
imshow(uint8(im));
title('Original')

figure
imshow(uint8(re));
title('Degradada')

figure
imshow(uint8(s));
title('Estimada')

figure
imshow((im(:,:,1)-s(:,:,1)),[]);
title('Residuo')