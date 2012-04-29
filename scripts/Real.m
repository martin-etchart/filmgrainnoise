%% Degradación Real

im = double(imread(NombreImagen));
[M,N,C]=size(im);

%% Film Grain Denoise
s = zeros(M,N,C);
for i=1:C
    s(:,:,i) = fgdenoise(im(:,:,i),Sv,Gamma);
end

%% Perfil de evolución
figure
plot(im(150,:),'b'); hold on
plot(s(150,:),'r'); hold on
title('Perfil de evolución para fila 150'); legend('Ruido real','Filtrada')
ylabel('Intensidad'); xlabel('Indices'); axis('tight')

%% Imágenes
figure
imshow(uint8(im));
title('Ruido Real')

figure
imshow(uint8(s));
title('Imagen denoised')

figure
imshow((im(:,:,1)-s(:,:,1)),[]);
title('Residuo')