Tm = 10e-3;
datos = ["velocidad", "lastAng", "currentAng", "refVal", "kp", "kd", "ki"];
leyenda = ["velocidad", "lastAng", "currentAng", "refVal", "kp", "kd", "ki"];

fichero = 'datos.txt';

formatSpec = sprintf('%s: %%f', datos(1));

for n=2:length(datos)
    formatSpec = sprintf('%s, %s: %%f', formatSpec, datos(n));
end

formatSpec = sprintf('%s\n', formatSpec);



fileID = fopen(fichero, 'r');
A = fscanf(fileID,formatSpec, [length(datos) Inf]);

t = (1:length(A))*Tm;

figure(402)
clf()
hold on

plot(t,A);
grid
legend(leyenda)

title("")
xlabel("t (s)");
ylim([-5,6])