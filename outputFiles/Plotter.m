startPoint = 4;
nTryouts = 4;
nSamples = 201;
nVal = 8;
from = 1;
X = 1:nSamples;
formatSpec = '%f %u %u %u %u %u %u %u';
fileID = fopen('randomMovementOutput', 'r');
size = [nVal Inf];
A = fscanf(fileID, formatSpec, size);
A = A';
for i = 1: nTryouts 
figure(i)
Y1 = A(from:(from + nSamples - 1), startPoint:nVal);
from = from + nSamples;
Y2 = A(from:(from + nSamples - 1), startPoint:nVal);
from = from + nSamples;
Y3 = A(from:(from + nSamples - 1), startPoint:nVal);
Y = [Y1, Y2, Y3];
h = stem(X, Y, 'filled', 'LineStyle', 'none', 'Marker', '.');
xlabel('Samples');
title (sprintf("Probabillity of random movement: %1.4f", A(from, 1)));
from = from + nSamples;
[h(1).Color, h(6).Color, h(11).Color] = deal('red');
[h(2).Color, h(7).Color, h(12).Color] = deal('green');
[h(3).Color, h(8).Color, h(13).Color] = deal('blue');
[h(4).Color, h(9).Color, h(14).Color] = deal('yellow');
[h(5).Color, h(10).Color, h(15).Color] = deal('black');
legend('Objects','Path','Preys','Nests', 'Packets Attended')
end
