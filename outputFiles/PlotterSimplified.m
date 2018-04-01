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
Y1 = A(from:(from + nSamples - 1), startPoint:nVal - 3);
from = from + nSamples;
Y2 = A(from:(from + nSamples - 1), startPoint:nVal - 3);
from = from + nSamples;
Y3 = A(from:(from + nSamples - 1), startPoint:nVal - 3);
Y = [Y1, Y2, Y3];
h = stem(X, Y, 'filled', 'LineStyle', 'none', 'Marker', '.');
xlabel('Samples');
title (sprintf("Probabillity of random movement: %1.4f", A(from, 1)));
from = from + nSamples;
h(1).Color = 'red';
h(2).Color = 'blue';
h(3).Color = 'red';
h(4).Color = 'blue';
h(5).Color = 'red';
h(6).Color = 'blue';
legend('Objects','Path')
end
