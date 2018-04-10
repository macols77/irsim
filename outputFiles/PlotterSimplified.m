startPoint = 4;
nTryouts = 2;
nSamples = 301;
nVal = 8;
from = 1;
X = 1:nSamples;
formatSpec = '%f %u %u %u %u %u %u %u';
fileID = fopen('randomMovementOutputFinal', 'r');
size = [nVal Inf];
A = fscanf(fileID, formatSpec, size);
A = A';

for i = 1: nTryouts 
Y1 = A(from:(from + nSamples - 1), startPoint:nVal - 3);
from = from + nSamples;
Y2 = A(from:(from + nSamples - 1), startPoint:nVal - 3);
from = from + nSamples;
Y3 = A(from:(from + nSamples - 1), startPoint:nVal - 3);
from = from + nSamples;
Y4 = A(from:(from + nSamples - 1), startPoint:nVal - 3);
Y = [Y1, Y2, Y3, Y4];

if i == 1
    Y_random = Y1 + Y2 + Y3 + Y4;
else 
    Y_no_random = Y1 + Y2 + Y3 + Y4;
end

figure(i)
h = stem(X, Y, 'filled', 'LineStyle', 'none', 'Marker', '.');
xlabel('Samples');
title (sprintf("Probabillity of random movement: %1.4f", A(from, 1)));
[h(1).Color, h(3).Color, h(5).Color, h(7).Color] = deal('red');
[h(2).Color, h(4).Color, h(6).Color, h(8).Color] = deal('blue');
legend('Objects','Path')

from = from + nSamples;
end

% D = [1:1:301; 1:1:301];
% D = D';
% 
% for i = 1 : nSamples
%     for j=1 : 2
%         D(i, j) = 4;
%     end
% end
% 
% Y_no_random = Y_no_random ./ D;
% Y_random = Y_random ./ D;
% Y = [Y_random, Y_no_random];
% 
% figure(10);
% h = stem(X, Y, 'filled', 'LineStyle', 'none', 'Marker', '.');
% xlabel('Samples');
% title (sprintf("Overlayed"));
% [h(1).Color, h(2).Color] = deal('red');
% [h(3).Color, h(4).Color] = deal('blue');
% legend('O p = 0.0', '!O p = 0.0','O p = 0.5', '!O p = 0.5');
