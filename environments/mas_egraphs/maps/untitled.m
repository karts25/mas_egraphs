temp = ones(100,100);
temp(1:20,:) = 0;
temp(40:60,:) = 0.1;
imwrite(temp, 'testmap.bmp');