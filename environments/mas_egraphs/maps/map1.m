w = 400;
h = 400;
room_h = h/4;
door_size = room_h/4;
Map = ones(h, w);
wall_w = 10;

% make border around map
Map(1:h, 1:wall_w) = 0;
Map(1:h, h-wall_w:w) = 0;
Map(1:wall_w, 1:w) = 0;
Map(h-wall_w:h, 1:w) = 0;

% vertical wall
leftcorridor_x = 150;
rightcorridor_x = 250;

% corridor
Map(1:h, leftcorridor_x:leftcorridor_x+wall_w) = 0;
Map(1:h, rightcorridor_x:rightcorridor_x+wall_w) = 0;

% horizontal walls
for i = 1:4
    % left room
    Map(room_h*i:room_h*i+wall_w, 1:leftcorridor_x) = 0;
    % left room vertical door
    Map(room_h*i - (room_h/2) - door_size: room_h*i - (room_h/2),...
        leftcorridor_x:leftcorridor_x+wall_w) = 1;
    % left room horizontal door
    if(i ~=4)
        Map(room_h*i:room_h*i+wall_w, ...
            leftcorridor_x/2-door_size:leftcorridor_x/2) = 1;
    end
    
    % right room
    Map(room_h*i:room_h*i+wall_w, rightcorridor_x:w) = 0;
    % right room vertical door
    Map(room_h*i - (room_h/2) - door_size: room_h*i - (room_h/2),...
        rightcorridor_x:rightcorridor_x+wall_w) = 1;
    % right room horizontal door
    if(i ~=4)
       Map(room_h*i:room_h*i+wall_w, ...
            rightcorridor_x + rightcorridor_x/2-door_size:3*rightcorridor_x/2) = 1;
    end
end


h = figure(1);
imshow(mat2gray(Map));
print('office.png', '-dpng');
