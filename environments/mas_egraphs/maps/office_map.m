w = 300;
h = 300;
room_h = h/4;
door_size = room_h/4;
Map = ones(h, w);
wall_w = 5;

obstacle_cost = 0.4;

% make border around map
Map(1:h, 1:wall_w) = 0;
Map(1:h, h-wall_w:w) = 0;
Map(1:wall_w, 1:w) = 0;
Map(h-wall_w:h, 1:w) = 0;

% vertical wall
leftcorridor_x = 120;
rightcorridor_x = 180;

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
            3*rightcorridor_x/2-door_size:3*rightcorridor_x/2) = 1;
    end
end

figure(1);
imshow(mat2gray(Map));
imwrite(Map,'office_known.pgm');

%% make unknown map
Map_unknown = Map;

% close some doors at random
horiz_doors_left = randperm(3);
horiz_doors_right = randperm(3);

for ctr = 1:3
    % left room horizontal door    
    i = horiz_doors_left(ctr);
    Map_unknown(room_h*i:room_h*i+wall_w, ...
        leftcorridor_x/2-door_size:leftcorridor_x/2) = obstacle_cost;
end

%for ctr = 1:3
%     right room horizontal door
%     i = 3; %horiz_doors_right(ctr);
%     Map_unknown(room_h*i:room_h*i+wall_w, ...
%             3*rightcorridor_x/2-door_size:3*rightcorridor_x/2) = obstacle_cost;        
% end

vert_doors_left = randperm(4);
vert_doors_right = randperm(4);

% for ctr = 1:0
%     i = vert_doors_left(ctr);
%     % left room vertical door
%     Map_unknown(room_h*i - (room_h/2) - door_size: room_h*i - (room_h/2),...
%         leftcorridor_x:leftcorridor_x+wall_w) = obstacle_cost;
%     
%     i = vert_doors_right(ctr);
%     % right room vertical door
%     Map_unknown(room_h*i - (room_h/2) - door_size: room_h*i - (room_h/2),...
%         rightcorridor_x:rightcorridor_x+wall_w) = obstacle_cost;
% end

% add some random obstacles
% num_obstacles = 10;
% obstacles_w = door_size/4;
% obstacles_h = 2;
% obstacles_x = h/2+randperm(h/2, num_obstacles);
% obstacles_y = 5+ randperm(w, num_obstacles);
% for i = 1:num_obstacles
%     Map_unknown(obstacles_x(i)-obstacles_h:obstacles_x(i) + obstacles_h, obstacles_y(i) - obstacles_w:obstacles_y(i) + obstacles_w) = obstacle_cost;
% end
figure(2);
imshow(mat2gray(Map_unknown));
imwrite(Map_unknown, 'office_unknown2.pgm');