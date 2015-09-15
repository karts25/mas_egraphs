function office_map(map_id)
rng(map_id);
w = 300;
h = 300;

wall_w = 5;
num_expts = 15;
num_goals = 10;
num_robots = 2;
obstacle_cost = 0.4;
room_h = h/4;
door_size = room_h/4;
Map = ones(h, w);
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

%% make unknown map
Map_unknown = Map;

% close some doors at random
horiz_doors_left = randperm(3);
horiz_doors_right = randperm(3);

for ctr = 1
    % left room horizontal door    
    i = horiz_doors_left(ctr);
    Map_unknown(room_h*i:room_h*i+wall_w, ...
        leftcorridor_x/2-door_size:leftcorridor_x/2) = obstacle_cost;
end

for ctr = 1
    % right room horizontal door
     i = horiz_doors_right(ctr);
     Map_unknown(room_h*i:room_h*i+wall_w, ...
                 3*rightcorridor_x/2-door_size:3*rightcorridor_x/2) = obstacle_cost;        
 end

vert_doors_left = randperm(4);
vert_doors_right = randperm(4);

 for ctr = 1
     i = vert_doors_left(ctr);
     % left room vertical door
     Map_unknown(room_h*i - (room_h/2) - door_size: room_h*i - (room_h/2),...
         leftcorridor_x:leftcorridor_x+wall_w) = obstacle_cost;
     
     i = vert_doors_right(ctr);
     % right room vertical door
     Map_unknown(room_h*i - (room_h/2) - door_size: room_h*i - (room_h/2),...
         rightcorridor_x:rightcorridor_x+wall_w) = obstacle_cost;
 end

% add some random obstacles
% num_obstacles = 20;
% obstacles_w = 2;
% obstacles_h = 2;
% i = 1;
% while(i < num_obstacles)
%     obstacles_x = randi(h-5);
%     obstacles_y = 5+ randi(w-5);
%     if any(Map_unknown(obstacles_x-obstacles_h:obstacles_x + obstacles_h, ...
%             obstacles_y - obstacles_w:obstacles_y + obstacles_w) <= obstacle_cost)
%         continue;
%    end
%     
%     Map_unknown(obstacles_x-obstacles_h:obstacles_x + obstacles_h, obstacles_y - obstacles_w:obstacles_y + obstacles_w) = obstacle_cost;
%     i = i+1;
% end

imshow(mat2gray(Map_unknown));

map_unknown_fname = sprintf('office_unknown%d.pgm', map_id);
imwrite(Map_unknown, map_unknown_fname);

imwrite(Map, 'office_known.pgm');

expt_i = 1;

goal_x = nan(num_expts, num_goals);
goal_y = nan(num_expts, num_goals);
while(expt_i <= num_expts)
    fprintf('Generating experiment %d\n', expt_i);
    Map_unknown_copy = Map_unknown;
    goal_x_expt = nan(1,num_goals);
    goal_y_expt = nan(1,num_goals);
    goal_i = 1;
    while(goal_i <= num_goals)
        goal_x_expt(goal_i) = 20 + randi(w-40); 
        goal_y_expt(goal_i) = 20 + randi(h-40);
        if any(any(Map_unknown(goal_x_expt(goal_i)-10: goal_x_expt(goal_i)+10, ...
            goal_y_expt(goal_i)-10:goal_y_expt(goal_i)+10) <= obstacle_cost))
                continue;
        end
        goal_x(expt_i, goal_i) = goal_x_expt(goal_i);
        goal_y(expt_i, goal_i) = goal_y_expt(goal_i);
        
        Map_unknown_copy(goal_x_expt(goal_i)-2: goal_x_expt(goal_i)+2, ...
            goal_y_expt(goal_i)-2:goal_y_expt(goal_i)+2) = obstacle_cost;
       
        goal_i = goal_i + 1;

    end
    figure(2);
    imshow(mat2gray(Map_unknown_copy));   
    expt_i = expt_i + 1;
end
filename = sprintf('r%1dg%02d_%d.yaml', num_robots, num_goals, map_id);
fileID = fopen(filename,'w');
fprintf(fileID, 'experiments:\n\n');

goal_x = 0.1*goal_x;
goal_y = 0.1*goal_y;
for expt_i = 1:num_expts
    fprintf(fileID,'  - test: test_%d\n', expt_i);  
    fprintf(fileID,'    num_goals: %d\n', num_goals); 
    fprintf(fileID,'    start: %.2f %.2f %.2f %.2f\n', 14, 15, 0, 1.58);
    %fprintf(fileID,'    start: %.2f %.2f %.2f %.2f\n', 16, 15, 0, 0.78);
    fprintf(fileID,'    start: %.2f %.2f %.2f %.2f\n', 15, 15, 1, 1.58);
        
    for goal_i = 1:num_goals
        fprintf(fileID,'    goal: %.2f %.2f %.2f\n', 0.1*h - goal_y(expt_i, goal_i), goal_x(expt_i, goal_i), 0);
    end
end

fclose(fileID);

