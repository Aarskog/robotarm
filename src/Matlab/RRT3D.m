pd = [0.1654;0.146;0.2918];
tod = [-pi/2; 0; pi/2];
p0 = [0,0,robotlength+0.08];

close all
figure(1)
show(robotShow)
hold on

originBox = [0.1 0.1 0.3];
boxsize =[ .04,  .04,  .04];

vmax      = originBox +boxsize;        % vertex max
vmin      = originBox-boxsize;        % vertex min
 %box (voxel)
    vertices = [vmax(1) vmin(2) vmin(3); vmax(1) vmax(2) vmin(3); vmin(1) vmax(2) vmin(3); vmin(1) vmax(2) vmax(3); vmin(1) vmin(2) vmax(3); vmax(1) vmin(2) vmax(3); vmin; vmax ];
    faces = [1 2 3 7; 1 2 8 6; 1 6 5 7; 7 5 4 3; 2 8 4 3; 8 6 5 4];
    h= patch('Vertices',vertices,'Faces',faces,'FaceColor','green');
    set(h,'FaceAlpha',1);



clear nodes

rs = norm(pd);


x_max = robotlength;
y_max = robotlength;
z_max = robotlength;

obstacle = [0,0,0,0];

EPS = 0.07;
r = 0.2;

erroracceptance = 0.5;

numNodes = 300;        

q_start.coord = p0;
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = pd;
q_goal.cost = 0;

nodes(1) = q_start;

xlim([-x_max x_max])
ylim([-y_max y_max])
zlim([0 robotlength+0.1])

for i = 1:1:numNodes
    fprintf('RRT running. %2.2f%% done \n',i/numNodes*100)
    q_rand = [(rand(1)-1/2)*2*x_max (rand(1)-1/2)*2*y_max rand(1)*z_max];
    plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  [0 0.4470 0.7410])
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if ((norm(nodes(j).coord - q_goal.coord))<erroracceptance)
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist_3d(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    q_new.coord = steer3d(q_rand, q_near.coord, val, EPS);
    if ~rayBoxIntersection(q_rand, q_rand-q_near.coord, vmin, vmax)
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;

        % Within a radius of r, find all existing nodes
        q_nearest = [];
        %r = 50;
        %r = .01;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if (~rayBoxIntersection(nodes(j).coord, nodes(j).coord-q_new.coord, vmin, vmax))&&((dist_3d(nodes(j).coord, q_new.coord)) <= r)
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end

        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;

        % Iterate through all nearest neighbors to find alternate lower
        % cost paths

        for k = 1:1:length(q_nearest)
            if q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'g');            
                hold on
            end
        end

        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end

        % Append to nodes
        nodes = [nodes q_new];
    end
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist_3d(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end
%
% show(robotShow)
% h= patch('Vertices',vertices,'Faces',faces,'FaceColor','green');
%     set(h,'FaceAlpha',1);
%     zlim([0 robotlength+0.1])
% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
datatas = q_end.coord;
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'r', 'LineWidth', 4);
    hold on
    q_end = nodes(start);
    datatas = [datatas,q_end.coord'];
end
fnplt(cscvn(datatas));
%%
figure(414)
fnplt(cscvn(datatas))





