function [ quad ] = quad_pos( pos,rot,params)
%QUAD_POS Calculates coordinates of quadrotor's position in world frame
% pos       3x1 position vector [x; y; z];
% rot       3x3 body-to-world rotation matrix
% L         1x1 length of the quad

H = params.arm_length/3;
L = params.arm_length;
r = params.arm_length/1.2;

wHb   = [rot pos(:); 0 0 0 1]; % homogeneous transformation from body to world

prop = [
        r*cos(pi/3)   r*sin(pi/3)   0 1;
        r*cos(2*pi/3) r*sin(2*pi/3) 0 1;
        r*cos(3*pi/3) r*sin(3*pi/3) 0 1;
        r*cos(4*pi/3) r*sin(4*pi/3) 0 1;
        r*cos(5*pi/3) r*sin(5*pi/3) 0 1;
        r*cos(6*pi/3) r*sin(6*pi/3) 0 1;
        r*cos(pi/3) r*sin(pi/3) 0 1;
        ];
    
p1 = [prop(:,1)+L,prop(:,2)+L,prop(:,3:4)];
p2 = [prop(:,1)-L,prop(:,2)+L,prop(:,3:4)];
p3 = [prop(:,1)-L,prop(:,2)-L,prop(:,3:4)];
p4 = [prop(:,1)+L,prop(:,2)-L,prop(:,3:4)];

quadBodyFrame  = [L L 0 1; 
                 -L L 0 1;
                 -L -L 0 1; 
                  L -L 0 1; 
                  0 0 0 1; 
                  0 0 H 1;
                  p1;
                  p2;
                  p3;
                  p4;
                  ]';
              
quadWorldFrame = wHb * quadBodyFrame;
quad           = quadWorldFrame(1:3, :);

end
