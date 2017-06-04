function R = YPRToDCM(yaw,pitch,roll)
% convert Euler angle to rotation matrix

% R - 3 x 3, rotation matrix from body frame to world frame

s_y = sin(yaw);
c_y = cos(yaw);
s_p = sin(pitch);
c_p = cos(pitch);
s_r = sin(roll);
c_r = cos(roll);

R = [c_y*c_p  c_y*s_p*s_r-s_y*c_r  c_y*s_p*c_r+s_y*s_r;
     s_y*c_p  s_y*s_p*s_r+c_y*c_r  s_y*s_p*c_r-c_y*s_r;
     -s_p     c_p*s_r              c_p*c_r            ];
 
end