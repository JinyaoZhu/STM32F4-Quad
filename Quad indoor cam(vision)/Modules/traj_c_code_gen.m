function traj_c_code_gen(x,y,z)
close all;

% x = smooth(x,5);
% y = smooth(y,5);
% z = smooth(z,5);

fid = fopen('trajectory.h','w');

fprintf(fid,'#ifndef __TRAJECTORY_H__\n');
fprintf(fid,'#define __TRAJECTORY_H__\n\n');
fprintf(fid,'#define TRAJ_LEN %d\n\n',length(x));
% X data
fprintf(fid,'const float Traj_X[TRAJ_LEN]={');
for k = 1:length(x)
if(k == length(x))
fprintf(fid,'%.8f',x(k));
else
    fprintf(fid,'%.8f,',x(k));
end
if(mod(k,15)==0)
  fprintf(fid,'\n');
end
end
fprintf(fid,'};\n\n');

% Y data
fprintf(fid,'const float Traj_Y[TRAJ_LEN]={');
for k = 1:length(y)
if(k == length(y))
fprintf(fid,'%.8f',y(k));
else
    fprintf(fid,'%.8f,',y(k));
end
if(mod(k,15)==0)
  fprintf(fid,'\n');
end
end
fprintf(fid,'};\n\n');

% Z data
fprintf(fid,'const float Traj_Z[TRAJ_LEN]={');
for k = 1:length(z)
if(k == length(z))
fprintf(fid,'%.8f',z(k));
else
    fprintf(fid,'%.8f,',z(k));
end
if(mod(k,15)==0)
  fprintf(fid,'\n');
end
end
fprintf(fid,'};\n\n');

fprintf(fid,'#endif /*__TRAJECTORY_H__*/ \n');
fclose(fid);

end