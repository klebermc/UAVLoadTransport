for i=1:max(size(X_response)); 
    error_x(i) = X_response(i,1) - X_response(i,2);
end
disp(strcat('X RMS=',num2str(sqrt(mean(error_x.^2)),4)))

for i=1:max(size(Y_response)); 
    error_y(i) = Y_response(i,1) - Y_response(i,2);
end
disp(strcat('Y RMS=',num2str(sqrt(mean(error_y.^2)),4)))

for i=1:max(size(Z_response)); 
    error_z(i) = Z_response(i,1) - Z_response(i,2);
end
disp(strcat('Z RMS=',num2str(sqrt(mean(error_z.^2)),4)))


disp(strcat('Int control effort=',num2str(trapz(control_effort)*0.01,4)))
