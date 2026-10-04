
%------------------------
%         RMS Error
ref = X_response(:,1);
output = X_response(:,2);

error = (output - ref);    % Errors
sq_error = (output - ref).^2;   % Squared Error
X_mse = mean((output - ref).^2)   % Mean Squared Error
X_RMSE = sqrt(mean((output - ref).^2))  % Root Mean Squared Error

X_reference_1 = X_response(2500:6000,1);
X_response_1 = X_response(2500:6000,2);
X_reference_2 = X_response(8000:13000,1);
X_response_2 = X_response(8000:13000,2);
X_RMSE = sqrt(mean((X_response_1 - X_reference_1).^2)) + sqrt(mean((X_response_2 - X_reference_2).^2))

% figure;
% hold on
% plot(ref,'r');
% plot(output,'b');
% plot(error,'m');
% plot(sq_error,'k');
% grid on
% legend('ref','output','error','sq_error');

%------------------------

ref = Y_response(:,1);
output = Y_response(:,2);

error = (output - ref);    % Errors
sq_error = (output - ref).^2;   % Squared Error
Y_mse = mean((output - ref).^2)   % Mean Squared Error
Y_RMSE = sqrt(mean((output - ref).^2))  % Root Mean Squared Error

Y_reference_1 = Y_response(2000:3000,1);
Y_response_1 = Y_response(2000:3000,2);
Y_reference_2 = Y_response(4500:6000,1);
Y_response_2 = Y_response(4500:6000,2);
Y_reference_3 = Y_response(7500:9000,1);
Y_response_3 = Y_response(7500:9000,2);
Y_reference_4 = Y_response(10500:13000,1);
Y_response_4 = Y_response(10500:13000,2);
Y_RMSE = sqrt(mean((Y_response_1 - Y_reference_1).^2)) + sqrt(mean((Y_response_2 - Y_reference_2).^2)) + sqrt(mean((Y_response_3 - Y_reference_3).^2)) + sqrt(mean((Y_response_4 - Y_reference_4).^2))

% figure;
% hold on
% plot(ref,'r');
% plot(output,'b');
% plot(error,'m');
% plot(sq_error,'k');
% grid on
% legend('ref','output','error','sq_error');

%------------------------

ref = Z_response(:,1);
output = Z_response(:,2);
error = (output - ref);    % Errors
sq_error = (output - ref).^2;   % Squared Error
Z_mse = mean((output - ref).^2)   % Mean Squared Error
Z_RMSE = sqrt(mean((output - ref).^2))  % Root Mean Squared Error

% figure;
% hold on
% plot(ref,'r');
% plot(output,'b');
% plot(error,'m');
% plot(sq_error,'k');
% grid on
% legend('ref','output','error','sq_error');


%------------------------------------
%       integral calculation

control_effort = pitch_desired;
trapz(tempo_sim,control_effort)


control_effort = roll_desired;
trapz(tempo_sim,control_effort)

control_effort = yaw_desired;
trapz(tempo_sim,control_effort)