% clear all;close all;
% clc;
% rand('seed',0);
% randn('seed',0);
% 
% %-------------------------------------------------------------------------%
% %                        Abre o arquivo de simulação
% %-------------------------------------------------------------------------%
% open_system('quad_control_elev8_attit_training');
% 
% %-------------------------------------------------------------------------%
% %                        INICIALIZACAO DOS PARAMETROS
% %-------------------------------------------------------------------------%
% t_inicio = 0; % Tempo inicial para a simulacao de cada iteracao.
% t_final = 10; % Tempo final para a simulacao de cada iteracao.
% t_regime = 0; % Tempo inicial de regime. 
% limite_iteracoes = 2000; % Numero limite de iterações para o aprendizado
% enable_gripper = 1;
% 
% %-------------------------------------------------------------------------%
% %             CONFIGURANDO VETORES DE ACOES E PROBABILIDADES
% %-------------------------------------------------------------------------%
% %controller 1
% [cont1_kp_value,cont1_kp_prob] = init_cla(1,5,100);
% [cont1_ki_value,cont1_ki_prob] = init_cla(0,0.020,100);
% [cont1_kd_value,cont1_kd_prob] = init_cla(0,1,100);
% 
% %controller 2
% [cont2_kp_value,cont2_kp_prob] = init_cla(1,5,100);
% [cont2_ki_value,cont2_ki_prob] = init_cla(0,0.020,100);
% [cont2_kd_value,cont2_kd_prob] = init_cla(0,1,100);
% 
% %controller 3
% [cont3_kp_value,cont3_kp_prob] = init_cla(2,20,100);
% [cont3_ki_value,cont3_ki_prob] = init_cla(0,1,100);
% [cont3_kd_value,cont3_kd_prob] = init_cla(0,5,100);
% 
% %-------------------------------------------------------------------------%
% %                       CONFIGURANDO DEMAIS VETORES
% %-------------------------------------------------------------------------%
% cst = 10 * t_final * (1/0.010); % cost of a reference at 1, and the system in 0, until tf, using a dt=10ms
% costs1(1,1) = cst;
% costs1(1,2) = 1;
% costs2(1,1) = cst;
% costs2(1,2) = 1;
% costs3(1,1) = cst;
% costs3(1,2) = 1;
% k=1;
% y1(:,k) = cont3_kp_value;
% z1(:,k) = cont3_kp_prob;
% y2(:,k) = cont3_ki_value;
% z2(:,k) = cont3_ki_prob;
% y3(:,k) = cont3_kd_value;
% z3(:,k) = cont3_kd_prob;
% costrecord1=cst;
% costrecord2=cst;
% costrecord3=cst;
% selected_gains=[];

for iteration=1749:limite_iteracoes,
    tic
    disp('----------------------------------')
    disp(['Iteration : ',num2str(iteration)]);
    %-------------------------------------------------------------------------
    %  SELECIONA OS VALORES DOS PARAMETROS NAS DADAS AS 
    %  DISTRIBUICOES DE PROBABILIDADES
    %-------------------------------------------------------------------------
    [Kp_pitch]=density(rand,cont1_kp_value,cont1_kp_prob);
    [I_pitch]=density(rand,cont1_ki_value,cont1_ki_prob);
    [D_pitch]=density(rand,cont1_kd_value,cont1_kd_prob);

    [Kp_roll]=density(rand,cont2_kp_value,cont2_kp_prob);
    [I_roll]=density(rand,cont2_ki_value,cont2_ki_prob);
    [D_roll]=density(rand,cont2_kd_value,cont2_kd_prob);

    [Kp_yaw]=density(rand,cont3_kp_value,cont3_kp_prob);
    [I_yaw]=density(rand,cont3_ki_value,cont3_ki_prob);
    [D_yaw]=density(rand,cont3_kd_value,cont3_kd_prob);
    
    %-------------------------------------------------------------------------%     
    %  APLICA OS PARAMETROS ESCOLHIDOS NOS CONTROLADORES DE ESTABILIZACAO 
    %  E OBTEM OS ESTADOS DO SISTEMA NAO LINEAR. 
    %-------------------------------------------------------------------------%
    disp(strcat('Pitch:',num2str([Kp_pitch I_pitch D_pitch])));
    disp(strcat('Roll:',num2str([Kp_roll I_roll D_roll])));
    disp(strcat('Yaw:',num2str([Kp_yaw I_yaw D_yaw])));
    selected_gains = [selected_gains; Kp_pitch I_pitch D_pitch Kp_roll I_roll D_roll Kp_yaw I_yaw D_yaw];
%     save('gains_pos.mat','selected_gains');
    save workspace_inner.mat

    [tempo_sim,X,Giro_pitch,Giro_roll,Giro_yaw] = sim('quad_control_elev8_attit_training',[t_inicio t_final]); 

    %-------------------------------------------------------------------------%
    %              ANALISA OS ESTADOS FORNECIDOS PELO SISTEMA                 %
    %-------------------------------------------------------------------------%
    tam_t = length(tempo_sim);
    pos_vetor_t = find(tempo_sim >= t_regime, 1, 'first');

    reference_controller1 = Giro_pitch(pos_vetor_t :tam_t,1); 
    output_controller1 = Giro_pitch(pos_vetor_t :tam_t,2);
    tam_sinal_output1 =  length(output_controller1);
    for i = 1 : tam_sinal_output1
        square_error1(i) = (((reference_controller1(i)-output_controller1(i)).^2));%/tam_sinal_output1); % Calcula o erro quadratico medio em regime
    end
    JL_controller1 = sum(square_error1(1:tam_sinal_output1));

    reference_controller2 = Giro_roll(pos_vetor_t :tam_t,1);
    output_controller2 = Giro_roll(pos_vetor_t :tam_t,2);
    tam_sinal_output2 =  length(output_controller2);
    for i = 1 : tam_sinal_output2
        square_error2(i) = (((reference_controller2(i)-output_controller2(i)).^2));%/tam_sinal_output2); % Calcula o erro quadratico medio em regime
    end
    JL_controller2 = sum(square_error2(1:tam_sinal_output2));

    reference_controller3 = Giro_yaw(pos_vetor_t :tam_t,1);
    output_controller3 = Giro_yaw(pos_vetor_t :tam_t,2);
    tam_sinal_output3 =  length(output_controller3);
    for i = 1 : tam_sinal_output3
        square_error3(i) = (((reference_controller3(i)-output_controller3(i)).^2));%/tam_sinal_output3); % Calcula o erro quadratico medio em regime
    end
    JL_controller3 = sum(square_error3(1:tam_sinal_output3));

    %-------------------------------------------------------------------------%
    %                   Calcula o valor das recompensas
    %-------------------------------------------------------------------------%
    % O pitch zera quando ele cai de cabeça pra baixo, pra evitar dar
    % reward positivo, vou avaliar os outros custos
    if (JL_controller3>1e05) && (JL_controller2>1e05)
        JL_controller1 = JL_controller1*100;
    end
    
    costrecord1 = [costrecord1 JL_controller1];
    [costs1,beta1]=calcbeta(costs1,JL_controller1,500);

    costrecord2 = [costrecord2 JL_controller2];
    [costs2,beta2]=calcbeta(costs2,JL_controller2,500);

    costrecord3 = [costrecord3 JL_controller3];
    [costs3,beta3]=calcbeta(costs3,JL_controller3,500);

    %-------------------------------------------------------------------------%
    %                   ATUALIZA AS DISTRIBUICOES DE PROBABILIDADE 
    %-------------------------------------------------------------------------%
    [cont1_kp_value,cont1_kp_prob]=ucp(Kp_pitch,beta1,cont1_kp_value,cont1_kp_prob);
    [cont1_ki_value,cont1_ki_prob]=ucp(I_pitch,beta1,cont1_ki_value,cont1_ki_prob);
    [cont1_kd_value,cont1_kd_prob]=ucp(D_pitch,beta1,cont1_kd_value,cont1_kd_prob);

    [cont2_kp_value,cont2_kp_prob]=ucp(Kp_roll,beta2,cont2_kp_value,cont2_kp_prob);
    [cont2_ki_value,cont2_ki_prob]=ucp(I_roll,beta2,cont2_ki_value,cont2_ki_prob);
    [cont2_kd_value,cont2_kd_prob]=ucp(D_roll,beta2,cont2_kd_value,cont2_kd_prob);

    [cont3_kp_value,cont3_kp_prob]=ucp(Kp_yaw,beta3,cont3_kp_value,cont3_kp_prob);
    [cont3_ki_value,cont3_ki_prob]=ucp(I_yaw,beta3,cont3_ki_value,cont3_ki_prob);
    [cont3_kd_value,cont3_kd_prob]=ucp(D_yaw,beta3,cont3_kd_value,cont3_kd_prob);
    
    %-------------------------------------------------------------------------%  
    %          APRESENTA GRAFICAMENTE O PROGRESSO DO APRENDIZADO              %
    %-------------------------------------------------------------------------%
    if rem(iteration,5)==0
        %disp(['Iteration : ',num2str(iteration)]);
        k=k+1;
        y1(:,k) = cont3_kp_value;
        z1(:,k) = cont3_kp_prob;
        y2(:,k) = cont3_ki_value;
        z2(:,k) = cont3_ki_prob;
        y3(:,k) = cont3_kd_value;
        z3(:,k) = cont3_kd_prob;
        %figure;
        subplot(3,3,1);show(cont1_kp_value,cont1_kp_prob);
        subplot(3,3,2);show(cont1_ki_value,cont1_ki_prob);
        subplot(3,3,3);show(cont1_kd_value,cont1_kd_prob);
        
        subplot(3,3,4);show(cont2_kp_value,cont2_kp_prob);
        subplot(3,3,5);show(cont2_ki_value,cont2_ki_prob);
        subplot(3,3,6);show(cont2_kd_value,cont2_kd_prob);
        
        subplot(3,3,7);show(cont3_kp_value,cont3_kp_prob);
        subplot(3,3,8);show(cont3_ki_value,cont3_ki_prob);
        subplot(3,3,9);show(cont3_kd_value,cont3_kd_prob);
        drawnow;
    end;
    
    %-------------------------------------------------------------------------%  
    %                  CONDICAO DE PARADA DO APRENDIZADO
    %-------------------------------------------------------------------------% 
    cont1_ok=0;cont2_ok=0;cont3_ok=0;
    if (max(cont1_kp_prob)>0.8) && (max(cont1_ki_prob)>0.8) && (max(cont1_kd_prob)>0.8)
        disp('Controlador 1 aprendido');
        cont1_ok=1;
    end
    if (max(cont2_kp_prob)>0.8) && (max(cont2_ki_prob)>0.8) && (max(cont2_kd_prob)>0.8)
        disp('Controlador 2 aprendido');
        cont2_ok=1;
    end
    if (max(cont3_kp_prob)>0.8) && (max(cont3_ki_prob)>0.8) && (max(cont3_kd_prob)>0.8)
        disp('Controlador 3 aprendido');
        cont3_ok=1;
    end
    if (cont1_ok==1) && (cont2_ok==1) && (cont3_ok==1)
        break;
    end
    
    %Toggle gripper
    if (enable_gripper==0)
        enable_gripper=1;
    elseif (enable_gripper==1)
        enable_gripper=0;
    end
    disp('**********************************')
    pause(0.5);
    toc
end;
% DISPLAY RESULTS
% figure;show(cont1_kp_value,cont1_kp_prob);
% figure;show(cont1_ki_value,cont1_ki_prob);
% figure;show(cont1_kd_value,cont1_kd_prob);
x=0:50:(50*(size(y1,2)-1));
figure;
mesh(x,y1,z1);xlabel('Iteration');ylabel('X');zlabel('Prob. Density');
figure;
mesh(x,y2,z2);xlabel('Iteration');ylabel('Y');zlabel('Prob. Density');
figure;
mesh(x,y3,z3);xlabel('Iteration');ylabel('Z');zlabel('Prob. Density');
