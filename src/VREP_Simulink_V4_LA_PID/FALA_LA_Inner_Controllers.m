%%
%**************************************************************************
%**************************************************************************

%               INSTITUTO TECNOL�GICO DE AERON�UTICA (ITA)
%                AUTOR: S�RGIO RONALDO BARROS DOS SANTOS
%                             Ph.D STUDENT

%                          APRENDIZADO POR REFOR�O 
%                      ALGORITMO LEARNING AUTOMATA (LA)
%             TREINAMENTO DA MALHA DE CONTROLE DE ESTABILIZA��O
%                                 01/08/2011

%**************************************************************************
%**************************************************************************

%%

%run mdl_quadrotor.m

%-------------------------------------------------------------------------%
%                        INICIALIZA��O DOS PARAMETROS                     %
%-------------------------------------------------------------------------%

%function Valores_Saida = Algoritmo_aprendizagem(NIterMax,NIter_Media,t_inicio,t_final)
warning off;
clc; clear; close all;

NIterMax=5000; % N�mero m�ximo de itera��es. 
NIter_Media = 10; % N�mero de itera��es para a m�dia.
REC_Media = 10;    
Novo_aprendizado = 0; % Se igual a '1' Aproveita o conhecimento j� existente.

RP = -20; % Penalidade.
RG = 20; % Gratifica��o.

t_inicio = 0; % Tempo inicial para a simula��o de cada itera��o.
t_final = 10; % Tempo final para a simula��o de cada itera��o.
t_regime = 5; % Tempo inicial de regime. 
nl = 20; % N�mero de linhas das matrizes ou o n�mero de poss�veis pontos.
selected_gains = [0 0 0 0 0 0 0 0 0];

Pitch_controller_coef = 3; % N�mero de colunas das matrizes ou 
Roll_controller_coef = 3;  % n�mero de par�metros dos controladores
Yaw_controller_coef = 3;   % de estabiliza��o.
%Altitude_controller_coef = 3;

% Pitch_controller_coef_inicial = [0.1 0.001 2];   % Faixa de valores ajustados para os  
% Pitch_controller_coef_final   = [0.1 0.001 2]; % par�metros do controlador de pitch 
% 
% Roll_controller_coef_inicial = [0.1 0.001 2];    % Faixa de valores ajustados para os
% Roll_controller_coef_final   = [0.1 0.001 2];  % par�metros do controlador de roll
% 
% Yaw_controller_coef_inicial = [40 0.01 2];      % Faixa de valores ajustados para os
% Yaw_controller_coef_final   = [40 0.01 2];  % par�metros do controlador de yaw

Pitch_controller_coef_inicial = [0 0 0];   % Faixa de valores ajustados para os  
Pitch_controller_coef_final   = [5 0.1 2]; % par�metros do controlador de pitch 

Roll_controller_coef_inicial = [0 0 0];    % Faixa de valores ajustados para os
Roll_controller_coef_final   = [5 0.1 2];  % par�metros do controlador de roll

Yaw_controller_coef_inicial = [0 0 0];      % Faixa de valores ajustados para os
Yaw_controller_coef_final   = [10 0.1 4];  % par�metros do controlador de yaw

iter_ant = 0; % Inicializa a variavel do loop de itera��es
contador = 0;
rec_avg_pitch =0;rec_avg_roll =0;rec_avg_yaw =0;rec_avg_altitude =0;  
i0=1;
inicio = 0;

%%
%-------------------------------------------------------------------------%
%        CONFIGURA��O DAS MATRIZES (COEFICIENTES E PROBABILIDADES)        %
%-------------------------------------------------------------------------%

if Novo_aprendizado == 0;
    
   if(Pitch_controller_coef >= 1) 
        Pitch_controller_mat_coef = zeros(nl,Pitch_controller_coef); % Gera a matriz de par�metros
        Pitch_controller_mat_prob = (1/nl) + zeros(nl,Pitch_controller_coef); %Gera a matriz de probabilidade
        for j = 1 : Pitch_controller_coef  
            Pitch_controller_mat_coef(1,j) = Pitch_controller_coef_inicial(1,j)+((Pitch_controller_coef_final(1,j)-...
            Pitch_controller_coef_inicial(1,j))/nl);
            for i = 2:nl
                Pitch_controller_mat_coef(i,j) =(((Pitch_controller_coef_final(1,j)-Pitch_controller_coef_inicial(1,j))/nl)+...
                Pitch_controller_mat_coef(i-1,j)); 
            end
        end
    end

   if(Roll_controller_coef>= 1) 
        Roll_controller_mat_coef = zeros(nl,Roll_controller_coef); % Gera a matriz de par�metros
        Roll_controller_mat_prob= (1/nl) + zeros(nl,Roll_controller_coef); %Gera a matriz de probabilidade
        for j = 1 : Roll_controller_coef 
            Roll_controller_mat_coef(1,j) = Roll_controller_coef_inicial(1,j)+((Roll_controller_coef_final(1,j)-...
            Roll_controller_coef_inicial(1,j))/nl);            
            for i = 2:nl
               Roll_controller_mat_coef(i,j) =(((Roll_controller_coef_final(1,j)-Roll_controller_coef_inicial(1,j))/nl)+...
               Roll_controller_mat_coef(i-1,j)); 
            end
        end
   end
    
   if(Yaw_controller_coef>= 1) % Gera a matriz de par�metros
        Yaw_controller_mat_coef = zeros(nl,Yaw_controller_coef); % Gera a matriz de probabilidade
        Yaw_controller_mat_prob = (1/nl) + zeros(nl,Yaw_controller_coef);
        for j = 1 : Yaw_controller_coef  
            Yaw_controller_mat_coef(1,j) = Yaw_controller_coef_inicial(1,j)+((Yaw_controller_coef_final(1,j)-...
            Yaw_controller_coef_inicial(1,j))/nl);
            for i = 2:nl
                Yaw_controller_mat_coef(i,j) =(((Yaw_controller_coef_final(1,j)-Yaw_controller_coef_inicial(1,j))/nl)+...
                Yaw_controller_mat_coef(i-1,j)); 
            end
        end
   end

%    if(Altitude_controller_coef >= 1) % Gera a matriz de par�metros
%         Altitude_controller_mat_coef = zeros(nl,Altitude_controller_coef); % Gera a matriz de probabilidade
%         Altitude_controller_mat_prob = (1/nl) + zeros(nl,Altitude_controller_coef);
%         for j = 1 : Altitude_controller_coef  
%              Altitude_controller_mat_coef(1,j) = Altitude_controller_coef_inicial(1,j)+((Altitude_controller_coef_final(1,j)-...
%              Altitude_controller_coef_inicial(1,j))/nl);
%             for i = 2:nl
%                 Altitude_controller_mat_coef(i,j) =(((Altitude_controller_coef_final(1,j)-Altitude_controller_coef_inicial(1,j))/nl)+...
%                 Altitude_controller_mat_coef(i-1,j)); 
%             end
%         end
%    end
          
end

%%
%-------------------------------------------------------------------------%
%                     INICIALIZA O LOOP DE TREINAMENTO
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%                     GERA AS MATRIZES DE SOMAT�RIAS                      %
%-------------------------------------------------------------------------%

 
for iter=1:NIterMax; % Inicializa o loop de treinamento
 
   if(Pitch_controller_coef >= 1)  
       for j = 1 : Pitch_controller_coef
        Pitch_controller_mat_prob_sum(:,j) = Pitch_controller_mat_prob(:,j);
         for i=2:nl
              Pitch_controller_mat_prob_sum(i,j)= Pitch_controller_mat_prob_sum(i-1,j)+ Pitch_controller_mat_prob(i,j);
         end
       end
   end
   
   if(Roll_controller_coef >= 1)
       for j = 1 : Roll_controller_coef  
           Roll_controller_mat_prob_sum(:,j) = Roll_controller_mat_prob(:,j);
           for i=2:nl
                Roll_controller_mat_prob_sum(i,j)= Roll_controller_mat_prob_sum(i-1,j)+ Roll_controller_mat_prob(i,j);
           end
       end
   end
    
   if(Yaw_controller_coef >= 1)
       for j = 1 : Yaw_controller_coef  
           Yaw_controller_mat_prob_sum(:,j) = Yaw_controller_mat_prob(:,j);
           for i=2:nl
                Yaw_controller_mat_prob_sum(i,j)= Yaw_controller_mat_prob_sum(i-1,j)+ Yaw_controller_mat_prob(i,j);
           end
       end
   end
    
%    if(Altitude_controller_coef >= 1)
%        for j = 1 : Altitude_controller_coef  
%            Altitude_controller_mat_prob_sum(:,j) = Altitude_controller_mat_prob(:,j);
%            for i=2:nl
%                 Altitude_controller_mat_prob_sum(i,j)= Altitude_controller_mat_prob_sum(i-1,j)+ Altitude_controller_mat_prob(i,j);
%            end
%        end
%    end
    
   if floor(iter/5)==iter/5
        fprintf( 'Itera��o = %d\n', iter);
   end
    
   iter_ant = iter;

%%
%-------------------------------------------------------------------------
%             SELECIONA OS VALORES NAS MATRIZES DE PARAMETROS           %%
%-------------------------------------------------------------------------
   
   if(Pitch_controller_coef >= 1)  
        for j = 1 : Pitch_controller_coef
            Pitch_controller_rd_lna(j) = rand(1);
            Pitch_controller_sel_linha(j) = find( Pitch_controller_mat_prob_sum(:,j)>Pitch_controller_rd_lna(j),1,'first');
            Pitch_controller_coef_escolhidos(j) = Pitch_controller_mat_coef(Pitch_controller_sel_linha(j),j);
        end
        Kp_pitch = Pitch_controller_coef_escolhidos(1); 
        Ki_pitch = Pitch_controller_coef_escolhidos(2); 
        Kd_pitch = Pitch_controller_coef_escolhidos(3); 
   end
          
   if(Roll_controller_coef >= 1)  
        for j = 1 : Roll_controller_coef
            Roll_controller_rd_lna(j) = rand(1);
            Roll_controller_sel_linha(j) = find(Roll_controller_mat_prob_sum(:,j)>Roll_controller_rd_lna(j),1,'first');
            Roll_controller_coef_escolhidos(j) = Roll_controller_mat_coef(Roll_controller_sel_linha(j),j);
        end  
        Kp_roll = Roll_controller_coef_escolhidos(1);
        Ki_roll = Roll_controller_coef_escolhidos(2);
        Kd_roll = Roll_controller_coef_escolhidos(3);
   end
   
   if(Yaw_controller_coef >= 1)  
        for j = 1 : Yaw_controller_coef
            Yaw_controller_rd_lna(j) = rand(1);
            Yaw_controller_sel_linha(j) = find(Yaw_controller_mat_prob_sum(:,j)>Yaw_controller_rd_lna(j),1,'first');
            Yaw_controller_coef_escolhidos(j) = Yaw_controller_mat_coef(Yaw_controller_sel_linha(j),j);
        end  
        Kp_yaw = Yaw_controller_coef_escolhidos(1);
        Ki_yaw = Yaw_controller_coef_escolhidos(2);
        Kd_yaw = Yaw_controller_coef_escolhidos(3);
   end
   
%    if(Altitude_controller_coef >= 1)  
%         for j = 1 : Altitude_controller_coef
%             Altitude_controller_rd_lna(j) = rand(1);
%             Altitude_controller_sel_linha(j) = find(Altitude_controller_mat_prob_sum(:,j)>Altitude_controller_rd_lna(j),1,'first');
%             Altitude_controller_coef_escolhidos(j) = Altitude_controller_mat_coef(Altitude_controller_sel_linha(j),j);
%         end  
%         coef_alt_a = Altitude_controller_coef_escolhidos(1);
%         coef_alt_b = Altitude_controller_coef_escolhidos(2);
%         coef_alt_c = Altitude_controller_coef_escolhidos(3);
%    end
    
     
%%  
%-------------------------------------------------------------------------%     
%  APLICA OS PAR�METROS ESCOLHIDOS NOS CONTROLADORES DE ESTABILIZA��O 
%  E OBTEM OS ESTADOS DO SISTEMA N�O LINEAR. 
%-------------------------------------------------------------------------%
   try 
     disp('Roll:')
     disp([Kp_roll Ki_roll Kd_roll])
     disp('Pitch:')
     disp([Kp_pitch Ki_pitch Kd_pitch])
     disp('Yaw:')
     disp([Kp_yaw Ki_yaw Kd_yaw])
     selected_gains = [selected_gains; Kp_roll Ki_roll Kd_roll Kp_pitch Ki_pitch Kd_pitch Kp_yaw Ki_yaw Kd_yaw];
     save('gains.mat','selected_gains');
     [tempo_sim,X,X_response,Y_response,Z_response,Giro_pitch,Giro_roll,Giro_yaw] = sim('quad_control_elev8',[t_inicio t_final]);
     %[tempo_sim,X,X_response,Y_response,Z_response,dx_response,dy_response,dz_response] = sim('controle',[t_inicio t_final]);   

   end
%%
%-------------------------------------------------------------------------%
%              ANALISA OS ESTADOS FORNECIDOS PELO SISTEMA                 %
%-------------------------------------------------------------------------%

   tam_t = length(tempo_sim);
   pos_vetor_t = find(tempo_sim >= t_regime, 1, 'first');
 
   sinal_desejado = X_response(pos_vetor_t :tam_t,1); 
   sinal_output = X_response(pos_vetor_t :tam_t,2);
   tam_sinal_output =  length(sinal_output);
   for i = 1 : tam_sinal_output
        square_erro(i) = (((sinal_desejado(i)-sinal_output(i)).^2)/tam_sinal_output); % Calcula o erro quadr�tico m�dio
   end                                                                                % em regime da resposta de pitch
   mse = sum(square_erro(1:tam_sinal_output));
       
   sinal_desejado_roll = Y_response(pos_vetor_t :tam_t,1);
   sinal_output_roll = Y_response(pos_vetor_t :tam_t,2);
   tam_sinal_output_roll =  length(sinal_output_roll);
   for i = 1 : tam_sinal_output_roll
        square_erro_roll(i) = (((sinal_desejado_roll(i)-sinal_output_roll(i)).^2)/tam_sinal_output_roll); % Calcula o erro quadr�tico m�dio
   end                                                                                                    % em regime da resposta de roll
   mse_roll = sum(square_erro_roll(1:tam_sinal_output_roll));
 
   sinal_desejado_yaw = Z_response(pos_vetor_t :tam_t,1);
   sinal_output_yaw = Z_response(pos_vetor_t :tam_t,2);
   tam_sinal_output_yaw =  length(sinal_output_yaw);
   for i = 1 : tam_sinal_output_yaw
        square_erro_yaw(i) = (((sinal_desejado_yaw(i)-sinal_output_yaw(i)).^2)/tam_sinal_output_yaw); % Calcula o erro quadr�tico m�dio
   end                                                                                                % em regime da resposta de yaw
   mse_yaw = sum(square_erro_yaw(1:tam_sinal_output_yaw));
        
      
%%  
%-------------------------------------------------------------------------%
%              ARMAZENA O HISTORICO DO ERRO QUADR�TICO M�DIO              %
%-------------------------------------------------------------------------%
 
   corte_pitch = 0.05; 
   if mse <= corte_pitch % Valor m�ximo de corte
       Hist_mse_regime(iter) = mse; % Armazena avalia��o da resposta de pitch
   else
       Hist_mse_regime(iter) = corte_pitch; 
   end
     
   corte_roll = 0.05;
   if mse_roll <= corte_roll % Valor m�ximo de corte
       Hist_mse_regime_roll(iter) = mse_roll; % Armazena avalia��o da resposta de roll
   else
       Hist_mse_regime_roll(iter) = corte_roll; 
   end
  
   corte_yaw = 0.05;
   if mse_yaw <= corte_yaw % Valor m�ximo de corte
       Hist_mse_regime_yaw(iter) = mse_yaw; % Armazena avalia��o da resposta de yaw
   else
       Hist_mse_regime_yaw(iter) = corte_yaw; 
   end
   
%%
%-------------------------------------------------------------------------%  
%                 CALCULA A MEDIA DAS ULTIMAS 20 ITERA��ES                %
%-------------------------------------------------------------------------%
 
   if iter <= NIter_Media % Define intervalo para a m�dia
      Intervalo_Medio_A = iter;
   else
      Intervalo_Medio_A = NIter_Media;
   end
 
   Hist_1 = Hist_mse_regime(iter-(Intervalo_Medio_A - 1):iter); % Calcula a m�dia do erro da resposta de pitch
   Hist_media_mse_regime(iter) = mean(Hist_1); % Armazena m�dia calculada
         
   Hist_roll = Hist_mse_regime_roll(iter-(Intervalo_Medio_A - 1):iter); % Calcula a m�dia do erro da resposta de roll
   Hist_media_mse_regime_roll(iter) = mean(Hist_roll); % Armazena m�dia calculada
       
   Hist_yaw = Hist_mse_regime_yaw(iter-(Intervalo_Medio_A - 1):iter); % Calcula a m�dia do erro da resposta de yaw
   Hist_media_mse_regime_yaw(iter) = mean(Hist_yaw); % Armazena m�dia calculada
     

%% 
%-------------------------------------------------------------------------%
%        AVALIA��ES DOS ERROS OBTIDOS NOS ESTADOS (FUN��O OBJETIVO)       %
%-------------------------------------------------------------------------%
  
       if Hist_mse_regime(iter) >= corte_pitch 
           R_pitch = 0; 
       elseif Hist_mse_regime(iter) < corte_pitch 
           R_pitch = 1*RG;
       end
       Hist_funcao_j(iter) = R_pitch;
     
       if Hist_mse_regime_roll(iter) >= corte_roll
           R_roll = 0; 
       elseif Hist_mse_regime_roll(iter) < corte_roll
           R_roll = 1*RG;
       end
       Hist_funcao_j_roll(iter) = R_roll;
      
       if Hist_mse_regime_yaw(iter) >= corte_yaw
           R_yaw = 0; 
       elseif Hist_mse_regime_yaw(iter) < corte_yaw
           R_yaw = 1*RG;
       end
       Hist_funcao_j_yaw(iter) = R_yaw;
       
%%
%-------------------------------------------------------------------------%      
%                       CALCULA A M�DIA DA AVALIA��O                      %
%-------------------------------------------------------------------------%

   if iter < NIter_Media % Define intervalo para a m�dia
        Intervalo_Medio_B = iter;
   else
        Intervalo_Medio_B = NIter_Media;
   end

   Hist_j_pitch(iter) = R_pitch;
   Hist_j_2_pitch = Hist_j_pitch(iter-(Intervalo_Medio_B - 1):iter); % Calcula a m�dia do valor da recompensa
   Hist_media_j_pitch(iter) = mean(Hist_j_2_pitch );                 % Armazena m�dia calculada
     
   Hist_j_roll(iter) = R_roll;
   Hist_j_2_roll = Hist_j_roll(iter-(Intervalo_Medio_B - 1):iter); % Calcula a m�dia do valor da recompensa 
   Hist_media_j_roll(iter) = mean(Hist_j_2_roll);                  % Armazena m�dia calculada
       
   Hist_j_yaw(iter) = R_yaw;
   Hist_j_2_yaw = Hist_j_yaw(iter-(Intervalo_Medio_B - 1):iter); % Calcula a m�dia do valor da recompensa 
   Hist_media_j_yaw(iter) = mean(Hist_j_2_yaw);                  % Armazena m�dia calculada
       
         
%%
%-------------------------------------------------------------------------%
%                   ATUALIZA A MATRIZ DE PROBABILIDADE                    %
%-------------------------------------------------------------------------%
              
   if(Pitch_controller_coef >= 1) % Atualiza a matriz de probabilidade relacionada os par�metros de controle de pitch
        for j=1 : Pitch_controller_coef 
             Pitch_controller_mat_prob(Pitch_controller_sel_linha(j),j)= ((1+(R_pitch/100))*  Pitch_controller_mat_prob(Pitch_controller_sel_linha(j),j));
             if Pitch_controller_mat_prob(Pitch_controller_sel_linha(j),j) < 0
                 Pitch_controller_mat_prob(Pitch_controller_sel_linha(j),j) = 0;
             end
             Pitch_controller_mat_prob(1:nl,j)= Pitch_controller_mat_prob(1:nl,j)/sum(Pitch_controller_mat_prob(1:nl,j));
        end
        conv_curva_pitch = max(Pitch_controller_mat_prob,[],1);  
        Pitch_resposta_convergencia(iter) = min(conv_curva_pitch);
   end
   Hist_rec_pitch(iter) = (R_pitch/RG);
       
   if(Roll_controller_coef >= 1)  % Atualiza a matriz de probabilidade relacionada os par�metros de controle de roll
        for j=1 : Roll_controller_coef 
            Roll_controller_mat_prob(Roll_controller_sel_linha(j),j)= ((1+(R_roll/100))* Roll_controller_mat_prob(Roll_controller_sel_linha(j),j));
            if Roll_controller_mat_prob(Roll_controller_sel_linha(j),j) < 0
                Roll_controller_mat_prob(Roll_controller_sel_linha(j),j) = 0;
            end
                Roll_controller_mat_prob(1:nl,j)= Roll_controller_mat_prob(1:nl,j)/sum(Roll_controller_mat_prob(1:nl,j));
        end
        conv_curva_roll = max(Roll_controller_mat_prob,[],1);  
        Roll_resposta_convergencia(iter) = min(conv_curva_roll);
   end
   Hist_rec_roll(iter) = (R_roll/RG);
   
   if(Yaw_controller_coef >= 1) % Atualiza a matriz de probabilidade relacionada os par�metros de controle de yaw
       for j=1 : Yaw_controller_coef 
             Yaw_controller_mat_prob(Yaw_controller_sel_linha(j),j)= ((1+(R_yaw/100))* Yaw_controller_mat_prob(Yaw_controller_sel_linha(j),j));
             if Yaw_controller_mat_prob(Yaw_controller_sel_linha(j),j) < 0
                  Yaw_controller_mat_prob(Yaw_controller_sel_linha(j),j) = 0;
             end
              Yaw_controller_mat_prob(1:nl,j)= Yaw_controller_mat_prob(1:nl,j)/sum(Yaw_controller_mat_prob(1:nl,j));
        end
        conv_curva_yaw = max(Yaw_controller_mat_prob,[],1);  
        Yaw_resposta_convergencia(iter) = min(conv_curva_yaw);
   end
   Hist_rec_yaw(iter) = (R_yaw/RG);       
      
   
 %%
%-------------------------------------------------------------------------%      
%                       CALCULA e A RECOMPENSA M�DIA                        %
%-------------------------------------------------------------------------%

   if iter < NIter_Media % Define intervalo para a m�dia
        Intervalo_Medio_C = iter;
   else
        Intervalo_Medio_C = NIter_Media;
   end
   
   contador = contador+1;
   
   if (contador == REC_Media)
       inicio = inicio + 1;
       rec_avg_pitch(inicio)=mean(Hist_rec_pitch(i0:i0+REC_Media-1));
       rec_avg_roll(inicio)=mean(Hist_rec_roll(i0:i0+REC_Media-1));
       rec_avg_yaw(inicio)=mean(Hist_rec_yaw(i0:i0+REC_Media-1));
%        rec_avg_altitude(inicio)=mean(Hist_rec_altitude(i0:i0+REC_Media-1));
       i0=i0+REC_Media;

      contador = 0;

   end 
       
    
 %%
%-------------------------------------------------------------------------%  
%          APRESENTA GRAFICAMENTE O PROGRESSO DO APRENDIZADO              %
%-------------------------------------------------------------------------%
 
   
   if(Pitch_controller_coef >= 1) % Plota pitch
      
     if floor(iter/5)==iter/5 %|| Pitch_resposta_convergencia(iter)> 0.95
          
          Nf=1;
          figure(Nf); set(gcf,'DoubleBuffer','on','Position',[10 200 400 500]);
          clf;
          title('Pitch Controller')
                       
          subplot 411;
          hold off;
          plot(Hist_mse_regime(1:iter));
          ylim([0 corte_pitch])
          xlabel(sprintf('No. de itera��es'))
          ylabel(sprintf('Erro'));
          hold on;
          plot(Hist_media_mse_regime(1:iter),'r');
          %axis([0 iter -1.e-1 1e-1])
          grid on;
      
          subplot 412;
          hold off;
          plot((Hist_funcao_j(1:iter)));
          xlabel(sprintf('No. de itera��es'));
          ylabel(sprintf('Refor�o'));
          hold on;
          plot(Hist_media_j_pitch(1:iter),'r');
          grid on;
      
          subplot 413;
          hold off;
          plot(rec_avg_pitch,'-*');
          ylim([-1 1]);
          xlabel(sprintf('No. de intervalos'));
          ylabel(sprintf('Recompensa\n m�dia'));
          hold on;
          grid on;
          
          subplot 414;
          hold off;
          plot( Pitch_resposta_convergencia(1:iter));
          xlabel(sprintf('No. de itera��es'));
          ylabel(sprintf('Converg�ncia'));
          grid on;
      
          drawnow;
          
          
     end
   end    
    
  
   if(Roll_controller_coef >= 1) % Plota roll
        
     if floor(iter/5)==iter/5 %|| Roll_resposta_convergencia(iter)> 0.95
            
          Nf=2;
          figure(Nf); set(gcf,'DoubleBuffer','on','Position',[420 200 400 500]);
          clf;
          title('Roll Controller')
          
          subplot 411;
          hold off;
          plot(Hist_mse_regime_roll(1:iter));
          ylim([0 corte_roll])
          xlabel(sprintf('No. de itera��es'))
          ylabel(sprintf('Erro'));
          hold on;
          plot( Hist_media_mse_regime_roll(1:iter),'r');
          %axis([0 iter -1.e-1 1e-1])
          grid on;
      
          subplot 412;
          hold off;
          plot((Hist_funcao_j_roll(1:iter)));
          xlabel(sprintf('No. de itera��es'));
          ylabel(sprintf('Refor�o'));
          hold on;
          plot(Hist_media_j_roll(1:iter),'r');
          grid on;
      
          subplot 413;
          hold off;
          plot(rec_avg_roll,'-*');
          ylim([-1 1]);
          xlabel(sprintf('No. de intervalos'));
          ylabel(sprintf('Recompensa\n m�dia'));
          hold on;
          grid on;
          
          subplot 414;
          hold off;
          plot( Roll_resposta_convergencia(1:iter));
          xlabel(sprintf('No. de itera��es'));
          ylabel(sprintf('Converg�ncia'));
          grid on;
      
          drawnow;
          
     end       
   end     
     
  
   if(Yaw_controller_coef >= 1) % Plota yaw 
        
     if floor(iter/5)==iter/5 %|| Roll_resposta_convergencia(iter)> 0.95
        
          Nf=3;
          figure(Nf); set(gcf,'DoubleBuffer','on','Position',[830 200 400 500]);
          clf;
          title('Yaw Controller')
            
          subplot 411;
          hold off;
          plot(Hist_mse_regime_yaw(1:iter));
          ylim([0 corte_yaw])
          xlabel(sprintf('No. de itera��es'))
          ylabel(sprintf('Erro'));
          hold on;
          plot( Hist_media_mse_regime_yaw(1:iter),'r');
          %axis([0 iter -1.e-1 1e-1])
          grid on;
              
          subplot 412;
          hold off;
          plot((Hist_funcao_j_yaw(1:iter)));
          xlabel(sprintf('No. de itera��es'));
          ylabel(sprintf('Refor�o'));
          hold on;
          plot(Hist_media_j_yaw(1:iter),'r');
          grid on;
      
          subplot 413;
          hold off;
          plot(rec_avg_yaw,'-*');
          ylim([-1 1]);
          xlabel(sprintf('No. de intervalos'));
          ylabel(sprintf('Recompensa\n m�dia'));
          hold on;
          grid on;
          
          subplot 414;
          hold off;
          plot( Yaw_resposta_convergencia(1:iter));
          xlabel(sprintf('No. de itera��es'));
          ylabel(sprintf('Converg�ncia'));
          grid on;
      
          drawnow;
     end       
   end      

%------------------------------------------------------------------------%
%         PLOTA A RESPOSTA AO FINAL DO PROCESSO DE APRENDIZADO           %
%------------------------------------------------------------------------%
      if Pitch_resposta_convergencia(iter)> 0.99
          if Roll_resposta_convergencia(iter)> 0.99
              if Yaw_resposta_convergencia(iter)> 0.99
                 break;
              end;
          end;
      end;
  end