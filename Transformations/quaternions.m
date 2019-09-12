clear;close all;clc
vetor = [1 1 1]; %vetor a ser girado
eixo = [1 0 0]; %eixo de giro
angulo = pi/4; %lembrando que myquatrotate gira 2*angulo!!
v = [0 vetor]'
q = [cos(angulo/2) sin(angulo/2)*eixo]' %lembrando que myquatrotate gira 2*angulo!!
k = myquatmult(q,q)
norma_q = myquatnorm(q)
q_normalizado = myquatnormalize(q)
norma_normalizado = myquatnorm(q_normalizado)
inv = myquatinv(q)
iden = myquatmult(q,inv)
noth = myquatmult(q,iden)
rotate = myquatrotate(q,v) 
% rotate = myquatrotateframe(q,v)
rotate_frame = myquatrotateframe(q,v)
n = quatrotate(q',vetor)   %igual!  /\

% return
plot3([0 1],[0 0],[0 0],'black','linewidth',2); hold on; grid on;
plot3([0 0],[0 1],[0 0],'black','linewidth',2);
plot3([0 0],[0 0],[0 1],'black','linewidth',2);

plot3([0 vetor(1)],[0 vetor(2)],[0 vetor(3)],'magenta'); %vetor original
plot3([0 eixo(1)],[0 eixo(2)],[0 eixo(3)],'red'); %eixo de giro
plot3([0 n(1)],[0 n(2)],[0 n(3)],'blue'); %vetor girado (matlab)
plot3([0 rotate(1)],[0 rotate(2)],[0 rotate(3)],'green'); %vetor girado (mylib) 


return

% rotated_frame = myquatrotateframe(q,v)

%% Plots
close all
%Plota os 3 eixos
figure;hold on;grid on;
% plot3([0 1],[0 0],[0 0],'black','linewidth',2); hold on; grid on;
% plot3([0 0],[0 1],[0 0],'black','linewidth',2);
% plot3([0 0],[0 0],[0 1],'black','linewidth',2);
xlabel('i');
ylabel('j');
zlabel('k');
%plota 'vetor' orignal
plot3([0 vetor(1)],[0 vetor(2)],[0 vetor(3)],'color',[0 0 0]); 
view(103,23)
% plot eixo de giro
plot3([0 eixo(1)],[0 eixo(2)],[0 eixo(3)],'red'); 



% %plota 'vetor' girado de 'angulo' em torno de 'q' (myquat lib)
% plot3([0 rotate(1)],[0 rotate(2)],[0 rotate(3)],'green'); 
% plot3([0 n(1)],[0 n(2)],[0 n(3)],'blue'); 
% legend('Vetor Original','Eixo de Giro','Vetor Rotacionado mylib','Vetor Rotacionado matlab')
% return
for angulo = 0.1:0.1:2*pi
q = [cos(angulo/2) sin(angulo/2)*eixo]';
eixo_norm = eixo / sqrt((eixo*eixo')); 
q2 = [cos(angulo/2) sin(angulo/2)*eixo_norm];
n = quatrotate(q',vetor);
n2 = myquatrotate(q,[0 vetor]'); %qpq^-1
 
plot3([0 n(1)],[0 n(2)],[0 n(3)],'blue'); 
plot3([0 n2(1)],[0 n2(2)],[0 n2(3)],'green'); 
drawnow
pause(0.5)
end


function m = myquatconj(q)
if(size(q,1) ~= 4)
    error('Requer entrada de tamanho (4x1)')
end 
m = [q(1) -q(2) -q(3) -q(4)]';
end

function m = myquatmult(q,v)
if(size(q,1) ~= 4 || size(v,1) ~= 4)
    error('Requer entradas de tamanho (4x1)')
end
M = [q(1) -q(2) -q(3) -q(4);
    q(2) q(1) -q(4) q(3);
    q(3) q(4) q(1) -q(2);
    q(4) -q(3) q(2) q(1)];
m = M*v;
end

function m = myquatnormalize(q)
if(size(q,1) ~= 4)
    error('Requer entrada de tamanho (4x1)')
end 
norm = myquatnorm(q);
m = q / norm;
end


function m = myquatinv(q)
if(size(q,1) ~= 4)
    error('Requer entrada de tamanho (4x1)')
end 
norm_sq = myquatnorm(q)^2; 
m = [q(1) -q(2) -q(3) -q(4)]' / norm_sq;
end

function m = myquatnorm(q)
if(size(q,1) ~= 4)
    error('Requer entrada de tamanho (4x1)')
end 
m = sqrt(q'*q);
end


%gira 'v' em torno de 'q', resultando em m => m = qvq^-1 . REGRA DA MAO DIREITA!!!!
function m = myquatrotate(q,v)
if(size(q,1) ~= 4 || size(v,1) ~= 4)
    error('Requer entradas de tamanho (4x1)')
end
% q_inv = myquatinv(q);
% p_ = myquatmult(v,q_inv);
% p = myquatmult(q,p_);
p_ = myquatmult(q,v);
q_ = myquatinv(q);
p = myquatmult(p_,q_);
m = p(2:end); %retorna 3D

end

%descreve 'v' após giro 'q' do quadro inicial. %Giro contrário à função
%anterior
function m = myquatrotateframe(q,v)
if(size(q,1) ~= 4 || size(v,1) ~= 4)
    error('Requer entradas de tamanho (4x1)')
end
% q_conj= myquatconj(q);
% 
% q_inv = myquatinv(q_conj);
% p_ = myquatmult(q_conj,v);
% p = myquatmult(p_,q_inv);

q_inv = myquatinv(q);
p_ = myquatmult(q_inv,v);
p = myquatmult(p_,q);
m = p(2:end); %retorna 3D
end