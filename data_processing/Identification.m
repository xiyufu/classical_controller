%PRBS: U = 6000*idinput(4000,'PRBS');[mV]
%output theta normalized to [rad], t in [s]
% 
%read raw data from xml.
DOM = xmlread('PRBS10Hz.xml');
data = DOM.getElementsByTagName('data').item(0);
rows = data.getElementsByTagName('row');
n1 = rows.getLength();
n2 = length(strsplit(char(rows.item(0).getFirstChild.getData),'\t'));
datas = zeros(n1,n2);
for k = 1:n1
    row = char(rows.item(k-1).getFirstChild.getData);
    datas(k,:) = cellfun(@(x)str2double(x),strsplit(row,'\t'));
end
t_r = datas(:,1);
u_r = datas(:,2);
y_r = datas(:,10);

%choose effactive data.
t1 = 126;
t2 = 1125;
x = u_r(t1:t2);
y = y_r(t1:t2);
t = t_r(t1:t2);

%scaling, dealing with encoder overflow
t=t/1000;
t=t-t(1);
% delta = max(y)-min(y);%NO overflow
% [ymax,i]=max(y);
% y(i:end) = y(i:end)-delta-1;

factor = (6320+7889+9445-4797-3197-1635)/9;
%how many counts for one round. data comes from test.
factor = 2*pi/factor;
theta = factor*y;

% %Substract the DC signal from input and output.
% u = x - mean(x);
% k = regress(theta,t);
% y = theta - k*t;
%No non-linear assumpution here 
u = x;

%%%%
y = theta;%we add a - here, just another way to read the encoder.
%%%%

y0 = y;
u0 = u;

%use filtered data
% [B,A] = butter(6,10/50,'low');
% u = filter(B,A,u);
% y = filter(B,A,y);
% %It performs better if I don't use filter...
% 
% 
%assume 2nd order system, dicreted via zoh
%least squire method
dy = y(2:end) - y(1:end-1);

b = dy(2:end);
K = [-dy(1:end-1),  u(1:end-2)];
para = K\b;

%check the result
as = [1, para(1)-1, -para(1)];
bs = [para(2)];
y_hat = filter(bs,as,u0);
figure(1); grid on;
subplot(1,2,1); 
plot(t, y_hat, 'r', t, y, 'b','linewidth',1);title('2nd system');

%iterative estimation. 

for i = 1:1
    %choose different rounds of iteration can change the result
    %significantly. Sadly, the best number change with different signal.
    y = filter(1,as,y);
    u = filter(1,as,u);
    
    dy = y(2:end) - y(1:end-1);
    
    b = dy(2:end);
    K = [-dy(1:end-1), u(1:end-2)];
    para = K\b;
    as = [1, para(1)-1, -para(1)];
    bs = [para(2)];
end

y_hat1 = filter(bs,as,u0);
subplot(1,2,2); 
plot(t, y_hat1, 'r', t, y0, 'b','linewidth',1);title('2nd system, iteration');
figure(5)
error = y0 - y_hat1;
norm(error)
plot(t,error,'linewidth',1);title('error'); grid on;

% 
% 
% % %assume 3rd order system
% %  b = y(4:end);
% % K = [-y(3:end-1), -y(2:end-2), -y(1:end-3) u(3:end-1), u(2:end-2), u(1:end-3)];
% % para = K\b;
% % as = [1, para(1), para(2), para(3)];
% % bs = [para(4), para(5),para(6)];
% % y_hat1 = filter(bs,as,u);
% % figure(3)
% % subplot(1,2,1);
% % plot(t,y_hat1,'r',t,y,'b');title('3rd order, first estimation');
% % 
% % for i = 1:1          
% %     %choose different rounds of iteration can change the result
% %     %significantly. Sadly, the best number change with different signal.
% %     %But they are all around 2~4 rounds
% %     y = filter(1,as,y);
% %     u = filter(1,as,u);
% %     b = y(4:end);
% %     K = [-y(3:end-1), -y(2:end-2), -y(1:end-3) u(3:end-1), u(2:end-2), u(1:end-3)];
% %     para = K\b;
% %     as = [1, para(1), para(2), para(3)];
% %     bs = [para(4), para(5),para(6)];
% % end
% % 
% % y_hat2 = filter(bs,as,u0);
% % subplot(1,2,2);
% % plot(t,y_hat2,'r',t,y0,'b');title('3rd order, iteration');
% % figure(4);
% % error = y0 - y_hat2;
% % plot(t,error);title('error');
% 
% 
% % %%%%%%%%4th order is bad, pretty bad...%%%%%%%%%%%%%
% 
% % b1 = y(5:end);
% % K1 = [-y(4:end-1), -y(3:end-2), -y(2:end-3) -y(1:end-4) u(4:end-1), u(3:end-2), u(2:end-3), u(1:end-4)];
% % para1 = K1\b1;
% % as1 = [1, para1(1), para1(2), para1(3), para1(4)];
% % bs1 = [para1(5), para1(6),para1(7), para1(8)];
% % y_hat1 = filter(bs1,as1,u0);
% % figure(3)
% % plot(t,y_hat1,'r',t,y,'b');title('3rd order, first estimation');
% % 
% % for i = 1:1
% %     %choose different rounds of iteration can change the result
% %     %significantly. Sadly, the best number change with different signal.
% %     %But they are all around 2~4 rounds
% %     y = filter(1,as1,y);
% %     u = filter(1,as1,u);
% %     b1 = y(5:end);
% %     K1 = [-y(4:end-1), -y(3:end-2), -y(2:end-3) -y(1:end-4) u(4:end-1), u(3:end-2), u(2:end-3), u(1:end-4)];
% %     para1 = K1\b1;
% %     as1 = [1, para1(1), para1(2), para1(3), para1(4)];
% %     bs1 = [para1(5), para1(6),para1(7), para1(8)];
% % end
% % 
% % y_hat2 = filter(bs1,as1,u0);
% % figure(4)
% % plot(t,y_hat2,'r',t,y0,'b');title('3rd order, iteration');
% % figure(5)
% % error = y0 - y_hat2;
% % plot(t,error);title('error');
% 
% %First assume 3rd order system, but iterate in 2nd order
% %This gives a sightly better results than pure 2nd order.
% %  b = y(4:end);
% % K = [-y(3:end-1), -y(2:end-2), -y(1:end-3) u(3:end-1), u(2:end-2), u(1:end-3)];
% % para = K\b;
% % as = [1, para(1), para(2), para(3)];
% % bs = [para(4), para(5),para(6)];
% % y_hat1 = filter(bs,as,u);
% % figure(3)
% % subplot(1,2,1);
% % plot(t,y_hat1,'r',t,y,'b');title('3rd order, first estimation');
% 
% 
% 
% b = y(5:end);
% K = [-y(4:end-1), -y(3:end-2), -y(2:end-3) -y(1:end-4) u(4:end-1), u(3:end-2), u(2:end-3), u(1:end-4)];
% para = K\b;
% as = [1, para(1), para(2), para(3), para(4)];
% bs = [para(5), para(6),para(7), para(8)];
% y_hat1 = filter(bs,as,u0);
% subplot(1,2,1);
% plot(t,y_hat1,'r',t,y,'b');title('4th order, first estimation');
% 
% for i = 1:3
%     y = filter(1,as,y);
%     u = filter(1,as,u);
%     b = y(3:end);
%     K = [-y(2:end-1), -y(1:end-2), u(2:end-1), u(1:end-2)];
%     para = K\b;
%     as = [1, para(1), para(2)];
%     bs = [para(3), para(4)];
% end
% 
% y_hat2 = filter(bs,as,u0);
% subplot(1,2,2);
% plot(t,y_hat2,'r',t,y0,'b');title('2nd order, iteration');
% 
% error = y0 - y_hat2;
% norm(error)
% figure(4)
% plot(t,error);title('error');

% %%%%%%%%%%Using known part%%%%%%%%%%
% %H(s) = K/s(as^2+bs+c)
% 
% dy = y(2:end) - y(1:end-1);
% 
% b = dy(3:end);
% K = [-dy(2:end-1), -dy(1:end-2), u(3:end-1), u(2:end-2), u(1:end-3)];
% 
% para = K\b;
% 
% bs = [para(3), para(4), para(5)];
% as = [1, para(1)-1, para(2)-para(1), -para(2)];
% 
% y_hat = filter(bs,as,u0);
% figure(1)
% subplot(1,2,1);grid on
% plot(t,y_hat,'r',t,y0,'b');
% 
% for i = 1:1
%     y = filter(1,as,y);
%     u = filter(1,as,u);
%     
%     dy = y(2:end) - y(1:end-1);
%     
%     b = dy(3:end);
%     K = [-dy(2:end-1), -dy(1:end-2), u(3:end-1), u(2:end-2), u(1:end-3)];
%     
%     para = K\b;
%     
%     bs = [para(3), para(4), para(5)];
%     as = [1, para(1)-1, para(2)-para(1), -para(2)];
% end
% 
% y_hat1 = filter(bs,as,u0);
% subplot(1,2,2);grid on;
% plot(t,y_hat1,'r',t,y0,'b');
% 
% figure(2);
% error = y0 - y_hat1;
% norm(error)
% plot(t,error);title('error');
% %The result is better, norm(error) goes to 34 which was over 90  above

sysd = tf(bs,as,0.01);
sysc = d2c(sysd,'zoh');

% figure(3);
% bode(sysc);