clear
clc

input=randn(2000,2);
output=sum(input.*input,2);

%从1到2000间随机排序
k=rand(1,2000);
[m,n]=sort(k);

%随机提取1900个样本为训练样本，100个样本为预测样本
input_train=input(n(1:1900),:)';
output_train=output(n(1:1900),:)';
input_test=input(n(1901:2000),:)';
output_test=output(n(1901:2000),:)';
 
%输入数据归一化
[inputn,inputps]=mapminmax(input_train);

%% 网络结构初始化
innum=2;
midnum=10;
outnum=1;
  
%权值初始化
w1=rands(midnum,innum);
b1=rands(midnum,1);
w2=rands(midnum,outnum);
b2=rands(outnum,1);
  
%学习率
xite=0.1; %学习速率
loopNumber=100; %循环次数
I=zeros(1,midnum);
Iout=zeros(1,midnum);
FI=zeros(1,midnum);
dw1=zeros(innum,midnum);
db1=zeros(1,midnum);

%% 网络训练
E=zeros(1,loopNumber);
for ii=1:loopNumber
    disp(ii);
    for i=1:1:1900
       %% 网络预测输出
        x=inputn(:,i);
        % 隐含层输出
        for j=1:1:midnum
            I(j)=inputn(:,i)'*w1(j,:)'+b1(j);   %矩阵乘法，inputn是n×1500，w1是l×n
            %inputn(:,i)'是1×n，w1(j,:)'是n×1
            Iout(j)=1/(1+exp(-I(j)));   %Iout是1×l
        end
        % 输出层输出
        yn=w2'*Iout'+b2;   %矩阵乘法，w2是l×m，Iout是1×l
        %w2'是m×l，Iout'是l×1，yn是m×1
         
       %% 权值阀值修正
        %计算误差
        e=output_train(:,i)-yn;    
        E(ii)=E(ii)+sum(abs(e));
         
        %计算权值变化率
        dw2=e*Iout; %m×l
        db2=e';
         
        for j=1:1:midnum
            S=1/(1+exp(-I(j)));
            FI(j)=S*(1-S);
        end     
        for k=1:1:innum
            for j=1:1:midnum
                dw1(k,j)=FI(j)*x(k)*sum(e'*w2(j,:)');
                db1(j)=FI(j)*sum(e'*w2(j,:)');
            end
        end
            
        w1=w1+xite*dw1';
        b1=b1+xite*db1';
        w2=w2+xite*dw2';
        b2=b2+xite*db2';
    end
end

%% 网络预测
inputn_test=mapminmax('apply',input_test,inputps);
fore=zeros(1,100);
for i=1:100
    %隐含层输出
    for j=1:1:midnum
        I(j)=inputn_test(:,i)'*w1(j,:)'+b1(j);
        Iout(j)=1/(1+exp(-I(j)));
    end
    fore(:,i)=w2'*Iout'+b2;
end

%% 结果分析
figure(1)
plot(fore,':og')
hold on
plot(output_test,'-*');
legend('预测输出','期望输出')
title('BP网络预测输出','fontsize',12)
ylabel('函数输出','fontsize',12)
xlabel('样本','fontsize',12)

%预测误差
error=fore-output_test;
 
figure(2)
plot(error,'-*')
title('BP网络预测误差','fontsize',12)
ylabel('误差','fontsize',12)
xlabel('样本','fontsize',12)

figure(3)
plot((fore-output_test)./output_test,'-*');
title('神经网络预测误差百分比')