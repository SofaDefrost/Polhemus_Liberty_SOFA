
%% Author : Ke Wu

%%Try to build the connection between Sofa framework and Simulink

%Reset TCP/IP configuration
delete(instrfindall) %%
clear 

 sCom=serial('COM4','baudrate',115200,'parity','none','databits',8,'stopbits',1,'inputBufferSize',1024);%
 fopen(sCom);%%
 
Uoo=[1100,800,-120,-70];


pause(2);
SofaOutput=[0,0,0,0]


dt=0.02;
U=Uoo;
sendSteps(U,sCom);
% pause(2)
CPID=CPIDoo;
Error_last=[0,0,0];
Error_rate_m=[0,0,0];
Error_integral=[0,0,0];
Error_rate2_m=[0,0,0];
Error_m=[];

Error_integral_m=[0,0,0];
REF=[];
Ref=[];
Ref_rate=[];
Ref_rate2=[];
Ref_rate3=[];

Kp=[5000,0,0;0,5000,0;0,0,0];
Ki=[1500000,0,0;0,1500000,0;0,0,0]; 
Kd=[100,0,0;0,100,0;0,0,100]; 

a=4.5;
b=0.02;
w=0.02;
DeltaU=[0,0,0,0];
K=0.99;
load U_tr.mat
U_tr=U_m(:,1:4);
U_m=[];
SofaOutput=[0,0,0,0];
x=Poo(1);
y=Poo(2);
z=Poo(3);
tic
for i=1:1:600
    while ~tcpipClient.BytesAvailable
    end
    if tcpipClient.BytesAvailable>0
        SofaOutput_new = fread(tcpipClient,48);
        if tcpipClient.BytesAvailable>0
            ad = fread(tcpipClient,tcpipClient.BytesAvailable);              
        end         
    end    
    pp=DataTransfer(uint8(SofaOutput_new(17:end)))';
    number_data = tcpipClient.BytesAvailable;
    SofaOutput(i,:) = 2.54*[pp(1:3)-PosOffset';0];
    
    Ref=[x+b*i, y-a+a*sin(w*i+pi/2), z];
    
    Error_present=Ref-SofaOutput(end,1:3);   
    
    Error_rate=((Error_present-Error_last)./dt+Error_rate_m(end,:))/2;
    
    Error_rate_m=[Error_rate_m;Error_rate];
    
    Error_rate2=((Error_rate_m(end,:)-Error_rate_m(end-1,:))./dt+Error_rate2_m(end,:))/2;
    
    Error_rate2_m=[Error_rate2_m;Error_rate2];
    
    Error_m=[Error_m;Error_present];    
    
    Error_integral= (Error_present+Error_last)/2.*dt+Error_integral; 
    
    Error_last=Error_present;    

    U=CPID*(Kp*Error_present'+Ki*Error_integral'+Kd*Error_rate')+U_tr(i,:)';
    
    i
    
% 
%     if i>3
%         U=(1*U_m(end,:)'+1*U_m(end-1,:)'+1*U_m(end-2,:)'+1*U)/4;
%     end
    
    U_m=[U_m;U']; 
    
    sendSteps(U,sCom);
    REF=[REF;Ref];
    pause(dt)
end
toc
sendSteps([0,0,0,0],sCom);
pause(1);
figure(3)

hold on
plot(REF(:,1))
plot(SofaOutput(:,1))
plot(REF(:,2))
plot(SofaOutput(:,2))
load newZ.mat
plot(smooth(newZ(:),30))
plot(SofaOutput(:,3))






figure(4)
hold on
plot(SofaOutput(:,1),SofaOutput(:,2))
axis equal
plot(REF(:,1),REF(:,2))
axis equal


delete(instrfindall) %%
clear sCom
