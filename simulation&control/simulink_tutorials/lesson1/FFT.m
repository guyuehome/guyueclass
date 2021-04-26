function [Fre,Amp,Ph] = FFT(data,Fs,ampDB,isDetrend)
    % 快速傅里叶变换
    % data:波形数据
    % Fs:采样率
    % ampDB:逻辑值，是否进行对数变换，默认为false
    % isDetrend:逻辑值，是否进行去均值处理，默认为true
    % 返回[Fre:频率,Amp:幅值,Ph:相位（弧度）]
    if nargin<3
        ampDB=false;
        isDetrend=true;
    elseif nargin<4
        isDetrend=true;
    end
    n=length(data);
    if mod(n,2)==1
        n=n-1;
        data=data(1:n);
    end
    if isDetrend
        data=detrend(data);
    end
    Y = fft(data);
    %频率
    Fre=(0:n-1)*Fs/n;
    Fre=Fre(1:n/2);
    %幅值
    Amp=abs(Y(1:n/2));
    Amp([1,n/2])=Amp([1,n/2])/n;
    Amp(2:n/2-1)=Amp(2:n/2-1)/(n/2);
    if ampDB
        Amp=20*log(Amp);
        Amp(Amp<0)=0;
    end
    %相位
    Ph=angle(Y(1:n/2));
end