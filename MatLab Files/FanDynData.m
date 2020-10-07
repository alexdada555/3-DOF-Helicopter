clc;

FanData = load('FanDynData.txt');

TimeArray = FanData(1,:);
Excitation = FanData(2,:);
Response = FanData(3,:);

%Design For A Low Pass Biquad Filter

Fs = 2000;           % Sampling Frequency
Fpass = 1;           % Passband Frequency
Fstop = 45;          % Stopband Frequency
Apass = 1;           % Passband Ripple (dB)
Astop = 1.2;          % Stopband Attenuation (dB)
match = 'stopband';  % Band to match exactly

% Construct an FDESIGN object and call its BUTTER method.
h  = fdesign.lowpass(Fpass, Fstop, Apass, Astop, Fs);
Hd = design(h, 'butter', 'MatchExactly', match);
 
filtRes = filter(Hd,Response); % low pass filter

figure(1);
plot(TimeArray,Response);
hold on;
plot(TimeArray,filtRes);
hold on;
step(FanModel)
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
legend('Force (g)');

[p] = pzmap(FanModel);

tau = 1/abs(p);