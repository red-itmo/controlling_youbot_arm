clear

function y = get_filtered(u)
    [y_filt_1, zf] = filter(fb_d.num, fb_d.den, u);
    y_inv = y_filt_1(length(y_filt_1):-1:1);
    [y_inv_filt, zf] = filter(fb_d.num, fb_d.den, y_inv);
    y = y_inv_filt(length(y_inv_filt):-1:1)
endfunction


//Beginning
path = "~/ident_data/";
name = "data_5";
data = read(path + name + ".txt", -1, 16);

N = max(size(data));
angle_raw = data(:,[1:5]);
vel_raw = data(:, [6:10]);
tau_raw = data(:,[11:15]);
time = data(:,16);

accel_raw(1,:) = (vel_raw(2, :) - vel_raw(1, :)) ./ (time(2) - time(1));
for i = 2:(N-1)
    accel_raw(i,:) = (vel_raw(i+1, :) - vel_raw(i-1, :)) ./ (time(i+1) - time(i-1));
end
accel_raw(N,:) = (vel_raw(N, :) - vel_raw(N-1, :)) ./ (time(N) - time(N-1));

for i = 2:(max(size(time)))
    delta_t(i-1) = time(i) - time(i-1);
end
dt = mean(delta_t)
printf("Average dt = %f\n", dt);


//Filter
cut_off = 7.4; //its cut off frequency (MANUAL SETTING)
[pols, gain] = zpbutt(10, 2*%pi*cut_off);
H=gain/real(poly(pols,'s'))
fb = syslin('c', H);
fb_d = ss2tf(cls2dls(tf2ss(fb), dt)) ;


//Filtering
elems = 94:7594; // rows which were corrected by filter (MANUAL SETTING)
mat_out = zeros(length(elems), 21)
for i = 1:5

    //Filtering angles
    angle_filt = get_filtered(angle_raw(:,i))
    subplot(5, 5, i)
    plot2d(time, angle_raw(:,i), 3);
    plot2d(time, angle_filt, 1);
    
    //Filtering velocities
    vel_filt = get_filtered(vel_raw(:,i))
    subplot(5, 5, 5+i)
    plot2d(time, vel_raw(:,i), 3);
    plot2d(time, vel_filt, 1);
    
    //Calculate accels from filtered velocities
    accel_new(1) = (vel_filt(2) - vel_filt(1)) ./ (time(2) - time(1));
    for j = 2:(N-1)
        accel_new(j) = (vel_filt(j+1) - vel_filt(j-1)) ./ (time(j+1) - time(j-1));
    end
    accel_new(N) = (vel_filt(N) - vel_filt(N-1)) ./ (time(N) - time(N-1));
    subplot(5, 5, 10+i)
    plot2d(time, accel_raw(:,i), 3);
    plot2d(time, accel_new, 1);

    //Filtering torques
    tau_filt = get_filtered(tau_raw(:,i))
    subplot(5, 5, 15+i)
    plot2d(time, tau_raw(:,i), 3);
    plot2d(time, tau_filt, 1);
    
    //Spectrums
    fff = abs(fft(tau_raw(:,i)));
    amp_raw = fff(1:floor(N/2));
    frq_raw = (0:length(amp_raw)-1) / ( time(floor(N/2)) - time(1) );
    max_frq = (length(amp_raw)-1) / (time(floor(N/2)) - time(1));
    [filts_frq, filts_amp] = repfreq(fb, 0,  max_frq)
    filts_amp = abs(filts_amp)*max(amp_raw);
    subplot(5, 5, 20+i);
    plot2d3(frq_raw, amp_raw, 2);
    plot2d(filts_frq, filts_amp, 5);
    
    //Collecting useful data
    mat_out(:,i) = angle_filt(elems);
    mat_out(:,5+i) = vel_filt(elems);
    mat_out(:,10+i) = accel_new(elems);
    mat_out(:,15+i) = tau_filt(elems);
end

//Writing to file
mat_out(:,21) = time(elems);
fprintfMat(path + name + "_filt.txt", mat_out);
