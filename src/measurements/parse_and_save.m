bag_name = 'simd_v1_v2_v3_00.bag';
save_name = 'd_patherror_d00.mat';

descr = struct;

descr.h = 15;
descr.dt = 0.1;
descr.delay = 0.0;
descr.z = 0.75;
descr.Qv = 1;
descr.Qs = 1;
descr.R = 1;
descr.vmin = 0;
descr.vmax = 2;
descr.amin = -1.5;
descr.amax = 1.5;
descr.truckl = 0.3;
descr.safe_d = 0.2;
descr.timegap = 1;
descr.xr = 1.6;
descr.yr = 1.2;
descr.xc = 0.2;
descr.yc = 0;
descr.kp = 0.5;
descr.ki = -0.02;
descr.kd = 3;
descr.vpmin = 0.8;
descr.vpmax = 1.2;
descr.vpperiod = 16;
descr.variance = 0.;


cells = parse_recording_bag(bag_name);


%%

save(save_name, 'cells', 'descr');