function [ traj ] = runsim( simpar, verbose, seed)
rng(seed);
%RUNSIM Runs a single trajectory given the parameters in simparams
tic;
%% Prelims
%Derive the number of steps in the simulation and the time
nstep = ceil(simpar.general.tsim/simpar.general.dt + 1);
nstep_aid = ceil(simpar.general.tsim/simpar.general.dt_kalmanUpdate);
t = (0:nstep-1)'*simpar.general.dt;
t_kalman = (0:nstep_aid)'.*simpar.general.dt_kalmanUpdate;
nstep_aid = length(t_kalman);
%If you are computing the nominal star tracker or other sensor orientations
%below is an example of one way to do this
qz = rotv2quat(simpar.general.thz_c,[0,0,1]');
qy = rotv2quat(simpar.general.thy_c,[0,1,0]');
qx = rotv2quat(simpar.general.thx_c,[1,0,0]');
simpar.general.q_b2c_nominal = qmult(qx,qmult(qy,qz));
%Acceleration due to moon gravity
r_i_reference = [0, 0, simpar.general.R_M]';
a_grav = -simpar.general.MU/norm(r_i_reference)^3*r_i_reference;
%% Pre-allocate buffers for saving data
% Truth, navigation, and error state buffers
x_buff          = zeros(simpar.states.nx,nstep);
xhat_buff       = zeros(simpar.states.nxf,nstep);
delx_buff       = zeros(simpar.states.nxfe,nstep);
% Navigation covariance buffer
P_buff       = zeros(simpar.states.nxfe,simpar.states.nxfe,nstep);
% Continuous measurement buffer
ytilde_buff     = zeros(simpar.general.n_inertialMeas,nstep);
% Residual buffers (star tracker is included as an example)
res_example          = zeros(3,nstep_aid);
resCov_example       = zeros(3,3,nstep_aid);
K_example_buff       = zeros(simpar.states.nxfe,3,nstep_aid);
loss_res = zeros(2, nstep_aid);
% accel_buff           = zeros(3,nstep_aid);
% cam_buff             = zeros(2,nstep_aid);
%% Initialize the navigation covariance matrix
% P_buff(:,:,1) = initialize_covariance();
%% Initialize the truth state vector
%Does need simpar
x_buff(:,1) = initialize_truth_state(simpar);
% disp('x_buff')
% disp(x_buff(:,1))
%% Initialize the navigation state vector
%Should be (x_buff,simpar)
xhat_buff(:,1) = initialize_nav_state(x_buff(:,1),simpar);
disp('xhat_buff')
disp(xhat_buff(:,1))
% %% Synthasize terrain and gravity error
% [h_terrain, r_terrain] = gen_terrain(simpar, simpar.general.seed_h);
% %% Synthesize landmarks
% [ lm ] = gen_landmarks(simpar.general.num_landmarks, h_terrain,...
%     r_terrain, simpar);
%% Miscellaneous calcs
%Guidance law
%[a] = guidance(r, v, af, vf, rf, tgo, g, varargin);
a_thr(:,1) = guidance(x_buff(simpar.states.ix.pos,1),...
    x_buff(simpar.states.ix.vel,1),...
    [simpar.general.astar_x_tf, simpar.general.astar_y_tf, simpar.general.astar_x_tf]',...
    [simpar.general.vstar_x_tf, simpar.general.vstar_y_tf, simpar.general.vstar_z_tf]',...
    [simpar.general.rstar_x_tf, simpar.general.rstar_y_tf, simpar.general.rstar_z_tf]',...
    simpar.general.tsim,...
    a_grav, 'apollo');
% Synthesize continuous sensor data at t_n-1
% ytilde_buff(:,1) = contMeas();
%Initialize the measurement counter
k = 1;
%Check that the error injection, calculation, and removal are all
%consistent if the simpar.general.checkErrDefConstEnable is enabled.
if simpar.general.checkErrDefConstEnable
    checkErrorDefConsistency(xhat_buff(:,1), x_buff(:,1), simpar)
end
%Inject errors if the simpar.general.errorPropTestEnable flag is enabled
if simpar.general.errorPropTestEnable
    fnames = fieldnames(simpar.errorInjection);
    for i=1:length(fnames)
        delx_buff(i,1) = simpar.errorInjection.(fnames{i});
    end
    xhat_buff(:,1) = injectErrors(truth2nav(x_buff(:,1)), delx_buff(:,1), simpar);
end
%% Loop over each time step in the simulation
for i=2:nstep
    % Propagate truth states to t_n
    %   Realize a sample of process noise (don't forget to scale Q by 1/dt!)
    %   Define any inputs to the truth state DE
    %   Perform one step of RK4 integration
%     vperp = abs(x_buff(simpar.states.ix.vel(1),i-1));
%     Q_g = 2*vperp*simpar.truth.params.sig_grav_ss^2/simpar.general.d_g;
%     w_g = sqrt(Q_g/simpar.general.dt)*randn(2,1);
%     w_a = sqrt(simpar.truth.params.Q_nongra/simpar.general.dt)*randn(2,1);
%     w_p = sqrt(Q_p/simpar.general.dt).*randn(length(simpar.states.ix.param),1);
%     input_truth.w_g = w_g;
    input_truth.w_a = zeros(3,1);
    input_truth.w_r = zeros(3,1);
    input_truth.w_d = 0;
    input_truth.w_h = 0;
    input_truth.w_accl = 0;
    input_truth.a_grav = a_grav;
    input_truth.a_thrust = a_thr(:,i-1);
    input_truth.simpar = simpar;
    %Perform one step of RK4 integration
    x_buff(:,i) = rk4('truthState_de', x_buff(:,i-1), input_truth,...
        simpar.general.dt);
    % Synthesize continuous sensor data at t_n
    ytilde_buff(:,i) = contMeas(x_buff(:,i), a_thr(:,i-1), input_truth);
    % Propagate navigation states to t_n using sensor data from t_n-1
    %   Assign inputs to the navigation state DE
    %   Perform one step of RK4 integration
    input_nav.ytilde = ytilde_buff(:,i);
    input_nav.v_perp = x_buff(simpar.states.ix.vel(2),i-1);
    input_nav.simpar = simpar;
%     input_nav.x = x_buff(:,i-1);
%     input_nav.a_grav = a_grav;
    xhat_buff(:,i) = rk4('navState_de', xhat_buff(:,i-1), input_nav, ...
        simpar.general.dt);
    % Propagate the covariance to t_n
    input_cov.simpar = simpar;
    input_cov.ytilde = [];
%     input_cov.x = x_buff(:,i-1);
%     input_cov.xhat = xhat_buff(:,i-1);
%     P_buff(:,:,i) = rk4('navCov_de', P_buff(:,:,i-1), input_cov, ...
%         simpar.general.dt);
    % Propagate the error state from tn-1 to tn if errorPropTestEnable == 1
    if simpar.general.errorPropTestEnable
        input_delx.xhat = xhat_buff(:,i-1);
        input_delx.ytilde = [];
        input_delx.simpar = simpar;
        delx_buff(:,i) = rk4('errorState_de', delx_buff(:,i-1), ...
            input_delx, simpar.general.dt);
    end
    
    % If discrete measurements are available, perform a Kalman update
    if abs(t(i)-t_kalman(k+1)) < simpar.general.dt*0.01
        %   Check error state propagation if simpar.general.errorPropTestEnable = true
        if simpar.general.errorPropTestEnable
            checkErrorPropagation(x_buff(:,i), xhat_buff(:,i),...
                delx_buff(:,i), simpar);
        end
        %Adjust the Kalman update index
        k = k + 1;
        %   For each available measurement
        %       Synthesize the noisy measurement, ztilde
        %       Predict the measurement, ztildehat
        %       Compute the measurement sensitivity matrix, H
        %       If simpar.general.measLinerizationCheckEnable == true
        %           Check measurement linearization
        %       Compute and save the residual
        %       Compute and save the residual covariance
        %       Compute and save the Kalman gain, K
        %       Estimate the error state vector
        %       Update and save the covariance matrix
        %       Correct and save the navigation states
% % %         ztilde_example = example.synthesize_measurement();
% % %         ztildehat_example = example.predict_measurement();
% % %         H_example = example.compute_H();
% % %         example.validate_linearization();
% % %         res_example(:,k) = example.compute_residual();
% % %         resCov_example(:,k) = compute_residual_cov();
% % %         K_example_buff(:,:,k) = compute_Kalman_gain();
% % %         del_x = estimate_error_state_vector();
% % %         P_buff(:,:,k) = update_covariance();
% % %         xhat_buff(:,i) = correctErrors();
        [loss_ztilde, r_fi] = loss.synth_measurement(x_buff(:,1),simpar);
        loss_ztildehat = loss.pred_measurement(xhat_buff(:,1),r_fi,simpar);
        loss_res(:,k) = loss_ztilde-loss_ztildehat;
    end
    
    %Recalculate a_thr by calling the guidance law again
a_thr(:,1) = guidance(x_buff(simpar.states.ix.pos,1),...
    x_buff(simpar.states.ix.vel,1),...
    [simpar.general.astar_x_tf, simpar.general.astar_y_tf, simpar.general.astar_x_tf]',...
    [simpar.general.vstar_x_tf, simpar.general.vstar_y_tf, simpar.general.vstar_z_tf]',...
    [simpar.general.rstar_x_tf, simpar.general.rstar_y_tf, simpar.general.rstar_z_tf]',...
    simpar.general.tsim,...
    a_grav, 'apollo');

    if verbose && mod(i,100) == 0
        fprintf('%0.1f%% complete\n',100 * i/nstep);
    end
end

if verbose
    fprintf('%0.1f%% complete\n',100 * t(i)/t(end));
end

T_execution = toc;
%Package up residuals
navRes.example = res_example;
navResCov.example = resCov_example;
kalmanGains.example = K_example_buff;
%Package up outputs
traj = struct('navState',xhat_buff,...
    'navCov',P_buff,...
    'navRes',navRes,...
    'navResCov',navResCov,...
    'truthState',x_buff,...
    'time_nav',t,...
    'time_kalman',t_kalman,...
    'executionTime',T_execution,...
    'continuous_measurements',ytilde_buff,...
    'kalmanGain',kalmanGains,...
    'simpar',simpar);
end