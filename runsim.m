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
%% Pre-allocate buffers for saving data
% Truth, navigation, and error state buffers
x_buff          = zeros(simpar.states.nx,nstep);
xhat_buff       = zeros(simpar.states.nxf,nstep);
delx_buff       = zeros(simpar.states.nxfe,simpar.states.nxfe,nstep);
% Navigation covariance buffer
P_buff       = zeros(simpar.states.nxfe,simpar.states.nxfe,nstep);
% Continuous measurement buffer
ytilde_buff     = zeros(simpar.general.n_inertialMeas,nstep);
% Residual buffers (star tracker is included as an example)
% res_example          = zeros(3,nstep_aid);
% resCov_example       = zeros(3,3,nstep_aid);
%K_example_buff       = zeros(simpar.states.nxfe,3,nstep_aid);
loss_resCov = zeros(2,2,nstep_aid);
K_buff = zeros(simpar.states.nxfe,2,nstep_aid);
loss_res = zeros(2, nstep_aid);
%% Initialize the navigation covariance matrix
P_buff(:,:,1)   = initialize_covariance(simpar);
%% Initialize the truth state vector
x_buff(:,1) = initialize_truth_state(simpar);
%% Initialize the navigation state vector
xhat_buff(:,1) = initialize_nav_state(x_buff(:,1),simpar);
%% Miscellaneous calcs
%  xhat_buff(:,1)
%Guidance law
%[a] = guidance(r, v, af, vf, rf, tgo, g, varargin);
%r_i_ref = [0, 0, simpar.general.R_M]';
a_grav = [0;0;1.6242];
%simpar.general.MU/norm(r_i_ref)^3*r_i_ref;

af = [simpar.general.astar_x_tf; simpar.general.astar_y_tf; simpar.general.astar_z_tf];
vf = [simpar.general.vstar_x_tf; simpar.general.vstar_y_tf; simpar.general.vstar_z_tf];
rf = [simpar.general.rstar_x_tf; simpar.general.rstar_y_tf; simpar.general.rstar_z_tf];

a_thr(:,1) = guidance(x_buff(simpar.states.ix.pos,1),...
                      x_buff(simpar.states.ix.vel,1),...
                      af, vf, rf,...
                      simpar.general.tsim,...
                      a_grav, 'apollo');
% Synthesize continuous sensor data at t_n-1
wa_0 = sqrt(simpar.truth.params.Q_nongrav/simpar.general.dt)*randn(3,1);
%ytilde_buff(:,1) = contMeas(x_buff(:,1), a_thr(:,1), wa_0, simpar);
%Initialize the measurement counter
k = 1;
%Check that the error injection, calculation, and removal are all
%consistent if the simpar.general.checkErrDefConstEnable is enabled.
if simpar.general.checkErrDefConstEnable
    checkErrorDefConsistency(xhat_buff(:,1), x_buff(:,1), simpar);
end
%Inject errors if the simpar.general.errorPropTestEnable flag is enabled
if simpar.general.errorPropTestEnable
    fnames = fieldnames(simpar.errorInjection);
    for i=1:length(fnames)
        delx_buff(i,1) = simpar.errorInjection.(fnames{i});
    end
    xhat_buff(:,1) = injectErrors(truth2nav(x_buff(:,1)), delx_buff(:,1), simpar);
end
%% Compute the Q matrices (constant process noise PSDs)
Q_rbias = 2*simpar.truth.params.sig_rbias_ss^2/simpar.general.tau_r;
Q_abias = 2*simpar.truth.params.sig_abias_ss^2/simpar.general.tau_a;
%% Loop over each time step in the simulation
for i=2:nstep
    % Propagate truth states to t_n
    %   Realize a sample of process noise (don't forget to scale Q by 1/dt!)
    %   Define any inputs to the truth state DE
    %   Perform one step of RK4 integration
    
    %We use the truth state for the position and velocity, and with v_perp
    r = x_buff(simpar.states.ix.pos, i-1);
    v = x_buff(simpar.states.ix.vel, i-1);
    omega = simpar.general.omega_moon;
    omega_mi_m = [0;0;omega];
    v_perp = v-cross(omega_mi_m,r)-dot(v, r/norm(r))*r/norm(r);
    
    %Acceleration due to thrust - we use the thrust from the previous time
    %step
    input_truth.a_thrust = a_thr(:,i-1);
    input_truth.v_perp = v_perp;
    
    %Calculate the noise values
    %Noise in 
    Q_g = 2*norm(v_perp)*simpar.truth.params.sig_grav_ss^2/simpar.general.d_g;
    w_g = sqrt(Q_g/simpar.general.dt).*randn(3,1);
    
    %Noise in 
    w_a = sqrt(simpar.truth.params.Q_nongrav/simpar.general.dt)*randn(3,1);
    w_r = sqrt(Q_rbias/simpar.general.dt)*randn;
    
    %Noise in 
    Q_h = 2*norm(v_perp)*simpar.truth.params.sig_h_ss^2/simpar.general.d_h;
    w_h = sqrt(Q_h/simpar.general.dt)*randn;
    w_accl = sqrt(Q_abias/simpar.general.dt).*randn(3,1);
    
    %Noise values - accelerometer, range, something, height, acceleration
    input_truth.w_a = w_a;
    input_truth.w_r = w_r;
    input_truth.w_g = w_g;
    input_truth.w_h = w_h;
    input_truth.w_accl = w_accl;
    input_truth.simpar = simpar;
    
    %Perform one step of RK4 integration
    x_buff(:,i) = rk4('truthState_de', x_buff(:,i-1), input_truth,...
        simpar.general.dt);
%     xhat_buff(:,1)
    % Synthesize continuous sensor data at t_n
    T_i2b = calc_attitude(x_buff(:,i-1), simpar);
    ytilde_buff(:,i) = contMeas(x_buff(:,i), a_thr(:,i-1), w_a, simpar,T_i2b);
    % Propagate navigation states to t_n using sensor data from t_n-1
    %   Assign inputs to the navigation state DE
    %   Perform one step of RK4 integration
    input_nav.Ti2b = T_i2b;
    input_nav.ytilde = ytilde_buff(:,i);
    input_nav.simpar = simpar;
    input_nav.v_perp = v_perp;
    xhat_buff(:,i) = rk4('navState_de', xhat_buff(:,i-1), input_nav, ...
        simpar.general.dt);
%      xhat_buff(:,i)
    % Propagate the covariance to t_n
    input_cov.simpar = simpar;
    input_cov.ytilde = ytilde_buff(:,i);
    input_cov.Ti2b = T_i2b;
    input_cov.xhat = xhat_buff(:,i-1);
    input_cov.v_perp = v_perp;
    P_buff(:,:,i) = rk4('navCov_de', P_buff(:,:,i-1), input_cov, ...
        simpar.general.dt);
    % Propagate the error state from tn-1 to tn if errorPropTestEnable == 1
%      P_buff(:,:,i)
    format long
    if simpar.general.errorPropTestEnable
        input_delx.xhat = xhat_buff(:,i-1);
        input_delx.Ti2b = T_i2b;
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
        
        %Validate Linearization
        if simpar.general.measLinerizationCheckEnable
            loss.validate_linearization(x_buff(:,i), simpar, T_i2b);
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
        %       Correct and save the navigationclose states
        %Synthesize the measurement
        [loss_ztilde, r_f_i] = loss.synth_measurement(x_buff(:,i),simpar, T_i2b);
        loss_res(:,k) = loss.compute_residual(xhat_buff(:,i), loss_ztilde, r_f_i, simpar, T_i2b);
        
        H = loss.compute_H(x_buff(:,i), xhat_buff(:,i), simpar, T_i2b);
        R = loss.compute_R(x_buff(:,i), H, simpar);
        loss_resCov(:,:,k) = loss.compute_covariance_residual(H,P_buff(:,:,i),R);
        K_buff(:,:,k) = compute_Kalman_gain(P_buff(:,:,i), H, loss_resCov(:,:,k), simpar.general.processLOSEnable);
        K_buff(simpar.states.ixfe.b_accl,:,k) = K_buff(simpar.states.ixfe.b_accl,:,k);
        delx_buff(:,i) = K_buff(:,:,k)*loss_res(:,k);
        P_buff(:,:,i) = update_covariance(K_buff(:,:,k), H, P_buff(:,:,i), R, simpar);
        xhat_buff(:,i) = correctErrors(xhat_buff(:,i),delx_buff(:,i),simpar);
         xhat_buff(:,i)
         disp(i)
%         save('EmilyWorkspace.mat')
    end
    
    %Recalculate a_thr by calling the guidance law again
    a_thr(:,i) = guidance(x_buff(simpar.states.ix.pos,i),...
                          x_buff(simpar.states.ix.vel,i),...
                          af, vf, rf,...
                          simpar.general.tsim-t(i),...
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
navRes.loss = loss_res;
navResCov.loss = loss_resCov;
kalmanGains.loss = K_buff;
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
    'a_thrust',a_thr,...
    'simpar',simpar);
end