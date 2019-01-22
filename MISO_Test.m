%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wl_example_mimo_ofdm_txrx.m
% 2x2 MIMO OFDM Example
% A detailed write-up of this example is available on the wiki:
% http://warpproject.org/trac/wiki/WARPLab/Examples/MIMO_OFDM
%
% Copyright (c) 2015 Mango Communications - All Rights Reserved
% Distributed under the WARP License (http://warpproject.org/license)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear

% Params:
USE_WARPLAB_TXRX        = 1;           % Enable WARPLab-in-the-loop (otherwise sim-only)
WRITE_PNG_FILES         = 0;           % Enable writing plots to PNG
CHANNEL                 = 11;          % Channel to tune Tx and Rx radios

% Waveform params
N_OFDM_SYMS             = 1000;        % Number of OFDM symbols (must be even valued)
MOD_ORDER               = 4;          % Modulation order (2/4/16 = BSPK/QPSK/16-QAM)
TX_SCALE                = 1.0;         % Scale for Tx waveform ([0:1])
INTERP_RATE             = 2;           % Interpolation rate (must be 2)
TX_SPATIAL_STREAM_SHIFT = 0;           % Number of samples to shift the transmission from RFB

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];                           % Pilot subcarrier indices
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
N_DATA_SYMS             = N_OFDM_SYMS * length(SC_IND_DATA);      % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)

% Rx processing params
FFT_OFFSET                    = 4;           % Number of CP samples to use in FFT (on average)
LTS_CORR_THRESH               = 0.7;         % Normalized threshold for LTS correlation
DO_APPLY_CFO_CORRECTION       = 1;           % Enable CFO estimation/correction
DO_APPLY_PHASE_ERR_CORRECTION = 1;           % Enable Residual CFO estimation/correction
DO_APPLY_SFO_CORRECTION       = 1;           % Enable SFO estimation/correction
DECIMATE_RATE                 = INTERP_RATE;

% WARPLab experiment params
USE_AGC                 = false;        % Use the AGC if running on WARP hardware
MAX_TX_LEN              = 2^19;        % Maximum number of samples to use for this experiment
SAMP_PADDING            = 200;         % Extra samples to receive to ensure both start and end of waveform visible


if(USE_WARPLAB_TXRX)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Set up the WARPLab experiment
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    NUMNODES = 3;

    % Create a vector of node objects
    nodes   = wl_initNodes(NUMNODES);
    node_tx = nodes(2);
    node_rx = nodes(1);

    % Create a UDP broadcast trigger and tell each node to be ready for it
    eth_trig = wl_trigger_eth_udp_broadcast;
    wl_triggerManagerCmd(nodes, 'add_ethernet_trigger', [eth_trig]);

    % Read Trigger IDs into workspace
    trig_in_ids  = wl_getTriggerInputIDs(nodes(1));
    trig_out_ids = wl_getTriggerOutputIDs(nodes(1));

    % For both nodes, we will allow Ethernet to trigger the buffer baseband and the AGC
    wl_triggerManagerCmd(nodes, 'output_config_input_selection', [trig_out_ids.BASEBAND, trig_out_ids.AGC], [trig_in_ids.ETH_A]);
    % Set the trigger output delays.
    wl_triggerManagerCmd(node_tx,'output_config_delay', [trig_out_ids.BASEBAND,trig_out_ids.AGC], 31.25);
    wl_triggerManagerCmd(node_rx,'output_config_delay', [trig_out_ids.AGC], 3000);     %3000 ns delay before starting the AGC

    % Get IDs for the interfaces on the boards. 
    ifc_ids_TX = wl_getInterfaceIDs(node_tx);
    ifc_ids_RX = wl_getInterfaceIDs(node_rx);

    % Set up the TX / RX nodes and RF interfaces
    TX_RF     = ifc_ids_TX.RF_ON_BOARD;
    TX_RF_VEC = ifc_ids_TX.RF_ON_BOARD_VEC;
    TX_RF_ALL = ifc_ids_TX.RF_ALL;
    
    RX_RF     = ifc_ids_RX.RF_A;
    RX_RF_VEC = ifc_ids_RX.RF_A;
    RX_RF_ALL = ifc_ids_RX.RF_ALL;

    % Set up the interface for the experiment
    wl_interfaceCmd(node_tx, TX_RF_ALL, 'channel',2.4, CHANNEL);
    %wl_interfaceCmd(node_tx1, TX_RF_ALL, 'channel',5, CHANNEL);
    wl_interfaceCmd(node_rx, RX_RF_ALL, 'channel',2.4, CHANNEL);

    wl_interfaceCmd(node_tx, TX_RF_ALL, 'tx_gains', 2, 13);
    
    if(USE_AGC)
        wl_interfaceCmd(node_rx, RX_RF_ALL, 'rx_gain_mode', 'automatic');
        wl_basebandCmd(node_rx, 'agc_target', -13); %-13
    else
        wl_interfaceCmd(node_rx, RX_RF_ALL, 'rx_gain_mode', 'manual');
        RxGainRF = 2;                  % Rx RF Gain in [1:3]
        RxGainBB = 10;                 % Rx Baseband Gain in [0:31]
        wl_interfaceCmd(node_rx, RX_RF_ALL, 'rx_gains', RxGainRF, RxGainBB);
    end

    % Get parameters from the node
    SAMP_FREQ    = wl_basebandCmd(nodes(1), 'tx_buff_clk_freq');
    Ts           = 1/SAMP_FREQ;

    % We will read the transmitter's maximum I/Q buffer length
    % and assign that value to a temporary variable.
    %
    % NOTE:  We assume that the buffers sizes are the same for all interfaces

    maximum_buffer_len = min(MAX_TX_LEN, wl_basebandCmd(node_tx, TX_RF_VEC, 'tx_buff_max_num_samples'));
    
    example_mode_string = 'hw';
else
    % Use sane defaults for hardware-dependent params in sim-only version
    maximum_buffer_len  = min(MAX_TX_LEN, 2^20);
    SAMP_FREQ           = 40e6;
    example_mode_string = 'sim';
end

%% Define a half-band 2x interpolation filter response
interp_filt2 = zeros(1,43);
interp_filt2([1 3 5 7 9 11 13 15 17 19 21]) = [12 -32 72 -140 252 -422 682 -1086 1778 -3284 10364];
interp_filt2([23 25 27 29 31 33 35 37 39 41 43]) = interp_filt2(fliplr([1 3 5 7 9 11 13 15 17 19 21]));
interp_filt2(22) = 16384;
interp_filt2 = interp_filt2./max(abs(interp_filt2));

% Define the preamble
% Note: The STS symbols in the preamble meet the requirements needed by the
% AGC core at the receiver. Details on the operation of the AGC are
% available on the wiki: http://warpproject.org/trac/wiki/WARPLab/AGC
sts_f = zeros(1,64);
sts_f(1:27) = [0 0 0 0 -1-1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0 1+1i 0 0 0 1+1i 0 0 0 1+1i 0 0];
sts_f(39:64) = [0 0 1+1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0 -1-1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0];
sts_t = ifft(sqrt(13/6).*sts_f, 64);
sts_t = sts_t(1:16);

% LTS for CFO and channel estimation
lts_f = [0 1 1 -1 -1 1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 -1 1 -1 1 -1 1 1 1 1];
lts_t = ifft(lts_f, 64);

% We break the construction of our preamble into two pieces. First, the
% legacy portion, is used for CFO recovery and timing synchronization at
% the receiver. The processing of this portion of the preamble is SISO.
% Second, we include explicit MIMO channel training symbols.

% Legacy Preamble

% Use 30 copies of the 16-sample STS for extra AGC settling margin
% To avoid accidentally beamforming the preamble transmissions, we will
% let RFA be dominant and handle the STS and first set of LTS. We will
% append an extra LTS sequence from RFB so that we can build out the
% channel matrix at the receiver
lts_f_B = [0 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 0 0 0 0 0 0 0 0 0 0 0 -1 -1 1 1 -1 -1 1 -1 1 -1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 -1 -1 -1];

lts_f_part1 = [0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0 0 0 0 0 0 0 0 0 0 0 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3];
lts_f_part2 = [0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0 0 0 0 0 0 0 0 0 0 0 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3];

lts_t_B = ifft(lts_f_B, 64);

lts_t_part1 = ifft(lts_f_part1,64);
lts_t_part2 = ifft(lts_f_part2,64);

lts_t_sum = lts_t_part1 + lts_t_part2;

lts_f_sum = lts_f_part1 + lts_f_part2;

sts_t_rep = repmat(sts_t, 1, 30);
sts_t_rep_B = repmat(sts_t, 1, 2);

preamble_legacy_B = [sts_t_rep,sts_t_rep_B, lts_t(33:64), lts_t, lts_t];%,lts_f_sum];%Remove sum
preamble_legacy_A = [zeros(1,32),sts_t_rep, lts_t(33:64), lts_t, lts_t];%,lts_f_sum];%

% MIMO Preamble

% There are many strategies for training MIMO channels. Here, we will use
% the LTS sequence defined before and orthogonalize over time. First we
% will send the sequence on stream A and then we will send it on stream B

preamble_mimo_A = [lts_t,lts_t, lts_t_sum]; % change to lts_f_sum
preamble_mimo_B = [lts_t,lts_t,lts_t_sum];

preamble_A = [preamble_legacy_A, preamble_mimo_A];
preamble_B = [preamble_legacy_B, preamble_mimo_B];

% Sanity check variables that affect the number of Tx samples
if(SAMP_PADDING + INTERP_RATE*((N_OFDM_SYMS/2 * (N_SC + CP_LEN)) + length(preamble_A) + 100) > maximum_buffer_len)
    fprintf('Too many OFDM symbols for TX_NUM_SAMPS!\n');
    fprintf('Raise TX_NUM_SAMPS to %d, or \n', SAMP_PADDING + INTERP_RATE*((N_OFDM_SYMS/2 * (N_SC + CP_LEN)) + length(preamble_A) + 100));
    fprintf('Reduce N_OFDM_SYMS to %d\n',  2*(floor(( (maximum_buffer_len/INTERP_RATE)-length(preamble_A)-100-SAMP_PADDING )/( N_SC + CP_LEN )) - 1));
    return;
end

%% Generate a payload of random integers
zero_ones = [0,1];%
ones_zeros = [1,0];
% QAMDATA = [1,3,3,11,2,6,15,12];
% QAMDATA1 = [1,2,3,15,6,6,14,4];
% tx_data1=repmat(zero_ones,1,N_DATA_SYMS/4);
% tx_data=repmat(ones_zeros,1,N_DATA_SYMS/4);
% tx_data1=ones(1,N_DATA_SYMS/2);
% tx_data = zeros(1, N_DATA_SYMS/2);%[randi(MOD_ORDER, 1, N_DATA_SYMS-10) - 1TX_RF_VEC, ];
% tx_data = randi(MOD_ORDER, 1, N_DATA_SYMS/2) - 1 ;
% tx_data1 = randi(MOD_ORDER, 1, N_DATA_SYMS/2) - 1;

QPSKDATA = [2,2];
QPSKDATA1 = [0,2];

tx_data = repmat(QPSKDATA, 1, N_DATA_SYMS/4)  ;
tx_data1 = repmat(QPSKDATA1, 1, N_DATA_SYMS/4) ;

% tx_data2 = randi(MOD_ORDER, 1, N_DATA_SYMS/2) - 1;
% 
% tx_data = repmat(QAMDATA, 1, N_DATA_SYMS/16)  ;
% tx_data1 = repmat(QAMDATA1, 1, N_DATA_SYMS/16) ;
% Functions for data -> complex symbol mapping (like qammod, avoids comm toolbox requirement)
% These anonymous functions implement the modulation mapping from IEEE 802.11-2012 Section 18.3.5.8
modvec_bpsk   =  (1/sqrt(2))  .* [-1 1];
modvec_16qam  = (1/sqrt(10))  .* [-3 -1 +3 +1];

mod_fcn_bpsk  = @(x) complex(modvec_bpsk(1+x),0);
mod_fcn_qpsk  = @(x) complex(modvec_bpsk(1+bitshift(x, -1)), modvec_bpsk(1+mod(x, 2)));
mod_fcn_16qam = @(x) complex(modvec_16qam(1+bitshift(x, -2)), modvec_16qam(1+mod(x,4)));

% Map the data values on to complex symbols
switch MOD_ORDER
    case 2         % BPSKTX_RF_VEC
        tx_syms = arrayfun(mod_fcn_bpsk, tx_data);
        tx_syms1 = arrayfun(mod_fcn_bpsk, tx_data1);
      
    case 4         % QPSK
        tx_syms = arrayfun(mod_fcn_qpsk, tx_data);
        tx_syms1 = arrayfun(mod_fcn_qpsk, tx_data1);
        
    case 16        % 16-QAM
        tx_syms = arrayfun(mod_fcn_16qam, tx_data);
        tx_syms1 = arrayfun(mod_fcn_16qam, tx_data1);
       
    otherwise
        fprintf('Invalid MOD_ORDER (%d)!  Must be in [2, 4, 16]\n', MOD_ORDER);
        return;
end



% Break up the matrix into a vector for each antenna
tx_syms_A = tx_syms;
tx_syms_B = tx_syms1;


% Reshape the symbol vector to a matrix with one column per OFDM symbol
tx_syms_mat_A = reshape(tx_syms_A, length(SC_IND_DATA), N_OFDM_SYMS/2);
tx_syms_mat_B = reshape(tx_syms_B, length(SC_IND_DATA), N_OFDM_SYMS/2);

% Define the pilot tone values as BPSK symbols
%  We will transmit pilots only on RF A
pilots_A = [1 1 -1 1].';
pilots_B = [1 1 -1 1].';

% Repeat the pilots across all OFDM symbols
pilots_mat_A = repmat(pilots_A, 1, N_OFDM_SYMS/2);
pilots_mat_B = repmat(pilots_B, 1, N_OFDM_SYMS/2);

%% IFFT

% Construct the IFFT input matrix
ifft_in_mat_A = zeros(N_SC, N_OFDM_SYMS/2);
ifft_in_mat_B = zeros(N_SC, N_OFDM_SYMS/2);

% Insert the data and pilot values; other subcarriers will remain at 0
ifft_in_mat_A(SC_IND_DATA, :)   = tx_syms_mat_A;
ifft_in_mat_A(SC_IND_PILOTS, :) = pilots_mat_A;

ifft_in_mat_B(SC_IND_DATA, :)   = tx_syms_mat_B;
ifft_in_mat_B(SC_IND_PILOTS, :) = pilots_mat_B;

%Perform the IFFT
tx_payload_mat_A = ifft(ifft_in_mat_A, N_SC, 1);
tx_payload_mat_B = ifft(ifft_in_mat_B, N_SC, 1);

% Insert the cyclic prefix
if(CP_LEN > 0)
    tx_cp = tx_payload_mat_A((end-CP_LEN+1 : end), :);
    tx_payload_mat_A = [tx_cp; tx_payload_mat_A];
    
    tx_cp = tx_payload_mat_B((end-CP_LEN+1 : end), :);
    tx_payload_mat_B = [tx_cp; tx_payload_mat_B];
end

% Reshape to a vector
tx_payload_vec_A = reshape(tx_payload_mat_A, 1, numel(tx_payload_mat_A));
tx_payload_vec_B = reshape(tx_payload_mat_B, 1, numel(tx_payload_mat_B));


% Construct the full time-domain OFDM waveform
tx_vec_A = [preamble_A tx_payload_vec_A];
tx_vec_B = [preamble_B tx_payload_vec_B];



% Pad with zeros for transmission
tx_vec_padded_A = [tx_vec_A zeros(1,50)];
tx_vec_padded_B = [tx_vec_B zeros(1,50)];


%% Interpolate
if(INTERP_RATE == 1)
    tx_vec_air_A = tx_vec_padded_A;
    tx_vec_air_B = tx_vec_padded_B;
    
    tx1_vec_air_A = tx_vec_padded_A;
elseif(INTERP_RATE == 2)
	% Zero pad then filter (same as interp or upfirdn without signal processing toolbox)
    tx_vec_2x_A = zeros(1, 2*numel(tx_vec_padded_A));
    tx_vec_2x_A(1:2:end) = tx_vec_padded_A;
    tx_vec_air_A = filter(interp_filt2, 1, tx_vec_2x_A);
   
    tx_vec_2x_B = zeros(1, 2*numel(tx_vec_padded_B));
    tx_vec_2x_B(1:2:end) = tx_vec_padded_B;
    tx_vec_air_B = filter(interp_filt2, 1, tx_vec_2x_B);
end

% Scale the Tx vector to +/- 1
tx_vec_air_A = TX_SCALE .* tx_vec_air_A ./ max(abs(tx_vec_air_A));
tx_vec_air_B = TX_SCALE .* tx_vec_air_B ./ max(abs(tx_vec_air_B));

TX_NUM_SAMPS = length(tx_vec_air_A);

if(USE_WARPLAB_TXRX)
    wl_basebandCmd(nodes, 'tx_delay', 0);
    wl_basebandCmd(nodes, 'tx_length', TX_NUM_SAMPS+100);                   % Number of samples to send
    wl_basebandCmd(nodes, 'rx_length', TX_NUM_SAMPS+SAMP_PADDING);      % Number of samples to receive
end

if(USE_WARPLAB_TXRX)
    %% WARPLab Tx/Rx

    tx_mat_air = [tx_vec_air_A(:) , tx_vec_air_B(:)];
    
    %tx_mat_air = [tx_vec_air_B(:) , tx_vec_air_A(:)];

    % Write the Tx waveform to the Tx node
    wl_basebandCmd(node_tx, TX_RF_VEC, 'write_IQ', tx_mat_air(:,1:2));
    %wl_basebandCmd(node_tx1, TX1_RF_VEC, 'write_IQ', tx_mat_air(:,3));

    % Enable the Tx and Rx radios
    wl_interfaceCmd(node_tx, TX_RF, 'tx_en');
    %wl_interfaceCmd(node_tx1, TX_RF, 'tx_en');
    wl_interfaceCmd(node_rx, RX_RF, 'rx_en');

    % Enable the Tx and Rx buffers
    wl_basebandCmd(node_tx, TX_RF, 'tx_buff_en');
    
    wl_basebandCmd(node_rx, RX_RF, 'rx_buff_en');

    % Trigger the Tx/Rx cycle at both nodes
    eth_trig.send();

    % Retrieve the received waveform from the Rx node
    rx_mat_air = wl_basebandCmd(node_rx, RX_RF_VEC, 'read_IQ', 0, TX_NUM_SAMPS+SAMP_PADDING);

    rx_vec_air_A = rx_mat_air(:,1).';
    rx_vec_air_B = rx_mat_air(:,1).';

    % Disable the Tx/Rx radios and buffers
    wl_basebandCmd(node_tx, TX_RF_ALL, 'tx_rx_buff_dis');
    wl_basebandCmd(node_rx, RX_RF_ALL, 'tx_rx_buff_dis');
    
    wl_interfaceCmd(node_tx, TX_RF_ALL, 'tx_rx_dis');
    wl_interfaceCmd(node_rx, RX_RF_ALL, 'tx_rx_dis');
else
1
    rx_vec_air_A = tx_vec_air_A + .5 * tx_vec_air_B;
    rx_vec_air_B = tx_vec_air_B + .5 * tx_vec_air_A;

    rx_vec_air_A = [rx_vec_air_A, zeros(1,SAMP_PADDING)];
    rx_vec_air_B = [rx_vec_air_B, zeros(1,SAMP_PADDING)];

    rx_vec_air_A = rx_vec_air_A + 1e-2*complex(randn(1,length(rx_vec_air_A)), randn(1,length(rx_vec_air_A)));
    rx_vec_air_B = rx_vec_air_B + 1e-2*complex(randn(1,length(rx_vec_air_B)), randn(1,length(rx_vec_air_B)));

end

%% Decimate
if(DECIMATE_RATE == 1)
    raw_rx_dec_A = rx_vec_air_A;
    raw_rx_dec_B = rx_vec_air_B;
elseif(DECIMATE_RATE == 2)
    raw_rx_dec_A = filter(interp_filt2, 1, rx_vec_air_A);
    raw_rx_dec_A = raw_rx_dec_A(1:2:end);
    raw_rx_dec_B = filter(interp_filt2, 1, rx_vec_air_B);
    raw_rx_dec_B = raw_rx_dec_B(1:2:end);
end

%% Correlate for LTS

% For simplicity, we'll only use RFA for LTS correlation and peak
% discovery. A straightforward addition would be to repeat this process for
% RFB and combine the results for detection diversity.

% Complex cross correlation of Rx waveform with time-domain LTS
lts_corr = abs(conv(conj(fliplr(lts_t)), sign(raw_rx_dec_A)));

% Skip early and late samples - avoids occasional false positives from pre-AGC samples
lts_corr = lts_corr(32:end-32);

% Find all correlation peaks
lts_peaks = find(lts_corr > LTS_CORR_THRESH*max(lts_corr));

% Select best candidate correlation peak as LTS-payload boundary
% In this MIMO example, we actually have 3 LTS symbols sent in a row.
% The first two are sent by RFA on the TX node and the last one was sent
% by RFB. We will actually look for the separation between the first and the
% last for synchronizing our starting index.

[LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
[lts_last_peak_index,y] = find(LTS2-LTS1 == length(lts_t));

% Stop if no valid correlation peak was found
if(isempty(lts_last_peak_index))
    fprintf('No LTS Correlation Peaks Found!\n');
    return;
end

% Set the sample indices of the payload symbols and preamble
% The "+32" here corresponds to the 32-sample cyclic prefix on the preamble LTS
% The "+192" corresponds to the length of the extra training symbols for MIMO channel estimation
mimo_training_ind = lts_peaks(max(lts_last_peak_index)) + 32;
payload_ind = mimo_training_ind + 64;

% Subtract of 2 full LTS sequences and one cyclic prefixes
% The "-160" corresponds to the length of the preamble LTS (2.5 copies of 64-sample LTS)
lts_ind = mimo_training_ind-224;

if(DO_APPLY_CFO_CORRECTION)
    %Extract LTS (not yet CFO corrected)
    rx_lts = raw_rx_dec_A(lts_ind : lts_ind+159+64); %Extract the first two LTS for CFO
    rx_lts1 = rx_lts(-128+-FFT_OFFSET + [97+64:160+64]);
    rx_lts2 = rx_lts(-64-FFT_OFFSET + [97+64:160+64]);
    rx_lts3 = rx_lts(-FFT_OFFSET + [97+64:160+64]);
    %Calculate coarse CFO est
    rx_cfo_est_lts = mean(unwrap(angle(rx_lts2 .* conj(rx_lts1))));
    rx_cfo_est_lts = rx_cfo_est_lts/(2*pi*64);
else
    rx_cfo_est_lts = 0;
end

% Apply CFO correction to raw Rx waveforms
rx_cfo_corr_t = exp(-1i*2*pi*rx_cfo_est_lts*[0:length(raw_rx_dec_A)-1]);
rx_dec_cfo_corr_A = raw_rx_dec_A .* rx_cfo_corr_t;
rx_dec_cfo_corr_B = raw_rx_dec_B .* rx_cfo_corr_t;

% MIMO Channel Estimatation
lts_ind_TXA_start = lts_ind + 159 + 32 + 96 -FFT_OFFSET;
%lts_ind_TXA_start = mimo_training_ind - FFT_OFFSET;
lts_ind_TXA_end = lts_ind_TXA_start + 64 - 1;

lts_ind_TXB_start = lts_ind + 159 + 32 -FFT_OFFSET;
lts_ind_TXB_end = lts_ind_TXB_start + 64 - 1;

lts_ind_TXsum_start = lts_ind + 159 -FFT_OFFSET; %Take the sum preamble start
lts_ind_TXsum_end = lts_ind_TXsum_start + 64 - 1;

rx_lts_AA= rx_dec_cfo_corr_A( lts_ind_TXA_start:lts_ind_TXA_end );
rx_lts_BA = rx_dec_cfo_corr_A( lts_ind_TXB_start:lts_ind_TXB_end );

rx_lts_AB = rx_dec_cfo_corr_B( lts_ind_TXA_start:lts_ind_TXA_end );
rx_lts_sum = rx_dec_cfo_corr_B( lts_ind_TXsum_start:lts_ind_TXsum_end );

%rx_lts_BA = (rx_lts_BA1 +rx_lts_BA2)/2;

rx_lts_f = fft(rx_lts1);
rx_lts_f2 = fft(rx_lts2);
rx_lts_f3 = fft(rx_lts3);
rx_lts_AA_f = fft(rx_lts_AA);

rx_lts_AB_f = fft(rx_lts1);
rx_lts_BB_f = fft(rx_lts_BA);
rx_lts_sum_f = fft(rx_lts_sum);
% Calculate channel estimate
rx_H_est_AA = lts_f .* ((rx_lts_f+rx_lts_f2+rx_lts_f3)/3);
rx_H_est_BA = lts_f_B .* rx_lts_BB_f;

rx_H_est_AB = lts_f .* rx_lts_AA_f;
rx_H_est_BB = lts_f_B .* rx_lts_BB_f;

rx_H_est_sum = lts_f_sum .* rx_lts_sum_f;%add channel estimation for sum

%% Rx payload processing

% Extract the payload samples (integral number of OFDM symbols following preamble)
payload_vec_A = rx_dec_cfo_corr_A(payload_ind : payload_ind+(N_OFDM_SYMS/2)*(N_SC+CP_LEN)-1);
payload_mat_A = reshape(payload_vec_A, (N_SC+CP_LEN), (N_OFDM_SYMS/2));

payload_vec_B = rx_dec_cfo_corr_B(payload_ind : payload_ind+(N_OFDM_SYMS/2)*(N_SC+CP_LEN)-1);
payload_mat_B = reshape(payload_vec_B, (N_SC+CP_LEN), (N_OFDM_SYMS/2));

% Remove the cyclic prefix, keeping FFT_OFFSET samples of CP (on average)
payload_mat_noCP_A = payload_mat_A(CP_LEN-FFT_OFFSET+[1:N_SC], :);
payload_mat_noCP_B = payload_mat_B(CP_LEN-FFT_OFFSET+[1:N_SC], :);

% Take the FFT2
syms_f_mat_A = fft(payload_mat_noCP_A, N_SC, 1);
syms_f_mat_B = fft(payload_mat_noCP_B, N_SC, 1);

% Equalize pilots
% Because we only used Tx RFA to send pilots, we can do SISO equalization
% here. This is zero-forcing (just divide by chan estimates)
syms_eq_mat_pilots = syms_f_mat_A ./ repmat(rx_H_est_AA.', 1, N_OFDM_SYMS/2);

if DO_APPLY_SFO_CORRECTION
    % SFO manifests as a frequency-dependent phase whose slope increases
    % over time as the Tx and Rx sample streams drift apart from one
    % another. To correct for this effect, we calculate this phase slope at
    % each OFDM symbol using the pilot tones and use this slope to
    % interpolate a phase correction for each data-bearing subcarrier.

	% Extract the pilot tones and "equalize" them by their nominal Tx values
    pilots_f_mat = syms_eq_mat_pilots(SC_IND_PILOTS,:);
    pilots_f_mat_comp = pilots_f_mat.*pilots_mat_A;

	% Calculate the phases of every Rx pilot tone
    pilot_phases = unwrap(angle(fftshift(pilots_f_mat_comp, 1)), [], 1);

	pilot_spacing_mat = repmat(mod(diff(fftshift(SC_IND_PILOTS)), 64).', 1, N_OFDM_SYMS/2);
    pilot_slope_mat = mean(diff(pilot_phases) ./ pilot_spacing_mat);

	% Calculate the SFO correction phases for each OFDM symbol
    pilot_phase_sfo_corr = fftshift((-32:31).' * pilot_slope_mat, 1);
    pilot_phase_corr = exp(-1i*(pilot_phase_sfo_corr));

    % Apply the pilot phase correction per symbol
    syms_f_mat_A = syms_f_mat_A .* pilot_phase_corr;
    syms_f_mat_B = syms_f_mat_B .* pilot_phase_corr;
else
	% Define an empty SFO correction matrix (used by plotting code below)
    pilot_phase_sfo_corr = zeros(N_SC, N_OFDM_SYMS);
end

% Extract the pilots and calculate per-symbol phase error
if DO_APPLY_PHASE_ERR_CORRECTION
    pilots_f_mat = syms_eq_mat_pilots(SC_IND_PILOTS, :);
    pilot_phase_err = angle(mean(pilots_f_mat.*pilots_mat_A));
else
	% Define an empty phase correction vector (used by plotting code below)
    pilot_phase_err = zeros(1, N_OFDM_SYMS/2);
end
pilot_phase_corr = repmat(exp(-1i*pilot_phase_err), N_SC, 1);

% Apply pilot phase correction to both received streams
syms_f_mat_pc_A = syms_f_mat_A .* pilot_phase_corr;
syms_f_mat_pc_B = syms_f_mat_B .* pilot_phase_corr;

% MIMO Equalization
% We need to apply the MIMO equalization to each subcarrier separately.
% There, unfortunately, is no great vector-y solution to do this, so we
% reluctantly employ a FOR loop.

syms_eq_mat_A = zeros(N_SC, N_OFDM_SYMS/2);
syms_eq_mat_B = zeros(N_SC, N_OFDM_SYMS/2);
channel_condition_mat = zeros(1,N_SC);
for sc_idx = [SC_IND_DATA, SC_IND_PILOTS]
   y = syms_f_mat_pc_A(sc_idx,:);
   H1 = rx_H_est_AA(sc_idx);
   H2 = rx_H_est_sum(sc_idx);
   H3 = -0.2*rx_H_est_sum(sc_idx)+rx_H_est_AA(sc_idx);%; rx_H_est_BB(sc_idx), rx_H_est_BB(sc_idx)];
   x1 = H1\y;
   x2 = H2\y;
   x3 = H3\y;
   syms_eq_mat_A(sc_idx, :) = x1;
   syms_eq_mat_B(sc_idx, :) = x3;
   %channel_condition_mat(sc_idx) = rcond(H);
end

%subplot(2,1,1)
%plot(syms_eq_mat_A,'.')
%subplot(2,1,2)
%plot(syms_eq_mat_B,'.')

payload_syms_mat_A = syms_eq_mat_A(SC_IND_DATA, :);
payload_syms_mat_B = syms_eq_mat_B(SC_IND_DATA, :);

%% Demodulate
rx_syms_A = reshape(payload_syms_mat_A, 1, N_DATA_SYMS/2);
rx_syms_B = reshape(payload_syms_mat_B, 1, N_DATA_SYMS/2);

% Combine both streams to a single vector of symbols
rx_syms_space_mat = [rx_syms_A; rx_syms_B];
rx_syms = reshape(rx_syms_space_mat, 1, length(rx_syms_A)*2);

demod_fcn_bpsk = @(x) double(real(x)>0);
demod_fcn_qpsk = @(x) double(2*(real(x)>0) + 1*(imag(x)>0));
demod_fcn_16qam = @(x) (8*(real(x)>0)) + (4*(abs(real(x))<0.6325)) + (2*(imag(x)>0)) + (1*(abs(imag(x))<0.6325));

switch(MOD_ORDER)
    case 2         % BPSK
        rx_data = arrayfun(demod_fcn_bpsk, rx_syms);
    case 4         % QPSK
        rx_data = arrayfun(demod_fcn_qpsk, rx_syms);
    case 16        % 16-QAM
        rx_data = arrayfun(demod_fcn_16qam, rx_syms);
end


%% Plot Results
cf = 0;

% Tx signal
cf = cf + 1;
figure(cf); clf;

subplot(2,2,1);
plot(real(tx_vec_air_A), 'b');
axis([0 length(tx_vec_air_A) -TX_SCALE TX_SCALE])
grid on;
title('RFA Tx Waveform (I)');

subplot(2,2,2);
plot(imag(tx_vec_air_A), 'r');
axis([0 length(tx_vec_air_A) -TX_SCALE TX_SCALE])
grid on;
title('RFA Tx Waveform (Q)');

subplot(2,2,3);
plot(real(tx_vec_air_B), 'b');
axis([0 length(tx_vec_air_B) -TX_SCALE TX_SCALE])
grid on;
title('RFB Tx Waveform (I)');

subplot(2,2,4);
plot(imag(tx_vec_air_B), 'r');
axis([0 length(tx_vec_air_B) -TX_SCALE TX_SCALE])
grid on;
title('RFB Tx Waveform (Q)');

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_mimo_ofdm_plots_%s_txIQ', example_mode_string), '-dpng', '-r96', '-painters')
end

% Rx signal
cf = cf + 1;
figure(cf); clf;
subplot(2,2,1);
plot(real(rx_vec_air_A), 'b');
axis([0 length(rx_vec_air_A) -TX_SCALE TX_SCALE])
grid on;
title('RFA Rx Waveform (I)');

subplot(2,2,2);
plot(imag(rx_vec_air_A), 'r');
axis([0 length(rx_vec_air_A) -TX_SCALE TX_SCALE])
grid on;
title('RFA Rx Waveform (Q)');

subplot(2,2,3);
plot(real(rx_vec_air_B), 'b');
axis([0 length(rx_vec_air_B) -TX_SCALE TX_SCALE])
grid on;
title('RFB Rx Waveform (I)');

subplot(2,2,4);
plot(imag(rx_vec_air_B), 'r');
axis([0 length(rx_vec_air_B) -TX_SCALE TX_SCALE])
grid on;
title('RFB Rx Waveform (Q)');

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_mimo_ofdm_plots_%s_rxIQ', example_mode_string), '-dpng', '-r96', '-painters')
end

% Rx LTS correlation
cf = cf + 1;
figure(cf); clf;
lts_to_plot = lts_corr;
plot(lts_to_plot, '.-b', 'LineWidth', 1);
hold on;
grid on;
line([1 length(lts_to_plot)], LTS_CORR_THRESH*max(lts_to_plot)*[1 1], 'LineStyle', '--', 'Color', 'r', 'LineWidth', 2);
title('LTS Correlation and Threshold')
xlabel('Sample Index')
myAxis = axis();
axis([1, 1000, myAxis(3), myAxis(4)])

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_mimo_ofdm_plots_%s_ltsCorr', example_mode_string), '-dpng', '-r96', '-painters')
end

% Channel Estimates
cf = cf + 1;

rx_H_est_plot_AA = rx_H_est_AA;
rx_H_est_plot_AB = rx_H_est_AB;
rx_H_est_plot_BA = rx_H_est_BA;
rx_H_est_plot_BB = rx_H_est_BB;

x = (20/N_SC) * (-(N_SC/2):(N_SC/2 - 1));

figure(cf); clf;

subplot(3,2,1);
bh = bar(x, fftshift(abs(rx_H_est_plot_AA)),1,'LineWidth', 1);
shading flat
set(bh,'FaceColor',[0 0 1])
axis([min(x) max(x) 0 1.1*max(abs(rx_H_est_plot_AA))])
grid on;
title('A->A Channel Estimates (Magnitude)')
xlabel('Baseband Frequency (MHz)')

subplot(3,2,2);
bh = bar(x, fftshift(abs(rx_H_est_plot_AB)),1,'LineWidth', 1);
shading flat
set(bh,'FaceColor',[0 0 1])
axis([min(x) max(x) 0 1.1*max(abs(rx_H_est_plot_AB))])
grid on;
title('A->B Channel Estimates (Magnitude)')
xlabel('Baseband Frequency (MHz)')

subplot(3,2,3);
bh = bar(x, fftshift(abs(rx_H_est_plot_BA)),1,'LineWidth', 1);
shading flat
set(bh,'FaceColor',[0 0 1])
axis([min(x) max(x) 0 1.1*max(abs(rx_H_est_plot_BA))])
grid on;
title('B->A Channel Estimates (Magnitude)')
xlabel('Baseband Frequency (MHz)')

subplot(3,2,4);
bh = bar(x, fftshift(abs(rx_H_est_plot_BB)),1,'LineWidth', 1);
shading flat
set(bh,'FaceColor',[0 0 1])
axis([min(x) max(x) 0 1.1*max(abs(rx_H_est_plot_BB))])
grid on;
title('B->B Channel Estimates (Magnitude)')
xlabel('Baseband Frequency (MHz)')


subplot(3,1,3);
bh = bar(x, fftshift(channel_condition_mat) ,1,'LineWidth', 1);
shading flat
set(bh,'FaceColor',[1 0 0])
axis([min(x) max(x) 0 1.1])
grid on;
title('Channel Condition')
xlabel('Baseband Frequency (MHz)')

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_mimo_ofdm_plots_%s_chanEst', example_mode_string), '-dpng', '-r96', '-painters')
end

% Pilot phase error estimate
cf = cf + 1;
figure(cf); clf;
subplot(2,1,1)
plot(pilot_phase_err, 'b', 'LineWidth', 2);
title('Phase Error Estimates')
xlabel('OFDM Symbol Index')
ylabel('Radians')
axis([1 N_OFDM_SYMS/2 -3.2 3.2])
grid on

h = colorbar;
set(h,'Visible','off');

subplot(2,1,2)
imagesc(1:N_OFDM_SYMS, (SC_IND_DATA - N_SC/2), fftshift(pilot_phase_sfo_corr,1))
xlabel('OFDM Symbol Index')
ylabel('Subcarrier Index')
title('Phase Correction for SFO')
colorbar
myAxis = caxis();
if(myAxis(2)-myAxis(1) < (pi))
   caxis([-pi/2 pi/2])
end

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_mimo_ofdm_plots_%s_phaseError', example_mode_string), '-dpng', '-r96', '-painters')
end

% Symbol constellation
cf = cf + 1;
figure(cf); clf;

subplot(2,2,1)
rx_syms_plot = syms_f_mat_pc_A(SC_IND_DATA,:);
zero_sub = length(find(abs(syms_f_mat_pc_A([2:7 9:21 23:27 39:43 45:57 59:64],:))<2))/(48*900);
plot(rx_syms_plot(:),'go','MarkerSize',1)

axis square
grid on
title('Unequalized Rx Symbols RFA')
myAxis = [];
myAxis(1,:) = axis();

subplot(2,2,2)
rx_syms_plot = syms_f_mat_A(SC_IND_DATA,:);
plot(rx_syms_plot(:),'go','MarkerSize',1)
axis square
grid on
title('Unequalized Rx Symbols RFB')
myAxis(2,:) = axis();

subplot(2,2,1); axis([-max(abs(myAxis(:))), max(abs(myAxis(:))), ...
                      -max(abs(myAxis(:))), max(abs(myAxis(:)))])
subplot(2,2,2); axis([-max(abs(myAxis(:))), max(abs(myAxis(:))), ...
                      -max(abs(myAxis(:))), max(abs(myAxis(:)))])



subplot(2,2,3)
rx_syms_plot = syms_eq_mat_A(SC_IND_DATA,:);
plot(rx_syms_plot(:),'ro','MarkerSize',1)
axis square; axis(2*[-1 1 -1 1]);
grid on
title('Equalized Rx Symbols RFA')
hold off;

subplot(2,2,4)
rx_syms_plot = syms_eq_mat_B(SC_IND_DATA,:);
plot(rx_syms_plot(:),'ro','MarkerSize',1)
axis square; axis(2*[-1 1 -1 1]);
grid on
title('Equalized Rx Symbols RFB')
hold on;
plot(tx1_syms_mat_A(:),'bo');
hold off;

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_mimo_ofdm_plots_%s_constellations', example_mode_string), '-dpng', '-r96', '-painters')
end



% EVM & SNR
cf = cf + 1;
figure(cf); clf;

evm_mat_A = abs(payload_syms_mat_A - tx_syms_mat_A).^2;
evm_mat_B = abs(payload_syms_mat_B - tx_syms_mat_B).^2;
aevms_A = mean(evm_mat_A(:));
aevms_B = mean(evm_mat_B(:));
snr_A = 10*log10(1./aevms_A);
snr_B = 10*log10(1./aevms_B);

subplot(2,2,1)
plot(100*evm_mat_A(:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(evm_mat_A(:))], 100*[aevms_A, aevms_A],'r','LineWidth',4)
hold off
xlabel('Data Symbol Index')
ylabel('EVM (%)');
legend('Per-Symbol EVM','Average EVM','Location','NorthWest');

title('Stream A')
grid on
myAxis_mat(1,:) = axis();

subplot(2,2,2)
plot(100*evm_mat_B(:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(evm_mat_B(:))], 100*[aevms_B, aevms_B],'r','LineWidth',4)
hold off
xlabel('Data Symbol Index')
ylabel('EVM (%)');
legend('Per-Symbol EVM','Average EVM','Location','NorthWest');
title('Stream B')
grid on
myAxis_mat(2,:) = axis();

subplot(2,2,1); axis([min(myAxis_mat(:,1)), max(myAxis_mat(:,2)) ...
                      min(myAxis_mat(:,3)), min(myAxis_mat(:,4))]);
myAxis = axis;
h = text(round(.05*length(evm_mat_A(:))), 100*aevms_A+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SINR: %.1f dB', snr_A));
set(h,'Color',[1 0 0])
set(h,'FontWeight','bold')
set(h,'FontSize',10)
set(h,'EdgeColor',[1 0 0])
set(h,'BackgroundColor',[1 1 1])
subplot(2,2,2); axis([min(myAxis_mat(:,1)), max(myAxis_mat(:,2)) ...
                      min(myAxis_mat(:,3)), min(myAxis_mat(:,4))]);
myAxis = axis;
h = text(round(.05*length(evm_mat_B(:))), 100*aevms_B+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SINR: %.1f dB', snr_B));
set(h,'Color',[1 0 0])
set(h,'FontWeight','bold')
set(h,'FontSize',10)
set(h,'EdgeColor',[1 0 0])
set(h,'BackgroundColor',[1 1 1])


subplot(2,2,3)
imagesc(1:N_OFDM_SYMS, (SC_IND_DATA - N_SC/2), 100*fftshift(evm_mat_A,1))

hold on
h = line([1,N_OFDM_SYMS],[0,0]);
set(h,'Color',[1 0 0])
set(h,'LineStyle',':')
hold off
%myAxis = gca;
%set(myAxis,'YTickLabel', fftshift(get(myAxis,'YTickLabel'),1))
grid on
xlabel('OFDM Symbol Index')
ylabel('Subcarrier Index')
title('Stream A')
h = colorbar;
set(get(h,'title'),'string','EVM (%)');
myAxis = caxis();
if (myAxis(2)-myAxis(1)) < 5
    caxis([myAxis(1), myAxis(1)+5])
end

myAxis_mat = [];
myAxis_mat(1,:) = caxis();

subplot(2,2,4)
imagesc(1:N_OFDM_SYMS, (SC_IND_DATA - N_SC/2), 100*fftshift(evm_mat_B,1))

hold on
h = line([1,N_OFDM_SYMS],[0,0]);
set(h,'Color',[1 0 0])
set(h,'LineStyle',':')
hold off
%myAxis = gca;
%set(myAxis,'YTickLabel', fftshift(get(myAxis,'YTickLabel'),1))
grid on
xlabel('OFDM Symbol Index')
ylabel('Subcarrier Index')
title('Stream B')
h = colorbar;
set(get(h,'title'),'string','EVM (%)');
myAxis = caxis();
if (myAxis(2)-myAxis(1)) < 5
    caxis([myAxis(1), myAxis(1)+5])
end

myAxis_mat(2,:) = caxis();

subplot(2,2,3); caxis([min(myAxis_mat(:,1)), max(myAxis_mat(:,2))])

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_mimo_ofdm_plots_%s_evm', example_mode_string), '-dpng', '-r96', '-painters')
end

%% Calculate the error
zero_0 = find(tx_data==0 & tx_data1==0);
zero_0_error = length(find(real(rx_syms_B(zero_0))<-0.7 & imag(rx_syms_B(zero_0))<-0.7));%6

zero_1 = find(tx_data==0 & tx_data1==1);
zero_1_error = length(find(imag(rx_syms_B(zero_1))<0.7 & imag(rx_syms_B(zero_1))>-0.7 & real(rx_syms_B(zero_1))<-0.7));%5

zero_2 = find(tx_data==0 & tx_data1==2);
zero_2_error = length(find(real(rx_syms_B(zero_2))<0.7 & real(rx_syms_B(zero_2))>-0.7 & imag(rx_syms_B(zero_2))<-0.5));%7

zero_3 = find(tx_data==0 & tx_data1==3);
zero_3_error = length(find(real(rx_syms_B(zero_3))<0.8 & real(rx_syms_B(zero_3))>-0.8 & imag(rx_syms_B(zero_3))<0.8 & imag(rx_syms_B(zero_3))>-0.8));%9

one_0 = find(tx_data==1 & tx_data1==0);8
one_0_error = length(find(imag(rx_syms_B(one_0))<0.7 & imag(rx_syms_B(one_0))>-0.7 & real(rx_syms_B(one_0))<-0.7));%5

one_1 = find(tx_data==1 & tx_data1==1);
one_1_error = length(find(real(rx_syms_B(one_1))<-0.7 & imag(rx_syms_B(one_1))>-0.7));%4

one_2 = find(tx_data==1 & tx_data1==2);
one_2_error = length(find(real(rx_syms_B(one_2))<0.7 & real(rx_syms_B(one_2))>-0.7 & imag(rx_syms_B(one_2))<0.7 & imag(rx_syms_B(one_2))>-0.7));%9

one_3 = find(tx_data==1 & tx_data1==3);
one_3_error = length(find(real(rx_syms_B(one_3))<0.7 & real(rx_syms_B(one_3))>-0.7 & imag(rx_syms_B(one_3))>0.7));  %3



two_0 = find(tx_data==2 & tx_data1==0);
two_0_error = length(find(real(rx_syms_B(two_0))<0.7 & real(rx_syms_B(two_0))>-0.7 & imag(rx_syms_B(two_0))<-0.7));%7

two_1 = find(tx_data==2 & tx_data1==1);
two_1_error = length(find(real(rx_syms_B(two_1))<0.7 & real(rx_syms_B(two_1))>-0.7 & imag(rx_syms_B(two_1))<0.7 & imag(rx_syms_B(two_1))>-0.7));%9

two_2 = find(tx_data==2 & tx_data1==2);
two_2_error = length(find(real(rx_syms_B(two_2))>0.7 & imag(rx_syms_B(two_2))<-0.7));%8

two_3 = find(tx_data==2 & tx_data1==3);
two_3_error = length(find(imag(rx_syms_B(two_3))<0.7 & imag(rx_syms_B(two_3))>-0.7 & real(rx_syms_B(two_3))>0.7));%1

three_0 = find(tx_data==3 & tx_data1==0);
three_0_error = length(find(real(rx_syms_B(three_0))<0.7 & real(rx_syms_B(three_0))>-0.7 & imag(rx_syms_B(three_0))<0.7 & imag(rx_syms_B(three_0))>-0.7));%9

three_1 = find(tx_data==3 & tx_data1==1);
three_1_error = length(find(real(rx_syms_B(three_1))<0.7 & real(rx_syms_B(three_1))>-0.7 & imag(rx_syms_B(three_1))>0.7));  %3

three_2 = find(tx_data==3 & tx_data1==2);
three_2_error = length(find(imag(rx_syms_B(three_2))<0.7 & imag(rx_syms_B(three_2))>-0.7 & real(rx_syms_B(three_2))>0.7));%1

three_3 = find(tx_data==3 & tx_data1==3);
three_3_error = length(find(real(rx_syms_B(three_3))>0.7 & imag(rx_syms_B(three_3))>0.7));%2

error=N_DATA_SYMS/2-zero_0_error-zero_1_error-zero_2_error-zero_3_error-one_0_error-one_1_error-one_2_error-one_3_error-two_0_error-two_1_error-two_2_error-two_3_error...
    -three_0_error-three_1_error-three_2_error-three_3_error;
error