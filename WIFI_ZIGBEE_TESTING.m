%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wl_example_siso_ofdm_txrx.m
% A detailed write-up of this example is available on the wiki:
% http://warpproject.org/trac/wiki/WARPLab/Examples/OFDM
%
% Copyright (c) 2015 Mango Communications - All Rights Reserved
% Distributed under the WARP License (http://warpproject.org/license)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all

% Params:
USE_WARPLAB_TXRX        = 1;           % Enable WARPLab-in-the-loop (otherwise sim-only)
WRITE_PNG_FILES         = 0;           % Enable writing plots to PNG
CHANNEL                 = 6;          % Channel to tune Tx and Rx radios

% Waveform params
N_OFDM_SYMS             = 1680;         % Number of OFDM symbols
N_OFDM_SYMS1            = 1680; 
MOD_ORDER               = 64;           % Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)
TX_SCALE                = 1.0;          % Scale for Tx waveform ([0:1])

% OFDM params
SC_IND_PILOTS           = [8 22 44 58];% Pilot subcarrier indices
SC_IND_PILOTSrx         = [8 22 58];
SC_IND_DATA             = [2:7 9:21 23:27 39:43 45:57 59:64];     % Data subcarrier indices
N_SC                    = 64;                                     % Number of subcarriers
CP_LEN                  = 16;                                     % Cyclic prefix length
N_DATA_SYMS             = N_OFDM_SYMS * length(SC_IND_DATA);      % Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
INTERP_RATE             = 2;                                      % Interpolation rate (must be 2)

% Rx processing params
FFT_OFFSET                    = 4;           % Number of CP samples to use in FFT (on average)
LTS_CORR_THRESH               = 0.8;         % Normalized threshold for LTS correlation
DO_APPLY_CFO_CORRECTION       = 1;           % Enable CFO estimation/correction
DO_APPLY_PHASE_ERR_CORRECTION = 1;           % Enable Residual CFO estimation/correction
DO_APPLY_SFO_CORRECTION       = 1;           % Enable SFO estimation/correction
conv_order                    = 1/2;
DECIMATE_RATE                 = INTERP_RATE;

% WARPLab experiment params
USE_AGC                 = true;        % Use the AGC if running on WARP hardware
MAX_TX_LEN              = 2^26;        % Maximum number of samples to use for this experiment
TRIGGER_OFFSET_TOL_NS   = 3000;        % Trigger time offset toleration between Tx and Rx that can be accomodated
%Zigbee Modes
%1chan11[39 40 41 42 43 44 45 46 47 48 49 50];
%1chan12[14 15 16 17 18 19 20 21 22 23 24 25 26 27];;
%1chan13[39 40 41 42 43 44 45 46 47 48 49 50];
%1chan14[4 5 6 7 8 9 10 11 12 13 14 15 16];
%6chan15[55 56 57 58 59 60 61 62 63 64]
%6chan16[39 40 41 42 43 44 45 46 47]
%6chan17[49 50 51 52 53 54 55 56 57 58 59 60 61 62 63]
%6chan18[22 23 24 25 26 27 39 40 41 42 43 44 45]
%6chan19[3 4 5 6 7 8 9 10 11 12 13 14]
%11chan20[57 58 59 60 61 62 63 64]
%11chan21[39 40 41 42 43 44 45 46 47 48]
%11chan22[55 56 57 58 59 60 61 62];
%11chan23[ 7 8 9 10 11 12 13 14 ];
%11chan24[25 26 27];%
%2chan12[39 40 41 42 43 44 45 46 47 48 49 50];
%2chan13[55 56 57 58 59 60 61 62 63 64];
%2chan14[3 4 5 6 7 8 9 10 11 12 13 14 15 16 17];
%2chan15[16 17 18 19 20 21 22 23 24 25 26 27];
%3chan13[39 40 41 42 43];
%3chan14[54 55 56 57 58 59 60 61 62];
%3chan15[6 7 8 9 10 11 12 13 14];
%3chan16[18 19 20 21 22 23 24 25 26 27];
%3chan17[1];
%4chan14[39 40 41 42 43 44 45 46 47 48 49];
%4chan15[52 53 54 55 56 57 58 59 60 61 62 63 64];
%4chan16[4 5 6 7 8 9 10 11 12 13 14 15 16 17];
%4chan17[24 25 26 27];
%5chan15[39 40 41 42 43 44 45 46 47 48 49];
%5chan16[52 53 54 55 56 57 58 59 60 61 62 63 64];
%5chan17[4 5 6 7 8 9 10 11 12 13 14 15 16 17];
%5chan18[24 25 26 27];
mode11_ind=[39 40 41 42 43 44 45 46 47 48];
mode12_ind=[16 17 18 19 20 21 22 23 24 25 26 27];%1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63
mode13_ind=[39 40 41 42 43 44 45 46 47 48 49 50];%chan18 
mode14_ind=[3 4 5 6 7 8 9 10 11 12 13 14];
modeALL_ind=[1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64];
NUM_SAMPLES     = 2^27;
tx_sym_del=[25 26 27 28 29 30 31 32 33];
%Convolution code
convEncoder = comm.ConvolutionalEncoder(poly2trellis(7, [171 133]));
% convEncoder.PuncturePatternSource = 'Property';
% convEncoder.PuncturePattern = [1;1;1;0;0;1]; %[1;1;1;1;0;1;1;1;1;1;0;0];
vitDecoder = comm.ViterbiDecoder(poly2trellis(7, [171 133]), ...
  'InputFormat', 'Unquantized');
vitDecoder.PuncturePatternSource =  'Property';
vitDecoder.PuncturePattern = convEncoder.PuncturePattern;
vitDecoder.TracebackDepth = 96;
errorCalc = comm.ErrorRate('ReceiveDelay', vitDecoder.TracebackDepth);

if(USE_WARPLAB_TXRX)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Set up the WARPLab experiment
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    NUMNODES = 2;

    % Create a vector of node objects
    nodes   = wl_initNodes(NUMNODES);
    node_tx = nodes(2);
    node_rx = nodes(1);
    %node_monitor=nodes(3);% Create channel scanning node
    
    % Create a UDP broadcast trigger and tell each node to be ready for it
    eth_trig = wl_trigger_eth_udp_broadcast;
    wl_triggerManagerCmd(nodes, 'add_ethernet_trigger', [eth_trig]);   
    % Read Trigger IDs into workspace
    trig_in_ids  = wl_getTriggerInputIDs(nodes(1));
    trig_out_ids = wl_getTriggerOutputIDs(nodes(1));
    % For both nodes, we will allow Ethernet to trigger the buffer baseband and the AGC
    wl_triggerManagerCmd(nodes, 'output_config_input_selection', [trig_out_ids.BASEBAND, trig_out_ids.AGC], [trig_in_ids.ETH_A]);

    % Set the trigger output delays.
    nodes.wl_triggerManagerCmd('output_config_delay', [trig_out_ids.BASEBAND], 0);
    nodes.wl_triggerManagerCmd('output_config_delay', [trig_out_ids.AGC], TRIGGER_OFFSET_TOL_NS);
    
    % Get IDs for the interfaces on the boards. 
    ifc_ids_TX = wl_getInterfaceIDs(node_tx);
    ifc_ids_RX = wl_getInterfaceIDs(node_rx);
    % Get IDs for the interfaces on the board.
%    ifc_ids = wl_getInterfaceIDs(node_monitor);

    % Use RFA as the receiver for monitor node
%     RF_RX     = ifc_ids.RF_A;
%     RF_RX_VEC = ifc_ids.RF_A;
    %set up monitor node
    %wl_triggerManagerCmd(node_monitor, 'output_config_input_selection', [trig_out_ids.BASEBAND, trig_out_ids.AGC], [trig_in_ids.ETH_A]);
    %node_monitor.wl_triggerManagerCmd('output_config_delay', [trig_out_ids.BASEBAND], 0);
        %node_monitor.wl_triggerManagerCmd('output_config_delay', [trig_out_ids.AGC], 3000);   
    % Check the number of samples
    
%     max_rx_samples = wl_basebandCmd(node_monitor, RF_RX, 'rx_buff_max_num_samples');
%     Ts_monitor = 1/(wl_basebandCmd(node_monitor, 'tx_buff_clk_freq'));

    % Print information to the console
    %fprintf('WARPLab Spectrogram Example:\n');
    %fprintf('    Generating spectrogram using %.4f seconds of data (%d samples).\n', (NUM_SAMPLES * Ts), NUM_SAMPLES );

    % Create a UDP broadcast trigger and tell each node to be ready for it
%     eth_trig1 = wl_trigger_eth_udp_broadcast;
%     wl_triggerManagerCmd(node_monitor, 'add_ethernet_trigger', [eth_trig1]);

    % Set up the interface for the experiment
%     wl_interfaceCmd(node_monitor, ifc_ids.RF_ALL, 'channel', 2.4, CHANNEL);
% 
%     % Set the gains manually
%     wl_interfaceCmd(node_monitor, ifc_ids.RF_ALL, 'rx_gain_mode', 'manual');
%     RxGainRF = 3;              % Rx RF Gain in [1:3]
%     RxGainBB = 10;               % Rx Baseband Gain in [0:31]
%     wl_interfaceCmd(node_monitor, ifc_ids.RF_ALL, 'rx_gains', RxGainRF, RxGainBB);
    % Set up the TX / RX nodes and RF interfaces
    TX_RF     = ifc_ids_TX.RF_A;
    TX_RF_VEC = ifc_ids_TX.RF_A;
    TX_RF_ALL = ifc_ids_TX.RF_ALL;
    
    RX_RF     = ifc_ids_RX.RF_A;
    RX_RF_VEC = ifc_ids_RX.RF_A;
    RX_RF_ALL = ifc_ids_RX.RF_ALL;

    % Set up the interface for the experiment
    wl_interfaceCmd(node_tx, TX_RF_ALL, 'channel', 2.4, CHANNEL);
    wl_interfaceCmd(node_rx, RX_RF_ALL, 'channel', 2.4, CHANNEL);

    wl_interfaceCmd(node_tx, TX_RF_ALL, 'tx_gains', 3, 21);

    if(USE_AGC)
        wl_interfaceCmd(node_rx, RX_RF_ALL, 'rx_gain_mode', 'automatic');
        wl_basebandCmd(nodes, 'agc_target', -13);
    else
        wl_interfaceCmd(node_rx, RX_RF_ALL, 'rx_gain_mode', 'manual');
        RxGainRF = 2;                  % Rx RF Gain in [1:3]
        RxGainBB = 20;                 % Rx Baseband Gain in [0:31]
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
    maximum_buffer_len = min(MAX_TX_LEN, 2^20);
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
sts_f(39:64) = [0 0 0 0 0 0 0 0 0 1+1i 0 1+1i 0 -1-1i 0 0 0 -1-1i 0 0 0 1+1i 0 0 0 -1-1i];
sts_f(1,mode11_ind)=0;
sts_t = ifft(sqrt(13/6).*sts_f, 64);
sts_t = sts_t(1:16);

% LTS for CFO and channel estimation
lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
%lts_f(1,mode11_ind)=[0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1];
lts_t = ifft(lts_f, 64);

% Use 30 copies of the 16-sample STS for extra AGC settling margin
preamble = [repmat(sts_t, 1, 30)  lts_t(33:64) lts_t lts_t];

% Sanity check variables that affect the number of Tx samples
num_samps_needed = ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)) + ...
                    INTERP_RATE*((N_OFDM_SYMS * (N_SC + CP_LEN)) + length(preamble) +  ceil(length(interp_filt2)/2));

                                
if(num_samps_needed > maximum_buffer_len)
    fprintf('Too many OFDM symbols for TX_NUM_SAMPS!\n');
    fprintf('Raise MAX_TX_LEN to %d, or \n', num_samps_needed);
    fprintf('Reduce N_OFDM_SYMS to %d\n', floor(((maximum_buffer_len - ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)))/INTERP_RATE - (length(preamble) +  ceil(length(interp_filt2)/2)))/(N_SC + CP_LEN)));
    return;
end

%% Generate a payload of random integers
data_mode=2;
tx_data_raw = randi(MOD_ORDER, 1, N_DATA_SYMS-1) - 1;

tx_data_raw_re = reshape(de2bi(tx_data_raw),[],1);
tx_data_encode = convEncoder(tx_data_raw_re);
tx_data = bi2de(reshape(tx_data_encode,[],6))';
tx_data = [data_mode data_mode tx_data];
% Functions for data -> complex symbol mapping (like qammod, avoids comm toolbox requirement)
% These anonymous functions implement the modulation mapping from IEEE 802.11-2012 Section 18.3.5.8
modvec_bpsk   =  (1/sqrt(2))  .* [-1 1];
modvec_16qam  =  (1/sqrt(10)) .* [-3 -1 +3 +1];
modvec_64qam  =  (1/sqrt(43)) .* [-7 -5 -1 -3 +7 +5 +1 +3];

mod_fcn_bpsk  = @(x) complex(modvec_bpsk(1+x),0);
mod_fcn_qpsk  = @(x) complex(modvec_bpsk(1+bitshift(x, -1)), modvec_bpsk(1+mod(x, 2)));
mod_fcn_16qam = @(x) complex(modvec_16qam(1+bitshift(x, -2)), modvec_16qam(1+mod(x,4)));
mod_fcn_64qam = @(x) complex(modvec_64qam(1+bitshift(x, -3)), modvec_64qam(1+mod(x,8)));

% Map the data values on to complex symbols
switch MOD_ORDER
    case 2         % BPSK
        tx_syms = arrayfun(mod_fcn_bpsk, tx_data);
    case 4         % QPSK
        tx_syms = arrayfun(mod_fcn_qpsk, tx_data);
    case 16        % 16-QAM
        tx_syms = arrayfun(mod_fcn_16qam, tx_data);
    case 64        % 64-QAM
        tx_syms = arrayfun(mod_fcn_64qam, tx_data);
    otherwise
        fprintf('Invalid MOD_ORDER (%d)!  Must be in [2, 4, 16, 64]\n', MOD_ORDER);
        return;
end
N_OFDM_SYMS = length(tx_syms)/48;
% Reshape the symbol vector to a matrix with one column per OFDM symbol
tx_syms_mat = reshape(tx_syms, length(SC_IND_DATA), N_OFDM_SYMS);

% Define the pilot tone values as BPSK symbols
pilots = [1 1 1 1].';

% Repeat the pilots across all OFDM symbols
pilots_mat = repmat(pilots, 1, N_OFDM_SYMS);

%% IFFT

% Construct the IFFT input matrix
ifft_in_mat = zeros(N_SC, N_OFDM_SYMS);

% Insert the data and pilot values; other subcarriers will remain at 0
ifft_in_mat(SC_IND_DATA, :)   = tx_syms_mat;
ifft_in_mat(SC_IND_PILOTS, :) = pilots_mat;
%ifft_in_mat(mode11_ind,:)=0;
%100% config
% ifft_in_mat(mode11_ind,1:4:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,3:4:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,7:16:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,9:16:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,11:16:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,13:16:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,15:16:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,3:16:N_OFDM_SYMS)=0;
%Testing Config 2-2-3-2-2-3-2-2-3-2-2-3-2-2-3-2
ifft_in_mat(mode11_ind,1:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,3:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,5:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,7:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,9:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,11:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,13:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,15:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,17:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,19:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,23:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,25:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,27:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,29:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,31:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,33:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,35:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,37:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,39:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,43:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,45:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,47:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,49:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,53:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,55:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,57:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,59:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,63:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,61:64:N_OFDM_SYMS)=0;
% ifft_in_mat(mode11_ind,51:64:N_OFDM_SYMS)=0;


%Perform the IFFT
tx_payload_mat = ifft(ifft_in_mat, N_SC, 1);

% Insert the cyclic prefix
if(CP_LEN > 0)
    tx_cp = tx_payload_mat((end-CP_LEN+1 : end), :);
    tx_payload_mat = [tx_cp; tx_payload_mat];
end

% Reshape to a vector
tx_payload_vec = reshape(tx_payload_mat, 1, numel(tx_payload_mat));

% Construct the full time-domain OFDM waveform
tx_vec = [preamble tx_payload_vec];

% Pad with zeros for transmission to deal with delay through the
% interpolation filter
tx_vec_padded = [tx_vec, zeros(1, ceil(length(interp_filt2)/2))];

%% Interpolate
% Zero pad then filter (same as interp or upfirdn without signal processing toolbox)
if( INTERP_RATE ~= 2)
   fprintf('Error: INTERP_RATE must equal 2\n'); 
   return;
end

tx_vec_2x = zeros(1, 2*numel(tx_vec_padded));
tx_vec_2x(1:2:end) = tx_vec_padded;
tx_vec_air = filter(interp_filt2, 1, tx_vec_2x);


% Scale the Tx vector to +/- 1
tx_vec_air = TX_SCALE .* tx_vec_air ./ max(abs(tx_vec_air));
%Adjust WiFi Channel Usage
Non_trans_time=zeros(1,0.25*length(tx_vec_air));
tx_vec_air = [tx_vec_air Non_trans_time];

TX_NUM_SAMPS = length(tx_vec_air);

if(USE_WARPLAB_TXRX)
    wl_basebandCmd(nodes, 'tx_delay', 0);
    wl_basebandCmd(nodes, 'tx_length', TX_NUM_SAMPS);                                                        % Number of samples to send
    wl_basebandCmd(nodes, 'rx_length', TX_NUM_SAMPS + ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)));   % Number of samples to receive
end


%% WARPLab Tx/Rx
if(USE_WARPLAB_TXRX)
    % Write the Tx waveform to the Tx node
    wl_basebandCmd(node_tx, TX_RF_VEC, 'write_IQ', tx_vec_air(:));
    %enable node monitor
%     wl_basebandCmd(node_monitor, 'rx_length', NUM_SAMPLES);
%        % Enable to node to receive data 
%     wl_interfaceCmd(node_monitor, RF_RX, 'rx_en');
%     wl_basebandCmd(node_monitor, RF_RX, 'rx_buff_en');
    
    % Open up the transceivITer's low-pass filter to its maximum bandwidth (36MHz)
    %wl_interfaceCmd(node_monitor, RF_RX, 'rx_lpf_corn_freq', 3);
    %eth_trig1.send();
    
    wl_basebandCmd(node_tx,'continuous_tx',1);
    % Enable the Tx and Rx radios
    wl_interfaceCmd(node_tx, TX_RF, 'tx_en');
    wl_interfaceCmd(node_rx, RX_RF, 'rx_en');

    % Enable the Tx and Rx buffers
    wl_basebandCmd(node_tx, TX_RF, 'tx_buff_en');
    wl_basebandCmd(node_rx, RX_RF, 'rx_buff_en');

    % Trigger the Tx/Rx cycle at both nodes
    eth_trig.send();
    tic    
    % Retrieve the received waveform from the Rx node
    rx_vec_air = wl_basebandCmd(node_rx, RX_RF_VEC, 'read_IQ', 0, TX_NUM_SAMPS+(ceil((TRIGGER_OFFSET_TOL_NS*1e-9)/(1/SAMP_FREQ))));
    tpass=toc;
%     rx_IQ = wl_basebandCmd(node_monitor, RF_RX_VEC, 'read_IQ', 0, NUM_SAMPLES);
    % Trigger the node to receive samples
    % Read the samples from the node
    pause(60);

    % Disable the RX buffers
%     wl_basebandCmd(node_monitor, ifc_ids.RF_ALL, 'tx_rx_buff_dis');
%     wl_interfaceCmd(node_monitor, ifc_ids.RF_ALL, 'tx_rx_dis');

    rx_vec_air = rx_vec_air(:).';

    % Disable the Tx/Rx radios and buffers
    wl_basebandCmd(node_tx, TX_RF_ALL, 'tx_rx_buff_dis');
    wl_basebandCmd(node_rx, RX_RF_ALL, 'tx_rx_buff_dis');
    
    wl_interfaceCmd(node_tx, TX_RF_ALL, 'tx_rx_dis');
    wl_interfaceCmd(node_rx, RX_RF_ALL, 'tx_rx_dis');
else
    % Sim-only mode: Apply wireless degradations here for sim (noise, fading, etc)

    % Perfect (ie. Rx=Tx):
    % rx_vec_air = tx_vec_air;

    % AWGN:
    rx_vec_air = [tx_vec_air, zeros(1,ceil((TRIGGER_OFFSET_TOL_NS*1e-9) / (1/SAMP_FREQ)))];
    rx_vec_air = rx_vec_air + 0*complex(randn(1,length(rx_vec_air)), randn(1,length(rx_vec_air)));

    % CFO:
    % rx_vec_air = tx_vec_air .* exp(-1i*2*pi*1e-4*[0:length(tx_vec_air)-1]);
end

%% Decimate
if( INTERP_RATE ~= 2)
   fprintf('Error: INTERP_RATE must equal 2\n'); 
   return;
end

raw_rx_dec = filter(interp_filt2, 1, rx_vec_air);
raw_rx_dec = raw_rx_dec(1:2:end);
% node monitor FFT
% M = 8192*8*4;
% N =512;% 
% t_vec = (0:(NUM_SAMPLES-1))/(20e6);
% 
%     rx_IQ_decimate = rx_IQ(1:floor(NUM_SAMPLES/10000):end);
%     t_vec_decimate = t_vec(1:floor(NUM_SAMPLES/10000):end);
% 
%     figure(8);clf;
%     ax(1) = subplot(2,1,1);
%     plot(t_vec_decimate, real(rx_IQ_decimate),'b-')
%     xlabel('Time (seconds)')
%     title('I Received Waveform')
% 
%     ax(2) = subplot(2,1,2);
%     plot(t_vec_decimate, imag(rx_IQ_decimate),'b-')
%     xlabel('Time (seconds)')
%     title('Q Received Waveform')

%     % Now we can reshape the received vector into a matrix of M rows and N columns;
%     % Reshape the long vector of M*N to a matrix so we can take M FFTs of length N
% 
      
%     rx_IQ_slice    = rx_IQ(1:(M*N));
%     rx_IQ_mat      = reshape(rx_IQ_slice, M, N).';
%     rx_spectrogram = fft(rx_IQ_mat, N, 2);
% 
%     % Zero out any DC offset
%     rx_spectrogram(:,1) = zeros(N,1);
% 
% %     % Perform an fftshift so that the DC frequency bin is in the middle
%     rx_spectrogram = fftshift(rx_spectrogram,2);
%     [M,I]=max(max(abs(rx_spectrogram)));
%    t_fft=toc;
   % Plot the Spectrogram on a dB scale
%     
%     h = figure('units','pixels','Position',[100 100 2000 1000]);clf;
%     set(h,'PaperPosition',[.25 .25 20 10]);
%     figure(10)
%     %Plot the entire view
%     x_axis = linspace(-10,10,N);
%     y_axis = (0:(M-1)) / (20e6 / N);
%     imagesc(x_axis,y_axis,20*log10(abs(rx_spectrogram)./max(abs(rx_spectrogram(:)))));
%     caxis([-50 0])
%     colorbar
%     axis square
% 
%     xlabel('Frequency (MHz)')
%     ylabel('Time (s)')
%     title(sprintf('Spectrogram on dB Scale (%1.4f second view)', max(t_vec)))
%% Correlate for LTS
% lts_f1 = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
% lts_t1 = ifft(lts_f1, 64);
% Complex cross correlation of Rx waveform with time-domain LTS
lts_corr = abs(conv(conj(fliplr(lts_t)), sign(raw_rx_dec)));

% Skip early and late samples - avoids occasional false positives from pre-AGC samples
lts_corr = lts_corr(32:end-32);

% Find all correlation peaks
lts_peaks = find(lts_corr(1:800) > LTS_CORR_THRESH*max(lts_corr));

% Select best candidate correlation peak as LTS-payload boundary
[LTS1, LTS2] = meshgrid(lts_peaks,lts_peaks);
[lts_second_peak_index,y] = find(LTS2-LTS1 == length(lts_t));

% Stop if no valid correlation peak was found
if(isempty(lts_second_peak_index))
    fprintf('No LTS Correlation Peaks Found!\n');
    return;
end

% Set the sample indices of the payload symbols and preamble
% The "+32" corresponds to the 32-sample cyclic prefix on the preamble LTS
% The "-160" corresponds to the length of the preamble LTS (2.5 copies of 64-sample LTS)
payload_ind = lts_peaks(max(lts_second_peak_index)) + 32;
lts_ind = payload_ind-160;

if(DO_APPLY_CFO_CORRECTION)
    %Extract LTS (not yet CFO corrected)
    rx_lts = raw_rx_dec(lts_ind : lts_ind+159);
    rx_lts1 = rx_lts(-64+-FFT_OFFSET + [97:160]);
    rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);

    %Calculate coarse CFO est
    rx_cfo_est_lts = mean(unwrap(angle(rx_lts2 .* conj(rx_lts1))));
    rx_cfo_est_lts = rx_cfo_est_lts/(2*pi*64);
else
    rx_cfo_est_lts = 0;
end
decode_lts=fft(rx_lts);
% Apply CFO correction to raw Rx waveform
rx_cfo_corr_t = exp(-1i*2*pi*rx_cfo_est_lts*[0:length(raw_rx_dec)-1]);
rx_dec_cfo_corr = raw_rx_dec .* rx_cfo_corr_t;

% Re-extract LTS for channel estimate
rx_lts = rx_dec_cfo_corr(lts_ind : lts_ind+159);
rx_lts1 = rx_lts(-64+-FFT_OFFSET + [97:160]);
rx_lts2 = rx_lts(-FFT_OFFSET + [97:160]);

rx_lts1_f = fft(rx_lts1);
rx_lts2_f = fft(rx_lts2);
% rx_lts_demod=rx_lts2_f;
% threshold_real=mean(abs(real(rx_lts_demod)));
% threshold_imag=mean(abs(imag(rx_lts_demod)));
% loc_N1=find(real(rx_lts2_f)<-threshold_real | imag(rx_lts2_f)<-threshold_imag);
% loc_0=find(abs(rx_lts2_f)<0.5*threshold_real);
% loc_1=find(real(rx_lts2_f)>threshold_real | imag(rx_lts2_f)>threshold_imag);
% rx_lts_demod(loc_N1)=-1;
% rx_lts_demod(loc_0)= 0;
% rx_lts_demod(loc_1)= 1;
% rx_lts_demod(27:39)=[];
% rx_lts_zero=find(rx_lts_demod(1,2:51)==0);
% symbol_zero_start=64-(51-rx_lts_zero(1));
% if CHANNEL==6
%     if (38 <=symbol_zero_start&& symbol_zero_start<=41)
%         rx_mode=2;
           receive_sym_zero=[25 26 27 28 29 30 31 32 33];
%     elseif(53 <symbol_zero_start&& symbol_zero_start<57)
%         rx_mode=1;
%     elseif(47 <symbol_zero_start&& symbol_zero_start<51)
%         rx_mode=3;
%     elseif(21 <symbol_zero_start&& symbol_zero_start<24)
%         rx_mode=4;
%     elseif(2 <symbol_zero_start&& symbol_zero_start<4)
%         rx_mode=5;    
%     end
% end
% Calculate channel estimate from average of 2 training symbols

%lts_f = [0 1 -1 -1 1 1 -1 1 -1 1 -1 -1 -1 -1 -1 1 1 -1 -1 1 -1 1 -1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1];
rx_H_est = lts_f .* (rx_lts1_f + rx_lts2_f)/2;

%% Rx payload processing

% Extract the payload samples (integral number of OFDM symbols following preamble)
payload_vec = rx_dec_cfo_corr(payload_ind : payload_ind+N_OFDM_SYMS*(N_SC+CP_LEN)-1);
payload_mat = reshape(payload_vec, (N_SC+CP_LEN), N_OFDM_SYMS);

% Remove the cyclic prefix, keeping FFT_OFFSET samples of CP (on average)
payload_mat_noCP = payload_mat(CP_LEN-FFT_OFFSET+[1:N_SC], :);

% Take the FFT
syms_f_mat = fft(payload_mat_noCP, N_SC, 1);

% Equalize (zero-forcing, just divide by complex chan estimates)
syms_eq_mat = syms_f_mat./ repmat(rx_H_est.', 1, N_OFDM_SYMS);

if DO_APPLY_SFO_CORRECTION
    % SFO manifests as a frequency-dependent phase whose slope increases
    % over time as the Tx and Rx sample streams drift apart from one
    % another. To correct for this effect, we calculate this phase slope at
    % each OFDM symbol using the pilot tones and use this slope to
    % interpolate a phase correction for each data-bearing subcarrier.
    pilotsrx = [1 1 1].';
    % Repeat the pilots across all OFDM symbols
    pilots_matrx = repmat(pilotsrx, 1, N_OFDM_SYMS);
	% Extract the pilot tones and "equalize" them by their nominal Tx values
    pilots_f_mat = syms_eq_mat(SC_IND_PILOTSrx, :);
    pilots_f_mat_comp = pilots_f_mat.*pilots_matrx;

	% Calculate the phases of every Rx pilot tone
    pilot_phases = unwrap(angle(fftshift(pilots_f_mat_comp,1)), [], 1);

	% Calculate slope of pilot tone phases vs frequency in each OFDM symbol
    pilot_spacing_mat = repmat(mod(diff(fftshift(SC_IND_PILOTSrx)),64).', 1, N_OFDM_SYMS);                        
	pilot_slope_mat = mean(diff(pilot_phases) ./ pilot_spacing_mat);

	% Calculate the SFO correction phases for each OFDM symbol
    pilot_phase_sfo_corr = fftshift((-32:31).' * pilot_slope_mat, 1);
    pilot_phase_corr = exp(-1i*(pilot_phase_sfo_corr));

    % Apply the pilot phase correction per symbol
    syms_eq_mat = syms_eq_mat .* pilot_phase_corr;
else
	% Define an empty SFO correction matrix (used by plotting code below)
    pilot_phase_sfo_corr = zeros(N_SC, N_OFDM_SYMS);
end


if DO_APPLY_PHASE_ERR_CORRECTION
    % Extract the pilots and calculate per-symbol phase error
    pilots_f_mat = syms_eq_mat(SC_IND_PILOTSrx, :);
    pilots_f_mat_comp = pilots_f_mat.*pilots_matrx;
    pilot_phase_err = angle(mean(pilots_f_mat_comp));
else
	% Define an empty phase correction vector (used by plotting code below)
    pilot_phase_err = zeros(1, N_OFDM_SYMS);
end
pilot_phase_err_corr = repmat(pilot_phase_err, N_SC, 1);
pilot_phase_corr = exp(-1i*(pilot_phase_err_corr));

% Apply the pilot phase correction per symbol
syms_eq_pc_mat = syms_eq_mat .* pilot_phase_corr;
payload_syms_mat = syms_eq_pc_mat(SC_IND_DATA, :);
%Tx and Rx Payload Processing

% payload_syms_mat(receive_sym_zero,3:12:N_OFDM_SYMS)=NaN;
% payload_syms_mat(receive_sym_zero,4:12:N_OFDM_SYMS)=NaN;
% payload_syms_mat(receive_sym_zero,5:12:N_OFDM_SYMS)=NaN;
% payload_syms_mat(receive_sym_zero,7:12:N_OFDM_SYMS)=NaN;

%payload_syms_mat(receive_sym_zero,:)=[];% delete invalid data points
tx_data_del = reshape(tx_data,48,N_OFDM_SYMS);%delete Tx invalid data points
tx_data_del(receive_sym_zero,1:4:N_OFDM_SYMS)=NaN;
tx_data_del(receive_sym_zero,3:4:N_OFDM_SYMS)=NaN;
tx_data_del = reshape(tx_data_del,1,N_OFDM_SYMS*48);%-length(tx_sym)*N_OFDM_SYMS);
invalid_tx=find(isnan(tx_data_del));
tx_data_del(invalid_tx)=[];
tx_syms = reshape(tx_syms,48,N_OFDM_SYMS);%delete Tx invalid data points
tx_syms(receive_sym_zero,1:4:N_OFDM_SYMS)=NaN;
tx_syms(receive_sym_zero,3:4:N_OFDM_SYMS)=NaN;

tx_syms = reshape(tx_syms,1,N_OFDM_SYMS*48);%-length(tx_sym_del)*N_OFDM_SYMS);
invalid_tx_sym=find(isnan(tx_syms));
tx_syms(invalid_tx_sym)=[];
%% Demodulate
demod_fcn_bpsk = @(x) double(real(x)>0);
demod_fcn_qpsk = @(x) double(2*(real(x)>0) + 1*(imag(x)>0));
demod_fcn_16qam = @(x) (8*(real(x)>0)) + (4*(abs(real(x))<0.6325)) + (2*(imag(x)>0)) + (1*(abs(imag(x))<0.6325));
demod_fcn_64qam = @(x) (32*(real(x)>0)) + (16*(abs(real(x))<0.6172)) + (8*((abs(real(x))<(0.9258))&&((abs(real(x))>(0.3086))))) + (4*(imag(x)>0)) + (2*(abs(imag(x))<0.6172)) + (1*((abs(imag(x))<(0.9258))&&((abs(imag(x))>(0.3086)))));

switch(MOD_ORDER)
    case 2         % BPSK
        rx_mode = arrayfun(demod_fcn_bpsk, payload_syms_mat(1,1));
    case 4         % QPSK
        rx_mode = arrayfun(demod_fcn_qpsk, payload_syms_mat(1,1));
    case 16        % 16-QAM
        rx_mode = arrayfun(demod_fcn_16qam, payload_syms_mat(1,1));
    case 64        % 64-QAM
        rx_mode = arrayfun(demod_fcn_64qam, payload_syms_mat(1,1));
end
payload_syms_mat(receive_sym_zero,1:4:N_OFDM_SYMS)=NaN;
payload_syms_mat(receive_sym_zero,3:4:N_OFDM_SYMS)=NaN;
rx_syms = reshape(payload_syms_mat, 1, N_OFDM_SYMS*48);%-length(tx_sym_del)*N_OFDM_SYMS);
% rx_syms(1)=[];
invalid_rx=find(isnan(rx_syms));
rx_syms(invalid_rx)=[];
switch(MOD_ORDER)
    case 2         % BPSK
        rx_data_raw = arrayfun(demod_fcn_bpsk, rx_syms);
    case 4         % QPSK
        rx_data_raw = arrayfun(demod_fcn_qpsk, rx_syms);
    case 16        % 16-QAM
        rx_data_raw = arrayfun(demod_fcn_16qam, rx_syms);
    case 64        % 64-QAM
        rx_data_raw = arrayfun(demod_fcn_64qam, rx_syms);
end

rx_data_raw_re = reshape(de2bi(rx_data_raw),[],1);
% rx_data_decode = vitDecoder(rx_data_raw_re);
% tx_data_erased = reshape(de2bi(tx_data),[],1);
% BERVec = errorCalc(tx_data_erased, rx_data_decode);
% rx_data = bi2de(reshape(rx_data_decode,[],4))';
%% Plot Results
 cf = 0;

% Tx signal
% cf = cf + 1;
% figure(cf); clf;
% 
% subplot(2,1,1);
% plot(real(tx_vec_air), 'b');
% axis([0 length(tx_vec_air) -TX_SCALE TX_SCALE])
% grid on;
% title('Tx Waveform (I)');
% 
% subplot(2,1,2);
% plot(imag(tx_vec_air), 'r');
% axis([0 length(tx_vec_air) -TX_SCALE TX_SCALE])
% grid on;
% title('Tx Waveform (Q)');
% 
% if(WRITE_PNG_FILES)
%     print(gcf,sprintf('wl_ofdm_plots_%s_txIQ', example_mode_string), '-dpng', '-r96', '-painters')
% end

% Rx signal
% cf = cf + 1;
% figure(cf); clf;
% subplot(2,1,1);
% plot(real(rx_vec_air), 'b');
% axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
% grid on;
% title('Rx Waveform (I)');
% 
% subplot(2,1,2);
% plot(imag(rx_vec_air), 'r');
% axis([0 length(rx_vec_air) -TX_SCALE TX_SCALE])
% grid on;
% title('Rx Waveform (Q)');
% 
% if(WRITE_PNG_FILES)
%     print(gcf,sprintf('wl_ofdm_plots_%s_rxIQ', example_mode_string), '-dpng', '-r96', '-painters')
% end

%Rx LTS correlation
% cf = cf + 1;
% figure(cf); clf;
% lts_to_plot = lts_corr;
% plot(lts_to_plot, '.-b', 'LineWidth', 1);
% hold on;
% grid on;
% line([1 length(lts_to_plot)], LTS_CORR_THRESH*max(lts_to_plot)*[1 1], 'LineStyle', '--', 'Color', 'r', 'LineWidth', 2);
% title('LTS Correlation and Threshold')
% xlabel('Sample Index')
% myAxis = axis();
% axis([1, 1000, myAxis(3), myAxis(4)])
% 
% if(WRITE_PNG_FILES)
%     print(gcf,sprintf('wl_ofdm_plots_%s_ltsCorr', example_mode_string), '-dpng', '-r96', '-painters')
% end

% Channel Estimates
% cf = cf + 1;
% 
% rx_H_est_plot = repmat(complex(NaN,NaN),1,length(rx_H_est));
% rx_H_est_plot(SC_IND_DATA) = rx_H_est(SC_IND_DATA);
% rx_H_est_plot(SC_IND_PILOTS) = rx_H_est(SC_IND_PILOTS);
% 
% x = (20/N_SC) * (-(N_SC/2):(N_SC/2 - 1));
% 
% figure(cf); clf;
% subplot(2,1,1);
% stairs(x - (20/(2*N_SC)), fftshift(real(rx_H_est_plot)), 'b', 'LineWidth', 2);
% hold on
% stairs(x - (20/(2*N_SC)), fftshift(imag(rx_H_est_plot)), 'r', 'LineWidth', 2);
% hold off
% axis([min(x) max(x) -1.1*max(abs(rx_H_est_plot)) 1.1*max(abs(rx_H_est_plot))])
% grid on;
% title('Channel Estimates (I and Q)')
% 
% subplot(2,1,2);
% bh = bar(x, fftshift(abs(rx_H_est_plot)),1,'LineWidth', 1);
% shading flat
% set(bh,'FaceColor',[0 0 1])
% axis([min(x) max(x) 0 1.1*max(abs(rx_H_est_plot))])
% grid on;
% title('Channel Estimates (Magnitude)')
% xlabel('Baseband Frequency (MHz)')
% 
% if(WRITE_PNG_FILES)
%     print(gcf,sprintf('wl_ofdm_plots_%s_chanEst', example_mode_string), '-dpng', '-r96', '-painters')
% end

%% Pilot phase error estimate
% cf = cf + 1;
% figure(cf); clf;
% subplot(2,1,1)
% plot(pilot_phase_err, 'b', 'LineWidth', 2);
% title('Phase Error Estimates')
% xlabel('OFDM Symbol Index')
% ylabel('Radians')
% axis([1 N_OFDM_SYMS -3.2 3.2])
% grid on
% 
% h = colorbar;
% set(h,'Visible','off');
% 
% subplot(2,1,2)
% imagesc(1:N_OFDM_SYMS, (SC_IND_DATA - N_SC/2), fftshift(pilot_phase_sfo_corr,1))
% xlabel('OFDM Symbol Index')
% ylabel('Subcarrier Index')
% title('Phase Correction for SFO')
% colorbar
% myAxis = caxis();
% if(myAxis(2)-myAxis(1) < (pi))
%    caxis([-pi/2 pi/2])
% end
% 
% 
% if(WRITE_PNG_FILES)
%     print(gcf,sprintf('wl_ofdm_plots_%s_phaseError', example_mode_string), '-dpng', '-r96', '-painters')
% end

%% Symbol constellation
cf = cf + 1;
figure(cf); clf;
% invalid_payload=find(isnan(payload_syms_mat));
% payload_syms_mat(invalid_payload)=0;
plot(payload_syms_mat(:),'ro','MarkerSize',1);
axis square; axis(1.5*[-1 1 -1 1]);
grid on;
hold on;

plot(tx_syms_mat(:),'bo');
title('Tx and Rx Constellations')
legend('Rx','Tx','Location','EastOutside');

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_ofdm_plots_%s_constellations', example_mode_string), '-dpng', '-r96', '-painters')
end


% EVM & SNR
cf = cf + 1;
figure(cf); clf;
tx_syms_mat(tx_sym_del,:)=[];
payload_syms_mat(receive_sym_zero,:)=[];
evm_mat = abs(payload_syms_mat - tx_syms_mat).^2;
aevms = mean(evm_mat(:));
snr = 10*log10(1./aevms);

subplot(2,1,1)
plot(100*evm_mat(:),'o','MarkerSize',1)
axis tight
hold on
plot([1 length(evm_mat(:))], 100*[aevms, aevms],'r','LineWidth',4)
myAxis = axis;
h = text(round(.05*length(evm_mat(:))), 100*aevms+ .1*(myAxis(4)-myAxis(3)), sprintf('Effective SNR: %.1f dB', snr));
set(h,'Color',[1 0 0])
set(h,'FontWeight','bold')
set(h,'FontSize',10)
set(h,'EdgeColor',[1 0 0])
set(h,'BackgroundColor',[1 1 1])
hold off
xlabel('Data Symbol Index')
ylabel('EVM (%)');
legend('Per-Symbol EVM','Average EVM','Location','NorthWest');
title('EVM vs. Data Symbol Index')
grid on

subplot(2,1,2)
imagesc(1:N_OFDM_SYMS, (SC_IND_DATA - N_SC/2), 100*fftshift(evm_mat,1))

grid on
xlabel('OFDM Symbol Index')
ylabel('Subcarrier Index')
title('EVM vs. (Subcarrier & OFDM Symbol)')
h = colorbar;
set(get(h,'title'),'string','EVM (%)');
myAxis = caxis();
if (myAxis(2)-myAxis(1)) < 5
    caxis([myAxis(1), myAxis(1)+5])
end

if(WRITE_PNG_FILES)
    print(gcf,sprintf('wl_ofdm_plots_%s_evm', example_mode_string), '-dpng', '-r96', '-painters')
end

%% Calculate Rx stats
% 
sym_errs = sum(rx_data_raw ~= tx_data_del);
bit_errs = length(find(dec2bin(bitxor(rx_data_raw, tx_data_del),8) == '1'));
%rx_evm   = sqrt(sum((real(rx_syms) - real(tx_syms)).^2 + (imag(rx_syms) - imag(tx_syms)).^2)/((length(SC_IND_DATA)-length(tx_sym_del)) * N_OFDM_SYMS));

fprintf('\nResults:\n');
fprintf('Num Bytes:   %d\n', length(tx_data_del)*conv_order * log2(MOD_ORDER) / 8);
fprintf('Sym Errors:  %d (of %d total symbols)\n', sym_errs, length(tx_data_del)*conv_order);
fprintf('Bit Errors:  %d (of %d total bits)\n', bit_errs, length(tx_data_del)*conv_order * log2(MOD_ORDER));
fprintf('Thoughput:   %d\n',(length(tx_data_del)*conv_order * log2(MOD_ORDER)-bit_errs)/0.017);
cfo_est_lts = rx_cfo_est_lts*(SAMP_FREQ/INTERP_RATE);
cfo_est_phaseErr = mean(diff(unwrap(pilot_phase_err)))/(4e-6*2*pi);
cfo_total_ppm = ((cfo_est_lts + cfo_est_phaseErr) /  ((2.412+(.005*(CHANNEL-1)))*1e9)) * 1e6;

fprintf('CFO Est:     %3.2f kHz (%3.2f ppm)\n', (cfo_est_lts + cfo_est_phaseErr)*1e-3, cfo_total_ppm);
fprintf('     LTS CFO Est:                  %3.2f kHz\n', cfo_est_lts*1e-3);
fprintf('     Phase Error Residual CFO Est: %3.2f kHz\n', cfo_est_phaseErr*1e-3);

if DO_APPLY_SFO_CORRECTION
    drift_sec = pilot_slope_mat / (2*pi*312500);
    sfo_est_ppm =  1e6*mean((diff(drift_sec) / 4e-6));
    sfo_est = sfo_est_ppm*20;
    fprintf('SFO Est:     %3.2f Hz (%3.2f ppm)\n', sfo_est, sfo_est_ppm);

end



