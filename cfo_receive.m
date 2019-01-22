%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wl_example_siso_spectrogram.m
%
% In this example, we will use a single WARPLab receiver to plot a
% spectrogram of a received waveform.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
    clear all
    close all
    count=0;
    count1=0;
    count2=0;
    count3=0;
    rx_cfo_est_ltsSTS=zeros(9600,1);
    maxProb=zeros(9600,1);
    Decision=zeros(9600,1);
    FinalZigChan=zeros(100,1);
 for n=1:1
    tic 
    CHANNEL         = 6;         % Choose the channel we will receive on
    
    NUM_SAMPLES     = 2^23;      % Number of samples to request

    WRITE_PNG_FILES = 1;         % Enable writing plots to PNG

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%4%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Set up the WARPLab experiment
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Create a node object
    node = wl_initNodes(1);
    %node=nodes(3);
    % Read Trigger IDs into workspace
    trig_in_ids  = wl_getTriggerInputIDs(node);
    trig_out_ids = wl_getTriggerOutputIDs(node);

    % For both nodes, we will allow Ethernet to trigger the buffer baseband and the AGC
    wl_triggerManagerCmd(node, 'output_config_input_selection', [trig_out_ids.BASEBAND, trig_out_ids.AGC], [trig_in_ids.ETH_A]);

    % Set the trigger output delays. 
    %
    % NOTE:  We are waiting 3000 ns before starting the AGC so that there is time for the inputs 
    %   to settle before sampling the waveform to calculate the RX gains.
    %
    node.wl_triggerManagerCmd('output_config_delay', [trig_out_ids.BASEBAND], 0);
    node.wl_triggerManagerCmd('output_config_delay', [trig_out_ids.AGC], 3000);     % 3000 ns delay before starting the AGC

    % Get IDs for the interfaces on the board.
    ifc_ids = wl_getInterfaceIDs(node);

    % Use RFA as the receiver
    RF_RX     = ifc_ids.RF_A;
    RF_RX_VEC = ifc_ids.RF_A;

    % Check the number of samples
    max_rx_samples = wl_basebandCmd(node, RF_RX, 'rx_buff_max_num_samples');

    if ( strcmp( class(node.transport), 'wl_transport_eth_udp_mex' ) )
        if ( NUM_SAMPLES > node.baseband.MEX_TRANSPORT_MAX_IQ )
            fprintf('\nWARNING:  Requested %d samples.  Due to Matlab memory limitations, the mex transport only supports %d samples.', NUM_SAMPLES, node.baseband.MEX_TRANSPORT_MAX_IQ);
            fprintf('WARNING:  If your computer has enough physical memory, you can adjust this limit using node.baseband.MEX_TRANSPORT_MAX_IQ ');
            fprintf('WARNING:  up to a maximum of %d samples.\n\n', max_rx_samples);

            NUM_SAMPLES = node.baseband.MEX_TRANSPORT_MAX_IQ;
        end
    else 
        if ( NUM_SAMPLES > node.baseband.JAVA_TRANSPORT_MAX_IQ )
            fprintf('\nWARNING:  WARPLab by default only supports %d samples for the spectrogram\n', node.baseband.JAVA_TRANSPORT_MAX_IQ);
            fprintf('WARNING:  using the java transport.  Please use the MEX transport to increase the\n');
            fprintf('WARNING:  number of samples in the spectrogram.\n\n');

            NUM_SAMPLES = node.baseband.JAVA_TRANSPORT_MAX_IQ;
        end
    end

    if ( NUM_SAMPLES > max_rx_samples ) 
        fprintf('\nWARNING:  Requested %d samples.  Node only supports %d samples.\n\n', NUM_SAMPLES, max_rx_samples);

        NUM_SAMPLES = max_rx_samples;    
    end

    % Get the sample rate of the node
    Ts = 1/(wl_basebandCmd(node, 'tx_buff_clk_freq'));

    % Print information to the console
    %fprintf('WARPLab Spectrogram Example:\n');
    %fprintf('    Generating spectrogram using %.4f seconds of data (%d samples).\n', (NUM_SAMPLES * Ts), NUM_SAMPLES );

    % Create a UDP broadcast trigger and tell each node to be ready for it
    eth_trig = wl_trigger_eth_udp_broadcast;
    wl_triggerManagerCmd(node, 'add_ethernet_trigger', [eth_trig]);

    % Set up the interface for the experiment
    wl_interfaceCmd(node, ifc_ids.RF_ALL, 'channel', 2.4, CHANNEL);

    % Set the gains manually
    wl_interfaceCmd(node, ifc_ids.RF_ALL, 'rx_gain_mode', 'manual');
    RxGainRF = 3;                % Rx RF Gain in [1:3]
    RxGainBB = 10;               % Rx Baseband Gain in [0:31]
    wl_interfaceCmd(node, ifc_ids.RF_ALL, 'rx_gains', RxGainRF, RxGainBB);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Receive signal using WARPLab
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Set the receive length to the number of samples
    wl_basebandCmd(node, 'rx_length', NUM_SAMPLES);

    % Open up the transceivITer's low-pass filter to its maximum bandwidth (36MHz)
    wl_interfaceCmd(node, RF_RX, 'rx_lpf_corn_freq', 3);

    % Enable to node to receive data 
    wl_interfaceCmd(node, RF_RX, 'rx_en');
    wl_basebandCmd(node, RF_RX, 'rx_buff_en');

    % Trigger the node to receive samples
    eth_trig.send();

    % Read the samples from the node
    rx_IQ = wl_basebandCmd(node, RF_RX_VEC, 'read_IQ', 0, NUM_SAMPLES);

    % Disable the RX buffers
    wl_basebandCmd(node, ifc_ids.RF_ALL, 'tx_rx_buff_dis');
    wl_interfaceCmd(node, ifc_ids.RF_ALL, 'tx_rx_dis');
    %%
    close all
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualize results
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Figure 1: Time Series
    t_vec = (0:(NUM_SAMPLES-1))/(40e6);

    % MATLAB can really slow down when trying to plot a lot of samples. There
    % isn't much point to plotting a vector that is much longer than the number
    % of pixels in the plot, so we will decimate the vector down to ~10000 samples.

    rx_IQ_decimate = rx_IQ(1:floor(NUM_SAMPLES/10000):end);
    t_vec_decimate = t_vec(1:floor(NUM_SAMPLES/10000):end);

%     figure(1);clf;
%     ax(1) = subplot(2,1,1);
%     plot(t_vec_decimate, real(rx_IQ_decimate),'b-')
%     xlabel('Time (seconds)')
%     title('I Received Waveform')
% 
%     ax(2) = subplot(2,1,2);
%     plot(t_vec_decimate, imag(rx_IQ_decimate),'b-')
%     xlabel('Time (seconds)')
%     title('Q Received Waveform')

%     linkaxes(ax,'xy')
%     axis tight
% 
%     if(WRITE_PNG_FILES)
%         print(gcf,'wl_rx_timeseries','-dpng','-r96','-painters')
%     end

    % Figure 2: Spectrogram

    % MATLAB has a sophisticated 'spectrogram' function in the Signal Processing
    % Toolbox. In this example, we'll build our own crude spectrogram by
    % reshaping the received vector into a matrix and taking many FFTs. Note:
    % our approach has zero overlap in the FFTs.

    % Limit under which we will not show a zoomed in portion of the spectrogram.  
     zoom_span_time    = 100/1000;                         % 100 ms 
     ZOOM_SAMPLE_LIMIT = 4 * (zoom_span_time * 40e6);


    % First, we need to split our receive vector into M pieces of N samples
    % each. The relative size of M and N affects whether you want more
    % resolution in time(M) or more resolution in frequency (N). By default, we
    % will make M equal to N, but this is not a fundamental requirement.
% 
    M = 8192*2;
    N = 512;% 
%     % Now we can reshape the received vector into a matrix of M rows and N columns;
%     % Reshape the long vector of M*N to a matrix so we can take M FFTs of length N
% 
    tic  
    rx_IQ_slice    = rx_IQ(1:(M*N));
    rx_IQ_mat      = reshape(rx_IQ_slice, M, N).';
    rx_spectrogram = fft(rx_IQ_mat, N, 2);
    t_fft=toc;
    % Zero out any DC offset
    rx_spectrogram(:,1) = zeros(N,1);

%     % Perform an fftshift so that the DC frequency bin is in the middle
    rx_spectrogram = fftshift(rx_spectrogram,2);
    [M,I]=max(abs(rx_spectrogram));
%     t_fft=toc;
%    % Plot the Spectrogram on a dB scale
%     h = figure('units','pixels','Position',[100 100 2000 1000]);clf;
%     set(h,'PaperPosition',[.25 .25 20 10]);

    %Plot the entire view
%     if ( NUM_SAMPLES >= ZOOM_SAMPLE_LIMIT )
%         subplot(1,2,1)
%     end
% 
%     x_axis = linspace(-20,20,N);
%     y_axis = (0:(M-1)) / (40e6 / N);
%     imagesc(x_axis,y_axis,20*log10(abs(rx_spectrogram1)./max(abs(rx_spectrogram1(:)))));
%     caxis([-50 0])
%     colorbar
%     axis square
% 
%     xlabel('Frequency (MHz)')
%     ylabel('Time (s)')
%     title(sprintf('Spectrogram on dB Scale (%1.4f second view)', max(t_vec)))
%     if ( NUM_SAMPLES >= ZOOM_SAMPLE_LIMIT )
%         % Zoom into a small piece in the middle of the spectrogram
%         subplot(1,2,2)
% 
%         % Let's zoom in on a chunk out of the entire reception
%         zoom_span_index = ceil(zoom_span_time * (40e6 / N));
%         index_range     = floor((M/2)-(zoom_span_index/2)):floor((M/2)+(zoom_span_index/2));
% 
%         y_axis_slice         = y_axis( index_range );
%         rx_spectrogram_slice = rx_spectrogram( index_range, : );
% 
%         imagesc(x_axis, y_axis_slice, 20 * log10(abs(rx_spectrogram_slice)./max(abs(rx_spectrogram(:)))));
%         caxis([-50 0])
%         colorbar
%         axis square
% 
%         xlabel('Frequency (MHz)')
%         ylabel('Time (s)')
%         title(sprintf('Spectrogram on dB Scale (%1.4f second view)', zoom_span_time))
%     end
% 
%     if(WRITE_PNG_FILES)
%         print(gcf,'wl_rx_spectrogram','-dpng','-r200','-painters')
%     end
% %     tic
    zigstart=find(abs(rx_IQ)>1.2);
    zigstart1=find(zigstart>3400000+zigstart(1));
    zigstart2=find(zigstart>6800000+zigstart(1));
    zigstart3=find(zigstart>00000+zigstart(1));
    zigstart4=find(zigstart>14400000+zigstart(1));
    zigstart5=find(zigstart>17200000+zigstart(1));
    zigstart6=find(zigstart>20800000+zigstart(1));
    zigstart7=find(zigstart>24400000+zigstart(1));
    zigstart8=find(zigstart>28000000+zigstart(1));
    zigstart9=find(zigstart>31600000+zigstart(1));
    zigstart10=find(zigstart>35200000+zigstart(1));
    zigstart11=find(zigstart>38800000+zigstart(1));
    det_set=rx_IQ(zigstart(1):zigstart(1)+16500,1);
    det_set(:,2)=rx_IQ(zigstart(zigstart1(1)):zigstart(zigstart1(1))+16500,1);
    det_set(:,3)=rx_IQ(zigstart(zigstart2(1)):zigstart(zigstart2(1))+16500,1);
    det_set(:,4)=rx_IQ(zigstart(zigstart3(1)):zigstart(zigstart3(1))+16500,1);
    det_set(:,5)=rx_IQ(zigstart(zigstart4(1)):zigstart(zigstart4(1))+16500,1);
    det_set(:,6)=rx_IQ(zigstart(zigstart5(1)):zigstart(zigstart5(1))+16500,1);
    det_set(:,7)=rx_IQ(zigstart(zigstart6(1)):zigstart(zigstart6(1))+16500,1);
    det_set(:,8)=rx_IQ(zigstart(zigstart7(1)):zigstart(zigstart7(1))+16500,1);  
    det_set(:,9)=rx_IQ(zigstart(zigstart8(1)):zigstart(zigstart8(1))+16500,1);
    det_set(:,10)=rx_IQ(zigstart(zigstart9(1)):zigstart(zigstart9(1))+16500,1);
    det_set(:,11)=rx_IQ(zigstart(zigstart10(1)):zigstart(zigstart10(1))+16500,1);
    det_set(:,12)=rx_IQ(zigstart(zigstart11(1)):zigstart(zigstart11(1))+16500,1);  
    rx_cfo_est_lts2=0;
    Nwin=100;
    rx_sts=zeros(10000:1);
    a=zeros(10000,1);
    p=zeros(10000,1);
    c=zeros(10000,1);
    tic
     for j=1:12
%         for sample=1:10000
%             temp=zeros(Nwin+1,1);
%             temp2=zeros(Nwin+1,1);
            for i=2:Nwin
                rx_sts = det_set(1:40:6401,j);
                rx_sts1 = rx_sts(i-1);
                rx_sts2 = rx_sts(i+15);
%                 temp(i+1,1) = rx_sts1.*conj(rx_sts2)+temp(i);
%                 temp2(i+1,1) = rx_sts1.*conj(rx_sts1)+temp2(i);
                rx_cfo_est_lts2 = rx_sts1 .* conj(rx_sts2)+rx_cfo_est_lts2;    
            end
%             a(sample) = temp(Nwin+1);
%             p(sample) = temp2(Nwin+1);
%             
%          end
        rx_cfo_est_ltsSTS(j+12*(n-1),1)=angle(rx_cfo_est_lts2)/16;
        Prob(1,1)=normpdf(rx_cfo_est_ltsSTS(j,1),-0.0500,0.0023);
        Prob(1,2)=normpdf(rx_cfo_est_ltsSTS(j,1),-0.0750,0.0089);
        Prob(1,3)=normpdf(rx_cfo_est_ltsSTS(j,1),-0.0969,0.0209);
        Prob(1,4)=normpdf(rx_cfo_est_ltsSTS(j,1),-0.0562,0.0105);
        Prob(1,5)=normpdf(rx_cfo_est_ltsSTS(j,1),-0.0789,0.0023);
        [maxProb(j+12*(n-1),1), Decision(j+12*(n-1),1)]=max(Prob);
        
     end
   t_delay=toc;
   Decide1=length(find(Decision==1));
   Decide2=length(find(Decision==2));
   Decide3=length(find(Decision==3));
   Decide4=length(find(Decision==4));
   Decide5=length(find(Decision==5));
   ZigbeeChanel=[Decide1;Decide2;Decide3;Decide4;Decide5];
   [Maxlength,FinalZigChan(n,1)]=max(ZigbeeChanel);
%     t=toc;
    
 end
