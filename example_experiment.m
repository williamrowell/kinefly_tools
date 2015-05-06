%% load or create protocol array, last protocol should be 'rest' with no stim

% array of structs of experimental protocols
protocol(1).pattern_id = 3;
protocol(1).mode_array = [0, 0];
protocol(1).gain_bias_array = [-32 0 0 0];
protocol(1).stimulus_array = [0.1 0.2 5 5 1 4.2];
protocol(1).pre_delay = 2;
protocol(1).post_delay = 5;

protocol(2).pattern_id = 2;
protocol(2).mode_array = [0, 0];
protocol(2).gain_bias_array = [-32 0 0 0];
protocol(2).stimulus_array = [0.1 0.2 5 5 1 4.2];
protocol(2).pre_delay = 2;
protocol(2).post_delay = 5;

protocol(3).pattern_id = 1;
protocol(3).mode_array = [1, 0];
protocol(3).gain_bias_array = [-40 0 0 0];
protocol(3).stimulus_array = [0 0 0 2.5 1 0];
protocol(3).pre_delay = 1;
protocol(3).post_delay = 2.5;
% last protocol will be rest interval with no stimulus

% instead of typing all protocols here, let's create a .mat with all of them
% and load it
% load('protocol_list.mat');

%% establish the sequence of protocols to be used in the experiment

% number of replicates of each protocol
num_replicates = 5;
protocol_type = 'random';

switch protocol_type
    case 'collated'
        % ABCABCABC
        protocol_order = repmat(1:length(protocol)-1, [1, num_replicates]);
    case 'grouped'
        % AAABBBCCC
        protocol_order = reshape(repmat(1:length(protocol)-1, [num_replicates, 1]), [1, (length(protocol)-1)*num_replicates]);
    case 'random'
        % random sequence containing each protocol num_replicates times each
        protocol_index = repmat(1:length(protocol)-1, [1, num_replicates]);
        protocol_order = protocol_index(randperm(length(protocol_index)));
end

%% run protocol

% save random_order and protocol struct to file with datestamp (and maybe driver name)
now_str = datestr(now,30);
save([now_str '.mat'], 'protocol', 'protocol_order', 'num_replicates', 'protocol_type');

for prot = protocol_order
    % run trial
    Interval( protocol(prot).pattern_id,...
        protocol(prot).mode_array,...
        protocol(prot).gain_bias_array,...
        protocol(prot).stimulus_array,...
        protocol(prot).pre_delay,...
        protocol(prot).post_delay )
    % rest interval
    Interval( protocol(end).pattern_id,...
        protocol(end).mode_array,...
        protocol(end).gain_bias_array,...
        protocol(end).stimulus_array,...
        protocol(end).pre_delay,...
        protocol(end).post_delay )
end

% output the random order of protocols as an analog signal on AO
pause(5);
Encode_protocol(protocol_order);
