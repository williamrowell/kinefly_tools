function kinefly_experiment( protocol_name, protocol_type, num_replicates, driver )
%KINEFLY_EXPERIMENT Summary of this function goes here
%   Detailed explanation goes here
% instead of typing all protocols here, let's create a .mat with all of them
% and load it

load(protocol_name);

%% establish the sequence of protocols to be used in the experiment

% number of replicates of each protocol
%num_replicates = 10;
%protocol_type = 'collated';

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
save([now_str '_' driver '.mat'], 'protocol', 'protocol_order', 'num_replicates', 'protocol_type');

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

end

