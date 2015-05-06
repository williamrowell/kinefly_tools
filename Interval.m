function Interval( pattern_id, mode_array, gain_bias_array, stimulus_array, pre_delay, post_delay )
%INTERVAL One trial or rest interval.
%   pattern_id         integer between 0 and 255
%   mode_array         [x y] 0 = open loop, 1 = closed loop
%   gain_bias_array    [x_gain x_bias y_gain y_bias]
%   stimulus_array     [pulse_length pulse_period num_pulses trial_length num_trials voltage]
%   pre_delay          time in s to display pattern before stimulus begins
%   post_delay         time in s to display pattern after stimulus begins

% set pattern
Panel_com('set_pattern_id', pattern_id);

% set gain/bias
% gain_bias_array = [x_gain x_bias y_gain y_bias]
Panel_com('send_gain_bias', gain_bias_array);

% set open vs closed loop mode
Panel_com('set_mode', mode_array);

% start pattern
Panel_com('start');

% pre-stim delay
pause(pre_delay);

% stimulus
% Stimulus( pulse_length, pulse_period, num_pulses, trial_length, num_trials, voltage )
Stimulus(stimulus_array);

% post-stim delay
pause(post_delay);

% stop pattern
Panel_com('stop');

end

