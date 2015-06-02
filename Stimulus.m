function Stimulus( argument )
%STIMULUS Sends a pulse over AO to control red LED.
%   pulse_length    width of one light pulse in s
%   pulse_period    width of one pulse period in s
%   num_pulses      number of pulses per trial
%   trial_length    total duration of one trial in s
%   num_trials      number of replicates
%   voltage         output voltage, must be in range 0 <= v <= ~4.2

SAMPLE_RATE=1000;   %assumes 1000hz analog output

pulse_length = argument(1);
pulse_period = argument(2);
num_pulses = argument(3);
trial_length = argument(4);
num_trials = argument(5);
voltage = argument(6);

% Error checking to make sure we can use products of input values as indices
if abs(round(pulse_length*SAMPLE_RATE)-...
        pulse_length*SAMPLE_RATE) > eps('double')
    error('The product of pulse_length and sample_rate must be an integer.')
elseif abs(round((pulse_period-pulse_length)*SAMPLE_RATE)-...
            (pulse_period-pulse_length)*SAMPLE_RATE) > eps('double')
    error('The product of pulse_period-pulse_length and sample rate must be an integer.')
elseif abs(round((trial_length-(pulse_period*num_pulses))*SAMPLE_RATE)-...
            (trial_length-(pulse_period*num_pulses))*SAMPLE_RATE) > eps('double')
    error('The product of trial_length-(pulse_period*num_pulses) and sample_rate must be an integer.')
end

% set up analog output object
if ~exist ('AO','var')
    AO = analogoutput('mcc',0);
    ch = addchannel(AO, [0 1],{'Stimulus','TAG'});
else
    stop(AO)
end
% set sample rate
AO.SampleRate = SAMPLE_RATE;
if exist ('outsamp', 'var')
    clear outsamp
end

% create high frequency pulse
pulse = [ones(1,pulse_length*SAMPLE_RATE) zeros(1,(pulse_period-pulse_length)*SAMPLE_RATE)]*voltage;
pulses = repmat(pulse,[1,num_pulses]);

% create one trial
trial = [pulses zeros(1,(trial_length-(pulse_period*num_pulses))*SAMPLE_RATE)];

% create all trials
outsamp = repmat(trial,[1,num_trials]);

putdata(AO,[outsamp',outsamp']);
start(AO)

end
