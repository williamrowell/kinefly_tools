function Encode_protocol( argument )
%ENCODE_PROTOCOL Encodes an array of integers into an analog signal.
%    argument is an array of integers.

SAMPLE_RATE=1000;   %assumes 1000hz analog output

% set up analog output object
if ~exist ('AO','var')
    AO = analogoutput('mcc',0);
    ch = addchannel(AO, [0 1],{'Stimulus','TAG'});
end
% set sample rate
AO.SampleRate = SAMPLE_RATE;
if exist ('outsamp', 'var')
    clear outsamp
end

% create output array
outsamp = zeros(1,length(argument)*100+100);
for i = 1:length(argument)
    outsamp((i-1)*100+1:i*100) = argument(i)/10;
end

putdata(AO,[outsamp',outsamp']);
start(AO)

end