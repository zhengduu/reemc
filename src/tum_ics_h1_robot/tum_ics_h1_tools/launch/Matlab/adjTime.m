function[out] = adjTime(in)
% Takes a cell array of structs with a 't' field in 'uint64'.
% Theses structs have been imported from ros bag files
% and the time of these structs need to be set to a common
% reference time. The time is converted from 'uint64' to
% 'double' and inserted to the output structs. The time
% 't' in the output struct is in seconds.

% TUM ICS
% Emmanuel Dean dean@tum.de
% Florian Bergner florian.bergner@tum.de

if ~iscell(in)
    error('input is not a cell array')
end

for k=1:length(in)
    if ~isfield(in{k},'t')
        error('struct at %d does not contain field ''t''',k);
    end
    
    if ~isa(in{k}.t(1),'uint64')
        error('struct at %d: ''t'' is not of type ''uint64''',k);
    end
end


% detect start time/end time
t_start = 0;
t_min = intmax('uint64');
t_end = intmax('uint64');
for k=1:length(in)
    t = in{k}.t;
    
    if(t(1) > t_start)
        t_start = t(1);
    end
    
    if(t(1) < t_min)
        t_min = t(1);
    end
    
    if(t(end) < t_end)
        t_end = t(end);
    end
end
 

t_start = t_start-t_min;
t_end = t_end-t_min;

for k=1:length(in)
    in{k}.t = in{k}.t-t_min;
    in{k}.t = double(in{k}.t);
%     in{k}.t = in{k}.t - double(t_start);
    in{k}.t = in{k}.t * 1e-9;
end

out = in;
end
