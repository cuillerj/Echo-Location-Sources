function [hours, mins, secs] = ToolSec2hms(t)
    hours = floor(t / 3600);
    t = t - hours * 3600;
    mins = floor(t / 60);
    secs = t - mins * 60;
end