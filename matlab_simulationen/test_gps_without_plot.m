close all; clear; clc;
delete(serialportfind);

s = serialport("/dev/cu.usbserial-110", 115200);
configureTerminator(s, "LF");
flush(s);

fprintf("Warte auf GPS-Daten...\n");
fprintf("%-6s  %-10s  %-12s  %-10s  %-8s  %-6s  %-8s\n", ...
    "Fix", "Lat", "Lon", "Alt (m)", "km/h", "Sats", "COG °");
fprintf("%s\n", repmat('-', 1, 65));

while true
    line = readline(s);
    if contains(line, "i2c")
        fprintf("%-6s  %-10s  %-12s  %-10s  %-8s  %-6s  %-8s\n", ...
            extractField(line, "fix"), ...
            extractField(line, "lat"), ...
            extractField(line, "lon"), ...
            extractField(line, "alt"), ...
            extractField(line, "speed"), ...
            extractField(line, "sats"), ...
            extractField(line, "cog"));
    end
end

function val = extractField(line, field)
    pat = field + ": " + wildcardPattern + asManyOfPattern(digitsPattern | "." | "-", 1);
    tok = extract(line, pat);
    if isempty(tok)
        val = "-";
    else
        val = extractAfter(tok{1}, ": ");
    end
end
