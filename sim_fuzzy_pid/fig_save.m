function fig_save(f, name, caption)
%FIG_SAVE  Save figure as 300-dpi PNG and vector PDF in results/figures and
%   append its caption (must contain "Simulated"/"Simulation") to captions.md.
assert(contains(caption,'imulat'), 'Caption must state that the figure is simulated.');
d = fullfile(fileparts(mfilename('fullpath')),'results','figures');
if ~exist(d,'dir'), mkdir(d); end
exportgraphics(f, fullfile(d,[name '.png']), 'Resolution',300);
if isempty(findobj(f,'Type','surface'))
    exportgraphics(f, fullfile(d,[name '.pdf']), 'ContentType','vector');
else   % 3-D surfaces: embedded high-resolution image keeps the PDF small
    exportgraphics(f, fullfile(d,[name '.pdf']), 'ContentType','image','Resolution',300);
end
% replace an existing entry of the same figure instead of appending a duplicate
cf = fullfile(d,'captions.md'); L = {};
if exist(cf,'file')
    L = splitlines(string(fileread(cf))); L = cellstr(L(strlength(L) > 0));
end
L = L(~startsWith(L, ['- **' name '**']));
L{end+1} = sprintf('- **%s**: %s', name, caption);
fid = fopen(cf,'w'); fprintf(fid,'%s\n', L{:}); fclose(fid);
close(f);
end
