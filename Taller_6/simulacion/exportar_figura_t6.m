function exportar_figura_t6(fig, nombre)
% EXPORTAR_FIGURA_T6  Guarda la figura en ../informe/figuras/<nombre>.pdf
aqui = fileparts(mfilename('fullpath'));
destino = fullfile(aqui, '..', 'informe', 'figuras');
if ~exist(destino, 'dir'), mkdir(destino); end
archivo = fullfile(destino, [nombre '.pdf']);
try
    exportgraphics(fig, archivo, 'ContentType', 'vector');
catch
    set(fig, 'PaperUnits', 'centimeters', 'PaperSize', [32 16.5], ...
        'PaperPosition', [0 0 32 16.5]);
    print(fig, archivo, '-dpdf');
end
fprintf('  figura: %s\n', archivo);
end
