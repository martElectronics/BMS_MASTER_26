#!/usr/bin/env python3
"""
aim_a_excel.py — Batería de una sesión del AiM en un Excel visual (módulos y paralelos).

Uso:
    python aim_a_excel.py                 # abre un selector de archivo
    python aim_a_excel.py sesion.csv      # directo (CSV exportado)
    python aim_a_excel.py sesion.xrk      # mejor: el .xrk de RaceStudio (py -m pip install libxrk)
o doble clic en convertir_excel.bat.

Hace dos pasos:
  1. Rescata las lecturas de los canales empaquetados BMS Pk* (IDs 393-396). Aunque
     RaceStudio no tenga "Stepped Values" e interpole, se queda solo con las filas
     enteras, con el mismo IDmod en todos los canales y con TODOS los valores dentro
     de la celda mín/máx y la T mín/máx que manda el BMS en ese instante (±30 mV, ±1,5 °C).
     -> modulos/<sesion>_pack_rescatado.csv (una fila por segundo; vacío si el dato
        tiene más de 15 s).
  2. Monta el Excel -> modulos/<sesion>_baterias.xlsx:
       Panel      resumen, paralelos más débiles y 4 gráficas
       Mapa       10 módulos x 11 paralelos / 9 NTC con colores
       Tira ...   cada 10 s, diferencia de cada paralelo con la mediana (franjas rojas
                  = se queda atrás) y temperatura de cada NTC
       Datos / Tiempo  las series y la tabla completa
Con el .xrk (C:\\AIM_SPORT\\RaceStudio3\\user\\data\\<fecha>\\) llegan todas las tramas sin
interpolar y cada paralelo se ve cada ~1 s. El CSV exportado viene interpolado a 20 Hz
(Stepped Values no le afecta): el rescate se queda con lo válido, cada ~5-8 s. Los picos rápidos están siempre en la celda mín del BMS.
Solo librería estándar (el .xlsx se escribe a mano): no hay que instalar nada.
"""
import csv
import os
import sys
import zipfile
from xml.sax.saxutils import escape

import aim_a_fila as m
import aim_a_modulos as a


def rescatar(path, salida):
    """Paso 1: CSV del AiM -> CSV por segundo con las lecturas Pk válidas."""
    s = m.Sesion(path)
    N, dt, t = len(s.t), s.dt, s.t
    minv, maxv, mint, maxt = s.canal('BMS MinV'), s.canal('BMS MaxV'), s.canal('BMS MinT'), s.canal('BMS MaxT')
    I = s.canal('BMS PackCurrent')
    gps = s.canal('GPS Speed')
    TOL_V, TOL_T, VENT = 0.03, 1.5, int(1.0 / dt)


    def rango(v, i, fn):
        x = [v[k] for k in range(max(0, i - VENT), min(N, i + VENT + 1)) if m.ok(v[k])]
        return fn(x) if x else float('nan')


    # min/max deslizantes (+-1 s): el BMS y la pagina no estan en el mismo instante
    lo_v = [0] * N; hi_v = [0] * N; lo_t = [0] * N; hi_t = [0] * N
    for i in range(0, N, 5):
        a0, b0 = rango(minv, i, min), rango(maxv, i, max)
        c0, d0 = rango(mint, i, min), rango(maxt, i, max)
        for k in range(i, min(i + 5, N)):
            lo_v[k], hi_v[k], lo_t[k], hi_t[k] = a0, b0, c0, d0

    series = {}
    for nombre, canales, fn in a.TRAMAS:
        cols = [s.canal(c) for c in canales]
        tot = ok = rech = 0
        previo = None
        for i in range(N):
            vals = [a.entero(c[i]) for c in cols]
            if any(v is None for v in vals) or not any(vals):
                continue
            tot += 1
            try:
                md, d = fn(*vals)
            except ValueError:
                continue
            if md >= 10:
                continue
            buenos = []
            valido = True
            for (ti, k), c in d.items():
                x = a.dec_t(c) if ti == 'T' else a.dec_v(c)
                if x is None:
                    continue
                if (md, k) in a.NTC_IGNORADOS and ti == 'T':
                    buenos.append((ti, k, x))
                    continue
                if ti == 'V' and not (lo_v[i] - TOL_V <= x <= hi_v[i] + TOL_V):
                    valido = False
                if ti == 'T' and not (lo_t[i] - TOL_T <= x <= hi_t[i] + TOL_T):
                    valido = False
                buenos.append((ti, k, x))
            if not valido or not buenos:
                rech += 1
                continue
            ok += 1
            for ti, k, x in buenos:
                series.setdefault((md, ti, k), []).append((i, x))
        print(f'{nombre}: {tot} filas enteras, {ok} validas, {rech} rechazadas por rango')

    # Cobertura
    vs = {k: p for k, p in series.items() if k[1] == 'V'}
    ts = {k: p for k, p in series.items() if k[1] == 'T'}
    print(f'paralelos con datos: {len(vs)}/110, NTC: {len(ts)}/90')
    huecos = sorted((t[p[-1][0]] - t[p[0][0]]) / max(1, len(set(i for i, _ in p)) - 1) for p in vs.values())
    print(f'lectura cada: mediana {huecos[len(huecos)//2]:.1f} s, peor {huecos[-1]:.1f} s')

    try:
        idx = [i for i in range(N) if m.ok(I[i]) and abs(I[i]) > 5]
        i0, i1 = idx[0], idx[-1]
        filas = []
        for md in range(10):
            for n in range(1, 12):
                p = vs.get((md, 'V', n))
                if not p:
                    continue
                ini = [x for i, x in p if i < i0 and abs(I[i]) < 1]
                fin = [x for i, x in p if t[i] > t[i1] + 45 and abs(I[i]) < 1]
                xy = [(t[i], I[i], x) for i, x in p if m.ok(I[i])]
                r = a.regresion_ventanas([q[0] for q in xy], [q[1] for q in xy], [q[2] for q in xy], 40.0)
                filas.append(dict(m=md, n=n, ini=a.mediana(ini), fin=a.mediana(fin), ni=len(ini), nf=len(fin),
                                  vmin=min(x for _, x in p), r=-r * 1000 if r is not None else None, nd=len(p)))
        con = [f for f in filas if f['ini'] and f['fin']]
        solo_fin = [f for f in filas if f['fin']]
        print(f'con reposo inicial y final: {len(con)}; con reposo final: {len(solo_fin)}')
        ref_ini = a.mediana([f['ini'] for f in filas if f['ini']])
        for f in filas:
            f['ini2'] = f['ini'] if f['ini'] else ref_ini
            if f['fin']:
                f['ds'] = a.soc_ocv(f['ini2']) - a.soc_ocv(f['fin'])
        ref = a.mediana([f['ds'] for f in filas if 'ds' in f])
        print(f'mediana: ini {ref_ini:.3f} fin {a.mediana([f["fin"] for f in solo_fin]):.3f}  SOC perdido {ref:.1f} %')
        print('Menos capacidad:')
        for f in sorted([f for f in filas if 'ds' in f], key=lambda f: -f['ds'])[:10]:
            print(f"  {a.mod(f['m'])} V{f['n']}: {f['ini2']:.3f}{'' if f['ini'] else '*'} -> {f['fin']:.3f} V  dSOC {f['ds']:.1f} %  rel {ref/f['ds']:.2f}  n={f['ni']},{f['nf']}  vmin {f['vmin']:.3f}  R {f['r'] if f['r'] is None else round(f['r'],2)}")
        print('Mas R (mOhm):')
        rr = [f for f in filas if f['r'] is not None and f['nd'] > 25]
        mr = a.mediana([f['r'] for f in rr])
        for f in sorted(rr, key=lambda f: -f['r'])[:8]:
            print(f"  {a.mod(f['m'])} V{f['n']}: {f['r']:.2f} ({f['r']/mr:.2f}x, n={f['nd']})")
        print(f'  mediana {mr:.2f}, paralelos con R: {len(rr)}')
        print('T max por modulo (con >=3 lecturas de ese valor o superior):')
        for md in range(10):
            best = None
            for k in range(1, 10):
                if (md, k) in a.NTC_IGNORADOS:
                    continue
                pts = sorted((x for _, x in ts.get((md, 'T', k), [])), reverse=True)
                if len(pts) >= 3 and (best is None or pts[2] > best[0]):
                    best = (pts[2], k, len(pts))
            print(f'  {a.mod(md)}: {best}')
        print('BMS MaxT real:', m.vmax(maxt))
    
    except Exception as e:
        print('El log no acaba con el coche parado un rato: no se calcula la capacidad de cada paralelo.')
    # Tabla por segundo: cada celda es la ultima lectura VALIDA y su antiguedad maxima se limita a 15 s
    claves = [(md, 'V', n) for md in range(10) for n in range(1, 12)] + [(md, 'T', k) for md in range(10) for k in range(1, 10)]
    por_fila = {}
    for key, p in series.items():
        for i, x in p:
            por_fila.setdefault(i, []).append((key, x))
    os.makedirs(os.path.dirname(salida), exist_ok=True)
    ult, edad = {}, {}
    fmt = lambda x, nd: '' if x is None or not m.ok(x) else f'{x:.{nd}f}'.replace('.', ',')
    llenas = total = 0
    with open(salida, 'w', newline='', encoding='utf-8-sig') as fo:
        w = csv.writer(fo, delimiter=';')
        w.writerow(['Tiempo (s)', 'I pack (A)', 'Velocidad (km/h)', 'MinV BMS', 'MaxV BMS', 'MaxT BMS'] + [f'{a.mod(q[0])}_{q[1]}{q[2]}' for q in claves])
        sig = t[0]
        for i in range(N):
            for key, x in por_fila.get(i, []):
                ult[key], edad[key] = x, t[i]
            if t[i] + 1e-9 >= sig:
                sig += 1.0
                cel = []
                for q in claves:
                    ok_ = q in ult and t[i] - edad[q] <= 15.0
                    total += 1
                    llenas += ok_
                    cel.append(fmt(ult[q], 3 if q[1] == 'V' else 0) if ok_ else '')
                w.writerow([fmt(t[i], 2), fmt(I[i], 1), fmt(gps[i], 1) if gps else '', fmt(minv[i], 3), fmt(maxv[i], 3), fmt(maxt[i], 0)] + cel)
    print(f'tabla: {llenas/total*100:.0f} % de celdas con dato de menos de 15 s -> {salida}')
    return s


def excel(src, titulo, dst):
    """Paso 2: CSV rescatado -> Excel con gráficas."""
    rows = list(csv.reader(open(src, encoding='utf-8-sig'), delimiter=';'))
    cab, D = rows[0], [[(float(x.replace(',', '.')) if x else None) for x in r] for r in rows[1:]]
    col = {n: i for i, n in enumerate(cab)}
    N = len(D)
    mods = [f'M{m:02d}' for m in range(1, 11)]
    PV = [f'{m}_V{j}' for m in mods for j in range(1, 12)]
    PT = [f'{m}_T{j}' for m in mods for j in range(1, 10)]
    EXCL = {'M05_T1'}                       # NTC excluido en el firmware
    g = lambda k, n: D[k][col[n]]
    I = [g(k, 'I pack (A)') for k in range(N)]
    t = [g(k, 'Tiempo (s)') for k in range(N)]
    cargado = [k for k in range(N) if I[k] is not None and I[k] > 20]
    reposo = [k for k in range(N) if I[k] is not None and abs(I[k]) < 1]


    def med(v):
        v = sorted(x for x in v if x is not None)
        return v[len(v) // 2] if v else None


    def serie(n, idx):
        return [g(k, n) for k in idx if g(k, n) is not None]


    def bajo(v):
        v = sorted(v)
        return v[2] if len(v) >= 3 else None          # tercer valor mas bajo: quita lecturas sueltas


    vmin_carga = {p: bajo(serie(p, cargado)) for p in PV}
    v_rep_ini = {p: med(serie(p, [k for k in reposo if cargado and k < cargado[0]])) for p in PV}
    v_rep_fin = {p: med(serie(p, [k for k in reposo if cargado and k > cargado[-1]][-90:])) for p in PV}
    t_max = {}
    for p in PT:
        v = sorted(serie(p, range(N)), reverse=True)
        t_max[p] = v[2] if len(v) >= 3 else (v[0] if v else None)

    # Debiles: los que mas se separan por debajo de la mediana del pack con carga
    ref_c = med(vmin_carga.values())
    con_fin = [p for p in PV if v_rep_fin[p] is not None]
    if len(con_fin) > 80 and cargado and t[-1] - t[cargado[-1]] >= 90:   # reposo final de verdad (>=90 s)
        debiles = sorted(con_fin, key=lambda p: v_rep_fin[p])[:6]
    else:
        debiles = sorted([p for p in PV if vmin_carga[p] is not None], key=lambda p: vmin_carga[p])[:6]

    # --------------------------------------------------------------------------- utilidades xlsx
    def L(c):
        s, c = '', c + 1
        while c:
            c, r = divmod(c - 1, 26)
            s = chr(65 + r) + s
        return s


    def celda(r, c, v, st=0):
        ref = f'{L(c)}{r + 1}'
        s = f' s="{st}"' if st else ''
        if v is None or v == '':
            return f'<c r="{ref}"{s}/>' if st else ''
        if isinstance(v, (int, float)):
            return f'<c r="{ref}"{s}><v>{round(v, 4)}</v></c>'
        return f'<c r="{ref}" t="inlineStr"{s}><is><t xml:space="preserve">{escape(str(v))}</t></is></c>'


    ESC = {
        'Vabs': '<cfvo type="num" val="2.8"/><cfvo type="num" val="3.4"/><cfvo type="num" val="4.1"/><color rgb="FFF8696B"/><color rgb="FFFFEB84"/><color rgb="FF63BE7B"/>',
        'Vrel': '<cfvo type="percentile" val="2"/><cfvo type="percentile" val="50"/><cfvo type="max"/><color rgb="FFF8696B"/><color rgb="FFFFEB84"/><color rgb="FF63BE7B"/>',
        'T': '<cfvo type="num" val="25"/><cfvo type="num" val="40"/><cfvo type="num" val="60"/><color rgb="FF5A8AC6"/><color rgb="FFFFEB84"/><color rgb="FFF8696B"/>',
        'dmV': '<cfvo type="num" val="-150"/><cfvo type="num" val="0"/><cfvo type="num" val="60"/><color rgb="FFC00000"/><color rgb="FFFFFFFF"/><color rgb="FF5A8AC6"/>',
    }


    def hoja(filas, anchos=(), congelar=None, escalas=(), estilos=None, drawing=False, alto=None, zoom=None):
        x = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
             '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
             'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">']
        sv = f' zoomScale="{zoom}"' if zoom else ''
        if congelar:
            c, r = congelar
            x.append(f'<sheetViews><sheetView workbookViewId="0"{sv}><pane xSplit="{c}" ySplit="{r}" topLeftCell="{L(c)}{r + 1}" '
                     'activePane="bottomRight" state="frozen"/></sheetView></sheetViews>')
        else:
            x.append(f'<sheetViews><sheetView workbookViewId="0" showGridLines="{0 if drawing else 1}"{sv}/></sheetViews>')
        if alto:
            x.append(f'<sheetFormatPr defaultRowHeight="{alto}" customHeight="1"/>')
        if anchos:
            x.append('<cols>' + ''.join(f'<col min="{a}" max="{b}" width="{w}" customWidth="1"/>' for a, b, w in anchos) + '</cols>')
        x.append('<sheetData>')
        for r, fila in enumerate(filas):
            est = (estilos or {}).get(r, {})
            x.append(f'<row r="{r + 1}">' + ''.join(celda(r, c, v, est.get(c, est.get('*', 0))) for c, v in enumerate(fila)) + '</row>')
        x.append('</sheetData>')
        for p, (rango, tipo) in enumerate(escalas, 1):
            x.append(f'<conditionalFormatting sqref="{rango}"><cfRule type="colorScale" priority="{p}">'
                     f'<colorScale>{ESC[tipo]}</colorScale></cfRule></conditionalFormatting>')
        if drawing:
            x.append('<drawing r:id="rId1"/>')
        x.append('</worksheet>')
        return '\n'.join(x)


    COLORES = ['C00000', 'FF7F00', '7030A0', '0070C0', '00B050', '808000', '00B0F0', 'FF66CC', '996633', '404040']


    def grafica(tit, series, ytit, ymin=None, ymax=None, xtit='Tiempo (s)'):
        """series: (nombre, ref_x, ref_y, color, grosor_pt, discontinua)"""
        def rich(txt, sz):
            return (f'<c:tx><c:rich><a:bodyPr/><a:p><a:pPr><a:defRPr sz="{sz}" b="1"/></a:pPr><a:r><a:rPr lang="es-ES" sz="{sz}" b="1"/>'
                    f'<a:t>{escape(txt)}</a:t></a:r></a:p></c:rich></c:tx><c:overlay val="0"/>')
        sers = []
        for k, (nom, rx, ry, colr, w, dash) in enumerate(series):
            dsh = '<a:prstDash val="sysDash"/>' if dash else ''
            sers.append(f'<c:ser><c:idx val="{k}"/><c:order val="{k}"/><c:tx><c:v>{escape(nom)}</c:v></c:tx>'
                        f'<c:spPr><a:ln w="{int(w * 12700)}" cap="rnd"><a:solidFill><a:srgbClr val="{colr}"/></a:solidFill>{dsh}</a:ln></c:spPr>'
                        f'<c:marker><c:symbol val="none"/></c:marker>'
                        f'<c:xVal><c:numRef><c:f>{rx}</c:f></c:numRef></c:xVal><c:yVal><c:numRef><c:f>{ry}</c:f></c:numRef></c:yVal>'
                        f'<c:smooth val="0"/></c:ser>')
        sc = '<c:orientation val="minMax"/>' + (f'<c:max val="{ymax}"/>' if ymax is not None else '') + (f'<c:min val="{ymin}"/>' if ymin is not None else '')
        gris = '<c:spPr><a:ln w="6350"><a:solidFill><a:srgbClr val="D9D9D9"/></a:solidFill></a:ln></c:spPr>'
        return ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                '<c:chartSpace xmlns:c="http://schemas.openxmlformats.org/drawingml/2006/chart" '
                'xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" '
                'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
                '<c:roundedCorners val="0"/><c:chart>'
                f'<c:title>{rich(tit, 1300)}</c:title><c:autoTitleDeleted val="0"/><c:plotArea><c:layout/>'
                '<c:scatterChart><c:scatterStyle val="lineMarker"/><c:varyColors val="0"/>' + ''.join(sers) +
                '<c:axId val="1001"/><c:axId val="1002"/></c:scatterChart>'
                '<c:valAx><c:axId val="1001"/><c:scaling><c:orientation val="minMax"/></c:scaling><c:delete val="0"/><c:axPos val="b"/>'
                f'<c:title>{rich(xtit, 900)}</c:title><c:numFmt formatCode="0" sourceLinked="0"/><c:majorTickMark val="out"/><c:minorTickMark val="none"/>'
                '<c:tickLblPos val="low"/><c:crossAx val="1002"/><c:crosses val="autoZero"/><c:crossBetween val="midCat"/></c:valAx>'
                f'<c:valAx><c:axId val="1002"/><c:scaling>{sc}</c:scaling><c:delete val="0"/><c:axPos val="l"/><c:majorGridlines>{gris}</c:majorGridlines>'
                f'<c:title>{rich(ytit, 900)}</c:title><c:numFmt formatCode="General" sourceLinked="0"/><c:majorTickMark val="out"/><c:minorTickMark val="none"/>'
                '<c:tickLblPos val="nextTo"/><c:crossAx val="1001"/><c:crosses val="min"/><c:crossBetween val="midCat"/></c:valAx>'
                '</c:plotArea><c:legend><c:legendPos val="r"/><c:overlay val="0"/></c:legend><c:plotVisOnly val="1"/><c:dispBlanksAs val="span"/>'
                '</c:chart></c:chartSpace>')


    # --------------------------------------------------------------------------- hoja Datos (para las graficas)
    tmod = {m: [max([g(k, f'{m}_T{j}') for j in range(1, 10) if f'{m}_T{j}' not in EXCL and g(k, f'{m}_T{j}') is not None] or [None])
                for k in range(N)] for m in mods}
    dcab = ['Tiempo (s)', 'I pack (A)', 'Velocidad (km/h)', 'Celda mín BMS (V)', 'Celda máx BMS (V)', 'Mediana pack (V)',
            'Desequilibrio BMS (mV)'] + [p.replace('_', ' ') for p in debiles] + [f'T máx {m} (°C)' for m in mods] + ['T máx BMS (°C)']
    datos = [dcab]
    for k in range(N):
        vs = [g(k, p) for p in PV]
        mn, mx = g(k, 'MinV BMS'), g(k, 'MaxV BMS')
        datos.append([t[k], I[k], g(k, 'Velocidad (km/h)'), mn, mx, med(vs),
                      (mx - mn) * 1000 if mn is not None and mx is not None else None] +
                     [g(k, p) for p in debiles] + [tmod[m][k] for m in mods] + [g(k, 'MaxT BMS')])
    R = lambda c: f"Datos!${L(c)}$2:${L(c)}${N + 1}"
    X = R(0)
    nd = len(debiles)
    c_T0 = 7 + nd
    vals_v = [x for r in datos[1:] for x in (r[3], r[4]) if x is not None]
    ymin_v = round(max(2.5, min(vals_v) - 0.05), 1) if vals_v else None
    ymax_v = round(min(4.3, max(vals_v) + 0.05), 1) if vals_v else None
    graficas = [
        grafica('Tensión: celda mínima, máxima y paralelos débiles', [
            ('Celda máx (BMS)', X, R(4), '00B050', 1.5, False),
            ('Mediana pack', X, R(5), '7F7F7F', 1.5, True),
            ('Celda mín (BMS)', X, R(3), '000000', 1.5, False)] +
            [(debiles[i].replace('_', ' '), X, R(7 + i), COLORES[i], 2.25, False) for i in range(nd)],
            'V', ymin_v, ymax_v),
        grafica('Corriente del pack', [('I pack (A)', X, R(1), '0070C0', 1.0, False)], 'A'),
        grafica('Desequilibrio (celda máx - mín)', [('Desequilibrio (mV)', X, R(6), 'C00000', 1.25, False)], 'mV', 0),
        grafica('Temperatura máxima de cada módulo', [(f'{mods[i]}', X, R(c_T0 + i), COLORES[i], 1.5, False) for i in range(10)] +
                [('BMS máx', X, R(c_T0 + 10), '000000', 1.0, True)], '°C'),
    ]

    # --------------------------------------------------------------------------- hoja Panel (texto)
    def f3(v):
        return '-' if v is None else f'{v:.3f}'.replace('.', ',')


    minI = [x for x in I if x is not None]
    mins = [g(k, 'MinV BMS') for k in range(N) if g(k, 'MinV BMS') is not None]
    kmin = min(range(N), key=lambda k: g(k, 'MinV BMS') if g(k, 'MinV BMS') is not None else 9)
    panel = [[f'Baterías — {titulo}'], [],
             ['Celda mínima de la sesión (BMS)', f'{f3(min(mins))} V a los {t[kmin]:.0f} s con {I[kmin] or 0:.0f} A'],
             ['Corriente máxima del pack', f'{max(minI):.0f} A'],
             ['Temperatura máxima de módulo', f"{max(x for x in serie('MaxT BMS', range(N))):.0f} °C"],
             ['Desequilibrio máximo', f"{max(r[6] for r in datos[1:] if r[6] is not None):.0f} mV"],
             [],
             ['Paralelos más débiles', 'mín con carga', 'reposo inicio', 'reposo final', 'vs. mediana con carga']]
    for p in debiles:
        panel.append([p.replace('_', ' '), vmin_carga[p], v_rep_ini[p], v_rep_fin[p],
                      (vmin_carga[p] - ref_c) * 1000 if ref_c else None])
    panel += [[], ['Cómo leerlo'],
              ['· Gráfica de tensión: si un paralelo de color cae por debajo de la línea negra (mín del BMS) es el que manda. 2,80 V = fallo del BMS.'],
              ['· Hoja Mapa: colores por módulo y paralelo. Hoja Tira: franjas rojas = paralelo que se queda por debajo del resto en el tiempo.'],
              ['· Canales Pk: con el .xrk cada paralelo se refresca cada ~1 s; con el CSV exportado (interpolado), cada ~5-8 s. Los picos rápidos están en Celda mín BMS.']]
    est_panel = {0: {'*': 3}, 7: {'*': 1}}
    for r in range(8, 8 + nd):
        est_panel[r] = {1: 2, 2: 2, 3: 2, 4: 4}
    est_panel[9 + nd] = {'*': 1}
    GH = 22                                      # filas por grafica
    fila0 = len(panel) + 1
    anchors = [(0, fila0, 8, fila0 + GH), (8, fila0, 16, fila0 + GH), (0, fila0 + GH + 1, 8, fila0 + 2 * GH + 1), (8, fila0 + GH + 1, 16, fila0 + 2 * GH + 1)]

    # --------------------------------------------------------------------------- hoja Mapa
    def bloque(tit, nota, fn, n, pref):
        out = [[tit], [nota], ['Módulo'] + [f'{pref}{j}' for j in range(1, n + 1)]]
        for m in mods:
            out.append([m] + [fn(f'{m}_{pref}{j}') for j in range(1, n + 1)])
        return out + [[]]


    mapa = []
    mapa += bloque('Tensión mínima con carga (V)', 'Lo que más baja tirando (3er valor más bajo). Rojo = cerca de 2,8 V (fallo del BMS).', vmin_carga.get, 11, 'V')
    mapa += bloque('Tensión en reposo al final (V)', 'Mediana de los últimos 90 s parados. Rojo = el más bajo del pack (poca capacidad o desequilibrio).', v_rep_fin.get, 11, 'V')
    mapa += bloque('Tensión en reposo al inicio (V)', 'Antes de rodar.', v_rep_ini.get, 11, 'V')
    mapa += bloque('Temperatura máxima (°C)', 'M05 T1 está excluido en el firmware.', lambda p: None if p in EXCL else t_max.get(p), 9, 'T')
    est_mapa = {}
    for b in range(4):
        est_mapa[14 * b] = {'*': 3}
        est_mapa[14 * b + 2] = {'*': 1}
        for r in range(14 * b + 3, 14 * b + 13):
            est_mapa[r] = {c: 2 for c in range(1, 12)} | {0: 1}
    esc_mapa = [('B4:L13', 'Vabs'), ('B18:L27', 'Vrel'), ('B32:L41', 'Vrel'), ('B46:J55', 'T')]

    # --------------------------------------------------------------------------- hoja Tira (heatmap en el tiempo)
    paso = 10
    tira = [['Tiempo (s)', 'I (A)'] + [p.replace('_V', ' V') for p in PV]]
    for k in range(0, N, paso):
        blk = range(k, min(N, k + paso))
        vals = {p: med([g(q, p) for q in blk]) for p in PV}
        mdn = med(vals.values())
        tira.append([t[k], med([I[q] for q in blk])] + [None if vals[p] is None or mdn is None else round((vals[p] - mdn) * 1000) for p in PV])
    tira_t = [['Tiempo (s)', 'I (A)'] + [p.replace('_T', ' T') for p in PT]]
    for k in range(0, N, paso):
        blk = range(k, min(N, k + paso))
        tira_t.append([t[k], med([I[q] for q in blk])] + [None if p in EXCL else med([g(q, p) for q in blk]) for p in PT])
    nt = len(tira)
    hoja_tira = hoja(tira, anchos=[(1, 2, 8), (3, 112, 4.2)], congelar=(2, 1), escalas=[(f'C2:{L(111)}{nt}', 'dmV')], estilos={0: {'*': 1}}, zoom=70)
    hoja_tirat = hoja(tira_t, anchos=[(1, 2, 8), (3, 92, 4.2)], congelar=(2, 1), escalas=[(f'C2:{L(91)}{len(tira_t)}', 'T')], estilos={0: {'*': 1}}, zoom=70)

    # --------------------------------------------------------------------------- hoja Tiempo
    iv = [i for i, n in enumerate(cab) if '_V' in n]
    it = [i for i, n in enumerate(cab) if '_T' in n]
    hoja_tiempo = hoja([cab] + D, anchos=[(1, 6, 10), (7, len(cab), 7)], congelar=(1, 1),
                       escalas=[(f'{L(iv[0])}2:{L(iv[-1])}{N + 1}', 'Vabs'), (f'{L(it[0])}2:{L(it[-1])}{N + 1}', 'T')], estilos={0: {'*': 1}})

    # --------------------------------------------------------------------------- ensamblado
    NS = 'http://schemas.openxmlformats.org'
    hojas = [('Panel', hoja(panel, anchos=[(1, 1, 34), (2, 5, 14), (6, 16, 11)], estilos=est_panel, drawing=True)),
             ('Mapa', hoja(mapa, anchos=[(1, 1, 10), (2, 12, 8)], escalas=esc_mapa, estilos=est_mapa)),
             ('Tira tensiones', hoja_tira), ('Tira temperaturas', hoja_tirat),
             ('Datos', hoja(datos, anchos=[(1, len(dcab), 12)], congelar=(1, 1), estilos={0: {'*': 1}})),
             ('Tiempo', hoja_tiempo)]
    styles = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
              f'<styleSheet xmlns="{NS}/spreadsheetml/2006/main">'
              '<numFmts count="2"><numFmt numFmtId="164" formatCode="0.000"/><numFmt numFmtId="165" formatCode="+0;-0;0"/></numFmts>'
              '<fonts count="3"><font><sz val="10"/><name val="Calibri"/></font><font><b/><sz val="10"/><name val="Calibri"/></font>'
              '<font><b/><sz val="14"/><color rgb="FF1F3864"/><name val="Calibri"/></font></fonts>'
              '<fills count="2"><fill><patternFill patternType="none"/></fill><fill><patternFill patternType="gray125"/></fill></fills>'
              '<borders count="1"><border/></borders><cellStyleXfs count="1"><xf/></cellStyleXfs>'
              '<cellXfs count="5"><xf fontId="0"/><xf fontId="1" applyFont="1"/><xf fontId="0" numFmtId="164" applyNumberFormat="1"/>'
              '<xf fontId="2" applyFont="1"/><xf fontId="0" numFmtId="165" applyNumberFormat="1"/></cellXfs></styleSheet>')
    wb = (f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><workbook xmlns="{NS}/spreadsheetml/2006/main" '
          f'xmlns:r="{NS}/officeDocument/2006/relationships"><sheets>' +
          ''.join(f'<sheet name="{escape(n)}" sheetId="{i + 1}" r:id="rId{i + 1}"/>' for i, (n, _) in enumerate(hojas)) + '</sheets></workbook>')
    rels_wb = (f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="{NS}/package/2006/relationships">' +
               ''.join(f'<Relationship Id="rId{i + 1}" Type="{NS}/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet{i + 1}.xml"/>'
                       for i in range(len(hojas))) +
               f'<Relationship Id="rId{len(hojas) + 1}" Type="{NS}/officeDocument/2006/relationships/styles" Target="styles.xml"/></Relationships>')
    drawing = [f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><xdr:wsDr xmlns:xdr="{NS}/drawingml/2006/spreadsheetDrawing" xmlns:a="{NS}/drawingml/2006/main">']
    for i, (c0, r0, c1, r1) in enumerate(anchors):
        drawing.append(f'<xdr:twoCellAnchor><xdr:from><xdr:col>{c0}</xdr:col><xdr:colOff>0</xdr:colOff><xdr:row>{r0}</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:from>'
                       f'<xdr:to><xdr:col>{c1}</xdr:col><xdr:colOff>0</xdr:colOff><xdr:row>{r1}</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:to>'
                       f'<xdr:graphicFrame macro=""><xdr:nvGraphicFramePr><xdr:cNvPr id="{i + 2}" name="Grafica {i + 1}"/><xdr:cNvGraphicFramePr/></xdr:nvGraphicFramePr>'
                       '<xdr:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/></xdr:xfrm><a:graphic>'
                       f'<a:graphicData uri="{NS}/drawingml/2006/chart"><c:chart xmlns:c="{NS}/drawingml/2006/chart" '
                       f'xmlns:r="{NS}/officeDocument/2006/relationships" r:id="rId{i + 1}"/></a:graphicData></a:graphic></xdr:graphicFrame><xdr:clientData/></xdr:twoCellAnchor>')
    drawing.append('</xdr:wsDr>')
    rels_dr = (f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="{NS}/package/2006/relationships">' +
               ''.join(f'<Relationship Id="rId{i + 1}" Type="{NS}/officeDocument/2006/relationships/chart" Target="../charts/chart{i + 1}.xml"/>'
                       for i in range(len(graficas))) + '</Relationships>')
    rels_s1 = (f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="{NS}/package/2006/relationships">'
               f'<Relationship Id="rId1" Type="{NS}/officeDocument/2006/relationships/drawing" Target="../drawings/drawing1.xml"/></Relationships>')
    ct = (f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Types xmlns="{NS}/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/>'
          '<Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
          '<Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>'
          '<Override PartName="/xl/drawings/drawing1.xml" ContentType="application/vnd.openxmlformats-officedocument.drawing+xml"/>' +
          ''.join(f'<Override PartName="/xl/worksheets/sheet{i + 1}.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>' for i in range(len(hojas))) +
          ''.join(f'<Override PartName="/xl/charts/chart{i + 1}.xml" ContentType="application/vnd.openxmlformats-officedocument.drawingml.chart+xml"/>' for i in range(len(graficas))) +
          '</Types>')
    with zipfile.ZipFile(dst, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml', ct)
        z.writestr('_rels/.rels', f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="{NS}/package/2006/relationships">'
                   f'<Relationship Id="rId1" Type="{NS}/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/></Relationships>')
        z.writestr('xl/workbook.xml', wb)
        z.writestr('xl/_rels/workbook.xml.rels', rels_wb)
        z.writestr('xl/styles.xml', styles)
        for i, (_, x) in enumerate(hojas):
            z.writestr(f'xl/worksheets/sheet{i + 1}.xml', x)
        z.writestr('xl/worksheets/_rels/sheet1.xml.rels', rels_s1)
        z.writestr('xl/drawings/drawing1.xml', ''.join(drawing))
        z.writestr('xl/drawings/_rels/drawing1.xml.rels', rels_dr)
        for i, x in enumerate(graficas):
            z.writestr(f'xl/charts/chart{i + 1}.xml', x)
    print('->', dst, os.path.getsize(dst) // 1024, 'KB | debiles:', [(p, f3(vmin_carga[p])) for p in debiles])
    return dst


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except AttributeError:
        pass
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    path = args[0] if args else m.elegir_archivo()
    if not path:
        sys.exit("No se eligió ningún archivo.")
    base = os.path.splitext(os.path.basename(path))[0]
    carpeta = os.path.join(os.path.dirname(os.path.abspath(__file__)), "modulos")
    print(f"\nLeyendo {os.path.basename(path)} ...")
    rescatado = os.path.join(carpeta, base + "_pack_rescatado.csv")
    cab = rescatar(path, rescatado).meta
    titulo = " · ".join(x for x in (cab.get("Racer", base), cab.get("Comment", ""),
                                     f'{cab.get("Date", "")} {cab.get("Time", "")}'.strip()) if x)
    dst = excel(rescatado, titulo, os.path.join(carpeta, base + "_baterias.xlsx"))
    print(f"\nExcel guardado en: {dst}")


if __name__ == "__main__":
    main()
