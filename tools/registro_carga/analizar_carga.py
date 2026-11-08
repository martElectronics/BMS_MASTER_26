#!/usr/bin/env python3
"""
analizar_carga.py — Capacidad de cada paralelo y del pack a partir de una carga
registrada con registrar_carga.py. Genera resultados/<carga>.xlsx.

Uso:
    python analizar_carga.py                       # abre un selector de archivo
    python analizar_carga.py registros/carga_X.csv
o doble clic en analizar_carga.bat.

Dos medidas:
  1. Capacidad RELATIVA (no necesita reposo ni curva de la celda): todos los paralelos
     están en serie y reciben los mismos Ah. Se mira cuántos Ah necesita cada uno para
     subir cada tramo de 0,1 V (3,6→3,7, 3,7→3,8...). Uno con 8 de 11 celdas lo hace con
     ~0,73 de los Ah de la mediana. Con corriente constante la caída I·R es casi igual en
     todo el tramo, así que se cancela.
  2. Capacidad ABSOLUTA (necesita reposo ANTES de cargar y >= 10 min DESPUÉS): tensión
     de reposo de cada paralelo -> SOC con la curva típica de la Samsung 40T -> Ah / SOC
     ganado. Es la que dice si el pack tiene 44 Ah o menos.
Además: resistencia de cada paralelo con el escalón de corriente al arrancar la carga.
Solo librería estándar.
"""
import csv
import math
import os
import sys

from xlsx_simple import (COLORES, DOS_DEC, NEGRITA, TITULO, TRES_DEC, Grafica, Libro, rango_col)

I_CARGA_A = 1.0          # por encima: cargando
I_REPOSO_A = 0.3         # por debajo: reposo
REPOSO_MIN_S = 600       # reposo final mínimo para fiarse de la capacidad absoluta
VENTANA_REPOSO_S = 120   # se usa el final de cada reposo (lo más relajado)
UMBRALES = [round(3.40 + 0.05 * k, 2) for k in range(16)]   # 3,40 ... 4,15 V
TRAMO_V = 0.10
CELDAS_PARALELO = 11
# Curva OCV-SOC típica de la Samsung 40T (la misma que aim_a_modulos.py).
OCV_V = [3.00, 3.25, 3.40, 3.55, 3.62, 3.68, 3.76, 3.85, 3.94, 4.02, 4.10, 4.19]
OCV_SOC = [0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]


def soc_ocv(v):
    if v <= OCV_V[0]:
        return 0.0
    for k in range(1, len(OCV_V)):
        if v <= OCV_V[k]:
            return OCV_SOC[k - 1] + (v - OCV_V[k - 1]) * (OCV_SOC[k] - OCV_SOC[k - 1]) / (OCV_V[k] - OCV_V[k - 1])
    return 100.0


def mediana(x):
    x = sorted(v for v in x if v is not None)
    n = len(x)
    return None if n == 0 else (x[n // 2] if n % 2 else 0.5 * (x[n // 2 - 1] + x[n // 2]))


def num(s):
    try:
        v = float(s)
        return None if math.isnan(v) else v
    except (TypeError, ValueError):
        return None


def elegir_archivo():
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        p = filedialog.askopenfilename(title="CSV de la carga", filetypes=[("CSV", "*.csv")],
                                       initialdir=os.path.join(os.path.dirname(os.path.abspath(__file__)), "registros"))
        root.destroy()
        return p
    except Exception:
        return input("Ruta del CSV: ").strip().strip('"')


def leer(path):
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        cab = next(r)
        filas = [x for x in r if len(x) == len(cab)]
    col = {n: i for i, n in enumerate(cab)}
    datos = {n: [num(x[i]) for x in filas] for n, i in col.items() if n != "pc_hora"}
    datos["pc_hora"] = [x[col["pc_hora"]] for x in filas]
    pv = [n for n in cab if "_V" in n and n[0] == "M"]
    pt = [n for n in cab if "_T" in n and n[0] == "M"]
    return datos, pv, pt


def analizar(path):
    d, PV, PT = leer(path)
    t = d["t_s"]
    N = len(t)
    if N < 20:
        sys.exit("El registro es demasiado corto.")
    mods = sorted({p.split("_")[0] for p in PV})

    # Corriente: la del cargador si la hay; si no, la del Hall
    ic = d["chg_outI"]
    usa_cargador = sum(1 for x in ic if x and x > I_CARGA_A) > 10
    I = [x if x is not None else 0.0 for x in (ic if usa_cargador else d["i_hall_A"])]
    ih = [x if x is not None else 0.0 for x in d["i_hall_A"]]
    ah = [0.0] * N
    ah_h = 0.0
    huecos = 0.0
    for k in range(1, N):
        dt = t[k] - t[k - 1]
        if dt > 5:
            huecos += dt
        ah[k] = ah[k - 1] + 0.5 * (I[k] + I[k - 1]) * dt / 3600
        ah_h += 0.5 * (ih[k] + ih[k - 1]) * dt / 3600
    cargando = [k for k in range(N) if I[k] > I_CARGA_A]
    if not cargando:
        sys.exit("No hay tramo de carga (corriente > 1 A) en el registro.")
    i0, i1 = cargando[0], cargando[-1]
    ah_total = ah[i1] - ah[i0]
    i_med = mediana(I[k] for k in cargando)

    def reposo(idx):
        idx = [k for k in idx if abs(I[k]) < I_REPOSO_A]
        if not idx:
            return [], 0.0
        tfin = t[idx[-1]]
        return [k for k in idx if t[k] >= tfin - VENTANA_REPOSO_S], t[idx[-1]] - t[idx[0]]

    rep_ini, dur_ini = reposo(range(0, i0))
    rep_fin, dur_fin = reposo(range(i1 + 1, N))

    res = {}
    for p in PV:
        v = d[p]
        # mediana móvil de 5 muestras para quitar el ruido de una lectura
        vs = [mediana(v[max(0, k - 2):k + 3]) for k in range(N)]
        cruce = {}
        for u in UMBRALES:
            if vs[i0] is not None and vs[i0] >= u:
                continue
            for k in range(i0 + 1, i1 + 1):
                if vs[k] is not None and vs[k - 1] is not None and vs[k] >= u > vs[k - 1]:
                    f = (u - vs[k - 1]) / (vs[k] - vs[k - 1]) if vs[k] != vs[k - 1] else 0
                    cruce[u] = ah[k - 1] + f * (ah[k] - ah[k - 1])
                    break
        vi = mediana(v[k] for k in rep_ini) if rep_ini else None
        vf = mediana(v[k] for k in rep_fin) if rep_fin else None
        # resistencia: escalón de tensión ~10-20 s después de arrancar la carga
        k10 = next((k for k in range(i0, i1) if t[k] - t[i0] >= 10), None)
        r = None
        if vi is not None and k10 is not None:
            vc = mediana(v[k10:k10 + 20])
            ic0 = mediana(I[k10:k10 + 20])
            if vc is not None and ic0 and ic0 > I_CARGA_A:
                r = (vc - vi) / ic0 * 1000
        res[p] = dict(cruce=cruce, vi=vi, vf=vf, r=r, vs=vs)

    # tramos de 0,1 V: Ah que necesita cada paralelo
    tramos = []
    for u in UMBRALES:
        u2 = round(u + TRAMO_V, 2)
        if u2 not in UMBRALES:
            continue
        dah = {p: res[p]["cruce"][u2] - res[p]["cruce"][u] for p in PV if u in res[p]["cruce"] and u2 in res[p]["cruce"]}
        if len(dah) >= 0.8 * len(PV) and mediana(dah.values()) and mediana(dah.values()) > 0.05:
            m = mediana(dah.values())
            tramos.append((u, u2, {p: x / m for p, x in dah.items()}, m))
    for p in PV:
        res[p]["rel"] = mediana(tr[2].get(p) for tr in tramos) if tramos else None

    # capacidad absoluta
    abs_ok = bool(rep_ini and rep_fin and dur_fin >= REPOSO_MIN_S)
    for p in PV:
        vi, vf = res[p]["vi"], res[p]["vf"]
        res[p]["cap"] = None
        if abs_ok and vi is not None and vf is not None:
            ds = soc_ocv(vf) - soc_ocv(vi)
            if ds > 2:
                res[p]["cap"] = ah_total / ds * 100
    return dict(d=d, PV=PV, PT=PT, mods=mods, t=t, N=N, I=I, ah=ah, ah_total=ah_total, ah_hall=ah_h, i0=i0, i1=i1,
                i_med=i_med, dur_ini=dur_ini, dur_fin=dur_fin, abs_ok=abs_ok, res=res, tramos=tramos,
                usa_cargador=usa_cargador, huecos=huecos)


def informe(A, path):
    PV, PT, mods, res, t, N = A["PV"], A["PT"], A["mods"], A["res"], A["t"], A["N"]
    d = A["d"]
    rel = {p: res[p]["rel"] for p in PV}
    debiles = sorted([p for p in PV if rel[p] is not None], key=lambda p: rel[p])[:6]
    caps = [res[p]["cap"] for p in PV if res[p]["cap"] is not None]
    nombre = lambda p: p.replace("_", " ")

    print(f"\nCarga: {A['ah_total']:.2f} Ah ({'cargador' if A['usa_cargador'] else 'Hall'}), Hall {A['ah_hall']:.2f} Ah,"
          f" corriente {A['i_med']:.1f} A, {(t[A['i1']] - t[A['i0']]) / 60:.0f} min")
    print(f"Reposo antes {A['dur_ini'] / 60:.0f} min, después {A['dur_fin'] / 60:.0f} min")
    if A["huecos"] > 0:
        print(f"! {A['huecos']:.0f} s sin datos (USB cortado): los Ah de esos huecos se interpolan.")
    print(f"Tramos de 0,1 V usados: {', '.join(f'{a:.2f}-{b:.2f}' for a, b, _, _ in A['tramos']) or 'ninguno'}")
    print("Paralelos con menos capacidad relativa (1,00 = mediana del pack):")
    for p in debiles:
        r = res[p]
        print(f"  {nombre(p)}: {rel[p]:.2f} (≈ {rel[p] * CELDAS_PARALELO:.0f} de {CELDAS_PARALELO} celdas)"
              f"  R {r['r']:.2f} mΩ" if r["r"] is not None else f"  {nombre(p)}: {rel[p]:.2f}")
    if A["abs_ok"] and caps:
        print(f"Capacidad absoluta: mediana {mediana(caps):.1f} Ah, mínima {min(caps):.1f} Ah (la que limita el pack)")
    else:
        print(f"Capacidad absoluta: no se calcula (hace falta reposo antes y >= {REPOSO_MIN_S // 60} min después).")

    # ---------------- Datos (cada 10 s) ----------------
    paso = max(1, int(round(10 / max(0.1, mediana([t[k] - t[k - 1] for k in range(1, min(N, 200))])))))
    idx = list(range(0, N, paso))
    tmod = {m: [max([d[q][k] for q in PT if q.startswith(m + "_") and q != "M05_T1" and d[q][k] is not None] or [None]) for k in idx] for m in mods}
    dcab = ["Tiempo (min)", "Ah", "I cargador (A)", "I Hall (A)", "Celda mín (V)", "Celda máx (V)", "Mediana (V)",
            "Desequilibrio (mV)"] + [nombre(p) for p in debiles] + [f"T máx {m}" for m in mods]
    datos = [dcab]
    for j, k in enumerate(idx):
        vs = [d[p][k] for p in PV if d[p][k] is not None]
        datos.append([t[k] / 60, A["ah"][k], d["chg_outI"][k], d["i_hall_A"][k], d["vMin"][k], d["vMax"][k], mediana(vs),
                      (max(vs) - min(vs)) * 1000 if vs else None] + [res[p]["vs"][k] for p in debiles] + [tmod[m][j] for m in mods])
    nr = len(datos)
    R = lambda c: rango_col("Datos", c, 2, nr)
    X_ah, X_t = R(1), R(0)
    nd = len(debiles)

    # ---------------- Panel ----------------
    panel = [[f"Carga — {os.path.basename(path)}"], [],
             ["Ah cargados", f"{A['ah_total']:.2f} Ah ({'cargador' if A['usa_cargador'] else 'Hall'}) · Hall {A['ah_hall']:.2f} Ah"],
             ["Corriente / duración", f"{A['i_med']:.1f} A · {(t[A['i1']] - t[A['i0']]) / 60:.0f} min"],
             ["Reposo antes / después", f"{A['dur_ini'] / 60:.0f} min / {A['dur_fin'] / 60:.0f} min"],
             ["Capacidad del pack", (f"mediana {mediana(caps):.1f} Ah · el más débil {min(caps):.1f} Ah" if A["abs_ok"] and caps
                                     else f"sin calcular: hace falta reposo antes y ≥ {REPOSO_MIN_S // 60} min después")],
             [], ["Paralelos con menos capacidad", "capacidad relativa", "≈ celdas", "R (mΩ)", "reposo antes (V)", "reposo después (V)", "capacidad (Ah)"]]
    for p in debiles:
        r = res[p]
        panel.append([nombre(p), rel[p], rel[p] * CELDAS_PARALELO, r["r"], r["vi"], r["vf"], r["cap"]])
    panel += [[], ["Cómo leerlo"],
              ["· Capacidad relativa: Ah que necesita el paralelo para subir cada 0,1 V frente a la mediana. 0,73 ≈ 8 de 11 celdas."],
              ["· Gráfica de tensión frente a Ah: un paralelo con menos capacidad sube más deprisa (su línea se despega hacia arriba)."],
              ["· Capacidad absoluta: tensión de reposo antes/después con la curva típica de la 40T. Necesita los dos reposos."]]
    est = {0: {"*": TITULO}, 7: {"*": NEGRITA}, 9 + nd: {"*": NEGRITA}}
    for r_ in range(8, 8 + nd):
        est[r_] = {1: DOS_DEC, 2: 0, 3: DOS_DEC, 4: TRES_DEC, 5: TRES_DEC, 6: DOS_DEC}
    f0 = len(panel) + 1
    GH = 22
    vv = [x for fila in datos[1:] for x in fila[4:6] if x is not None]
    graf = [
        Grafica("Tensión frente a Ah cargados", [("Celda máx", X_ah, R(5), "00B050", 1.5, False), ("Mediana", X_ah, R(6), "7F7F7F", 1.5, True),
                                                 ("Celda mín", X_ah, R(4), "000000", 1.5, False)] +
                [(nombre(debiles[i]), X_ah, R(8 + i), COLORES[i], 2.25, False) for i in range(nd)],
                "V", "Ah", (0, f0, 8, f0 + GH), round(min(vv) - 0.05, 1) if vv else None, round(max(vv) + 0.05, 1) if vv else None),
        Grafica("Corriente", [("Cargador", X_t, R(2), "0070C0", 1.5, False), ("Hall", X_t, R(3), "FF7F00", 1.0, True)],
                "A", "Tiempo (min)", (8, f0, 16, f0 + GH)),
        Grafica("Desequilibrio (celda máx - mín)", [("mV", X_ah, R(7), "C00000", 1.25, False)], "mV", "Ah", (0, f0 + GH + 1, 8, f0 + 2 * GH + 1), 0),
        Grafica("Temperatura máxima de cada módulo", [(mods[i], X_t, R(8 + nd + i), COLORES[i % 10], 1.5, False) for i in range(len(mods))],
                "°C", "Tiempo (min)", (8, f0 + GH + 1, 16, f0 + 2 * GH + 1)),
    ]

    # ---------------- Mapa ----------------
    def bloque(tit, nota, fn, pref, n):
        out = [[tit], [nota], ["Módulo"] + [f"{pref}{j}" for j in range(1, n + 1)]]
        for m in mods:
            out.append([m] + [fn(f"{m}_{pref}{j}") for j in range(1, n + 1)])
        return out + [[]]
    nv = max(int(p.split("_V")[1]) for p in PV)
    nt = max(int(p.split("_T")[1]) for p in PT) if PT else 0
    tmaxp = {p: max([x for x in d[p] if x is not None] or [None]) for p in PT}
    mapa = (bloque("Capacidad relativa (1,00 = mediana)", "Rojo ≈ 0,7 (unas 8 de 11 celdas). Verde = normal.", lambda p: rel.get(p), "V", nv) +
            bloque("Resistencia (mΩ)", "Escalón de tensión al arrancar la carga. Rojo = alta (unión, fusibles o celdas que faltan).", lambda p: res[p]["r"] if p in res else None, "V", nv) +
            bloque("Reposo antes de cargar (V)", "Rojo = el más bajo del pack.", lambda p: res[p]["vi"] if p in res else None, "V", nv) +
            bloque("Reposo después de cargar (V)", "", lambda p: res[p]["vf"] if p in res else None, "V", nv) +
            bloque("Temperatura máxima (°C)", "", lambda p: tmaxp.get(p), "T", nt))
    b = len(mods) + 4
    from xlsx_simple import letra
    ult_v, ult_t = letra(nv), letra(nt)
    rng = lambda k, u: f"B{k * b + 4}:{u}{k * b + 3 + len(mods)}"
    esc = [(rng(0, ult_v), "capacidad"), (rng(1, ult_v), "verde_rojo"), (rng(2, ult_v), "rojo_verde"), (rng(3, ult_v), "rojo_verde"), (rng(4, ult_t), "temperatura")]
    est_m = {}
    for k in range(5):
        est_m[k * b] = {"*": TITULO}
        est_m[k * b + 2] = {"*": NEGRITA}
        for r_ in range(k * b + 3, k * b + 3 + len(mods)):
            est_m[r_] = {c: (DOS_DEC if k < 2 else TRES_DEC) for c in range(1, nv + 1)} | {0: NEGRITA}

    # ---------------- Tramos ----------------
    tr = [["Paralelo"] + [f"{a:.2f}-{b_:.2f} V" for a, b_, _, _ in A["tramos"]] + ["Mediana"]]
    tr.append(["Ah de la mediana"] + [m for _, _, _, m in A["tramos"]] + [None])
    for p in sorted(PV, key=lambda p: rel[p] if rel[p] is not None else 9):
        tr.append([nombre(p)] + [x[2].get(p) for x in A["tramos"]] + [rel[p]])
    ntr = len(A["tramos"])
    est_t = {0: {"*": NEGRITA}, 1: {"*": DOS_DEC}}
    for r_ in range(2, len(tr)):
        est_t[r_] = {c: DOS_DEC for c in range(1, ntr + 2)}

    libro = Libro()
    libro.hoja("Panel", panel, anchos=[(1, 1, 32), (2, 7, 15), (8, 16, 10)], estilos=est, graficas=graf)
    libro.hoja("Mapa", mapa, anchos=[(1, 1, 10), (2, nv + 1, 8)], escalas=esc, estilos=est_m)
    libro.hoja("Tramos", tr, anchos=[(1, 1, 16), (2, ntr + 2, 12)], congelar=(1, 2),
               escalas=[(f"B3:{letra(ntr + 1)}{len(tr)}", "capacidad")] if ntr else (), estilos=est_t)
    libro.hoja("Datos", datos, anchos=[(1, len(dcab), 12)], congelar=(1, 1), estilos={0: {"*": NEGRITA}})
    carpeta = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultados")
    os.makedirs(carpeta, exist_ok=True)
    dst = os.path.join(carpeta, os.path.splitext(os.path.basename(path))[0] + ".xlsx")
    libro.guardar(dst)
    print(f"\nExcel guardado en: {dst}")
    return dst


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except AttributeError:
        pass
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    path = args[0] if args else elegir_archivo()
    if not path:
        sys.exit("No se eligió ningún archivo.")
    print(f"Leyendo {os.path.basename(path)} ...")
    informe(analizar(path), path)


if __name__ == "__main__":
    main()
