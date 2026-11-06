#!/usr/bin/env python3
"""
aim_a_modulos.py — Deshace la telemetría EMPAQUETADA del BMS (IDs 393-396) del
CSV del AiM: tensión de los 110 paralelos y temperatura de los 90 NTC del pack
frente al tiempo.

Uso:
    python aim_a_modulos.py                     # abre un selector de archivo
    python aim_a_modulos.py sesion.csv          # directo
    python aim_a_modulos.py sesion.csv --patron # valida el AiM con el patrón del comando 'P'

Hace tres cosas:
  1. Imprime un resumen: módulo/NTC más caliente, diferencia de temperatura
     entre módulos, paralelos más bajos, paralelos que más caen bajo carga y
     capacidad relativa de cada paralelo (necesita reposo GRABADO antes de
     rodar y ≥1 min después: compara el SOC de reposo inicial y final).
  2. Guarda modulos/<sesion>_pack.csv: una fila por segundo con M01_V1..M10_V11
     (V) y M01_T1..M10_T9 (°C), más corriente del pack y velocidad. Cada valor
     es la última lectura recibida de ese paralelo/NTC (se refrescan cada ~1 s).
     Separador ';' y coma decimal, para abrirlo directamente en Excel.
  3. Con --patron, comprueba que lo que grabó el AiM coincide bit a bit con el
     patrón que manda el BMS con el comando 'P' por serie. Hacerlo una vez en
     banco antes de fiarse de los datos.

En RaceStudio los 10 canales BMS Pk* necesitan "Stepped Values" y 24/16 bits:
si el AiM interpola, el valor empaquetado no vale y la fila se descarta (se
avisa de cuántas). Formato de bits en docs/CAN_Solo2DL.md §17b; el codificador
es src/main.cpp (codeT/codeV/patT/patV): mantener los dos iguales.
Solo librería estándar de Python: no hay que instalar nada.
"""

import csv
import os
import sys

from aim_a_fila import Sesion, elegir_archivo, ok

NUM_MOD = 10
N_T, N_V = 9, 11
T_INVALIDO, V_INVALIDO = 63, 1023
DT_SALIDA = 1.0                 # s entre filas del CSV de salida
I_REPOSO_A = 1.0                # |I| por debajo de esto = reposo
I_CARGA_A = 5.0                 # |I| por encima de esto = rodando
RELAJ_S = 60.0                  # s tras la última carga antes de fiarse de la OCV final
VENTANA_R_S = 20.0              # s: ventana de la regresión V-I (la OCV casi no cambia dentro)
CELDAS_PARALELO = 11            # 11P
# Curva OCV-SOC típica de la Samsung 40T (la del balanceador desde el 10 %;
# 0-5 % aproximado, para que un paralelo muy bajo no se quede en el 10 %).
OCV_V = [3.00, 3.25, 3.40, 3.55, 3.62, 3.68, 3.76, 3.85, 3.94, 4.02, 4.10, 4.19]
OCV_SOC = [0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
# NTCs excluidos de los fallos en el firmware (NTC_IGNORED en BQ79606.h):
# se decodifican igual, pero no entran en los rankings. (IDmod, T)
NTC_IGNORADOS = {(4, 1)}        # board 8 canal 0 = IDmod 4 (M05), T1


def dec_t(c):
    return None if c == T_INVALIDO else float(c)


def dec_v(c):
    return None if c == V_INVALIDO else 2.5 + 0.002 * c


def pat_t(m, k):
    return (m * 9 + k * 5) % 63


def pat_v(m, n):
    return (m * 97 + n * 31) % 1023


def mod(m):
    return f"M{m + 1:02d}"


def entero(x):
    """Valor de canal como entero, o None si falta o está interpolado."""
    if not ok(x):
        return None
    r = round(x)
    return int(r) if abs(x - r) < 1e-3 and r >= 0 else None


# Cada trama: canales y función que devuelve (IDmod, {('T'|'V', índice): código})
# o lanza ValueError si los IDmod de sus canales no coinciden.
def trama_393(a, b, c):
    m = a >> 20
    if (c & 0xF) != m or (a & 0x3):
        raise ValueError
    t = [(a >> 14) & 63, (a >> 8) & 63, (a >> 2) & 63,
         (b >> 18) & 63, (b >> 12) & 63, (b >> 6) & 63, b & 63,
         (c >> 10) & 63, (c >> 4) & 63]
    return m, {("T", k + 1): v for k, v in enumerate(t)}


def trama_v(base):
    def f(a, b, c):
        m = a >> 20
        if (b >> 20) != m or ((c >> 2) & 0xF) != m or (c & 0x3):
            raise ValueError
        v = [(a >> 10) & 1023, a & 1023, (b >> 10) & 1023, b & 1023, (c >> 6) & 1023]
        return m, {("V", base + i): x for i, x in enumerate(v)}
    return f


def trama_396(a):
    m = a >> 20
    if a & 0x3FF:
        raise ValueError
    return m, {("V", 11): (a >> 10) & 1023}


TRAMAS = [
    ("393 (T1..T9)", ["BMS PkT A", "BMS PkT B", "BMS PkT C"], trama_393),
    ("394 (V1..V5)", ["BMS PkV1 A", "BMS PkV1 B", "BMS PkV1 C"], trama_v(1)),
    ("395 (V6..V10)", ["BMS PkV2 A", "BMS PkV2 B", "BMS PkV2 C"], trama_v(6)),
    ("396 (V11)", ["BMS PkV3 A"], trama_396),
]


def decodificar(s):
    """Recorre el CSV y devuelve la lista de lecturas (i_fila, m, clave, código) y estadísticas."""
    lecturas, stats = [], {}
    columnas = []
    for nombre, canales, fn in TRAMAS:
        cols = [s.canal(c, c.replace(" ", "_")) for c in canales]
        if any(c is None for c in cols):
            stats[nombre] = "falta en el CSV"
            continue
        columnas.append((nombre, cols, fn))
        stats[nombre] = {"ok": 0, "interpolada": 0, "incoherente": 0}
    # El AiM muestrea más rápido que las páginas (20 Hz frente a 100 ms): la
    # misma página sale en varias filas seguidas. Solo cuenta como lectura
    # nueva cuando cambia el módulo.
    previo = {}
    for i in range(len(s.t)):
        for nombre, cols, fn in columnas:
            vals = [entero(c[i]) for c in cols]
            if any(v is None for v in vals):
                if all(ok(c[i]) for c in cols):
                    stats[nombre]["interpolada"] += 1
                continue
            # Todo a 0: aún no ha llegado ninguna trama. No es una lectura de M01:
            # sus tensiones saldrían a 2,500 V y el patrón daría un falso "NO".
            if not any(vals):
                continue
            try:
                m, d = fn(*vals)
            except ValueError:
                stats[nombre]["incoherente"] += 1
                continue
            if m >= NUM_MOD:
                stats[nombre]["incoherente"] += 1
                continue
            stats[nombre]["ok"] += 1
            if previo.get(nombre) == m:
                continue
            previo[nombre] = m
            for clave, codigo in d.items():
                lecturas.append((i, m, clave, codigo))
    return lecturas, stats


def comprobar_patron(lecturas):
    bien = mal = 0
    vistos = set()
    for _, m, (tipo, k), c in lecturas:
        esperado = pat_t(m, k) if tipo == "T" else pat_v(m, k)
        if c == esperado:
            bien += 1
        else:
            mal += 1
        vistos.add(m)
    print("\n--- Patrón de prueba ---")
    print(f"  Valores que coinciden: {bien}   no coinciden: {mal}")
    print(f"  Módulos vistos: {len(vistos)}/{NUM_MOD}")
    if bien and not mal and len(vistos) == NUM_MOD:
        print("  OK: el AiM graba los canales empaquetados sin perder bits.")
    else:
        print("  NO: revisar Stepped Values, nº de bits (24/16) y start bit en RaceStudio.")


def regresion_ventanas(ts, xs, ys, ventana_s):
    """Pendiente de y frente a x restando la media de cada ventana de tiempo.
    Con x = corriente separa la caída I·R de lo que baja la OCV durante la
    sesión (en 20 s la OCV casi no se mueve); con una regresión global un
    paralelo con menos capacidad parecía tener más o menos resistencia de la
    real según lo bajo que acabara."""
    grupos = {}
    for t_, x, y in zip(ts, xs, ys):
        # Dos rejillas desfasadas media ventana: un cambio de corriente que cae
        # justo en el borde de una ventana queda dentro de la otra.
        grupos.setdefault((0, int(t_ // ventana_s)), []).append((x, y))
        grupos.setdefault((1, int((t_ + ventana_s / 2) // ventana_s)), []).append((x, y))
    sxx = sxy = 0.0
    n = 0
    for g in grupos.values():
        if len(g) < 3:
            continue
        mx = sum(x for x, _ in g) / len(g)
        my = sum(y for _, y in g) / len(g)
        sxx += sum((x - mx) ** 2 for x, _ in g)
        sxy += sum((x - mx) * (y - my) for x, y in g)
        n += len(g)
    if n < 10 or sxx < 1e-6:
        return None
    return sxy / sxx


def soc_ocv(v):
    """SOC (%) a partir de la tensión en reposo, interpolando OCV_V/OCV_SOC."""
    if v <= OCV_V[0]:
        return OCV_SOC[0]
    for k in range(1, len(OCV_V)):
        if v <= OCV_V[k]:
            return OCV_SOC[k - 1] + (v - OCV_V[k - 1]) * (OCV_SOC[k] - OCV_SOC[k - 1]) / (OCV_V[k] - OCV_V[k - 1])
    return OCV_SOC[-1]


def mediana(x):
    x = sorted(x)
    n = len(x)
    return None if n == 0 else (x[n // 2] if n % 2 else 0.5 * (x[n // 2 - 1] + x[n // 2]))


def capacidad_relativa(s, filas, corriente, i_carga0, i_carga1):
    """Todos los paralelos están en serie: pasan los mismos Ah. El que pierde
    más SOC entre el reposo inicial y el final tiene menos capacidad.
    Pasar por la curva OCV (y no comparar mV a pelo) evita que un paralelo
    que simplemente está más bajo parezca peor por estar en la zona empinada."""
    print("\n--- Capacidad relativa (reposo antes de rodar → reposo final) ---")
    if i_carga0 is None:
        print("  Sin tramo con corriente: no se puede calcular.")
        return
    con = [f for f in filas if f["ocv_ini"] is not None and f["ocv_fin"] is not None]
    if not con:
        falta = []
        if not any(f["ocv_ini"] is not None for f in filas):
            falta.append("reposo grabado ANTES de rodar")
        if not any(f["ocv_fin"] is not None for f in filas):
            falta.append(f"reposo grabado ≥{RELAJ_S:.0f} s DESPUÉS de la última carga")
        print("  No se puede calcular: falta " + " y ".join(falta) + ".")
        return
    for f in con:
        f["dsoc"] = soc_ocv(f["ocv_ini"]) - soc_ocv(f["ocv_fin"])
    ref = mediana([f["dsoc"] for f in con])
    ah = sum(corriente[i] for i in range(i_carga0, i_carga1 + 1) if ok(corriente[i])) * s.dt / 3600.0
    reposo_fin = s.t[-1] - s.t[i_carga1]
    print(f"  Descargado: {ah:.2f} Ah según el Hall (por encima de 30 A puede leer alto: ver HallSensor.h)."
          f"  SOC perdido, mediana: {ref:.1f} %.  Reposo final: {reposo_fin:.0f} s")
    if ref < 2.0:
        print("  Muy poca descarga (<2 % de SOC): el resultado es poco fiable; hace falta rodar más.")
        return
    for f in con:
        f["rel"] = ref / f["dsoc"] if f["dsoc"] > 0.1 else None
    validos_rel = [f for f in con if f["rel"] is not None]
    print("  Los 5 paralelos con menos capacidad (1,00 = como la mediana del pack):")
    for f in sorted(validos_rel, key=lambda f: f["rel"])[:5]:
        print(f"    {mod(f['m'])} V{f['n']}: {f['rel']:.2f}  (≈ {f['rel'] * CELDAS_PARALELO:.0f} de"
              f" {CELDAS_PARALELO} celdas; {f['ocv_ini']:.3f} → {f['ocv_fin']:.3f} V,"
              f" {f['dsoc']:.1f} % de SOC)")
    print("  Por debajo de ~0,85 apunta a celdas desconectadas (fusibles de celda): medir ese paralelo.")


def analizar(s, lecturas, salida):
    t = s.t
    corriente = s.canal("BMS PackCurrent")
    velocidad = s.canal("GPS Speed")

    # Tramo con carga: de la primera a la última muestra con |I| > I_CARGA_A.
    # El reposo de ANTES y el de DESPUÉS (pasado RELAJ_S) dan la OCV de cada
    # paralelo para la capacidad relativa.
    carga = [i for i in range(len(t)) if corriente and ok(corriente[i]) and abs(corriente[i]) > I_CARGA_A]
    i_carga0 = carga[0] if carga else None
    i_carga1 = carga[-1] if carga else None

    # Valor físico de cada lectura
    series = {}          # (m, tipo, k) -> [(i_fila, valor)]
    for i, m, (tipo, k), c in lecturas:
        v = dec_t(c) if tipo == "T" else dec_v(c)
        if v is not None:
            series.setdefault((m, tipo, k), []).append((i, v))

    # Refresco por módulo
    por_mod = {}
    for i, m, (tipo, k), _ in lecturas:
        if tipo == "V" and k == 1:
            por_mod.setdefault(m, []).append(t[i])
    print("\n--- Cobertura ---")
    for m in range(NUM_MOD):
        ts = sorted(set(por_mod.get(m, [])))
        if len(ts) > 1:
            print(f"  {mod(m)} (IDmod {m}): {len(ts)} lecturas, una cada {(ts[-1] - ts[0]) / (len(ts) - 1):.1f} s")
        else:
            print(f"  {mod(m)} (IDmod {m}): sin datos")

    # ---- temperaturas ----
    print("\n--- Temperaturas ---")
    tmax_mod = {}
    for m in range(NUM_MOD):
        mejor = None
        for k in range(1, N_T + 1):
            if (m, k) in NTC_IGNORADOS:
                continue
            for i, v in series.get((m, "T", k), []):
                if mejor is None or v > mejor[0]:
                    mejor = (v, k, t[i])
        if mejor:
            tmax_mod[m] = mejor
    for m, (v, k, ti) in sorted(tmax_mod.items(), key=lambda x: -x[1][0]):
        print(f"  {mod(m)}: máx {v:.0f} °C en T{k} (t={ti:.0f} s)")
    for (m, k) in sorted(NTC_IGNORADOS):
        vs = [v for _, v in series.get((m, "T", k), [])]
        if vs:
            print(f"  ({mod(m)} T{k} excluido de fallos: {min(vs):.0f}..{max(vs):.0f} °C, fuera del ranking)")
    if len(tmax_mod) > 1:
        vals = [x[0] for x in tmax_mod.values()]
        print(f"  Diferencia entre el módulo más caliente y el más frío: {max(vals) - min(vals):.0f} °C")

    # ---- tensiones ----
    print("\n--- Paralelos ---")
    filas = []
    for m in range(NUM_MOD):
        for n in range(1, N_V + 1):
            pts = series.get((m, "V", n), [])
            if not pts:
                continue
            vmin_i, vmin_v = min(pts, key=lambda p: p[1])
            reposo = [v for i, v in pts if corriente and ok(corriente[i]) and abs(corriente[i]) < I_REPOSO_A]
            r = None
            if corriente:
                xy = [(t[i], corriente[i], v) for i, v in pts if ok(corriente[i])]
                pend = regresion_ventanas([a for a, _, _ in xy], [x for _, x, _ in xy],
                                          [y for _, _, y in xy], VENTANA_R_S)
                r = -pend * 1000 if pend is not None else None   # mΩ "vistos" desde la corriente del pack
            ocv_ini = ocv_fin = None
            if i_carga0 is not None:
                quieto = [(i, v) for i, v in pts if ok(corriente[i]) and abs(corriente[i]) < I_REPOSO_A]
                ocv_ini = mediana([v for i, v in quieto if i < i_carga0])
                ocv_fin = mediana([v for i, v in quieto if t[i] > t[i_carga1] + RELAJ_S])
            filas.append({"m": m, "n": n, "vmin": vmin_v, "t_vmin": t[vmin_i],
                          "i_vmin": corriente[vmin_i] if corriente and ok(corriente[vmin_i]) else None,
                          "v_ini": reposo[0] if reposo else None,
                          "v_fin": reposo[-1] if reposo else None, "r": r,
                          "ocv_ini": ocv_ini, "ocv_fin": ocv_fin})
    if filas:
        print("  Los 5 paralelos con la tensión más baja:")
        for f in sorted(filas, key=lambda f: f["vmin"])[:5]:
            i_txt = f" con {f['i_vmin']:.0f} A" if f["i_vmin"] is not None else ""
            print(f"    {mod(f['m'])} V{f['n']}: {f['vmin']:.3f} V (t={f['t_vmin']:.0f} s{i_txt})")
        con_r = [f for f in filas if f["r"] is not None]
        if con_r:
            media = sum(f["r"] for f in con_r) / len(con_r)
            print(f"  Caída con la corriente (mV por A del pack; media {media:.2f}). Los 5 que más caen:")
            for f in sorted(con_r, key=lambda f: -f["r"])[:5]:
                print(f"    {mod(f['m'])} V{f['n']}: {f['r']:.2f}  ({f['r'] / media:.1f}x la media)")
        con_fin = [f for f in filas if f["v_fin"] is not None]
        if con_fin:
            print("  Los 5 más bajos en reposo al final:")
            for f in sorted(con_fin, key=lambda f: f["v_fin"])[:5]:
                print(f"    {mod(f['m'])} V{f['n']}: {f['v_fin']:.3f} V")
            vf = [f["v_fin"] for f in con_fin]
            print(f"  Desequilibrio en reposo al final: {(max(vf) - min(vf)) * 1000:.0f} mV")
        capacidad_relativa(s, filas, corriente, i_carga0, i_carga1)
    else:
        print("  Sin tensiones decodificadas.")

    # ---- CSV de salida ----
    claves = [(m, "V", n) for m in range(NUM_MOD) for n in range(1, N_V + 1)] + \
             [(m, "T", k) for m in range(NUM_MOD) for k in range(1, N_T + 1)]
    cab = ["Tiempo (s)", "I pack (A)", "Velocidad (km/h)"] + \
          [f"{mod(m)}_{tipo}{k}" for m, tipo, k in claves]
    ultimo = {}
    por_fila = {}
    for i, m, (tipo, k), c in lecturas:
        v = dec_t(c) if tipo == "T" else dec_v(c)
        por_fila.setdefault(i, []).append(((m, tipo, k), v))

    def f(x, nd):
        return "" if x is None or not ok(x) else f"{x:.{nd}f}".replace(".", ",")

    os.makedirs(os.path.dirname(salida), exist_ok=True)
    with open(salida, "w", newline="", encoding="utf-8-sig") as fo:
        w = csv.writer(fo, delimiter=";")
        w.writerow(cab)
        siguiente = t[0] if t else 0
        for i in range(len(t)):
            for clave, v in por_fila.get(i, []):
                ultimo[clave] = v
            if t[i] + 1e-9 >= siguiente:
                siguiente += DT_SALIDA
                w.writerow([f(t[i], 2),
                            f(corriente[i], 1) if corriente else "",
                            f(velocidad[i], 1) if velocidad else ""] +
                           [f(ultimo.get(c), 3 if c[1] == "V" else 0) for c in claves])
    print(f"\nTabla por segundo guardada en: {salida}")


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except AttributeError:
        pass
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    patron = "--patron" in sys.argv
    path = args[0] if args else elegir_archivo()
    if not path:
        sys.exit("No se eligió ningún archivo.")

    print(f"\nLeyendo {os.path.basename(path)} ...")
    s = Sesion(path)
    lecturas, stats = decodificar(s)

    print("\n--- Tramas empaquetadas ---")
    for nombre, st in stats.items():
        if isinstance(st, str):
            print(f"  {nombre}: {st}")
        else:
            print(f"  {nombre}: {st['ok']} filas buenas, {st['interpolada']} interpoladas, "
                  f"{st['incoherente']} con IDmod distinto entre canales")
            # Con Stepped Values no sale ni una: una sola ya indica que está sin marcar,
            # y una fila interpolada puede pasar por buena y dar un valor falso.
            if st["interpolada"]:
                print("    ! Hay filas interpoladas: marca 'Stepped Values' en estos canales en "
                      "RaceStudio. Los valores de este log pueden ser falsos.")
    if not lecturas:
        sys.exit("\nNo hay nada que decodificar: ¿están los canales BMS Pk* en el log?")

    if patron:
        comprobar_patron(lecturas)
        return

    salida = os.path.join(os.path.dirname(os.path.abspath(__file__)), "modulos",
                          os.path.splitext(os.path.basename(path))[0] + "_pack.csv")
    analizar(s, lecturas, salida)


if __name__ == "__main__":
    main()
