#!/usr/bin/env python3
"""
aim_a_fila.py — Saca la fila de la hoja "Testing 26/27" (pestaña Hoja 1) a partir
del CSV exportado del AiM (RaceStudio 3 → Export → CSV).

Uso:
    python aim_a_fila.py                  # abre un selector de archivo
    python aim_a_fila.py sesion.csv       # directo
    python aim_a_fila.py sesion.csv 7     # 7 = código de la prueba en la Hoja 2

Hace tres cosas:
  1. Imprime un resumen legible y el veredicto de los criterios de paso.
  2. Copia al portapapeles la fila (65 columnas separadas por tabulador, coma
     decimal) para pegarla en la primera celda de una fila vacía de la Hoja 1.
  3. Guarda la misma fila en filas/<sesion>.tsv.

Si se da el código de prueba, descarga la Hoja 2 (plan de pruebas) y avisa si la
configuración del inversor que se ve en el log no coincide con la planificada.

Nombres de canal: los del DBC (docs/BMS_MART.dbc) tal como los importa RaceStudio
(guion bajo -> espacio). Si se renombra una señal en el DBC, cambiar aquí también.
Solo librería estándar de Python: no hay que instalar nada.
"""

import csv
import math
import os
import sys
import urllib.parse
import urllib.request
from datetime import datetime

# ===================== CONFIG =====================
SHEET_ID = "18oA7q569xaZiNq7OMyFt_6w63z2O7QIieH2IaMMF_7o"
PLAN_TAB = "Hoja 2"

RELACION = 4.0            # transmisión 44/11
CIRCUNFERENCIA_M = 1.221  # rodadura Hoosier 16.0x7.5-10, medida con GPS vs ERPM (sesión 0934)
PARES_POLOS = 10          # EMRAX 188 -> RPM motor = ERPM / 10

HV_ON_V = 300             # bus DC por encima de esto = alta tensión conectada
SDC_POR_BMS_VENTANA_S = 1.0

# Criterios de paso (los de la Hoja 2)
CRIT_MOTOR_MAX_C = 80.0
CRIT_PENDIENTE_C_MIN = 1.0
CRIT_REC_MAX = 128
CRIT_CELDA_MIN_V = 3.0
# ==================================================

CABECERA = [
    "Fecha", "Nº ID", "Tipo de prueba", "Versión firmware BMS", "Versión firmware VCU",
    "Versión firmware PDM", "FaultTimer K (muestras)", "Motor Current Max (Apk)",
    "Absolute Max Current (Apk)", "Motor Current Max Brake (Apk)", "Max DC (A)",
    "Min DC freno (A)", "RPM max conf", "kW max conf", "Relación de transmisión",
    "Masa piloto (kg)", "Temperatura ambiente (°C)", "SOC inicial (%)",
    "V pack BMS inicial (V)", "Desbalanceo inicial (mV)", "Celda min inicial (V)",
    "Temperatura max modulos inicial (°C)", "Temperatura min modulos inicial (°C)",
    "Temperatura motor inicial (°C)", "Temperatura inversor inicial (°C)",
    "V pack BMS final (V)", "Desbalanceo final (mV)", "Celda min final (V)",
    "Celda min bajo carga (V)", "Módulo y paralelo de la celda min",
    "Temperatura max modulos (°C)", "Temperatura min modulos (°C)",
    "Temperatura max motor (°C)", "Temperatura max inversor (°C)",
    "Pendiente temp. motor final (°C/min)", "Duración conduciendo (min)",
    "Nº aceleraciones a fondo", "Tiempo 0-75 m (s)", "Metros totales",
    "Velocidad max (km/h)", "DC max (A)", "DC reg max (A)", "AC max (Apk)",
    "AC reg max (Apk)", "RPM motor max", "kW max batería (BMS)",
    "REC max (errores CAN AiM)", "Bits de límite del inversor activados",
    "VCU viva toda la sesión", "Fallo BMS", "FirstFault", "INV FaultCode",
    "SDC abierto por otro nodo", "Reintentos reInit BQ", "MaxFailMs", "MaxCommFailMs",
    "Nº fallos V", "Nº fallos T", "Nº fallos comm BQ", "Nº fallos Hall",
    "Nº fallos init BQ", "Terminada", "Causa del fallo", "Comentarios",
    "Enlace Telemetria",
]
FIRST_FAULT = {0: "0 ninguno", 1: "1 V", 2: "2 T", 3: "3 NTC", 4: "4 Comm", 5: "5 Hall", 6: "6 NoInit"}
BITS_IGNORADOS = ("INV DrvEnLim", "INV VinLim")   # se activan solos al habilitar / al apagar

avisos = []


def aviso(msg):
    avisos.append(msg)


# --------------------------------------------------------------------------
#  Lectura del CSV del AiM
# --------------------------------------------------------------------------
def num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def ok(x):
    return x is not None and not (isinstance(x, float) and math.isnan(x))


class Sesion:
    def __init__(self, path):
        with open(path, newline="", encoding="utf-8", errors="replace") as f:
            rows = list(csv.reader(f))
        self.meta = {r[0]: r[1] for r in rows[:14] if len(r) >= 2}
        # La cabecera es la fila "Time" con muchas columnas: los metadatos también traen "Time","9:00 PM".
        hi = next(i for i, r in enumerate(rows) if len(r) > 2 and r[0] == "Time")
        self.nombres = rows[hi]
        self.unidades = rows[hi + 1]
        datos = [r for r in rows[hi + 2:] if len(r) == len(self.nombres) and any(r[1:])]
        self.col = {n: i for i, n in enumerate(self.nombres)}
        fs = num(self.meta.get("Sample Rate", "20")) or 20.0
        t = [num(r[0]) for r in datos]
        if not t or any(math.isnan(x) for x in t[:5]):
            aviso("El CSV no trae columna de tiempo (sesión sin GPS): se usa la frecuencia de muestreo.")
            t = [i / fs for i in range(len(datos))]
        self.t = t
        self.datos = datos
        self.dt = 1.0 / fs

    def nombre(self, *candidatos):
        """Nombre real del canal: exacto, o por prefijo (el AiM recorta a 16 caracteres)."""
        for c in candidatos:
            if c in self.col:
                return c
        for c in candidatos:
            for n in self.nombres:
                if n.startswith(c[:16]) or c.startswith(n) and len(n) >= 12:
                    return n
        return None

    def canal(self, *candidatos):
        n = self.nombre(*candidatos)
        if n is None:
            return None
        i = self.col[n]
        return [num(r[i]) for r in self.datos]


def validos(v):
    return [x for x in v if ok(x)]


def vmax(v):
    v = validos(v or [])
    return max(v) if v else None


def vmin(v):
    v = validos(v or [])
    return min(v) if v else None


def moda_redondeada(v, nd=0):
    v = [round(x, nd) for x in validos(v)]
    if not v:
        return None
    return max(set(v), key=v.count)


# --------------------------------------------------------------------------
#  Cálculos
# --------------------------------------------------------------------------
def escala_pack(v):
    """BMS PackVoltage: el AiM a veces pierde el gain 0,1 y/o aplica x1000 por 'mV'."""
    vals = validos(v)
    if not vals:
        return v
    m = sorted(vals)[len(vals) // 2]
    if m > 1e5:
        aviso("BMS PackVoltage venía en 'mV' y sin el gain 0,1 (corregido aquí). Arreglar el canal en RaceStudio.")
        return [x / 10000 if ok(x) else x for x in v]
    if m > 2000:
        aviso("BMS PackVoltage venía sin el gain 0,1 (corregido aquí). Arreglar el canal en RaceStudio.")
        return [x / 10 if ok(x) else x for x in v]
    return v


def a_voltios_celda(v):
    """BMS MinV/MaxV llegan en mV."""
    return [x / 1000 if ok(x) and x > 100 else x for x in v]


def suavizar(v, n):
    out, acc, q = [], 0.0, []
    for x in v:
        x = x if ok(x) else 0.0
        q.append(x)
        acc += x
        if len(q) > n:
            acc -= q.pop(0)
        out.append(acc / len(q))
    return out


def calcular(s, plan):
    t, dt = s.t, s.dt
    N = len(t)
    fila = {c: "" for c in CABECERA}
    res = {}   # valores para el resumen

    vdc = s.canal("INV VinDC")
    if vdc:
        vdc = [x / 1000 if ok(x) and x > 2000 else x for x in vdc]
    erpm = s.canal("INV ERPM")
    thr = s.canal("INV Throttle")
    iac = s.canal("INV AC Current")
    idc = s.canal("INV DC Current")
    drive = s.canal("INV DriveEnable")
    hv = [ok(x) and x > HV_ON_V for x in vdc] if vdc else [False] * N
    if not any(hv):
        aviso("No se ve alta tensión en el inversor (INV VinDC) en toda la sesión.")

    rueda = [(abs(e) / PARES_POLOS / RELACION / 60 * CIRCUNFERENCIA_M) if ok(e) else 0.0
             for e in (erpm or [0] * N)]          # m/s
    moviendo = [hv[i] and rueda[i] > 0.5 for i in range(N)]

    # ---- reposo inicial / final (sin corriente) ----
    def en_reposo(i):
        a = iac[i] if iac else 0
        d = idc[i] if idc else 0
        return (not ok(a) or abs(a) < 1) and (not ok(d) or abs(d) < 0.5)

    minv = a_voltios_celda(s.canal("BMS MinV") or [])
    maxv = a_voltios_celda(s.canal("BMS MaxV") or [])
    reposo = [i for i in range(N) if en_reposo(i) and minv and ok(minv[i]) and minv[i] > 1]
    ini = reposo[0] if reposo else None
    fin = reposo[-1] if reposo else None

    # ---- configuración del inversor (la que había con HV) ----
    def cfg(nombre):
        v = s.canal(nombre)
        if not v:
            return None
        sel = [v[i] for i in range(N) if hv[i]] or v
        return moda_redondeada(sel, 1)

    mcm, brake = cfg("INV MaxAC cfg"), cfg("INV MinAC cfg")
    maxdc, mindc = cfg("INV MaxDC cfg"), cfg("INV MinDC cfg")
    res["cfg"] = (mcm, brake, maxdc, mindc)
    fila["Motor Current Max (Apk)"] = mcm
    fila["Motor Current Max Brake (Apk)"] = brake
    fila["Max DC (A)"] = maxdc
    fila["Min DC freno (A)"] = mindc
    fila["Relación de transmisión"] = RELACION

    # ---- identificación ----
    fecha = s.meta.get("Date", "")
    try:
        fila["Fecha"] = datetime.strptime(fecha, "%A, %B %d, %Y").strftime("%d/%m/%Y")
    except ValueError:
        fila["Fecha"] = fecha
    if plan:
        fila["Nº ID"] = plan.get("Código de prueba", "")
        fila["Tipo de prueba"] = plan.get("Tipo de prueba", "")
        fila["Absolute Max Current (Apk)"] = plan.get("Absolute Max (Apk)", "")

    # ---- estado inicial / final ----
    soc = s.canal("BMS SOC")
    if soc and ini is not None:
        fila["SOC inicial (%)"] = soc[ini]
    pack = s.canal("BMS PackVoltage")
    if pack:
        pack = escala_pack(pack)
        if ini is not None:
            fila["V pack BMS inicial (V)"] = pack[ini]
            fila["V pack BMS final (V)"] = pack[fin]
    else:
        aviso("Falta BMS PackVoltage (ID 18): V pack inicial/final y kW quedan vacíos.")
    if ini is not None and maxv:
        fila["Desbalanceo inicial (mV)"] = (maxv[ini] - minv[ini]) * 1000
        fila["Desbalanceo final (mV)"] = (maxv[fin] - minv[fin]) * 1000
        fila["Celda min inicial (V)"] = minv[ini]
        fila["Celda min final (V)"] = minv[fin]
    carga = [minv[i] for i in range(N) if moviendo[i] and minv and ok(minv[i]) and minv[i] > 1]
    celda_carga = min(carga) if carga else None
    fila["Celda min bajo carga (V)"] = celda_carga

    maxt, mint = s.canal("BMS MaxT"), s.canal("BMS MinT")
    if ini is not None:
        if maxt: fila["Temperatura max modulos inicial (°C)"] = maxt[ini]
        if mint: fila["Temperatura min modulos inicial (°C)"] = mint[ini]
    fila["Temperatura max modulos (°C)"] = vmax(maxt)
    fila["Temperatura min modulos (°C)"] = vmin(mint)

    mot, ctrl = s.canal("INV MotorTemp"), s.canal("INV CtrlTemp")
    primero_hv = next((i for i in range(N) if hv[i]), None)
    if primero_hv is not None:
        if mot: fila["Temperatura motor inicial (°C)"] = mot[primero_hv]
        if ctrl: fila["Temperatura inversor inicial (°C)"] = ctrl[primero_hv]
    t_mot = vmax(mot)
    fila["Temperatura max motor (°C)"] = t_mot
    fila["Temperatura max inversor (°C)"] = vmax(ctrl)

    # ---- conducción ----
    mov_idx = [i for i in range(N) if moviendo[i]]
    pendiente = None
    if mov_idx and mot:
        t_fin = t[mov_idx[-1]]
        cola = [i for i in mov_idx if t[i] >= t_fin - 120 and ok(mot[i])]
        if len(cola) > 20:
            xs = [t[i] / 60 for i in cola]
            ys = [mot[i] for i in cola]
            mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
            sxx = sum((x - mx) ** 2 for x in xs)
            if sxx > 0:
                pendiente = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    fila["Pendiente temp. motor final (°C/min)"] = pendiente
    fila["Duración conduciendo (min)"] = len(mov_idx) * dt / 60

    acel, start = 0, None
    for i in range(N):
        a_fondo = thr and ok(thr[i]) and thr[i] >= 95
        if a_fondo and start is None:
            start = i
        if (not a_fondo or i == N - 1) and start is not None:
            if (t[i] - t[start]) >= 1.0 and iac and vmax(iac[start:i + 1]) and vmax(iac[start:i + 1]) > 5:
                acel += 1
            start = None
    fila["Nº aceleraciones a fondo"] = acel

    # 0-75 m: arrancadas desde parado sin levantar el pie
    mejor = None
    i = 1
    while thr and erpm and i < N:
        if ok(thr[i]) and thr[i] >= 95 and not (ok(thr[i - 1]) and thr[i - 1] >= 95) and rueda[i] < 0.6:
            x, j = 0.0, i
            while j < N and ok(thr[j]) and thr[j] >= 90 and x < 75:
                x += rueda[j] * dt
                j += 1
            if x >= 75:
                tt = t[j - 1] - t[i]
                mejor = tt if mejor is None else min(mejor, tt)
            i = j
        i += 1
    fila["Tiempo 0-75 m (s)"] = mejor

    dist = s.canal("Distance on GPS Speed")
    gps = s.canal("GPS Speed")
    if dist and vmax(dist) and vmax(dist) > 10:
        fila["Metros totales"] = vmax(dist) - (vmin(dist) or 0)
    else:
        fila["Metros totales"] = sum(rueda[i] for i in range(N) if hv[i]) * dt
        aviso("Metros totales calculados con la velocidad de rueda (sin GPS).")
    if gps and vmax(gps) and vmax(gps) > 2:
        fila["Velocidad max (km/h)"] = vmax(gps)
    else:
        fila["Velocidad max (km/h)"] = max(rueda) * 3.6
        aviso("Velocidad máxima calculada con la velocidad de rueda (sin GPS).")

    fila["DC max (A)"] = vmax(idc)
    fila["DC reg max (A)"] = vmin(idc)
    fila["AC max (Apk)"] = vmax(iac)
    fila["AC reg max (Apk)"] = vmin(iac)
    fila["RPM motor max"] = (vmax(erpm) or 0) / PARES_POLOS if erpm else None

    pcur = s.canal("BMS PackCurrent")
    kw = None
    if pack and pcur:
        sin_corriente = (vmax([abs(x) for x in validos(pcur)]) or 0) < 0.05
        if sin_corriente and (vmax(idc) or 0) > 10:
            aviso("BMS PackCurrent está a 0 toda la sesión aunque el inversor ve corriente: kW vacío. Revisar el Hall/canal.")
        else:
            p = [pack[i] * pcur[i] / 1000 if ok(pack[i]) and ok(pcur[i]) else float("nan") for i in range(N)]
            kw = max(suavizar(p, int(0.5 / dt) or 1))
    elif not pcur:
        aviso("Falta BMS PackCurrent (ID 18): kW max batería queda vacío.")
    fila["kW max batería (BMS)"] = kw
    if vdc and idc:
        p_inv = [vdc[i] * idc[i] / 1000 if ok(vdc[i]) and ok(idc[i]) else 0 for i in range(N)]
        res["kw_inv"] = max(suavizar(p_inv, int(0.5 / dt) or 1))

    # ---- salud ----
    rec = s.canal("ECU CAN - REC Rx Error")
    rec_mov = [rec[i] for i in mov_idx if ok(rec[i])] if rec else []
    fila["REC max (errores CAN AiM)"] = max(rec_mov) if rec_mov else None

    bits = []
    for n in s.nombres:
        if n.startswith("INV ") and n.endswith("Lim") and n not in BITS_IGNORADOS:
            v = s.canal(n)
            if any(hv[i] and ok(v[i]) and v[i] > 0.5 for i in range(N)):
                bits.append(n[4:])
    fila["Bits de límite del inversor activados"] = ", ".join(bits) if bits else "ninguno"

    vcu = None
    hb = s.canal("VCU Heartbeat")
    prox = hb or s.canal("APPS1 raw") or s.canal("VCU Vbat")
    if prox and any(hv):
        hv_idx = [i for i in range(N) if hv[i]]
        ultimo = None
        hueco = 0.0
        for i in hv_idx:
            if i > 0 and prox[i] != prox[i - 1]:
                ultimo = t[i]
            ref = ultimo if ultimo is not None else t[hv_idx[0]]
            hueco = max(hueco, t[i] - ref)
        vcu = "sí" if hueco < 2.0 else "no"
        if not hb:
            aviso("VCU Heartbeat no está en el log: 'VCU viva' se deduce de si cambian sus otros canales.")
        if vcu == "no":
            aviso(f"La VCU estuvo {hueco:.1f} s sin mandar datos con la alta tensión conectada.")
    fila["VCU viva toda la sesión"] = vcu

    stk = s.canal("BMS StkBmsFault")
    flt = s.canal("BMS Fault")
    hubo_fallo = (vmax(stk) or 0) > 0.5 or (vmax(flt) or 0) > 0.5
    fila["Fallo BMS"] = "sí" if hubo_fallo else "no"
    ff = s.canal("FirstFault")
    ff_val = 0
    if hubo_fallo and ff and flt:
        k = next((i for i in range(N) if ok(flt[i]) and flt[i] > 0.5), None)
        if k is not None:
            ventana = [ff[j] for j in range(max(0, k - 20), min(N, k + 20)) if ok(ff[j]) and ff[j] > 0.5]
            ff_val = moda_redondeada(ventana) or 0
    fila["FirstFault"] = FIRST_FAULT.get(int(ff_val), str(int(ff_val)))

    fc = s.canal("INV FaultCode")
    codigos = sorted({int(round(fc[i])) for i in range(N) if hv[i] and ok(fc[i]) and round(fc[i]) != 0}) if fc else []
    fila["INV FaultCode"] = ", ".join(map(str, codigos)) if codigos else 0

    sdc = s.canal("Pin SDC 3V3")
    aperturas = 0
    if sdc:
        w = int(SDC_POR_BMS_VENTANA_S / dt)
        for i in range(1, N):
            if ok(sdc[i - 1]) and ok(sdc[i]) and sdc[i - 1] > 0.5 >= sdc[i] and hv[i - 1]:
                bms = flt and any(ok(flt[j]) and flt[j] > 0.5 for j in range(max(0, i - w), min(N, i + w)))
                if not bms:
                    aperturas += 1
        fila["SDC abierto por otro nodo"] = "sí" if aperturas else "no"
    res["aperturas_sdc"] = aperturas

    fila["Reintentos reInit BQ"] = vmax(s.canal("BMS NumTryReset"))
    fila["MaxFailMs"] = vmax(s.canal("BMS MaxFailMs"))
    fila["MaxCommFailMs"] = vmax(s.canal("BMS MaxCommFailMs", "BMS MaxCommFailM"))
    fila["Nº fallos V"] = vmax(s.canal("BMS CntV"))
    fila["Nº fallos T"] = vmax(s.canal("BMS CntT"))
    fila["Nº fallos comm BQ"] = vmax(s.canal("BMS CntComm"))
    fila["Nº fallos Hall"] = vmax(s.canal("BMS CntHall"))
    fila["Nº fallos init BQ"] = vmax(s.canal("BMS CntInit"))
    fila["Terminada"] = "FALSE"

    # ---- criterios de paso ----
    res["criterios"] = [
        ("Motor < 80 °C", t_mot is not None and t_mot < CRIT_MOTOR_MAX_C, t_mot),
        ("Motor subiendo < 1 °C/min al final", pendiente is not None and pendiente < CRIT_PENDIENTE_C_MIN, pendiente),
        ("Sin bits de límite térmico", not bits, ", ".join(bits) or "ninguno"),
        ("REC < 128", fila["REC max (errores CAN AiM)"] is not None and fila["REC max (errores CAN AiM)"] < CRIT_REC_MAX,
         fila["REC max (errores CAN AiM)"]),
        ("VCU viva", vcu == "sí", vcu),
        ("Sin fallo del BMS", not hubo_fallo, fila["FirstFault"]),
        ("Celda mínima bajo carga ≥ 3,0 V", celda_carga is not None and celda_carga >= CRIT_CELDA_MIN_V, celda_carga),
    ]
    return fila, res


# --------------------------------------------------------------------------
#  Plan de pruebas (Hoja 2)
# --------------------------------------------------------------------------
def cargar_plan(codigo):
    url = (f"https://docs.google.com/spreadsheets/d/{SHEET_ID}/gviz/tq?tqx=out:csv&sheet="
           + urllib.parse.quote(PLAN_TAB))
    try:
        with urllib.request.urlopen(url, timeout=15) as r:
            texto = r.read().decode("utf-8")
    except Exception as e:
        aviso(f"No se pudo descargar la Hoja 2 ({e}): no se compara con el plan.")
        return None
    for fila in csv.DictReader(texto.splitlines()):
        if fila.get("Código de prueba", "").strip() == str(codigo).strip():
            return fila
    aviso(f"El código {codigo} no está en la Hoja 2.")
    return None


def comparar_con_plan(plan, cfg):
    pares = [("Motor Current Max (Apk)", cfg[0]), ("Motor Current Max Brake (Apk)", cfg[1]),
             ("Max DC (A)", cfg[2]), ("Min DC freno (A)", cfg[3])]
    for nombre, real in pares:
        previsto = num(plan.get(nombre, "").replace(",", "."))
        if not ok(previsto) or real is None:
            continue
        if abs(abs(previsto) - abs(real)) > 0.5:
            aviso(f"CONFIGURACIÓN DISTINTA AL PLAN: {nombre} planificado {previsto:g}, en el inversor {real:g}.")


# --------------------------------------------------------------------------
#  Salida
# --------------------------------------------------------------------------
def fmt(x):
    if x is None or x == "":
        return ""
    if isinstance(x, bool):
        return "TRUE" if x else "FALSE"
    if isinstance(x, (int, float)):
        if isinstance(x, float) and math.isnan(x):
            return ""
        if float(x).is_integer():
            return str(int(x))
        return f"{x:.2f}".rstrip("0").rstrip(".").replace(".", ",")
    return str(x)


def al_portapapeles(texto):
    try:
        import tkinter
        r = tkinter.Tk()
        r.withdraw()
        r.clipboard_clear()
        r.clipboard_append(texto)
        r.update()
        return r
    except Exception as e:
        aviso(f"No se pudo copiar al portapapeles ({e}); copia la fila del archivo .tsv.")
        return None


def elegir_archivo():
    try:
        import tkinter
        from tkinter import filedialog
        r = tkinter.Tk()
        r.withdraw()
        p = filedialog.askopenfilename(title="CSV exportado del AiM",
                                       filetypes=[("CSV del AiM", "*.csv"), ("Todos", "*.*")])
        r.destroy()
        return p
    except Exception:
        return input("Ruta del CSV del AiM: ").strip().strip('"')


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except AttributeError:
        pass
    args = sys.argv[1:]
    path = args[0] if args else elegir_archivo()
    if not path:
        sys.exit("No se eligió ningún archivo.")
    codigo = args[1] if len(args) > 1 else input("Código de la prueba en la Hoja 2 (ENTER para omitir): ").strip()

    print(f"\nLeyendo {os.path.basename(path)} ...")
    s = Sesion(path)
    plan = cargar_plan(codigo) if codigo else None
    fila, res = calcular(s, plan)
    if plan:
        comparar_con_plan(plan, res["cfg"])

    print(f"\nSesión: {s.meta.get('Racer', '')}  {s.meta.get('Date', '')} {s.meta.get('Time', '')}"
          f"  ({len(s.t) * s.dt / 60:.1f} min de log)")
    if plan:
        print(f"Prueba {codigo}: {plan.get('Tipo de prueba', '')} · escalón {plan.get('Escalón', '')}")
        print(f"  Requisitos: {plan.get('Requisitos previos', '')}")
        print(f"  Criterio:   {plan.get('Criterio de paso', '')}")
    print("\n--- Valores ---")
    for c in CABECERA:
        v = fmt(fila[c])
        if v != "":
            print(f"  {c:40s} {v}")
    if "kw_inv" in res:
        print(f"  {'(kW max según el inversor)':40s} {fmt(res['kw_inv'])}")
    if res.get("aperturas_sdc"):
        print(f"  {'(aperturas del SDC sin fallo del BMS)':40s} {res['aperturas_sdc']} (incluye las manuales)")

    print("\n--- Criterios de paso ---")
    for nombre, cumple, valor in res["criterios"]:
        print(f"  [{'OK' if cumple else 'NO'}] {nombre:36s} {fmt(valor)}")
    if avisos:
        print("\n--- Avisos ---")
        for a in avisos:
            print("  ! " + a)

    linea = "\t".join(fmt(fila[c]) for c in CABECERA)
    os.makedirs(os.path.join(os.path.dirname(os.path.abspath(__file__)), "filas"), exist_ok=True)
    salida = os.path.join(os.path.dirname(os.path.abspath(__file__)), "filas",
                          os.path.splitext(os.path.basename(path))[0] + ".tsv")
    with open(salida, "w", encoding="utf-8") as f:
        f.write("\t".join(CABECERA) + "\n" + linea + "\n")
    print(f"\nFila guardada en: {salida}")

    root = al_portapapeles(linea)
    if root:
        print("Fila copiada al portapapeles: en la Hoja 1, clic en la columna 'Fecha' de una fila vacía y Ctrl+V.")
        print("Quedan a mano: versiones de firmware, FaultTimer K, RPM/kW conf, masa del piloto,"
              " temperatura ambiente, módulo de la celda min, causa, comentarios y enlace.")
        input("\nPulsa ENTER cuando hayas pegado (cierra el portapapeles)...")
        root.destroy()


if __name__ == "__main__":
    main()
