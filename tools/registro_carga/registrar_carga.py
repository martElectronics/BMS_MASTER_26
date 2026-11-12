#!/usr/bin/env python3
"""
registrar_carga.py — Guarda TODO lo que manda el firmware del cargador (charger.cpp)
durante una carga: tensión de los 110 paralelos, temperatura de los 90 NTC, corriente
del Hall y la tensión/corriente que reporta el cargador. Una fila cada 0,5 s.

Uso:
    python registrar_carga.py            # autodetecta el puerto (pregunta si hay varios)
    python registrar_carga.py --port COM5
o doble clic en registrar_carga.bat. Ctrl+C para terminar.

Comandos al cargador desde esta misma ventana: escribe y pulsa ENTER
(g = empezar a cargar, x = parar, c,7 = fijar 7 A, d = estado del cargador).
Las respuestas del firmware ([CHG] ..., [I] ...) salen en pantalla y el JSON
se sigue grabando mientras tanto.

Cómo funciona: activa el JSON del dashboard (tecla 'j' del charger) y escribe cada
línea en registros/carga_<fecha>.csv. Si se corta el USB, reintenta y sigue en el
mismo fichero. Arráncalo ANTES de dar corriente y déjalo 30 min después de parar:
los reposos de antes y después son los que dan la capacidad absoluta.
Luego: analizar_carga.py (o analizar_carga.bat) con ese CSV.
⚠ Cierra el monitor de PlatformIO antes: el puerto COM no se comparte.
"""
import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

import serial
import serial.tools.list_ports

try:
    import msvcrt          # Windows: leer el teclado sin bloquear
except ImportError:
    msvcrt = None

BAUD = 115200
CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "registros")
AVISO_S = 10.0          # cada cuánto imprime el estado en pantalla


def elegir_puerto(fijo):
    if fijo:
        return fijo
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        sys.exit("[ERROR] No hay puertos serie. Conecta el STM del cargador por USB.")
    if len(ports) == 1:
        return ports[0].device
    print("Puertos disponibles:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    sel = input(f"Elige puerto [0-{len(ports) - 1}] (ENTER=0): ").strip()
    return ports[int(sel) if sel.isdigit() else 0].device


def cabecera(d):
    """Columnas a partir de la primera línea JSON (nº de módulos, paralelos y NTC reales)."""
    cols = ["pc_hora", "t_s", "i_hall_A", "chg_outV", "chg_outI", "chg_iSet", "chg_cargando", "chg_estado",
            "vMin", "vMax", "tMin", "tMax", "bmsOk", "f_v", "f_t", "f_ntc", "f_comm", "f_hall", "f_init"]
    for m, mod in enumerate(d["modules"]):
        cols += [f"M{m + 1:02d}_V{n + 1}" for n in range(len(mod["v"]))]
    for m, mod in enumerate(d["modules"]):
        cols += [f"M{m + 1:02d}_T{k + 1}" for k in range(len(mod["t"]))]
    return cols


def fila(d, t0):
    c = d.get("chg", {})
    f = d.get("faults", {})
    b = lambda x: 1 if x else 0
    out = [datetime.now().isoformat(timespec="milliseconds"), round(time.time() - t0, 2),
           d.get("i"), c.get("outV"), c.get("outI"), c.get("i"), b(c.get("charging")), c.get("st"),
           d.get("vMin"), d.get("vMax"), d.get("tMin"), d.get("tMax"), b(d.get("bmsOk")),
           b(f.get("v")), b(f.get("t")), b(f.get("ntc")), b(f.get("comm")), b(f.get("hall")), b(f.get("init"))]
    for mod in d["modules"]:
        out += mod["v"]
    for mod in d["modules"]:
        out += [None if x <= -900 else x for x in mod["t"]]
    return out


_tecleado = []


def teclado(ser):
    """Lo que se escribe en la ventana se manda al cargador al pulsar ENTER."""
    if msvcrt is None:
        return
    while msvcrt.kbhit():
        ch = msvcrt.getwch()
        if ch in ("\r", "\n"):
            cmd = "".join(_tecleado).strip()
            _tecleado.clear()
            print()
            if cmd:
                ser.write((cmd + "\n").encode())
                print(f"  >> enviado al cargador: {cmd}")
        elif ch == "\b":
            if _tecleado:
                _tecleado.pop()
                print("\b \b", end="", flush=True)
        elif ch.isprintable():
            _tecleado.append(ch)
            print(ch, end="", flush=True)


def leer_json(ser, espera_s):
    """Devuelve la primera línea JSON válida que llegue en espera_s, o None.
    Las demás líneas del firmware (respuestas a los comandos) se muestran."""
    fin = time.time() + espera_s
    while time.time() < fin:
        teclado(ser)
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        if raw.startswith("{"):
            if raw.endswith("}"):
                try:
                    return json.loads(raw)
                except json.JSONDecodeError:
                    pass
        elif raw:
            print(f"  [cargador] {raw}")
    return None


def activar_json(ser):
    """Si el dashboard JSON no está activo, lo activa con 'j' (es un conmutador:
    primero se mira si ya llega JSON, para no apagarlo)."""
    d = leer_json(ser, 2.5)
    if d is not None:
        return d
    ser.write(b"j")
    return leer_json(ser, 3.0)


def main():
    ap = argparse.ArgumentParser(description="Registro completo de una carga (charger.cpp)")
    ap.add_argument("--port", help="puerto serie, p.ej. COM5")
    args = ap.parse_args()
    puerto = elegir_puerto(args.port)
    os.makedirs(CARPETA, exist_ok=True)
    ruta = os.path.join(CARPETA, datetime.now().strftime("carga_%Y%m%d_%H%M.csv"))

    t0 = time.time()
    fo = None
    w = None
    n = 0
    ah_chg = ah_hall = 0.0
    t_prev = None
    t_aviso = 0.0
    print(f"Guardando en {ruta}\nCtrl+C para terminar (deja 30 min de reposo tras parar la carga).")
    print("Comandos (escribe y ENTER): g = cargar, x = parar, c,7 = fijar 7 A, d = estado del cargador.")
    try:
        while True:
            try:
                with serial.Serial(puerto, BAUD, timeout=1) as ser:
                    time.sleep(2)                  # el VCP del ST-Link tarda en estabilizarse
                    d = activar_json(ser)
                    if d is None:
                        print(f"[{puerto}] no llega el JSON del cargador. ¿Firmware charger cargado? Reintento...")
                        time.sleep(3)
                        continue
                    print(f"[{puerto}] recibiendo datos.")
                    while True:
                        if d is not None:
                            if w is None:
                                fo = open(ruta, "w", newline="", encoding="utf-8")
                                w = csv.writer(fo)
                                w.writerow(cabecera(d))
                            r = fila(d, t0)
                            w.writerow(r)
                            n += 1
                            ahora = time.time()
                            if t_prev is not None and ahora - t_prev < 5:
                                dt_h = (ahora - t_prev) / 3600
                                ah_chg += (d.get("chg", {}).get("outI") or 0) * dt_h
                                ah_hall += (d.get("i") or 0) * dt_h
                            t_prev = ahora
                            if ahora - t_aviso >= AVISO_S:
                                t_aviso = ahora
                                fo.flush()
                                c = d.get("chg", {})
                                print(f"  {(ahora - t0) / 60:6.1f} min | I cargador {c.get('outI', 0):5.1f} A, Hall {d.get('i', 0):5.1f} A"
                                      f" | {ah_chg:6.2f} Ah | celda {d.get('vMin', 0):.3f}-{d.get('vMax', 0):.3f} V"
                                      f" | T máx {d.get('tMax', 0):.0f} °C | {n} filas")
                        d = leer_json(ser, 5.0)
                        if d is None:
                            print("  (5 s sin datos: ¿se ha parado el JSON? Lo reactivo)")
                            d = activar_json(ser)
            except (serial.SerialException, OSError) as e:
                print(f"[{puerto}] conexión perdida ({e}). Reintento en 3 s; sigue en el mismo fichero.")
                t_prev = None
                time.sleep(3)
    except KeyboardInterrupt:
        pass
    finally:
        if fo:
            fo.close()
    print(f"\nFin: {n} filas, {ah_chg:.2f} Ah según el cargador ({ah_hall:.2f} según el Hall).")
    print(f"Fichero: {ruta}\nAhora: analizar_carga.bat (o python analizar_carga.py \"{ruta}\")")


if __name__ == "__main__":
    main()
