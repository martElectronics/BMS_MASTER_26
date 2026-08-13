/**
 * BMS Telemetry Dashboard — Web Serial API client
 *
 * Protocol: STM32 emits one JSON line per refresh tick when in 'j' mode.
 * Format:
 *   {"bmsOk":true,"vMin":3.920,"vMax":3.973,"vDelta":53.0,"i":12.34,
 *    "tMin":23.4,"tMax":28.6,"ntcOpen":0,"soc":84,"fan":0,
 *    "faults":{"v":false,"t":false,"ntc":false,"comm":false,"hall":false,"init":false},
 *    "modules":[{"v":[3.96,...],"t":[25.6,...]}, ...]}
 */

// ── Thresholds (match BMS defines) ──────────────────────────────────
const UV = 2.8, OV = 4.2, UT = -20, OT = 60;
const UV_WARN = UV + 0.03, OV_WARN = OV - 0.03;
const OT_WARN = OT - 5;

// ── State ────────────────────────────────────────────────────────────
let port       = null;
let reader     = null;
let lineBuffer = '';
let currentTab = 'v';

// ── DOM refs ─────────────────────────────────────────────────────────
const connectBtn  = document.getElementById('connectBtn');
const statusEl    = document.getElementById('connectionStatus');
const statusText  = document.getElementById('statusText');
const bmsOkEl     = document.getElementById('bmsOk');
const bmsCardEl   = document.getElementById('bmsCard');

const currentText = document.getElementById('currentText');
const fanText     = document.getElementById('fanText');
const vMinText    = document.getElementById('vMinText');
const vMaxText    = document.getElementById('vMaxText');
const tMinText    = document.getElementById('tMinText');
const tMaxText    = document.getElementById('tMaxText');
const vDeltaText  = document.getElementById('vDeltaText');
const ntcOpenText = document.getElementById('ntcOpenText');
const container   = document.getElementById('modulesContainer');
const faultEls = {
    v:    document.getElementById('faultV'),
    t:    document.getElementById('faultT'),
    ntc:  document.getElementById('faultNTC'),
    comm: document.getElementById('faultCOMM'),
    hall: document.getElementById('faultHALL'),
    init: document.getElementById('faultINIT'),
};

// ── Charger control (solo si el firmware publica "chg" en el JSON) ────
const chargePanel     = document.getElementById('chargePanel');
const chgStateText    = document.getElementById('chgStateText');
const chgTsonText     = document.getElementById('chgTsonText');
const chgVoutText     = document.getElementById('chgVoutText');
const chgIoutText     = document.getElementById('chgIoutText');
const chgCurrentInput = document.getElementById('chgCurrentInput');
const chgCurrentHint  = document.getElementById('chgCurrentHint');
const chgSetCurrentBtn= document.getElementById('chgSetCurrentBtn');
const chgStartBtn     = document.getElementById('chgStartBtn');
const chgStopBtn      = document.getElementById('chgStopBtn');

// ── Envío de comandos al firmware ─────────────────────────────────────
// El writable es independiente del readable, así que se puede usar sin tocar
// el reader. Hay que soltar el lock tras cada escritura o la siguiente falla.
async function sendCmd(cmd) {
    if (!port || !port.writable) return false;
    const w = port.writable.getWriter();
    try {
        await w.write(new TextEncoder().encode(cmd + '\n'));
        return true;
    } catch (e) {
        console.warn('Serial write error:', e.message);
        return false;
    } finally {
        w.releaseLock();
    }
}

// 'g' = solicitar carga. El firmware solo la concede si la seguridad da OK y
// el TSON está armado, y la cancela ante cualquier fallo — este botón NO puede
// forzar una carga insegura. Aun así se pide confirmación: es 400 V.
chgStartBtn?.addEventListener('click', () => {
    if (confirm('¿Iniciar la carga?\n\nSe enviará la petición al BMS; solo arrancará '
              + 'si la seguridad da OK y el TSON está armado.')) {
        sendCmd('g');
    }
});

chgStopBtn?.addEventListener('click', () => sendCmd('x'));

// 'c,<I>' = fijar corriente DC. El firmware la capa a CHG_MAX_CURRENT_A de
// todos modos; aquí se capa también para no mandar valores absurdos.
chgSetCurrentBtn?.addEventListener('click', () => {
    const max = parseFloat(chgCurrentInput.max) || 0;
    let v = parseFloat(chgCurrentInput.value);
    if (!isFinite(v) || v < 0) v = 0;
    if (v > max) { v = max; chgCurrentInput.value = v; }
    sendCmd('c,' + v.toFixed(1));
});

// ── Connect / Disconnect ─────────────────────────────────────────────
connectBtn.addEventListener('click', () => { port ? disconnect() : connect(); });

async function connect() {
    try {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });
    } catch (e) {
        console.error('Serial open error:', e);
        port = null;
        return;
    }
    lineBuffer = '';
    setConnected(true);
    readLoop();  // fire-and-forget — manages its own lifecycle
}

async function disconnect() {
    if (reader) { try { await reader.cancel(); } catch (_) {} }
    if (port)   { try { await port.close();    } catch (_) {} }
    reader = null;
    port   = null;
    setConnected(false);
}

function setConnected(on) {
    statusEl.className  = 'connection-status' + (on ? ' connected' : '');
    statusText.textContent = on ? 'Connected' : 'Disconnected';
    connectBtn.textContent = on ? '✕ Disconnect' : '⚡ Connect Serial';
    connectBtn.className   = 'btn ' + (on ? 'danger' : 'primary');
}

// ── Serial read loop ─────────────────────────────────────────────────
async function readLoop() {
    // pipeThrough is the idiomatic, lock-safe pattern for Web Serial
    reader = port.readable.pipeThrough(new TextDecoderStream()).getReader();

    try {
        while (true) {
            const { value, done } = await reader.read();
            if (done) break;
            lineBuffer += value;

            let nl;
            while ((nl = lineBuffer.indexOf('\n')) !== -1) {
                const line = lineBuffer.slice(0, nl).trim();
                lineBuffer  = lineBuffer.slice(nl + 1);
                if (line.startsWith('{')) {
                    try   { updateUI(JSON.parse(line)); }
                    catch (pe) { console.warn('JSON parse:', pe.message, line.slice(0, 60)); }
                }
            }
        }
    } catch (e) {
        if (e.name !== 'AbortError') console.warn('Serial read error:', e.message);
    } finally {
        reader = null;
        if (port) { port = null; setConnected(false); }
    }
}

// ── UI rendering ─────────────────────────────────────────────────────
// Panel de carga. `requested` es lo que pediste; `charging` lo que el firmware
// está mandando de verdad al OBC. Que difieran NO es un bug: significa que la
// seguridad o el TSON están bloqueando la petición, y es justo lo que interesa
// ver de un vistazo.
let chgCurrentTouched = false;   // no pisar el input mientras el usuario escribe
chgCurrentInput?.addEventListener('input', () => { chgCurrentTouched = true; });

function updateCharger(c) {
    chargePanel.hidden = false;

    let estado, cls;
    if (c.charging)       { estado = 'CARGANDO';         cls = 'ok';    }
    else if (c.requested) { estado = 'esperando permiso'; cls = 'warn'; }
    else                  { estado = 'parado';            cls = '';     }
    chgStateText.textContent = estado;
    chgStateText.className   = 'value mono ' + cls;

    chgTsonText.textContent = c.tson ? 'armado' : 'abierto';
    chgTsonText.className   = 'value mono ' + (c.tson ? 'ok' : 'warn');

    if (c.rxAlive) {
        chgVoutText.textContent = c.outV.toFixed(1) + ' V';
        chgIoutText.textContent = c.outI.toFixed(1) + ' A';
    } else {
        chgVoutText.textContent = 'sin OBC';
        chgIoutText.textContent = 'sin OBC';
    }

    // El tope lo manda el firmware: la UI no puede pedir más que el límite real.
    if (c.iMax !== undefined) {
        chgCurrentInput.max = c.iMax;
        chgCurrentHint.textContent = 'tope ' + c.iMax.toFixed(1) + ' A · Vmax '
                                   + (c.vSet !== undefined ? c.vSet.toFixed(0) : '?') + ' V';
    }
    if (!chgCurrentTouched) chgCurrentInput.value = c.i;
}

function updateUI(d) {
    // ── Hero row ──────────────────────────────────────────────────────
    const ok = d.bmsOk;
    bmsOkEl.textContent = ok ? 'HIGH (OK)' : 'LOW (FAULT)';
    bmsOkEl.className   = 'value bms-value ' + (ok ? 'ok' : 'fault');
    bmsCardEl.style.borderColor = ok ? 'rgba(16,185,129,0.35)' : 'rgba(239,68,68,0.5)';



    const i = d.i;
    currentText.textContent  = (i >= 0 ? '+' : '') + i.toFixed(2) + ' A';
    currentText.style.color  = Math.abs(i) > 150 ? 'var(--red)' : Math.abs(i) > 100 ? 'var(--yellow)' : 'var(--text)';
    fanText.textContent      = d.fan + '%';

    // ── Pack extremes ─────────────────────────────────────────────────
    vMinText.textContent   = d.vMin.toFixed(3) + ' V';
    vMaxText.textContent   = d.vMax.toFixed(3) + ' V';
    tMinText.textContent   = d.tMin.toFixed(1) + ' °C';
    tMaxText.textContent   = d.tMax.toFixed(1) + ' °C';
    vDeltaText.textContent = d.vDelta.toFixed(1) + ' mV';
    ntcOpenText.textContent = d.ntcOpen;

    colourV(vMinText, d.vMin);
    colourV(vMaxText, d.vMax);
    colourT(tMinText, d.tMin);
    colourT(tMaxText, d.tMax);

    // ── Charger ───────────────────────────────────────────────────────
    // El bloque "chg" solo lo publica charger.cpp, así que el panel aparece
    // únicamente cuando estás conectado al cargador (en main.cpp no existe).
    if (d.chg) updateCharger(d.chg);

    // ── Faults ────────────────────────────────────────────────────────
    const f = d.faults;
    setFault(faultEls.v,    f.v);
    setFault(faultEls.t,    f.t);
    setFault(faultEls.ntc,  f.ntc);
    setFault(faultEls.comm, f.comm);
    setFault(faultEls.hall, f.hall);
    setFault(faultEls.init, f.init);

    // ── Module cards ──────────────────────────────────────────────────
    renderModules(d.modules);
}

function colourV(el, v) {
    el.style.color = (v < UV || v > OV) ? 'var(--red)'
        : (v < UV + 0.03 || v > OV - 0.03) ? 'var(--yellow)' : 'var(--text)';
}
function colourT(el, t) {
    el.style.color = (t < UT || t > OT) ? 'var(--red)'
        : t > OT_WARN ? 'var(--yellow)' : 'var(--text)';
}
function setFault(el, active) {
    el.className = 'fault-badge ' + (active ? 'active' : 'ok');
}

// ── Module cards ─────────────────────────────────────────────────────
let lastModCount = 0;

function renderModules(modules) {
    if (!modules?.length) return;

    // Full rebuild only when number of modules changes
    if (modules.length !== lastModCount) {
        container.innerHTML = '';
        lastModCount = modules.length;
        modules.forEach((mod, m) => container.appendChild(buildCard(m, mod)));
    }

    modules.forEach((mod, m) => updateCard(m, mod));
}

function buildCard(m, mod) {
    const card = el('div', 'module-card', `mod-${m}`);

    const hdr  = el('div', 'module-header');
    hdr.appendChild(el('span', 'module-label', null, `Module ${m}`));
    hdr.appendChild(el('span', 'module-range', `mod-${m}-range`));
    card.appendChild(hdr);

    const vRow = el('div', 'cells-row', `mod-${m}-v`);
    (mod.v || []).forEach((_, n) => {
        const d = el('div', 'cell-dot cell-ok', `mod-${m}-v-${n}`, `c${n + 1}`);
        vRow.appendChild(d);
    });

    const tRow = el('div', 'cells-row', `mod-${m}-t`);
    tRow.style.marginTop = '4px';
    (mod.t || []).forEach((_, k) => {
        const d = el('div', 'cell-dot cell-ok', `mod-${m}-t-${k}`, `n${k + 1}`);
        tRow.appendChild(d);
    });

    card.appendChild(vRow);
    card.appendChild(tRow);
    applyTabVisibility(vRow, tRow);
    return card;
}

function updateCard(m, mod) {
    const vRow = document.getElementById(`mod-${m}-v`);
    const tRow = document.getElementById(`mod-${m}-t`);
    if (!vRow || !tRow) return;

    applyTabVisibility(vRow, tRow);

    let hasFault = false;
    let vMin = Infinity, vMax = -Infinity;

    (mod.v || []).forEach((v, n) => {
        const dot = document.getElementById(`mod-${m}-v-${n}`);
        if (!dot) return;
        if (v < vMin) vMin = v;
        if (v > vMax) vMax = v;
        let cls = 'cell-ok';
        if      (v <= 0.05)                       cls = 'cell-dim';
        else if (v < UV || v > OV)                { cls = 'cell-bad'; hasFault = true; }
        else if (v < UV + 0.03 || v > OV - 0.03) cls = 'cell-warn';
        dot.className = `cell-dot ${cls}`;
        dot.textContent = `C${n + 1}: ${v.toFixed(3)}V`;
    });

    (mod.t || []).forEach((t, k) => {
        const dot = document.getElementById(`mod-${m}-t-${k}`);
        if (!dot) return;
        let cls = 'cell-ok';
        if      (t < -90 || t > 150)  cls = 'cell-dim';
        else if (t < UT  || t > OT)   { cls = 'cell-bad'; hasFault = true; }
        else if (t > OT_WARN)          cls = 'cell-warn';
        dot.className = `cell-dot ${cls}`;
        dot.textContent = `T${k + 1}: ${t.toFixed(1)}°C`;
    });

    const card  = document.getElementById(`mod-${m}`);
    if (card)  card.className = 'module-card' + (hasFault ? ' has-fault' : '');

    const range = document.getElementById(`mod-${m}-range`);
    if (range && isFinite(vMin))
        range.textContent = `${vMin.toFixed(3)}–${vMax.toFixed(3)} V`;
}

function applyTabVisibility(vRow, tRow) {
    vRow.style.display = currentTab === 'v' ? 'flex' : 'none';
    tRow.style.display = currentTab === 't' ? 'flex' : 'none';
}

// ── Helper to create elements ─────────────────────────────────────────
function el(tag, cls, id, text) {
    const e = document.createElement(tag);
    if (cls)  e.className   = cls;
    if (id)   e.id          = id;
    if (text) e.textContent = text;
    return e;
}

// ── Tab switching ─────────────────────────────────────────────────────
window.switchTab = function(tab) {
    currentTab = tab;
    document.getElementById('tabV').className = 'tab' + (tab === 'v' ? ' active' : '');
    document.getElementById('tabT').className = 'tab' + (tab === 't' ? ' active' : '');
    document.querySelectorAll('[id^="mod-"]').forEach(e => {
        if (/^mod-\d+-v$/.test(e.id)) e.style.display = tab === 'v' ? 'flex' : 'none';
        if (/^mod-\d+-t$/.test(e.id)) e.style.display = tab === 't' ? 'flex' : 'none';
    });
};

// ── Graceful degradation ──────────────────────────────────────────────
if (!('serial' in navigator)) {
    connectBtn.disabled    = true;
    connectBtn.textContent = '🚫 Web Serial not supported';
    connectBtn.title       = 'Use Google Chrome or Microsoft Edge';
    document.querySelector('.info-box').insertAdjacentHTML(
        'beforeend',
        '<p style="color:var(--red);margin-top:.5rem">⚠ Web Serial API requires Chrome or Edge.</p>'
    );
}
