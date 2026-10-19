// Polls the board's /api endpoints: the ADC scope a few times a second, status once a second.

const $ = (id) => document.getElementById(id);
const canvas = $('scope');
const g = canvas.getContext('2d');

function drawScope(samples) {
  const W = canvas.width, H = canvas.height;
  g.fillStyle = '#000';
  g.fillRect(0, 0, W, H);

  let min = 4095, max = 0;
  for (const v of samples) {
    if (v < min) min = v;
    if (v > max) max = v;
  }

  // 12-bit ADC codes; auto-scale zooms in on whatever is there
  let lo = 0, hi = 4095;
  if ($('auto').checked) {
    const margin = Math.max(16, (max - min) * 0.1);
    lo = min - margin;
    hi = max + margin;
  }
  const y = (v) => H - (v - lo) / (hi - lo) * H;

  g.strokeStyle = '#333';
  g.beginPath();
  for (const v of [0, 1024, 2048, 3072, 4095]) {
    g.moveTo(0, y(v));
    g.lineTo(W, y(v));
  }
  g.stroke();

  g.strokeStyle = '#5f5';
  g.beginPath();
  samples.forEach((v, i) => {
    const x = i * W / (samples.length - 1);
    if (i) g.lineTo(x, y(v)); else g.moveTo(x, y(v));
  });
  g.stroke();

  const clipping = min === 0 || max === 4095 ? '  CLIPPING' : '';
  $('scope-info').textContent = `min ${min} max ${max} p-p ${max - min}${clipping}`;
}

async function pollScope() {
  if ($('run').checked) {
    try {
      const r = await fetch('/api/scope?n=' + $('n').value);
      drawScope(new Uint16Array(await r.arrayBuffer()));
    } catch (e) {
      // board busy or unreachable, try again next time
    }
  }
  setTimeout(pollScope, 300);
}

const escapeHtml = (s) => s.replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));

async function pollStatus() {
  try {
    const s = await (await fetch('/api/status')).json();
    $('status').textContent =
      `up ${s.uptime}s  wifi ${s.rssi} dBm  ADC ${s.rate.toFixed(0)} S/s  ` +
      `range ${s.adc_min}..${s.adc_max}  mean ${s.mean.toFixed(0)}  rms ${s.rms.toFixed(1)}\n` +
      `tones mark ${s.mark.toFixed(3)} space ${s.space.toFixed(3)} FS  flags/s ${s.flags}  ` +
      `DCD ${s.dcd ? 'ON' : 'off'} (${s.dcd_ms} ms/s)  KISS clients ${s.kiss}  ${s.tx ? 'TRANSMITTING' : ''}\n` +
      `frames ok ${s.ok}  bad CRC ${s.bad}  sent ${s.sent}  ADC overruns ${s.overruns}`;

    const rows = s.frames.map((f) =>
      `<tr class="${f.ok ? '' : 'bad'}"><td>${f.age}s</td><td>${f.ok ? 'ok' : 'BAD'}</td><td>${f.len}</td>` +
      `<td>${f.mark.toFixed(2)}/${f.space.toFixed(2)}</td><td>${escapeHtml(f.text)}</td></tr>`);
    $('frames').innerHTML = '<tr><td>age</td><td>crc</td><td>len</td><td>mark/space</td><td>data</td></tr>' + rows.join('');
  } catch (e) {
    $('status').textContent = 'no connection';
  }
  setTimeout(pollStatus, 1000);
}

$('send').onsubmit = async (e) => {
  e.preventDefault();
  const r = await fetch('/api/send', { method: 'POST', body: $('msg').value });
  $('send-result').textContent = await r.text();
};

pollScope();
pollStatus();
