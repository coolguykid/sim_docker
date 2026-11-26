'use strict';

const $ = id => document.getElementById(id);
const palette = ['#18735c', '#8b55ad'];
let rounds = [[], []], request = 0, playback = null;
const format = (value, digits) => value == null ? '—' : value.toLocaleString('zh-CN', {
  maximumFractionDigits: digits,
});

function draw(canvas, field, time) {
  const w = canvas.clientWidth, h = canvas.clientHeight, dpr = window.devicePixelRatio || 1;
  canvas.width = w * dpr;
  canvas.height = h * dpr;
  const ctx = canvas.getContext('2d');
  ctx.scale(dpr, dpr);
  const left = 52, top = 12, right = w - 16, bottom = h - 30;
  const maxTime = Number($('seek').max) || 1;
  const high = Math.max(1, ...rounds.flatMap(points => points.map(point => point[field] ?? 0)));
  const unit = 10 ** Math.floor(Math.log10(high));
  const maxY = Math.ceil(high / unit * 2) * unit / 2;
  const x = t => left + t / maxTime * (right - left);
  const y = v => bottom - v / maxY * (bottom - top);
  ctx.font = '12px system-ui';
  ctx.fillStyle = '#45655b';
  for (let i = 0; i <= 4; i++) {
    const v = maxY * i / 4;
    ctx.textAlign = 'right';
    ctx.fillText(format(v, 1), left - 6, y(v) + 4);
    ctx.strokeStyle = '#e2e9e5';
    ctx.beginPath(); ctx.moveTo(left, y(v)); ctx.lineTo(right, y(v)); ctx.stroke();
    const t = Math.round(maxTime * i / 4);
    ctx.textAlign = 'center';
    ctx.fillText(t + ' s', x(t), bottom + 20);
  }
  for (const [index, points] of rounds.entries()) {
    ctx.strokeStyle = palette[index];
    ctx.lineWidth = 2;
    ctx.beginPath();
    let connected = false;
    for (const point of points) {
      const value = point[field];
      if (value == null) { connected = false; continue; }
      if (connected) ctx.lineTo(x(point[0]), y(value));
      else ctx.moveTo(x(point[0]), y(value));
      connected = true;
    }
    ctx.stroke();
  }
  ctx.strokeStyle = '#879991'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(x(time), top); ctx.lineTo(x(time), bottom); ctx.stroke();
}

function render() {
  const time = Number($('seek').value);
  $('seconds').textContent = time + ' s';
  for (const [index, points] of rounds.entries()) {
    const point = points.find(item => item[0] === time);
    $('count' + index).textContent = format(point?.[1], 0);
    $('speed' + index).textContent = format(point?.[2], 2);
  }
  draw($('count'), 1, time);
  draw($('speed'), 2, time);
}

async function get(path) {
  const response = await fetch(path);
  if (!response.ok) throw new Error('HTTP ' + response.status);
  return response.json();
}

async function load() {
  const id = ++request;
  const selection = $('run').value;
  $('status').textContent = '正在读取推演帧…';
  try {
    const runs = await get('/api/runs');
    if (id !== request) return;
    if (!runs.length) throw new Error('暂无推演输出');
    $('run').replaceChildren(...runs.map(run => new Option(run)));
    $('run').value = runs.includes(selection) ? selection : runs[0];
    const query = new URLSearchParams({run: $('run').value, region: $('region').value});
    const data = await get('/api/comparison?' + query);
    if (id !== request) return;
    rounds = data;
    const ends = rounds.map(points => points.at(-1)?.[0]);
    $('seek').max = Math.max(...ends.map(end => end ?? 0));
    $('status').textContent = ends.map((end, index) =>
      (index === 0 ? '调整前' : '调整后') + (end == null ? '暂无完整帧' : '可回放至 ' + end + ' s')
    ).join(' · ');
    render();
  } catch (error) {
    if (id === request) $('status').textContent = '读取失败 · ' + error.message;
  }
}

for (let region = 1; region <= 35; region++) {
  $('region').add(new Option('R' + String(region).padStart(2, '0'), String(region)));
}
$('run').onchange = $('region').onchange = $('refresh').onclick = load;
$('seek').oninput = render;
$('play').onclick = () => {
  if (playback !== null) {
    clearInterval(playback);
    playback = null;
    $('play').textContent = '播放';
    return;
  }
  const seek = $('seek');
  if (+seek.max === 0) return;
  if (+seek.value >= +seek.max) seek.value = 0;
  $('play').textContent = '暂停';
  playback = setInterval(() => {
    seek.value = Number(seek.value) + 1;
    render();
    if (+seek.value >= +seek.max) $('play').click();
  }, 125);
};
new ResizeObserver(render).observe($('count'));
render();
load();
