/* Shared chart reading tools. Reuse the live chart so values and clock stay identical. */
(function () {
  'use strict';
  const icon = '<svg class="dt-icon" viewBox="0 0 24 24" aria-hidden="true"><path d="M9 3H3v6m12-6h6v6M3 15v6h6m12-6v6h-6M8 8l-5-5m13 5 5-5M8 16l-5 5m13-5 5 5"/></svg>';
  const dialog = document.createElement('dialog');
  dialog.id = 'dtChartDialog';
  dialog.className = 'dt-chart-dialog';
  dialog.setAttribute('aria-labelledby', 'dtFocusTitle');
  dialog.setAttribute('aria-describedby', 'dtFocusDescription');
  dialog.innerHTML = '<div class="dt-focus-header"><h2 id="dtFocusTitle">图表详情</h2><button type="button" id="dtFocusClose">关闭 · Esc</button></div><p class="dt-focus-description" id="dtFocusDescription">与当前页面使用相同的数据与时间。关闭后返回原位置。</p><div class="dt-focus-body"></div>';
  document.body.append(dialog);
  let expanded = null;
  function redraw() {
    window.dispatchEvent(new Event('resize'));
    if (typeof S !== 'undefined' && S.view === 'center') window.CenterDemo?.resize();
    if (typeof S !== 'undefined' && S.view === 'region') window.RegionDemo?.resize();
  }
  function restore() {
    if (!expanded) return;
    const {target, placeholder, trigger} = expanded;
    placeholder.replaceWith(target);
    target.classList.remove('dt-chart-expanded');
    expanded = null;
    document.body.classList.remove('dt-focusing');
    requestAnimationFrame(() => { redraw(); trigger.focus({preventScroll: true}); });
  }
  dialog.addEventListener('close', restore);
  dialog.addEventListener('cancel', e => { e.stopImmediatePropagation(); });
  document.addEventListener('keydown', e => { if (dialog.open && e.key === 'Escape') { e.preventDefault(); e.stopImmediatePropagation(); dialog.close(); } }, true);
  dialog.querySelector('#dtFocusClose').addEventListener('click', () => dialog.close());
  dialog.addEventListener('click', e => { if (e.target === dialog) { const r = dialog.getBoundingClientRect(); if (e.clientX < r.left || e.clientX > r.right || e.clientY < r.top || e.clientY > r.bottom) dialog.close(); } });
  function open(target, title, trigger) {
    if (expanded) return;
    const placeholder = document.createElement('div');
    placeholder.className = 'dt-focus-placeholder';
    placeholder.style.height = target.getBoundingClientRect().height + 'px';
    target.replaceWith(placeholder);
    dialog.querySelector('.dt-focus-body').append(target);
    target.classList.add('dt-chart-expanded');
    dialog.querySelector('#dtFocusTitle').textContent = title;
    expanded = {target, placeholder, trigger};
    document.body.classList.add('dt-focusing');
    dialog.showModal();
    requestAnimationFrame(redraw);
  }
  function enhance(root) {
    if (!root) return;
    const containers = root.querySelectorAll('.cd-mini-chart, .cd-chart, .rd-eval-plot, .rd-history, .rd-mini-chart');
    for (const target of containers) {
      if (target.dataset.dtEnhanced || !target.querySelector('canvas')) continue;
      const figure = target.closest('figure');
      const panel = target.closest('.cd-card, .panel');
      const heading = figure?.querySelector('figcaption') || panel?.querySelector('.cd-head, .rd-head');
      if (!heading) continue;
      target.dataset.dtEnhanced = 'true';
      const title = (figure?.querySelector('figcaption')?.textContent || panel?.querySelector('h3, .panel-title')?.textContent || '图表详情').trim();
      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'dt-chart-zoom';
      button.setAttribute('aria-label', '放大' + title);
      button.title = '放大查看图表';
      button.innerHTML = icon + '<span>放大</span>';
      heading.classList.add('dt-chart-heading');
      heading.append(button);
      button.addEventListener('click', () => open(target, title, button));
    }
  }
  function syncNavigation() {
    for (const button of document.querySelectorAll('nav [data-view]')) {
      if (button.classList.contains('active')) button.setAttribute('aria-current', 'page');
      else button.removeAttribute('aria-current');
    }
  }
  for (const id of ['centerDemo', 'regionalDemo']) {
    const root = document.getElementById(id);
    enhance(root);
    new MutationObserver(() => { enhance(root); syncNavigation(); }).observe(root, {childList: true, attributes: true, attributeFilter: ['hidden']});
  }
  for (const button of document.querySelectorAll('nav [data-view]')) new MutationObserver(syncNavigation).observe(button, {attributes: true, attributeFilter: ['class']});
  syncNavigation();
  window.DTDesktop = {enhance};
})();
