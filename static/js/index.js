(function () {
  'use strict';

  var mq = window.matchMedia('(max-width: 768px)');
  function isMobile() { return mq.matches; }
  function clamp(v, a, b) { return Math.max(a, Math.min(b, v)); }
  function ease(t) { return t < .5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2; }
  function tryPlay(v) { if (!v) return; var p = v.play(); if (p && p.catch) p.catch(function () {}); }
  function loadVideo(v) {
    if (!v || v.dataset.loaded) return;
    v.src = v.getAttribute('data-src'); v.dataset.loaded = '1'; v.load();
  }

  /* =====================================================================
     Hero: sticky stage. Scrolling the first ~70vh shrinks the full-bleed
     video into a card and reveals the neighbouring slides; after that the
     stage scrolls away like any other section. Left/right, swipe, dots and
     arrow keys move between the four runs.
     ===================================================================== */
  var stage = document.querySelector('.hero-stage');
  var sticky = document.querySelector('.hero-sticky');
  var slider = document.getElementById('hero-slider');
  var copy = document.getElementById('hero-copy');
  var slides = slider ? Array.prototype.slice.call(slider.querySelectorAll('.slide')) : [];
  var dotsWrap = document.getElementById('slider-dots');
  var captionEl = document.getElementById('slide-caption');
  var prevBtn = slider ? slider.querySelector('.slider-nav.prev') : null;
  var nextBtn = slider ? slider.querySelector('.slider-nav.next') : null;
  var GAP = 28;
  var idx = 0, progress = 0, ticking = false;

  if (stage && slider && slides.length) {
    slides.forEach(function (s, i) {
      var b = document.createElement('button');
      b.type = 'button'; b.setAttribute('role', 'tab');
      b.setAttribute('aria-label', s.getAttribute('data-label'));
      b.addEventListener('click', function () { go(i); });
      dotsWrap.appendChild(b);
      s.addEventListener('click', function () { if (i !== idx) go(i); });
    });
    // when a run finishes, move on to the next one (wrapping back to the first)
    slides.forEach(function (s, i) {
      var v = s.querySelector('video');
      if (v) v.addEventListener('ended', function () { if (i === idx) go(idx + 1); });
    });
    if (prevBtn) prevBtn.addEventListener('click', function () { go(idx - 1); });
    if (nextBtn) nextBtn.addEventListener('click', function () { go(idx + 1); });

    // swipe (pointer events; vertical scrolling stays native via touch-action: pan-y)
    var px = null, py = null;
    slider.addEventListener('pointerdown', function (e) { px = e.clientX; py = e.clientY; }, { passive: true });
    slider.addEventListener('pointerup', function (e) {
      if (px === null) return;
      var dx = e.clientX - px, dy = e.clientY - py; px = py = null;
      if (Math.abs(dx) > 48 && Math.abs(dx) > Math.abs(dy) * 1.3) go(idx + (dx < 0 ? 1 : -1));
    }, { passive: true });
    slider.addEventListener('pointercancel', function () { px = py = null; });

    // arrow keys while the stage is on screen
    window.addEventListener('keydown', function (e) {
      if (e.target && /^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName)) return;
      var r = stage.getBoundingClientRect();
      if (r.bottom < 0 || r.top > window.innerHeight) return;
      if (e.key === 'ArrowRight') { e.preventDefault(); go(idx + 1); }
      if (e.key === 'ArrowLeft') { e.preventDefault(); go(idx - 1); }
    });

    function geom() {
      var vw = document.documentElement.clientWidth;
      var vh = sticky.clientHeight;
      var mob = isMobile();
      var areaH = mob ? slider.clientHeight : vh;
      var W0 = vw, H0 = mob ? Math.min(vw * 0.75, areaH) : vh;
      var W1 = Math.min(960, vw - 40), H1 = W1 * 0.75;   // same 960px column as the rest of the page
      var maxH = areaH * (mob ? 0.9 : 0.82);   // lets the card reach the full 960px column on a 900px-tall window
      if (H1 > maxH) { H1 = maxH; W1 = H1 / 0.75; }
      return { W0: W0, H0: H0, W1: W1, H1: H1 };
    }

    function render() {
      var g = geom();
      var e = ease(progress);
      var W = g.W0 + (g.W1 - g.W0) * e, H = g.H0 + (g.H1 - g.H0) * e;
      var collapsed = progress > 0.92;

      slider.style.setProperty('--w1', g.W1 + 'px');
      slider.style.setProperty('--w', W + 'px');
      slider.style.setProperty('--h', H + 'px');
      slider.style.setProperty('--ui', String(clamp((progress - 0.35) / 0.55, 0, 1)));
      // arrows fade in faster than the rest, so they are legible the moment they become clickable
      slider.style.setProperty('--ui-nav', String(clamp((progress - 0.34) / 0.16, 0, 1)));
      slider.classList.toggle('is-collapsed', collapsed);
      // arrows, dots and caption become clickable as soon as they start fading in
      slider.classList.toggle('is-nav-ready', progress > 0.38);

      slides.forEach(function (s, i) {
        var d = i - idx;
        var v = s.querySelector('video');
        if (d === 0) {
          s.style.setProperty('--w', W + 'px');
          s.style.setProperty('--h', H + 'px');
          s.style.setProperty('--x', '0px');
          s.style.setProperty('--s', '1');
          s.style.setProperty('--o', '1');
          s.style.setProperty('--r', (14 * e) + 'px');
          s.style.setProperty('--shade', String(1 - e));
          s.classList.add('is-active');
          if (v) { if (collapsed) v.setAttribute('controls', ''); else v.removeAttribute('controls'); }
        } else {
          var side = d < 0 ? -1 : 1;
          s.style.setProperty('--w', g.W1 + 'px');
          s.style.setProperty('--h', g.H1 + 'px');
          s.style.setProperty('--x', (d * (g.W1 + GAP) + side * (1 - e) * 160) + 'px');
          s.style.setProperty('--s', '0.94');
          s.style.setProperty('--o', String(Math.abs(d) === 1 ? e * 0.75 : 0));
          s.style.setProperty('--r', '14px');
          s.classList.remove('is-active');
          if (v) v.removeAttribute('controls');
        }
      });

      // title fades out as the hero collapses (desktop only; phones keep the title above the card)
      copy.style.setProperty('--copy', isMobile() ? '1' : String(clamp(1 - progress * 1.8, 0, 1)));

      // preload neighbours once the stage starts collapsing
      if (progress > 0.3) {
        [idx - 1, idx + 1].forEach(function (k) { if (slides[k]) loadVideo(slides[k].querySelector('video')); });
      }

    }

    function updateMeta() {
      var s = slides[idx];
      captionEl.innerHTML = '<strong>' + s.getAttribute('data-label') + '.</strong> ' + s.getAttribute('data-caption');
      Array.prototype.forEach.call(dotsWrap.children, function (b, i) {
        b.classList.toggle('is-active', i === idx);
        b.setAttribute('aria-selected', i === idx ? 'true' : 'false');
      });
    }

    function go(i) {
      i = (i + slides.length) % slides.length;   // wraps: last -> first
      if (i === idx) return;
      var oldV = slides[idx].querySelector('video');
      if (oldV) oldV.pause();
      idx = i;
      var v = slides[idx].querySelector('video');
      loadVideo(v);
      try { v.currentTime = 0; } catch (e) {}
      tryPlay(v);
      updateMeta(); render();
    }


    function onScroll() {
      if (ticking) return;
      ticking = true;
      requestAnimationFrame(function () {
        // phones skip the scroll-collapse: the slider is shown in its card state right away
        if (isMobile()) {
          progress = 1;
        } else {
          var collapse = stage.offsetHeight - sticky.clientHeight;
          progress = clamp((-stage.getBoundingClientRect().top) / Math.max(1, collapse), 0, 1);
        }
        render();
        ticking = false;
      });
    }

    window.addEventListener('scroll', onScroll, { passive: true });
    window.addEventListener('resize', onScroll);
    if (mq.addEventListener) mq.addEventListener('change', onScroll);

    var first = slides[0].querySelector('video');
    loadVideo(first); tryPlay(first);
    updateMeta(); onScroll();

    // some browsers only allow playback after the first interaction; retry once then
    function kick() {
      var v = slides[idx].querySelector('video');
      if (v && v.paused) tryPlay(v);
      ['scroll', 'touchstart', 'click', 'keydown'].forEach(function (ev) { window.removeEventListener(ev, kick); });
    }
    ['scroll', 'touchstart', 'click', 'keydown'].forEach(function (ev) { window.addEventListener(ev, kick, { passive: true }); });

    // pause the hero video once the stage is off screen, resume when it returns
    if ('IntersectionObserver' in window) {
      new IntersectionObserver(function (entries) {
        var v = slides[idx].querySelector('video');
        if (!v) return;
        if (entries[0].isIntersecting) tryPlay(v); else v.pause();
      }, { threshold: 0.05 }).observe(stage);
    }
  }

  /* ---------- Lazy videos elsewhere: load on approach, play when visible ---------- */
  var lazyVideos = Array.prototype.slice.call(document.querySelectorAll('video.lazy-video'));
  if ('IntersectionObserver' in window) {
    var loader = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) { if (e.isIntersecting) { loadVideo(e.target); loader.unobserve(e.target); } });
    }, { rootMargin: '400px 0px' });
    var player = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) {
        if (e.isIntersecting) { loadVideo(e.target); tryPlay(e.target); } else { e.target.pause(); }
      });
    }, { threshold: 0.25 });
    lazyVideos.forEach(function (v) { loader.observe(v); player.observe(v); });
  } else {
    lazyVideos.forEach(function (v) { loadVideo(v); tryPlay(v); });
  }

  /* ---------- Copy BibTeX ---------- */
  var copyBtn = document.getElementById('copy-bib');
  var bib = document.getElementById('bib-text');
  if (copyBtn && bib) {
    copyBtn.addEventListener('click', function () {
      var text = bib.innerText;
      var done = function () {
        copyBtn.innerHTML = '<i class="fa-solid fa-check"></i> Copied';
        setTimeout(function () { copyBtn.innerHTML = '<i class="fa-regular fa-copy"></i> Copy'; }, 1800);
      };
      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(text).then(done, done);
      } else {
        var range = document.createRange(); range.selectNodeContents(bib);
        var sel = window.getSelection(); sel.removeAllRanges(); sel.addRange(range);
        try { document.execCommand('copy'); } catch (e) {}
        sel.removeAllRanges(); done();
      }
    });
  }
})();
