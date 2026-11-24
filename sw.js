'use strict';
const CACHE = 'truckvision360-v80';
const CORE = ['./', './index.html', './truckvision.js', './sensor-perception.js', './vehicle-physics.js', './articulated-planner.js', './planner-worker.js', './voice.js', './camera-geometry.js', './camera-measure.js', './manifest.webmanifest', './icon.svg'];
self.addEventListener('install', event => {
  event.waitUntil(caches.open(CACHE).then(cache => cache.addAll(CORE)).then(() => self.skipWaiting()));
});
self.addEventListener('activate', event => {
  event.waitUntil(caches.keys().then(keys => Promise.all(keys.filter(key => key.startsWith('truckvision360-') && key !== CACHE).map(key => caches.delete(key)))).then(() => self.clients.claim()));
});
self.addEventListener('fetch', event => {
  const request = event.request;
  if (request.method !== 'GET' || new URL(request.url).origin !== self.location.origin) return;
  const url = new URL(request.url);
  const isAppCode = request.mode === 'navigate' || /\.(?:html|js)$/.test(url.pathname);
  if (isAppCode) {
    event.respondWith(fetch(request).then(response => {
      if (response && response.ok) {
        const copy = response.clone();
        caches.open(CACHE).then(cache => cache.put(request, copy));
      }
      return response;
    }).catch(() => caches.match(request, {ignoreSearch: true}).then(cached =>
      cached || (request.mode === 'navigate' ? caches.match('./index.html') : Promise.reject(new Error('Offline resource unavailable')))
    )));
  } else {
    event.respondWith(caches.match(request, {ignoreSearch: true}).then(cached => {
      if (cached) return cached;
      return fetch(request).then(response => {
        if (response && response.ok) {
          const copy = response.clone();
          caches.open(CACHE).then(cache => cache.put(request, copy));
        }
        return response;
      });
    }));
  }
});
