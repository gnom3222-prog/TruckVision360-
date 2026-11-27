'use strict';
const CACHE = 'truckvision360-v155';
const CORE = ['./', './index.html', './truckvision.js?v=20261127-1', './sensor-perception.js?v=20261126-2', './vehicle-physics.js?v=20261126-15', './articulated-planner.js?v=20261127-1', './planner-worker.js?v=20261127-1', './voice.js?v=20261123-4', './camera-geometry.js?v=20261126-5', './camera-measure.js?v=20261126-10', './manifest.webmanifest', './icon.svg'];
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
