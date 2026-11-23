'use strict';
const CACHE = 'truckvision360-v16';
const CORE = ['./', './index.html', './truckvision.js', './vehicle-physics.js', './voice.js', './camera-measure.js', './manifest.webmanifest', './icon.svg'];
self.addEventListener('install', event => {
  event.waitUntil(caches.open(CACHE).then(cache => cache.addAll(CORE)).then(() => self.skipWaiting()));
});
self.addEventListener('activate', event => {
  event.waitUntil(caches.keys().then(keys => Promise.all(keys.filter(key => key.startsWith('truckvision360-') && key !== CACHE).map(key => caches.delete(key)))).then(() => self.clients.claim()));
});
self.addEventListener('fetch', event => {
  const request = event.request;
  if (request.method !== 'GET' || new URL(request.url).origin !== self.location.origin) return;
  event.respondWith(caches.match(request, {ignoreSearch: true}).then(cached => {
    if (cached) return cached;
    return fetch(request).catch(() => request.mode === 'navigate' ? caches.match('./index.html') : Promise.reject(new Error('Offline resource unavailable')));
  }));
});
