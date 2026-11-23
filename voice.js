/* TruckVision 360 — standalone Russian voice assistant */
(function () {
  'use strict';

  if (window.__truckVisionVoiceLoaded) return;
  window.__truckVisionVoiceLoaded = true;

  function initVoiceAssistant() {
    if (document.getElementById('truckVisionVoicePanel')) return;

    const panel = document.createElement('section');
    panel.id = 'truckVisionVoicePanel';
    panel.setAttribute('aria-label', 'Голосовой помощник TruckVision 360');
    panel.style.cssText = [
      'box-sizing:border-box',
      'width:calc(100% - 24px)',
      'max-width:900px',
      'margin:16px auto 24px',
      'padding:16px',
      'border:1px solid #3983a8',
      'border-radius:16px',
      'background:#142b40',
      'color:#f3f7fb',
      'font:16px system-ui,-apple-system,sans-serif',
      'position:relative',
      'z-index:10'
    ].join(';');

    panel.innerHTML = `
      <div style="font-size:19px;font-weight:700;margin-bottom:10px">
        🔊 Голосовой помощник TruckVision 360
      </div>
      <div style="display:flex;gap:10px;flex-wrap:wrap">
        <button id="tvVoiceToggle" type="button"
          style="border:0;border-radius:10px;padding:12px 16px;background:#0789ad;color:white;font-weight:700;font-size:15px">
          Включить голос
        </button>
        <button id="tvVoiceTest" type="button"
          style="border:1px solid #6b8ba5;border-radius:10px;padding:12px 16px;background:#203e58;color:white;font-weight:700;font-size:15px">
          Проверить голос
        </button>
      </div>
      <p id="tvVoiceStatus" aria-live="polite" style="margin:10px 0 0;color:#b8c9d8">
        Голос выключен. Нажми «Включить голос», затем «Проверить голос».
      </p>
      <p id="tvVoiceAdvice" aria-live="polite" style="margin:8px 0 0;color:#83d6a8">
        Ожидание подсказки симулятора.
      </p>
    `;

    const main = document.querySelector('main');
    if (main && main.parentNode) main.insertAdjacentElement('afterend', panel);
    else document.body.appendChild(panel);

    const toggle = panel.querySelector('#tvVoiceToggle');
    const test = panel.querySelector('#tvVoiceTest');
    const status = panel.querySelector('#tvVoiceStatus');
    const adviceOutput = panel.querySelector('#tvVoiceAdvice');
    let enabled = false;
    let lastMessage = '';
    let lastSpokenAt = 0;

    function supported() {
      return 'speechSynthesis' in window &&
        typeof window.SpeechSynthesisUtterance === 'function';
    }

    function speak(message, force) {
      if (!enabled || !message || !supported()) return;
      const now = Date.now();
      if (!force && message === lastMessage) return;
      if (!force && now - lastSpokenAt < 2500) return;

      lastMessage = message;
      lastSpokenAt = now;
      window.speechSynthesis.cancel();

      const utterance = new window.SpeechSynthesisUtterance(message);
      utterance.lang = 'ru-RU';
      utterance.rate = 0.92;
      utterance.pitch = 1;
      utterance.volume = 1;
      utterance.onerror = function () {
        status.textContent = 'Не удалось воспроизвести речь. Проверь звук и настройки Safari.';
      };
      window.speechSynthesis.speak(utterance);
    }

    toggle.addEventListener('click', function () {
      if (!supported()) {
        status.textContent = 'Синтез речи недоступен в этом браузере.';
        return;
      }

      enabled = !enabled;
      toggle.textContent = enabled ? 'Выключить голос' : 'Включить голос';
      status.textContent = enabled ? 'Голос включён.' : 'Голос выключен.';

      if (!enabled) {
        window.speechSynthesis.cancel();
        lastMessage = '';
        return;
      }

      // The greeting is started directly from the user's tap (required by iOS Safari).
      const greeting = new window.SpeechSynthesisUtterance(
        'Голосовой помощник TruckVision 360 включён.'
      );
      greeting.lang = 'ru-RU';
      greeting.rate = 0.92;
      window.speechSynthesis.cancel();
      window.speechSynthesis.speak(greeting);
    });

    test.addEventListener('click', function () {
      if (!enabled) {
        status.textContent = 'Сначала нажми «Включить голос».';
        return;
      }
      lastMessage = '';
      lastSpokenAt = 0;
      speak('Проверка голосовой системы. TruckVision 360 готов к работе.', true);
    });

    function syncAdvice() {
      const advice = document.getElementById('assistantAdvice');
      if (!advice) return;
      const message = (advice.textContent || '').trim();
      if (!message) return;
      adviceOutput.textContent = message;
      speak(message, false);
    }

    const advice = document.getElementById('assistantAdvice');
    if (advice) {
      new MutationObserver(syncAdvice).observe(advice, {
        childList: true,
        subtree: true,
        characterData: true
      });
      syncAdvice();
    } else {
      status.textContent = 'Панель голосового помощника загружена; подсказки симулятора на этой странице не найдены.';
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initVoiceAssistant, { once: true });
  } else {
    initVoiceAssistant();
  }
})();
