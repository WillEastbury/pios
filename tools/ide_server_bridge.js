/* ide_server_bridge.js -- injected by tools/gen_ide_assets.py directly before
 * </body> in the VENDORED PicoScript WebIDE (../picoscript/docs/index.html,
 * built by that repo's gen_site.py). This file is the ONLY place that wires the
 * real, upstream WebIDE to a running PIOS instance -- the vendored HTML/CSS/JS
 * above it is never hand-edited, so upstream regenerations
 * (`python ../picoscript/gen_site.py`) always drop in cleanly.
 *
 * PIOS has no external PicoWAL daemon and no PicoSTS: persistent records live in
 * the on-board capsule/WALFS store, reachable over the kernel's existing
 * core0-safe HTTP endpoints:
 *     /api/capsule?op=list|get|puthex|status&pack=&card=&offset=&hex=
 *     /api/walfs?path=&mode=dir|text|hex&offset=&max=
 *     /api/terminal?cmd=
 * So, unlike picoweb's bridge, this one adds NO OAuth and does NOT try to drive
 * the portal's built-in "live server" REST simulator (PIOS does not expose that
 * shape). Instead it:
 *   1. Loads /picoscript/config (endpoint prefixes + build/hook metadata).
 *   2. Adds a first-class "PicoWAL" portal tab whose same-origin iframe hosts
 *      the PIOS PicoWAL workspace (picowal.html) -- the real, live data view.
 *   3. Adds Deploy controls (Save/Load Source, Deploy Bytecode) into the
 *      WebIDE's existing Compile & Run controls row, mapped to /api/capsule.
 *
 * Everything is defensive: if a portal hook (showView, getSrc/setSrc, DBG,
 * filesStatus, the .tabs/.controls/.main containers) is missing after an
 * unrelated upstream refactor, the feature degrades to a no-op/alert instead of
 * throwing and breaking the rest of the page. */
(function () {
  "use strict";

  var IDE_PREFIX = "/picoscript/";
  var PWB = { config: null };
  window.PiosIdeBridge = PWB;

  function byId(id) { return document.getElementById(id); }
  function mk(tag, cls, text) {
    var el = document.createElement(tag);
    if (cls) el.className = cls;
    if (text !== undefined) el.textContent = text;
    return el;
  }
  function report(msg, isErr) {
    if (typeof filesStatus === "function") { try { filesStatus(msg, !!isErr); return; } catch (e) { /* fall through */ } }
    if (isErr) alert(msg); else if (typeof console !== "undefined") console.log(msg);
  }

  /* ---------- config ---------- */
  function loadConfig() {
    return fetch(IDE_PREFIX + "config", { credentials: "same-origin" })
      .then(function (r) { if (!r.ok) throw new Error("config " + r.status); return r.json(); })
      .then(function (c) { PWB.config = c; return c; })
      .catch(function (e) { PWB.configError = e.message; return null; });
  }
  function capsulePrefix() { return (PWB.config && PWB.config.capsule_prefix) || "/api/capsule"; }

  /* ---------- hex helpers ---------- */
  var HEX = "0123456789abcdef";
  function bytesToHex(bytes) {
    var s = "";
    for (var i = 0; i < bytes.length; i++) {
      var b = bytes[i] & 0xFF;
      s += HEX[b >> 4] + HEX[b & 0xF];
    }
    return s;
  }
  function hexToBytes(hex) {
    hex = (hex || "").replace(/[^0-9a-fA-F]/g, "");
    if (hex.length & 1) hex = hex.slice(0, hex.length - 1);
    var out = new Uint8Array(hex.length / 2);
    for (var i = 0; i < out.length; i++) out[i] = parseInt(hex.substr(i * 2, 2), 16);
    return out;
  }
  function utf8Bytes(str) {
    if (typeof TextEncoder !== "undefined") return new TextEncoder().encode(str);
    var utf = unescape(encodeURIComponent(str)), a = new Uint8Array(utf.length);
    for (var i = 0; i < utf.length; i++) a[i] = utf.charCodeAt(i);
    return a;
  }
  function utf8Decode(bytes) {
    if (typeof TextDecoder !== "undefined") return new TextDecoder().decode(bytes);
    var s = "";
    for (var i = 0; i < bytes.length; i++) s += String.fromCharCode(bytes[i]);
    try { return decodeURIComponent(escape(s)); } catch (e) { return s; }
  }

  /* ---------- capsule I/O (chunked; puthex hex arg <=2048 chars = 1024 bytes) ---------- */
  var PUT_CHUNK = 1024;   /* bytes per /api/capsule puthex call */
  var GET_CHUNK = 4096;   /* bytes per /api/capsule get call (max=4096) */
  function capsulePutBytes(pack, card, bytes) {
    var base = capsulePrefix();
    var chain = Promise.resolve();
    var total = bytes.length;
    for (var off = 0; off < total; off += PUT_CHUNK) {
      (function (offset) {
        chain = chain.then(function () {
          var slice = bytes.subarray(offset, Math.min(offset + PUT_CHUNK, total));
          var url = base + "?op=puthex&pack=" + pack + "&card=" + card +
                    "&offset=" + offset + "&hex=" + bytesToHex(slice);
          return fetch(url, { method: "GET", credentials: "same-origin" })
            .then(function (r) { return r.json(); })
            .then(function (j) { if (!j || j.ok !== true) throw new Error((j && j.error) || "put failed"); });
        });
      })(off);
    }
    return chain.then(function () { return total; });
  }
  function capsuleGetBytes(pack, card) {
    var base = capsulePrefix();
    var acc = [];
    function step(off) {
      var url = base + "?op=get&pack=" + pack + "&card=" + card + "&offset=" + off + "&max=" + GET_CHUNK;
      return fetch(url, { credentials: "same-origin" }).then(function (r) { return r.json(); })
        .then(function (j) {
          if (!j || j.ok !== true) throw new Error((j && j.error) || "get failed");
          var chunk = hexToBytes(j.data || "");
          acc.push(chunk);
          var next = off + chunk.length;
          if (j.truncated && chunk.length > 0 && next < j.totalBytes) return step(next);
          var out = new Uint8Array(next), p = 0;
          acc.forEach(function (c) { out.set(c, p); p += c.length; });
          return out.subarray(0, j.totalBytes || next);
        });
    }
    return step(0);
  }

  /* ---------- deploy controls in the WebIDE controls row ---------- */
  function askPackCard(action) {
    var pack = prompt(action + " -- capsule pack:", (localStorage.getItem("pios.ide.pack") || "0"));
    if (pack === null) return null;
    var card = prompt(action + " -- card:", (localStorage.getItem("pios.ide.card") || "0"));
    if (card === null) return null;
    try { localStorage.setItem("pios.ide.pack", pack); localStorage.setItem("pios.ide.card", card); } catch (e) { }
    return { pack: (pack | 0), card: (card | 0) };
  }
  function saveSource() {
    var pc = askPackCard("Save source"); if (!pc) return;
    var src = (typeof getSrc === "function") ? getSrc() : "";
    capsulePutBytes(pc.pack, pc.card, utf8Bytes(src))
      .then(function (n) { report("saved " + n + " bytes to capsule " + pc.pack + "/" + pc.card); })
      .catch(function (e) { report("save failed: " + e.message, true); });
  }
  function loadSource() {
    var pc = askPackCard("Load source"); if (!pc) return;
    capsuleGetBytes(pc.pack, pc.card)
      .then(function (bytes) {
        var text = utf8Decode(bytes);
        if (typeof setSrc === "function") setSrc(text);
        report("loaded " + bytes.length + " bytes from capsule " + pc.pack + "/" + pc.card);
      })
      .catch(function (e) { report("load failed: " + e.message, true); });
  }
  function deployBytecode() {
    var words = window.DBG && window.DBG.words;
    if (!words || !words.length) { report("Compile first -- no bytecode to deploy", true); return; }
    var pc = askPackCard("Deploy bytecode"); if (!pc) return;
    var buf = new Uint8Array(words.length * 4), dv = new DataView(buf.buffer);
    for (var i = 0; i < words.length; i++) dv.setUint32(i * 4, words[i] >>> 0, true);
    capsulePutBytes(pc.pack, pc.card, buf)
      .then(function (n) { report("deployed " + words.length + " words (" + n + " bytes) to capsule " + pc.pack + "/" + pc.card); })
      .catch(function (e) { report("deploy failed: " + e.message, true); });
  }
  function buildDeployControls() {
    var controls = document.querySelector("#view-play .controls") || document.querySelector(".controls");
    if (!controls || byId("pios-btn-save-source")) return;
    var sep = mk("span"); sep.style.cssText = "width:1px;align-self:stretch;background:#2c313f;margin:0 4px";
    controls.appendChild(sep);
    function addBtn(id, cls, label, title, fn) {
      var b = mk("button", cls, label); if (id) b.id = id; b.title = title;
      b.addEventListener("click", fn); controls.appendChild(b); return b;
    }
    addBtn("pios-btn-save-source", "ghost", "Save Source", "PUT active source to /api/capsule pack/card", saveSource);
    addBtn(null, "ghost", "Load Source", "GET source from /api/capsule pack/card", loadSource);
    addBtn("pios-btn-deploy", "act", "Deploy Bytecode", "PUT last compiled bytecode to /api/capsule pack/card", deployBytecode);
  }

  /* ---------- PicoWAL workspace tab ---------- */
  var baseShowView = window.showView;
  function ensurePicowalView() {
    if (byId("view-picowal")) return;
    var main = document.querySelector(".main"); if (!main) return;
    var view = mk("div", "view"); view.id = "view-picowal";
    view.style.cssText = "padding:0;overflow:hidden";
    var iframe = document.createElement("iframe");
    iframe.id = "picowalFrame"; iframe.title = "PicoWAL workspace";
    iframe.src = IDE_PREFIX + "picowal.html?embedded=1";
    iframe.style.cssText = "width:100%;height:100%;border:0;display:block;background:#16213e";
    view.appendChild(iframe); main.appendChild(view);
  }
  function installUnifiedShowView() {
    baseShowView = window.showView;
    window.showView = function (v) {
      var tabBtn = byId("pios-tab-picowal");
      if (v === "picowal") {
        ensurePicowalView();
        document.querySelectorAll(".view").forEach(function (e) { e.classList.remove("active"); });
        byId("view-picowal").classList.add("active");
        document.querySelectorAll(".tabs .tab").forEach(function (b) { b.classList.remove("active"); });
        if (tabBtn) tabBtn.classList.add("active");
        return;
      }
      if (typeof baseShowView === "function") baseShowView(v);
      if (tabBtn) tabBtn.classList.remove("active");
    };
  }
  function buildPicowalTab() {
    var tabs = document.querySelector(".tabs");
    if (!tabs || byId("pios-tab-picowal")) return;
    var btn = mk("button", "tab", "PicoWAL");
    btn.id = "pios-tab-picowal";
    btn.title = "Open the on-board PicoWAL workspace (WALFS + capsule store)";
    btn.addEventListener("click", function () { window.showView("picowal"); });
    tabs.appendChild(btn);
  }

  /* ---------- boot ---------- */
  loadConfig().then(function () {
    try { buildDeployControls(); } catch (e) { }
    try { installUnifiedShowView(); } catch (e) { }
    try { buildPicowalTab(); } catch (e) { }
  });
})();
