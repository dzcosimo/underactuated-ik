/* Simulation viewers: a row of buttons selects which video plays in the frame, and the caption follows. */
(function () {
  "use strict";
  document.querySelectorAll(".simviewer").forEach(function (root) {
    var video = root.querySelector("video"), cap = root.querySelector(".simcap");
    var btns = root.querySelectorAll("[data-sim]");
    function pick(b, play) {
      btns.forEach(function (x) { x.setAttribute("aria-pressed", x === b ? "true" : "false"); });
      var m = matchMedia("(max-width: 600px)").matches ? "_m" : "";
      video.poster = "sim_" + b.dataset.sim + m + ".jpg";
      video.src = "sim_" + b.dataset.sim + m + ".mp4";
      video.style.aspectRatio = m ? "1 / 1" : (b.dataset.ar || "2 / 1");
      cap.innerHTML = b.querySelector("template").innerHTML;
      if (window.MathJax && MathJax.typesetPromise) MathJax.typesetPromise([cap]);
      if (play) video.play().catch(function () {});
    }
    btns.forEach(function (b) { b.addEventListener("click", function () { pick(b, true); }); });
    pick(btns[0], false);
    var cur = function () { return root.querySelector("[data-sim][aria-pressed=true]"); };
    matchMedia("(max-width: 600px)").addEventListener("change", function () { pick(cur(), !video.paused); });
    video.addEventListener("click", function () { if (video.paused) video.play().catch(function () {}); else video.pause(); });
    if ("IntersectionObserver" in window && !matchMedia("(prefers-reduced-motion: reduce)").matches) {
      new IntersectionObserver(function (es) { es.forEach(function (e) { if (e.isIntersecting) video.play().catch(function () {}); else video.pause(); }); }, { threshold: 0.4 }).observe(video);
    }
  });
})();
