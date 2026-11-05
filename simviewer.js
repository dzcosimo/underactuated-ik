/* Simulation viewers: a row of buttons selects which video plays in the frame, and the caption follows. */
(function () {
  "use strict";
  document.querySelectorAll(".simviewer").forEach(function (root) {
    var video = root.querySelector("video"), cap = root.querySelector(".simcap");
    var btns = root.querySelectorAll("[data-sim]");
    function pick(b, play) {
      btns.forEach(function (x) { x.setAttribute("aria-pressed", x === b ? "true" : "false"); });
      video.poster = "sim_" + b.dataset.sim + ".jpg";
      video.src = "sim_" + b.dataset.sim + ".mp4";
      video.style.aspectRatio = b.dataset.ar || "2 / 1";
      cap.innerHTML = b.querySelector("template").innerHTML;
      if (window.MathJax && MathJax.typesetPromise) MathJax.typesetPromise([cap]);
      if (play) video.play().catch(function () {});
    }
    btns.forEach(function (b) { b.addEventListener("click", function () { pick(b, true); }); });
    pick(btns[0], false);
    video.addEventListener("click", function () { if (video.paused) video.play().catch(function () {}); else video.pause(); });
    if ("IntersectionObserver" in window && !matchMedia("(prefers-reduced-motion: reduce)").matches) {
      new IntersectionObserver(function (es) { es.forEach(function (e) { if (e.isIntersecting) video.play().catch(function () {}); else video.pause(); }); }, { threshold: 0.4 }).observe(video);
    }
  });
})();
