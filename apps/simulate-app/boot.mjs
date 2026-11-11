// Trunk's initializer hook: shows the simulator's download on the page's
// progress surface. Trunk takes the hook as a module file. From its first
// line the app drives the same elements, so nothing here outlives start-up.
export default function bootProgress() {
  const root = document.getElementById("boot");
  const phase = document.getElementById("boot-phase");
  const bar = document.getElementById("boot-bar");
  const fill = document.getElementById("boot-fill");
  const detail = document.getElementById("boot-detail");
  const megabytes = (bytes) => (bytes / 1048576).toFixed(1);
  // The app reads the link in the address once, at start: another fragment
  // is another simulation.
  window.addEventListener("hashchange", () => window.location.reload());
  return {
    onStart() {
      phase.textContent = "Downloading the simulator";
    },
    // `current` counts decoded bytes. Trunk measures `total` before it
    // optimises the wasm, so it bounds the size from above: the bar uses it,
    // the text does not claim it.
    onProgress({ current, total }) {
      detail.textContent = `${megabytes(current)} MB`;
      if (total > 0 && current < total) {
        bar.removeAttribute("data-unknown");
        fill.style.width = `${(100 * current) / total}%`;
        return;
      }
      if (!(total > 0)) {
        bar.setAttribute("data-unknown", "");
        return;
      }
      // The browser compiles while it downloads; what remains has no measure.
      fill.style.width = "";
      bar.setAttribute("data-unknown", "");
      phase.textContent = "Compiling and starting the simulator";
      detail.textContent = "";
    },
    onFailure(error) {
      root.setAttribute("data-state", "failed");
      phase.textContent = "The simulator could not be loaded";
      detail.textContent = String(error);
    },
  };
}
