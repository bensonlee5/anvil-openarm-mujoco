import "./styles.css";

import { DEMOS, getDemo, type DemoDefinition } from "./demos";
import { ViewerApp } from "./simulator";

const queriedRoot = document.querySelector<HTMLDivElement>("#app");
if (!queriedRoot) {
  throw new Error("missing #app root");
}
const appRoot: HTMLDivElement = queriedRoot;

let activeViewer: ViewerApp | undefined;

function renderSplash(): void {
  activeViewer?.dispose();
  activeViewer = undefined;

  appRoot.innerHTML = "";
  const splash = document.createElement("main");
  splash.className = "splash";

  const intro = document.createElement("section");
  intro.className = "splash__intro";
  intro.innerHTML = `
    <div>
      <div class="brand">
        <div class="brand__mark">AO</div>
        <span>Anvil OpenARM MuJoCo</span>
      </div>
      <h1>Hosted robot demos</h1>
      <p class="splash__copy">
        Select a browser-based MuJoCo scene, then operate either arm with joint-space keyboard teleop or the actuator sliders.
      </p>
    </div>
    <figure class="splash__media">
      <img src="${import.meta.env.BASE_URL}sim-assets/anvil_openarm/preview.png" alt="OpenArm bimanual robot preview" />
    </figure>
  `;

  const demos = document.createElement("section");
  demos.className = "splash__demos";
  const grid = document.createElement("div");
  grid.className = "demo-grid";
  for (const demo of DEMOS) {
    const button = document.createElement("button");
    button.className = "demo-card";
    button.type = "button";
    button.innerHTML = `
      <div>
        <div class="demo-card__eyebrow">${demo.eyebrow}</div>
        <h2>${demo.title}</h2>
        <p>${demo.description}</p>
      </div>
      <span class="demo-card__launch">Open demo</span>
    `;
    button.addEventListener("click", () => {
      history.pushState({}, "", `?demo=${demo.id}`);
      void openDemo(demo);
    });
    grid.append(button);
  }
  demos.append(grid);
  splash.append(intro, demos);
  appRoot.append(splash);
}

async function openDemo(demo: DemoDefinition): Promise<void> {
  activeViewer?.dispose();
  activeViewer = undefined;
  renderLoading(demo);
  try {
    activeViewer = await ViewerApp.create(appRoot, demo, () => {
      history.pushState({}, "", window.location.pathname);
      renderSplash();
    });
  } catch (error) {
    console.error(error);
    renderError(demo, error);
  }
}

function renderLoading(demo: DemoDefinition): void {
  appRoot.innerHTML = `
    <div class="loading">
      <div class="loading__box">
        <h1>Loading ${demo.title}</h1>
        <p>Preparing MuJoCo WASM, model XML, and mesh assets.</p>
      </div>
    </div>
  `;
}

function renderError(demo: DemoDefinition, error: unknown): void {
  const message = error instanceof Error ? error.message : String(error);
  appRoot.innerHTML = `
    <div class="error">
      <div class="error__box">
        <h1>${demo.title} did not load</h1>
        <p>${message}</p>
      </div>
    </div>
  `;
}

function route(): void {
  const params = new URLSearchParams(window.location.search);
  const demo = getDemo(params.get("demo"));
  if (demo) {
    void openDemo(demo);
  } else {
    renderSplash();
  }
}

window.addEventListener("popstate", route);
route();
