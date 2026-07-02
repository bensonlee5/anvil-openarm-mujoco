import "./styles.css";

import { DEMOS, getDemo, type DemoDefinition } from "./demos";
import {
  describeCommandSurface,
  getSelectedLoaderProfile,
  loadLoaderProfiles,
  type LoaderProfile,
  type LoaderProfilePayload,
} from "./loaderProfiles";
import { ViewerApp } from "./simulator";

const queriedRoot = document.querySelector<HTMLDivElement>("#app");
if (!queriedRoot) {
  throw new Error("missing #app root");
}
const appRoot: HTMLDivElement = queriedRoot;

let activeViewer: ViewerApp | undefined;
let routeGeneration = 0;

function renderSplash(
  loaderProfiles: LoaderProfilePayload,
  selectedProfile: LoaderProfile | undefined,
): void {
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
      <h1>Anvil Openarm V2 demo</h1>
      <p class="splash__copy">
        Select an OpenArm v2 loader profile and browser-based MuJoCo scene, then operate either arm with joint-space keyboard teleop or the actuator sliders.
      </p>
    </div>
    <figure class="splash__media">
      <img src="${import.meta.env.BASE_URL}sim-assets/anvil_openarm/preview.png" alt="OpenArm bimanual robot preview" />
    </figure>
  `;

  const demos = document.createElement("section");
  demos.className = "splash__demos";

  const configPicker = createConfigPicker(loaderProfiles, selectedProfile);
  demos.append(configPicker);

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
      const configId = currentConfigId();
      history.pushState({}, "", buildUrl(demo.id, configId));
      void openDemo(demo, profileById(loaderProfiles.profiles, configId));
    });
    grid.append(button);
  }
  demos.append(grid);
  splash.append(intro, demos);
  appRoot.append(splash);
}

function createConfigPicker(
  loaderProfiles: LoaderProfilePayload,
  selectedProfile: LoaderProfile | undefined,
): HTMLElement {
  const section = document.createElement("section");
  section.className = "loader-config";

  const header = document.createElement("div");
  header.className = "loader-config__header";
  header.innerHTML = `
    <div>
      <div class="loader-config__eyebrow">OpenArm v2 config</div>
      <h2>Runtime profile</h2>
    </div>
  `;

  const details = document.createElement("div");
  details.className = "loader-config__details";

  if (loaderProfiles.profiles.length === 0) {
    details.textContent =
      "No loader profiles found. Run npm run prepare:assets after initializing upstream/anvil_loader.";
    section.append(header, details);
    return section;
  }

  const select = document.createElement("select");
  select.id = "loader-profile";
  select.className = "loader-config__select";
  select.setAttribute("aria-label", "OpenArm v2 loader profile");
  for (const profile of loaderProfiles.profiles) {
    const option = document.createElement("option");
    option.value = profile.id;
    option.textContent = profile.title;
    select.append(option);
  }
  if (selectedProfile) {
    select.value = selectedProfile.id;
  }

  header.append(select);
  section.append(header, details);

  const updateDetails = (profile: LoaderProfile | undefined): void => {
    if (!profile) {
      details.textContent = "";
      return;
    }
    details.innerHTML = `
      <p>${escapeHtml(profile.summary)}</p>
      <dl>
        <div><dt>YAML</dt><dd><a href="${profile.sourceUrl}" target="_blank" rel="noreferrer">${profile.filename}</a></dd></div>
        <div><dt>Mode</dt><dd>${escapeHtml(profile.controlMode)}</dd></div>
        <div><dt>Surface</dt><dd>${escapeHtml(describeCommandSurface(profile))}</dd></div>
        <div><dt>Arms</dt><dd>${escapeHtml(formatArms(profile))}</dd></div>
      </dl>
      <p class="loader-config__support">${escapeHtml(profile.repoSupport)}</p>
    `;
  };

  select.addEventListener("change", () => {
    const profile = profileById(loaderProfiles.profiles, select.value);
    history.replaceState({}, "", buildUrl(undefined, profile?.id));
    updateDetails(profile);
  });
  updateDetails(selectedProfile ?? loaderProfiles.profiles[0]);
  return section;
}

async function openDemo(
  demo: DemoDefinition,
  loaderProfile: LoaderProfile | undefined,
): Promise<void> {
  activeViewer?.dispose();
  activeViewer = undefined;
  renderLoading(demo, loaderProfile);
  try {
    activeViewer = await ViewerApp.create(appRoot, demo, loaderProfile, () => {
      history.pushState({}, "", buildUrl(undefined, loaderProfile?.id));
      void route();
    });
  } catch (error) {
    console.error(error);
    renderError(demo, error);
  }
}

function renderLoading(
  demo: DemoDefinition,
  loaderProfile: LoaderProfile | undefined,
): void {
  appRoot.innerHTML = `
    <div class="loading">
      <div class="loading__box">
        <h1>Loading ${demo.title}</h1>
        <p>Preparing MuJoCo WASM, model XML, mesh assets, and ${loaderProfile?.title ?? "OpenArm v2"} profile metadata.</p>
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

async function route(): Promise<void> {
  const generation = ++routeGeneration;
  const loaderProfiles = await loadLoaderProfiles();
  if (generation !== routeGeneration) {
    return;
  }
  const params = new URLSearchParams(window.location.search);
  const profile = getSelectedLoaderProfile(
    loaderProfiles.profiles,
    params.get("config"),
  );
  const demo = getDemo(params.get("demo"));
  if (demo) {
    void openDemo(demo, profile);
  } else {
    renderSplash(loaderProfiles, profile);
  }
}

function currentConfigId(): string | undefined {
  return document.querySelector<HTMLSelectElement>("#loader-profile")?.value;
}

function profileById(
  profiles: LoaderProfile[],
  id: string | undefined,
): LoaderProfile | undefined {
  return profiles.find((profile) => profile.id === id);
}

function buildUrl(
  demoId: string | undefined,
  configId: string | undefined,
): string {
  const params = new URLSearchParams();
  if (demoId) {
    params.set("demo", demoId);
  }
  if (configId) {
    params.set("config", configId);
  }
  const query = params.toString();
  return query
    ? `${window.location.pathname}?${query}`
    : window.location.pathname;
}

function formatArms(profile: LoaderProfile): string {
  return profile.arms
    .map((arm) => {
      const extras = [
        arm.canInterfaceName ? `CAN ${arm.canInterfaceName}` : "",
        arm.vrController ? `VR ${arm.vrController}` : "",
      ].filter(Boolean);
      return extras.length ? `${arm.name} (${extras.join(", ")})` : arm.name;
    })
    .join(", ");
}

function escapeHtml(value: string): string {
  return value.replace(/[&<>"']/g, (char) => {
    switch (char) {
      case "&":
        return "&amp;";
      case "<":
        return "&lt;";
      case ">":
        return "&gt;";
      case '"':
        return "&quot;";
      default:
        return "&#039;";
    }
  });
}

window.addEventListener("popstate", () => {
  void route();
});
void route();
