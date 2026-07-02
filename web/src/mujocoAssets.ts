import loadMujoco, { type MainModule } from "@mujoco/mujoco";
import mujocoWasmUrl from "@mujoco/mujoco/mujoco.wasm?url";

const ASSET_ROOT = "sim-assets/anvil_openarm";
const WORKING_ROOT = "/working/anvil_openarm";

let mujocoPromise: Promise<MainModule> | undefined;
let prepared = false;

export { WORKING_ROOT };

export async function getMujoco(): Promise<MainModule> {
  mujocoPromise ??= loadMujoco({
    locateFile: (path: string) =>
      path.endsWith(".wasm") ? mujocoWasmUrl : path,
  });
  return mujocoPromise;
}

export async function prepareAssets(mujoco: MainModule): Promise<void> {
  if (prepared) {
    return;
  }

  mkdirTree(mujoco, WORKING_ROOT);

  const manifest = await fetchAsset("manifest.json").then((response) =>
    response.json() as Promise<{ files: string[] }>,
  );

  await Promise.all(
    manifest.files.map(async (file) => {
      const response = await fetchAsset(file);
      if (!response.ok) {
        throw new Error(`Failed to fetch ${file}: ${response.status}`);
      }
      mkdirTree(mujoco, `${WORKING_ROOT}/${parentDir(file)}`);
      const bytes = new Uint8Array(await response.arrayBuffer());
      mujoco.FS.writeFile(`${WORKING_ROOT}/${file}`, bytes);
    }),
  );

  prepared = true;
}

function fetchAsset(file: string): Promise<Response> {
  const base = import.meta.env.BASE_URL || "./";
  return fetch(`${base}${ASSET_ROOT}/${file}`);
}

function parentDir(file: string): string {
  const idx = file.lastIndexOf("/");
  return idx === -1 ? "" : file.slice(0, idx);
}

function mkdirTree(mujoco: MainModule, path: string): void {
  const parts = path.split("/").filter(Boolean);
  let current = "";
  for (const part of parts) {
    current += `/${part}`;
    if (!mujoco.FS.analyzePath(current, false).exists) {
      mujoco.FS.mkdir(current);
    }
  }
}
