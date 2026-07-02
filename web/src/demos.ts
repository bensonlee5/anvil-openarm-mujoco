export type DemoId =
  | "bimanual"
  | "pedestal"
  | "cell"
  | "manipulation"
  | "wrist-sweep"
  | "full-rom";

export type ScriptedController = "wristSweep" | "fullRom";

export interface DemoDefinition {
  id: DemoId;
  title: string;
  eyebrow: string;
  description: string;
  xmlPath: string;
  script?: ScriptedController;
  camera: {
    position: [number, number, number];
    target: [number, number, number];
  };
}

export const DEMOS: DemoDefinition[] = [
  {
    id: "bimanual",
    title: "Bimanual Arm",
    eyebrow: "Base model",
    description: "Open the generated Anvil OpenARM bimanual model in a neutral scene.",
    xmlPath: "scene.xml",
    camera: {
      position: [2.15, 1.75, 2.05],
      target: [0, 0.85, 0],
    },
  },
  {
    id: "pedestal",
    title: "Pedestal",
    eyebrow: "Mounted view",
    description: "Operate the arm on its pedestal with floor, lights, and the home keyframe.",
    xmlPath: "pedestal.xml",
    camera: {
      position: [2.25, 1.7, 2.2],
      target: [0, 0.82, 0],
    },
  },
  {
    id: "cell",
    title: "Workcell",
    eyebrow: "Cell scene",
    description: "Explore the OpenArm workcell with lifter, sheet, walls, and the bimanual arm.",
    xmlPath: "cell.xml",
    camera: {
      position: [2.35, 1.95, 2.3],
      target: [0.15, 0.9, -0.05],
    },
  },
  {
    id: "manipulation",
    title: "Manipulation Demo",
    eyebrow: "Props",
    description: "Load the workcell with manipulable objects for browser-side interaction.",
    xmlPath: "demo.xml",
    camera: {
      position: [2.2, 1.85, 2.05],
      target: [0.25, 0.94, -0.02],
    },
  },
  {
    id: "wrist-sweep",
    title: "Wrist Sweep",
    eyebrow: "Scripted motion",
    description: "Run the Anvil J6 and J7 range-of-motion sweep in the hosted viewer.",
    xmlPath: "pedestal.xml",
    script: "wristSweep",
    camera: {
      position: [1.6, 1.55, 1.45],
      target: [0, 0.92, 0],
    },
  },
  {
    id: "full-rom",
    title: "Full Range of Motion",
    eyebrow: "Scripted motion",
    description:
      "Sweep every joint J1–J7 and the gripper through its full Anvil range, one joint at a time on both arms.",
    xmlPath: "scene.xml",
    script: "fullRom",
    camera: {
      position: [2.6, 1.95, 2.45],
      target: [0, 0.95, 0],
    },
  },
];

export function getDemo(id: string | null): DemoDefinition | undefined {
  return DEMOS.find((demo) => demo.id === id);
}
