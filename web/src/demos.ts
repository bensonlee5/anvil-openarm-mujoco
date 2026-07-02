export type DemoId =
  | "bimanual"
  | "pedestal"
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
