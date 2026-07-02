export const DEFAULT_LOADER_PROFILE_ID = "openarm_v2_quest_teleop_commanded_ee";

export interface LoaderArm {
  name: string;
  canInterfaceName?: string;
  vrController?: string;
}

export interface LoaderProfile {
  id: string;
  title: string;
  summary: string;
  filename: string;
  sourcePath: string;
  sourceUrl: string;
  armType: string;
  controlMode: string;
  commandedEe: boolean;
  homingVelocity?: number;
  commandSurface: string;
  arms: LoaderArm[];
  repoSupport: string;
}

export interface LoaderProfilePayload {
  sourceRepo: string;
  sourcePath: string;
  profiles: LoaderProfile[];
}

let cachedProfiles: Promise<LoaderProfilePayload> | undefined;

export async function loadLoaderProfiles(): Promise<LoaderProfilePayload> {
  cachedProfiles ??= fetch(
    `${import.meta.env.BASE_URL}sim-assets/anvil_openarm/openarm_v2_configs.json`,
  )
    .then((response) => {
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      return response.json() as Promise<LoaderProfilePayload>;
    })
    .catch((error) => {
      console.warn("Could not load OpenArm v2 loader profiles", error);
      return {
        sourceRepo: "https://github.com/anvil-robotics/anvil-loader",
        sourcePath: "upstream/anvil_loader/config",
        profiles: [],
      };
    });
  return cachedProfiles;
}

export function getSelectedLoaderProfile(
  profiles: LoaderProfile[],
  requestedId: string | null,
): LoaderProfile | undefined {
  return (
    profiles.find((profile) => profile.id === requestedId) ??
    profiles.find((profile) => profile.id === DEFAULT_LOADER_PROFILE_ID) ??
    profiles[0]
  );
}

export function describeCommandSurface(profile: LoaderProfile): string {
  switch (profile.commandSurface) {
    case "commanded_ee":
      return "Commanded EE";
    case "joint_position_policy":
      return "Joint policy";
    case "quest_joint_position":
      return "Quest joint";
    case "leader_follower":
      return "Leader-follower";
    case "leader_only":
      return "Leader only";
    default:
      return profile.controlMode.replaceAll("_", " ");
  }
}
