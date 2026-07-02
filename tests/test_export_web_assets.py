from scripts.export_web_assets import (
    ANVIL_LOADER_CONFIGS,
    OPENARM_V2_CONFIG_FILES,
    extract_loader_profile,
)


def test_extracts_openarm_v2_loader_profiles_from_submodule():
    profiles = [
        extract_loader_profile(ANVIL_LOADER_CONFIGS / filename)
        for filename in OPENARM_V2_CONFIG_FILES
    ]

    assert [profile["id"] for profile in profiles] == [
        "openarm_v2_inference",
        "openarm_v2_leader_follower_teleop",
        "openarm_v2_leader_only",
        "openarm_v2_quest_teleop",
        "openarm_v2_quest_teleop_commanded_ee",
    ]

    by_id = {profile["id"]: profile for profile in profiles}
    assert by_id["openarm_v2_inference"]["armType"] == "openarm_v2"
    assert by_id["openarm_v2_inference"]["commandSurface"] == "joint_position_policy"
    assert by_id["openarm_v2_quest_teleop_commanded_ee"]["commandedEe"] is True
    assert (
        by_id["openarm_v2_quest_teleop_commanded_ee"]["commandSurface"]
        == "commanded_ee"
    )
    assert by_id["openarm_v2_quest_teleop"]["arms"] == [
        {
            "name": "follower_l",
            "canInterfaceName": "follower_l",
            "vrController": "left",
        },
        {
            "name": "follower_r",
            "canInterfaceName": "follower_r",
            "vrController": "right",
        },
    ]
