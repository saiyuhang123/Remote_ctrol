"""Keep each embedded RViz lifetime in a separate process (Ogre globals)."""
import os
import subprocess
import sys
import tempfile


def main():
    binary = sys.argv[1]
    cases = [
        "initialWindowFitsAvailableScreen",
        "odometrySourceFollowsModeAndPreservesNavigationChoice",
        "responsiveLayout:full-hd",
        "responsiveLayout:laptop",
        "responsiveLayout:compact",
        "presentationKeepsUnknownReverseAndInterlockSeparate",
        "patrolSummarySurvivesIdleNavigationUpdates",
        "writesOnlyLayoutPreferences",
        "restoresLayoutWithoutAuthorization",
        "corruptedLayoutFallsBackToAccessiblePanels",
    ]
    with tempfile.TemporaryDirectory(prefix="desktop-console-settings-") as settings:
        for case in cases:
            env = dict(os.environ, DESKTOP_CONSOLE_SETTINGS_DIR=settings)
            print(f"Console scenario: {case}", flush=True)
            result = subprocess.run([binary, case], env=env, timeout=30, check=False)
            if result.returncode:
                return result.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
