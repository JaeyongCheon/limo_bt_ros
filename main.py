import asyncio
import argparse
import cProfile

from modules.utils import set_config

# -----------------------------
# Parse command line arguments
# -----------------------------
parser = argparse.ArgumentParser(description="py_bt_ros")
parser.add_argument(
    "--config",
    type=str,
    default="config.yaml",
    help="Path to config yaml",
)
args = parser.parse_args()

# -----------------------------
# Load configuration & BT runner
# -----------------------------
set_config(args.config)
from modules.utils import config
from modules.bt_runner import BTRunner

bt_runner = BTRunner(config)


# -----------------------------
# Main async loop
# -----------------------------
async def loop():
    while bt_runner.running:
        bt_runner.handle_keyboard_events()
        if not bt_runner.paused:
            await bt_runner.step()
        bt_runner.render()

    bt_runner.close()


def main():
    asyncio.run(loop())


# -----------------------------
# Entry point
# -----------------------------
if __name__ == "__main__":
    # profiling_mode이 없으면 기본값 False 사용
    profiling = config.get("bt_runner", {}).get("profiling_mode", False)

    if profiling:
        cProfile.run("main()", sort="cumulative")
    else:
        main()
