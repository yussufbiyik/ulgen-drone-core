import importlib
import argparse
import sys
import os
import asyncio
import inspect

def parse_args():
    parser = argparse.ArgumentParser(
        description="Run main() from a testable module (supports sync & async)."
    )
    parser.add_argument(
        "module",
        help="Module path in format: <package>.<module> (e.g., controllers.drone_controller)",
    )
    return parser.parse_args()

async def run_main(main_func):
    if inspect.iscoroutinefunction(main_func):
        await main_func()
    else:
        main_func()

def main():
    args = parse_args()
    module_path = args.module

    try:
        mod = importlib.import_module(module_path)
        if not hasattr(mod, "main"):
            print(f"The module '{module_path}' does not have a 'main()' function.")
            sys.exit(1)

        main_func = getattr(mod, "main")
        print(f"Running main() from {module_path}...")

        asyncio.run(run_main(main_func))

    except ModuleNotFoundError as e:
        print(f"Module not found: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Error while running the module: {e}")
        sys.exit(1)

if __name__ == "__main__":
    # Ensure root is in path
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    main()
