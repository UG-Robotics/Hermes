# Entry point to run simulations or tests
import argparse, pathlib, sys

# Ensure src is on sys.path so package imports work when running from repo root
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[0]))

from testing.test_transitions import run_simulation


def main():
    parser = argparse.ArgumentParser(description='Hermes entrypoint')
    parser.add_argument('--mode', choices=['simulate','test'], default='simulate')
    args = parser.parse_args()

    if args.mode in ('simulate','test'):
        run_simulation()


if __name__ == '__main__':
    main()

    
