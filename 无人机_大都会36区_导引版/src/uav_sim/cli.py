"""Command-line entry point for the UAV simulation application."""

import argparse
import json
from pathlib import Path
from uav_sim.app import main as run_app
from uav_sim.guidance import GuidanceConfig, PARAMETERS


def main(argv=None):
    parser = argparse.ArgumentParser(description='UAV simulation: original GUI or a reproducible JSON configuration')
    parser.add_argument('--config', type=Path, help='Run the same application headlessly using a JSON configuration')
    guidance = parser.add_argument_group('guidance overrides (use with --config)')
    guidance.add_argument('--guidance', dest='enabled', action=argparse.BooleanOptionalAction,
                          default=None, help='enable or disable traffic guidance')
    guidance.add_argument('--guidance-evaluate', dest='evaluate', action=argparse.BooleanOptionalAction,
                          default=None, help='sample congestion metrics even if guidance is disabled')
    for key, (label, low, high, decimals) in PARAMETERS.items():
        guidance.add_argument('--guidance-' + key.replace('_', '-'), dest=key,
                              type=float if decimals else int, help='{} [{}, {}]'.format(label, low, high))
    args = parser.parse_args(argv)
    overrides = {key: getattr(args, key) for key in ('enabled', 'evaluate', *PARAMETERS)
                 if getattr(args, key) is not None}
    if args.config:
        from uav_sim.app import run_config
        config = json.loads(args.config.read_text(encoding='utf-8-sig'))
        if not isinstance(config, dict):
            parser.error('Configuration must be a JSON object')
        if overrides or 'guidance' in config:
            settings = config.get('guidance', {})
            if not isinstance(settings, dict):
                parser.error('guidance must be a JSON object')
            settings = dict(settings, **overrides)
            try:
                GuidanceConfig.from_mapping(settings)
            except ValueError as error:
                parser.error(str(error))
            config['guidance'] = settings
        return run_config(config)
    if overrides:
        parser.error('Guidance overrides require --config.')
    return run_app()


if __name__ == "__main__":
    main()

