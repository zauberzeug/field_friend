#!/usr/bin/env python3
"""
System update script for Field Friend with Air Link and Nebula VPN.
Checks internet connection, installs/updates Air Link service with Air Link token,
and updates Nebula certificate files.

Usage: python update_system.py [options] <cert_file1> <cert_file2> ...
Example: python update_system.py --on-air-token "your-token-here" host.crt host.key
         python update_system.py host.crt host.key ca.crt config.yaml
"""

import argparse
import logging
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
log = logging.getLogger(__name__)

# Internet connectivity check (from rosys codebase)
IP_ADDRESS_LIST = [
    '1.1.1.1',  # Cloudflare
    '1.0.0.1',
    '8.8.8.8',  # Google DNS
    '8.8.4.4',
    '208.67.222.222',  # Open DNS
    '208.67.220.220',
]
PORT = 53
NEBULA_CONFIG_DIR = Path('/etc/nebula')


def has_internet() -> bool:
    """Returns True if there's a connection"""
    for host in IP_ADDRESS_LIST:
        try:
            socket.setdefaulttimeout(3)
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, PORT))
            log.info(f'Internet connection confirmed via {host}')
            return True
        except OSError:
            continue
    log.warning('No internet connection detected')
    return False


def install_air_link_package() -> bool:
    """Install or update the air-link package"""
    try:
        log.info('Installing/updating air-link package...')
        subprocess.run(['pip', 'install', '--upgrade', 'air-link'],
                       check=True, capture_output=True, text=True, timeout=120)
        log.info('Air Link package installed/updated successfully')
        return True
    except subprocess.CalledProcessError as e:
        log.error(f'Failed to install air-link package: {e}')
        log.error(f'Error output: {e.stderr}')
        return False
    except subprocess.TimeoutExpired:
        log.error('Package installation timed out')
        return False


def setup_air_link_service(on_air_token: str | None = None) -> bool:
    """Setup Air Link as a system service with On Air token"""
    try:
        # Check if air-link command is available
        result = subprocess.run(['which', 'air-link'],
                                check=False, capture_output=True, text=True)
        if result.returncode != 0:
            log.error('air-link command not found after installation')
            return False

        # Install air-link as service
        log.info('Setting up Air Link service...')
        install_cmd = ['air-link', 'install']

        if on_air_token:
            install_cmd.append(on_air_token)
            log.info('Installing Air Link service with On Air token')
        else:
            log.info('Installing Air Link service without On Air token')

        # NOTE: air-link install handles sudo internally
        result = subprocess.run(install_cmd, check=False, text=True, timeout=120)

        if result.returncode == 0:
            log.info('Air Link service setup completed')
            return True
        log.error('Air Link service setup failed')
        return False

    except subprocess.TimeoutExpired:
        log.error('Air Link service setup timed out')
        return False
    except Exception as e:
        log.error(f'Failed to setup Air Link service: {e}')
        return False


def set_air_link_token(token: str) -> bool:
    """Set On Air token for existing Air Link installation"""
    try:
        log.info('Setting On Air token for Air Link...')
        subprocess.run(['air-link', 'set-token', token],
                       check=True, capture_output=True, text=True, timeout=30)
        log.info('On Air token set successfully')
        return True
    except subprocess.CalledProcessError as e:
        log.error(f'Failed to set On Air token: {e}')
        log.error(f'Error output: {e.stderr}')
        return False
    except subprocess.TimeoutExpired:
        log.error('Setting token timed out')
        return False


def restart_air_link_service() -> bool:
    """Restart Air Link service with sudo if needed"""
    try:
        log.info('Restarting Air Link service...')

        # Check if we need sudo
        restart_cmd = ['systemctl', 'restart', 'air-link']
        if os.getuid() != 0:
            restart_cmd = ['sudo', *restart_cmd]

        result = subprocess.run(restart_cmd, check=False, capture_output=True, text=True, timeout=30)

        if result.returncode != 0:
            log.error(f'Failed to restart service: {result.stderr}')
            return False

        # Wait a moment and check if service is running
        time.sleep(3)
        status_cmd = ['systemctl', 'is-active', 'air-link']
        if os.getuid() != 0:
            status_cmd = ['sudo', *status_cmd]

        status_result = subprocess.run(status_cmd, check=False, capture_output=True, text=True)

        if status_result.returncode == 0:
            log.info('Air Link service is running')
            return True
        log.error('Air Link service failed to start')
        return False

    except subprocess.TimeoutExpired:
        log.error('Service restart timed out')
        return False
    except Exception as e:
        log.error(f'Failed to restart Air Link service: {e}')
        return False


def update_air_link(on_air_token: str | None = None) -> bool:
    """Install/update Air Link service with On Air token"""
    try:
        # Only proceed if a token is provided
        if not on_air_token:
            log.info('No On Air token provided, skipping Air Link setup')
            return True

        # First, install or update the package
        if not install_air_link_package():
            return False

        # Check if service already exists
        service_cmd = ['systemctl', 'list-units', '--full', '--all', 'air-link.service']
        if os.getuid() != 0:
            service_cmd = ['sudo', *service_cmd]

        service_check = subprocess.run(service_cmd, check=False, capture_output=True, text=True)
        service_exists = 'air-link.service' in service_check.stdout

        if not service_exists:
            # Service doesn't exist, install it
            if not setup_air_link_service(on_air_token):
                return False
        else:
            # Service exists, set token if provided and restart
            if on_air_token:
                if not set_air_link_token(on_air_token):
                    log.warning('Failed to set token, but continuing...')

            if not restart_air_link_service():
                return False

        log.info('✓ Air Link service ready')
        return True

    except Exception as e:
        log.error(f'Failed to update Air Link: {e}')
        return False


def update_nebula_file(source_file: str, target_name: str | None = None) -> bool:
    """Update a Nebula certificate file"""
    try:
        source = Path(source_file)

        if not source.exists():
            log.error(f'Source file does not exist: {source}')
            return False

        # Use source filename if target not specified
        if target_name is None:
            target_name = source.name

        target = NEBULA_CONFIG_DIR / target_name

        # Create target directory if it doesn't exist
        if not NEBULA_CONFIG_DIR.exists():
            mkdir_cmd = ['sudo', 'mkdir', '-p', str(NEBULA_CONFIG_DIR)]
            subprocess.run(mkdir_cmd, check=True, capture_output=True, text=True)
            log.info(f'Created directory {NEBULA_CONFIG_DIR}')

        # Backup existing file if it exists
        if target.exists():
            backup_path = target.with_suffix(target.suffix + '.backup')
            backup_cmd = ['sudo', 'cp', str(target), str(backup_path)]
            subprocess.run(backup_cmd, check=True, capture_output=True, text=True)
            log.info(f'Backed up existing file to {backup_path}')

        # Copy the file with sudo
        copy_cmd = ['sudo', 'cp', str(source), str(target)]
        subprocess.run(copy_cmd, check=True, capture_output=True, text=True)

        log.info(f'Successfully updated {target} from {source}')
        return True

    except subprocess.CalledProcessError as e:
        log.error(f'Failed to update Nebula file {source_file}: {e}')
        return False
    except Exception as e:
        log.error(f'Failed to update Nebula file {source_file}: {e}')
        return False


def restart_nebula_service() -> bool:
    """Restart Nebula service if it exists"""
    try:
        # Check if nebula service exists
        check_cmd = ['systemctl', 'is-enabled', 'nebula']
        if os.getuid() != 0:
            check_cmd = ['sudo', *check_cmd]

        result = subprocess.run(check_cmd, check=False, capture_output=True, text=True)

        if result.returncode == 0:
            log.info('Restarting Nebula service...')
            restart_cmd = ['systemctl', 'restart', 'nebula']
            if os.getuid() != 0:
                restart_cmd = ['sudo', *restart_cmd]

            subprocess.run(restart_cmd, check=True, capture_output=True, text=True)
            log.info('Nebula service restarted successfully')
            return True
        log.info('Nebula service not found, skipping restart')
        return True

    except subprocess.CalledProcessError as e:
        log.error(f'Failed to restart Nebula service: {e}')
        return False


def suggest_reboot() -> None:
    """Suggest system reboot for certificate changes to take effect"""
    log.info('=' * 50)
    log.info('IMPORTANT: Certificate files have been updated.')
    log.info('Please reboot the device for changes to take effect:')
    log.info('  sudo reboot')
    log.info('=' * 50)


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Update system: check internet, install/update Air Link with On Air token, and update Nebula certificate files',
        epilog='''Examples:
  python update_system.py host.crt host.key
  python update_system.py --on-air-token "your-token" host.crt host.key
  python update_system.py host.crt host.key ca.crt config.yaml''',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('files', nargs='*',
                        help='Certificate files to update (e.g., host.crt host.key ca.crt config.yaml)')
    parser.add_argument('--on-air-token', '-t', type=str,
                        help='On Air token for NiceGUI remote access')
    parser.add_argument('--skip-internet-check', action='store_true',
                        help='Skip internet connectivity check')
    parser.add_argument('--auto-reboot', action='store_true',
                        help='Automatically reboot after successful update')

    args = parser.parse_args()

    # If no files provided and not updating air-link, show help
    if not args.files and not args.on_air_token:
        parser.print_help()
        sys.exit(1)

    success = True

    log.info('Starting system update process...')

    # Check internet connection
    if not args.skip_internet_check:
        log.info('Checking internet connection...')
        if not has_internet():
            log.error('No internet connection available')
            success = False
        else:
            log.info('✓ Internet connection OK')

    # Update Air Link
    if success:
        if not update_air_link(args.on_air_token):
            log.error('✗ Failed to update Air Link')
            success = False
        else:
            log.info('✓ Air Link updated successfully')

    # Update Nebula certificate files
    nebula_files_updated = False
    for cert_file in args.files:
        if update_nebula_file(cert_file):
            nebula_files_updated = True
        else:
            success = False

    # Restart Nebula service if files were updated
    if nebula_files_updated:
        if restart_nebula_service():
            log.info('✓ Nebula service handling completed')
        else:
            log.warning('⚠ Nebula service restart failed, but continuing...')

    if success:
        log.info('✓ All updates completed successfully')

        if nebula_files_updated:
            if args.auto_reboot:
                log.info('Auto-rebooting in 5 seconds...')
                time.sleep(5)
                subprocess.run(['sudo', 'reboot'], check=False)
            else:
                suggest_reboot()

        sys.exit(0)
    else:
        log.error('✗ Some updates failed')
        sys.exit(1)


if __name__ == '__main__':
    main()
