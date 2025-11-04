#!/usr/bin/env python3
"""
Monitor i.MX 93 extraction progress in real-time.

Usage:
    python scripts/monitor_imx93_progress.py
    python scripts/monitor_imx93_progress.py --watch  # Auto-refresh every 5 seconds
"""

import argparse
import json
import time
from pathlib import Path
from datetime import datetime

CHECKPOINT_FILE = Path('/Users/alexjokela/projects/ballistics/data/imx93_extraction.checkpoint.json')


def load_checkpoint():
    """Load the checkpoint file."""
    if not CHECKPOINT_FILE.exists():
        return None

    try:
        with open(CHECKPOINT_FILE, 'r') as f:
            return json.load(f)
    except Exception as e:
        return {'error': str(e)}


def display_progress(checkpoint, clear_screen=False):
    """Display extraction progress."""
    if clear_screen:
        print('\033[2J\033[H')  # Clear screen and move cursor to top

    print("=" * 80)
    print("i.MX 93 Datasheet Extraction - Real-Time Progress")
    print("=" * 80)
    print(f"Last Updated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

    if checkpoint is None:
        print("⏳ Status: Checkpoint file not found")
        print("   Either extraction hasn't started or is still converting PDF pages.\n")
        return

    if 'error' in checkpoint:
        print(f"❌ Error reading checkpoint: {checkpoint['error']}\n")
        return

    # Progress bar
    progress = checkpoint.get('progress_percent', 0)
    bar_length = 50
    filled = int(bar_length * progress / 100)
    bar = '█' * filled + '░' * (bar_length - filled)

    print(f"📊 Progress: [{bar}] {progress}%")
    print(f"   Pages: {checkpoint.get('pages_completed', 0)}/{checkpoint.get('total_pages', 109)}")
    print(f"   Last completed: Page {checkpoint.get('last_completed_page', 0)}")
    print()

    # Extraction statistics
    print("📈 Extracted Data:")
    print(f"   - Tables: {checkpoint.get('tables_extracted', 0)}")
    print(f"   - Specifications: {checkpoint.get('specs_extracted', 0)}")
    print(f"   - Registers: {checkpoint.get('registers_extracted', 0)}")
    print()

    # Time estimates
    pages_done = checkpoint.get('pages_completed', 0)
    pages_remaining = checkpoint.get('total_pages', 109) - pages_done

    if pages_done > 0:
        # Estimate based on 30 seconds per page
        seconds_remaining = pages_remaining * 30
        minutes_remaining = seconds_remaining / 60

        print("⏱️  Estimated Time:")
        print(f"   - Pages remaining: {pages_remaining}")
        print(f"   - Time remaining: ~{int(minutes_remaining)} minutes")
        print()

    print("=" * 80)


def main():
    parser = argparse.ArgumentParser(description='Monitor i.MX 93 extraction progress')
    parser.add_argument('--watch', action='store_true', help='Auto-refresh every 5 seconds')
    parser.add_argument('--interval', type=int, default=5, help='Refresh interval in seconds (default: 5)')
    args = parser.parse_args()

    try:
        if args.watch:
            print("📡 Monitoring extraction progress (Ctrl+C to stop)...\n")
            while True:
                checkpoint = load_checkpoint()
                display_progress(checkpoint, clear_screen=True)

                # Check if complete
                if checkpoint and checkpoint.get('progress_percent', 0) >= 100:
                    print("\n✅ Extraction complete! Checkpoint file will be removed soon.")
                    break

                time.sleep(args.interval)
        else:
            checkpoint = load_checkpoint()
            display_progress(checkpoint)

    except KeyboardInterrupt:
        print("\n\n⏸️  Monitoring stopped by user.\n")


if __name__ == '__main__':
    main()
