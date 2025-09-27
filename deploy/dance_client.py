#!/usr/bin/env python3
"""
Dance Client - Simple HTTP client for dance_server.py
Usage: python3 dance_client.py
"""

import requests
import time
import json


class DanceClient:
    def __init__(self, base_url="http://localhost:8080"):
        self.base_url = base_url

    def start_timing(self):
        """Start timing collection"""
        try:
            response = requests.post(f"{self.base_url}/start")
            if response.status_code == 200:
                data = response.json()
                print(f"✓ {data['message']}")
                print(f"Total cues needed: {data['total_cues']}")
                return True
            else:
                print(f"✗ Error: {response.json().get('error', 'Unknown error')}")
                return False
        except requests.exceptions.ConnectionError:
            print("✗ Cannot connect to server. Is it running on localhost:8080?")
            return False

    def send_input(self):
        """Send timing cue (equivalent to pressing Enter)"""
        try:
            response = requests.post(f"{self.base_url}/input")
            if response.status_code == 200:
                data = response.json()
                print(f"✓ {data['message']}")
                if 'delay' in data:
                    print(f"  Delay: {data['delay']:.3f}s")
                if 'consensus_time' in data:
                    print(f"  🎭 Dance scheduled at: {data['consensus_time']:.3f}s")
                return True
            else:
                error_data = response.json()
                print(f"✗ Error: {error_data.get('error', 'Unknown error')}")
                return False
        except requests.exceptions.ConnectionError:
            print("✗ Cannot connect to server")
            return False

    def get_status(self):
        """Get current status"""
        try:
            response = requests.get(f"{self.base_url}/status")
            if response.status_code == 200:
                data = response.json()
                print(f"Status: {'Active' if data['active'] else 'Inactive'}")
                print(f"Progress: {data['cues_collected']}/{data['total_cues']} cues")
                if data['scheduled']:
                    print("🎭 Dance sequence scheduled!")
                return data
            else:
                print(f"✗ Error getting status")
                return None
        except requests.exceptions.ConnectionError:
            print("✗ Cannot connect to server")
            return None

    def get_cues(self):
        """Get collected cues and timing data"""
        try:
            response = requests.get(f"{self.base_url}/cues")
            if response.status_code == 200:
                data = response.json()
                print("Collected timing data:")
                print(json.dumps(data, indent=2))
                return data
            else:
                print(f"✗ Error getting cues")
                return None
        except requests.exceptions.ConnectionError:
            print("✗ Cannot connect to server")
            return None


def simple_mode(server_urls=None):
    """Simple mode - just press Enter for timing cues like original dance.py"""
    if server_urls is None:
        server_urls = ["http://localhost:8080"]

    # Create clients for each server
    clients = [DanceClient(url) for url in server_urls]

    print(f"🎭 Dance Timing Client - Controlling {len(clients)} robot(s)")
    for i, url in enumerate(server_urls, 1):
        print(f"  Robot {i}: {url}")

    print("Connecting to servers...")

    # Auto-start timing collection on all robots
    for i, client in enumerate(clients, 1):
        print(f"Starting Robot {i}...")
        if not client.start_timing():
            print(f"✗ Failed to start Robot {i}")
            return

    print("\nPress Enter for each timing cue (Ctrl+C to quit):")
    print("=" * 50)

    try:
        while True:
            # Check status of first robot (assuming all are in sync)
            status = clients[0].get_status()
            if not status:
                break

            if status['scheduled']:
                print("✓ All cues collected! Dance sequence scheduled.")
                print("Servers are handling the robot control pipeline.")
                break

            # Show progress and prompt
            prompt = f"Press Enter for cue {status['cues_collected'] + 1}/{status['total_cues']} (all {len(clients)} robots): "
            input(prompt)

            # Send the timing cue to all robots
            for i, client in enumerate(clients, 1):
                if not client.send_input():
                    print(f"✗ Failed to send cue to Robot {i}")

    except KeyboardInterrupt:
        print("\nStopped by user")
    except EOFError:
        print("\nStopped")




if __name__ == "__main__":
    import sys

    # Parse arguments for server URLs
    server_urls = None
    if "--servers" in sys.argv:
        server_idx = sys.argv.index("--servers")
        if server_idx + 1 < len(sys.argv):
            server_list = sys.argv[server_idx + 1]
            server_urls = server_list.split(",")
            print(f"Using servers: {server_urls}")

    # Default to localhost if no servers specified
    if server_urls is None:
        server_urls = ["http://localhost:8080"]

    # Just run simple mode (removed other unused modes)
    simple_mode(server_urls)