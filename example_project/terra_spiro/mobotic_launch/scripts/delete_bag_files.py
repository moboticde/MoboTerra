import argparse
import os
import shutil
import time

import psutil  # For disk usage


def delete_old_folders(directory, days, prefix):
    """Deletes folders older than the specified number of days."""
    now = time.time()
    cutoff = now - (days * 24 * 60 * 60)  # Calculate cutoff time

    deleted_count = 0
    for item in os.listdir(directory):
        item_path = os.path.join(directory, item)
        if os.path.isdir(item_path) and item.startswith(prefix):
            try:
                # Get modification time (metadata may not always be reliable)
                modified_time = os.path.getmtime(item_path)

                if modified_time < cutoff:
                    shutil.rmtree(item_path)  # Delete the folder and its content.
                    print(f"Deleted folder: {item_path}")
                    deleted_count += 1
            except OSError as e:
                print(f"Error deleting {item_path}: {e}")
    return deleted_count


def check_and_delete_if_disk_full(directory, threshold_percent, num_to_delete, prefix):
    """Checks disk usage and deletes oldest folders if above threshold."""

    disk = psutil.disk_usage(directory)  # Get disk usage info
    if disk.percent > threshold_percent:
        print(f"Disk usage above threshold ({disk.percent}%)")

        folders = []
        for item in os.listdir(directory):
            item_path = os.path.join(directory, item)
            if os.path.isdir(item_path) and item.startswith(prefix):
                try:
                    modified_time = os.path.getmtime(item_path)
                    folders.append((modified_time, item_path))
                except OSError as e:
                    print(f"Error getting time for {item_path}: {e}")

        if folders:  # Check if any folders were found
            folders.sort(key=lambda x: x[0])  # Sort by modification time (oldest first)
            folders_to_delete = folders[:num_to_delete]

            for _, folder_path in folders_to_delete:
                try:
                    shutil.rmtree(folder_path)
                    print(f"Deleted folder (disk full): {folder_path}")
                except OSError as e:
                    print(f"Error deleting {folder_path}: {e}")
        else:
            print("No folders found to delete (disk full)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Clean up old folders and manage disk space.")

    parser.add_argument("directory", help="The directory to clean.")
    parser.add_argument("-d", "--days", type=int, default=30,
                        help="Keep folders younger than this many days (default: 30).")
    parser.add_argument("-t", "--threshold", type=int, default=80,
                        help="Delete folders if disk usage is above this percentage (default: 80).")
    parser.add_argument("-n", "--num-delete", type=int, default=5,
                        help="Number of folders to delete if disk is full (default: 5).")
    parser.add_argument("-p", "--prefix", type=str, default="rosbag2",
                        help="Prefix of the folders to delete (default: 'rosbag2').")

    args = parser.parse_args()

    deleted_old = delete_old_folders(args.directory, args.days, args.prefix)

    check_and_delete_if_disk_full(args.directory, args.threshold, args.num_delete, args.prefix)
